use std::collections::HashSet;
use std::time::Instant;

use crate::config::*;
use crate::stats::Statistics;
use crate::vehicle::{Direction, Route, Vehicle, VehicleState};

pub struct Intersection {
    pub vehicles: Vec<Vehicle>,
    pub reserved_slots: HashSet<u64>,
    pub total_passed: usize,
}

impl Intersection {
    pub fn new() -> Self {
        Intersection {
            vehicles: Vec::new(),
            reserved_slots: HashSet::new(),
            total_passed: 0,
        }
    }

    pub fn add_vehicle(&mut self, v: Vehicle) {
        self.vehicles.push(v);
    }

    pub fn update(&mut self, dt: f64, stats: &mut Statistics) {
        self.apply_smart_control();
        self.apply_safe_distance();

        for v in &mut self.vehicles {
            v.update(dt);
        }

        self.check_intersection_entry();
        self.check_intersection_exit(stats);

        let close_calls = self.detect_close_calls();
        stats.close_calls += close_calls;

        self.remove_done_vehicles(stats);
    }

    fn apply_smart_control(&mut self) {
        let ix = INTERSECTION_X;
        let iy = INTERSECTION_Y;
        let iw = ROAD_WIDTH;
        let approach_zone = iw * 0.8;

        let reservation_holders: Vec<(u64, Direction, Route, usize)> = self
            .vehicles
            .iter()
            .filter(|v| v.state == VehicleState::InIntersection)
            .map(|v| (v.id, v.direction, v.route, v.lane_index))
            .collect();

        for v in &mut self.vehicles {
            if v.state != VehicleState::Approaching {
                continue;
            }

            let dist_to_intersection = distance_to_intersection_entry(v, ix, iy, iw);

            if dist_to_intersection > approach_zone {
                v.target_velocity = SPEED_HIGH;
                continue;
            }

            let conflicts = reservation_holders.iter().any(|(rid, rdir, rroute, _rlane)| {
                *rid != v.id && paths_conflict(v.direction, v.route, *rdir, *rroute)
            });

            if conflicts {
                if dist_to_intersection < SAFE_DISTANCE * 1.2 {
                    v.target_velocity = 0.0;
                } else {
                    v.target_velocity = SPEED_LOW;
                }
            } else {
                v.target_velocity = SPEED_MED;
            }
        }
    }

    fn apply_safe_distance(&mut self) {
        let count = self.vehicles.len();
        for i in 0..count {
            for j in 0..count {
                if i == j {
                    continue;
                }
                let (ahead, behind) = if self.vehicles[i].id > self.vehicles[j].id {
                    let (a, b) = self.vehicles.split_at_mut(i);
                    (&b[0], &a[j])
                } else {
                    let (a, b) = self.vehicles.split_at_mut(j);
                    (&a[i], &b[0])
                };

                if ahead.direction != behind.direction {
                    continue;
                }
                if ahead.lane_index != behind.lane_index
                    && ahead.state != VehicleState::InIntersection
                    && behind.state != VehicleState::InIntersection
                {
                    continue;
                }

                let dist = ((ahead.x - behind.x).powi(2) + (ahead.y - behind.y).powi(2)).sqrt();
                let _ = (dist, ahead, behind);
            }
        }

        let snapshots: Vec<(usize, f64, f64, Direction, usize, VehicleState)> = self
            .vehicles
            .iter()
            .enumerate()
            .map(|(i, v)| (i, v.x, v.y, v.direction, v.lane_index, v.state))
            .collect();

        for i in 0..self.vehicles.len() {
            let vi = &snapshots[i];
            for j in 0..snapshots.len() {
                if i == j {
                    continue;
                }
                let vj = &snapshots[j];

                if vi.3 != vj.3 {
                    continue;
                }
                if vi.4 != vj.4
                    && vi.5 != VehicleState::InIntersection
                    && vj.5 != VehicleState::InIntersection
                {
                    continue;
                }

                let is_ahead = match vi.3 {
                    Direction::South => vj.2 < vi.2,
                    Direction::North => vj.2 > vi.2,
                    Direction::East => vj.1 > vi.1,
                    Direction::West => vj.1 < vi.1,
                };

                if !is_ahead {
                    continue;
                }

                let dist = ((vi.1 - vj.1).powi(2) + (vi.2 - vj.2).powi(2)).sqrt();
                if dist < SAFE_DISTANCE {
                    let target = if dist < STOP_DISTANCE {
                        0.0
                    } else {
                        SPEED_LOW
                    };
                    if self.vehicles[i].target_velocity > target {
                        self.vehicles[i].target_velocity = target;
                    }
                }
            }
        }
    }

    fn check_intersection_entry(&mut self) {
        let ix = INTERSECTION_X;
        let iy = INTERSECTION_Y;
        let iw = ROAD_WIDTH;

        for v in &mut self.vehicles {
            if v.state != VehicleState::Approaching {
                continue;
            }
            if is_in_intersection_box(v.x, v.y, ix, iy, iw) {
                v.state = VehicleState::InIntersection;
                v.entry_time = Some(Instant::now());
                if v.route != Route::Straight {
                    v.setup_turn();
                }
                v.target_velocity = SPEED_MED;
            }
        }
    }

    fn check_intersection_exit(&mut self, stats: &mut Statistics) {
        let ix = INTERSECTION_X;
        let iy = INTERSECTION_Y;
        let iw = ROAD_WIDTH;

        for v in &mut self.vehicles {
            if v.state != VehicleState::InIntersection {
                continue;
            }
            if !is_in_intersection_box(v.x, v.y, ix, iy, iw) {
                v.state = VehicleState::Exiting;
                v.exit_time = Some(Instant::now());
                v.target_velocity = SPEED_HIGH;

                if let Some(elapsed) = v.elapsed_seconds() {
                    stats.record_time(elapsed);
                }
                stats.record_velocity(v.max_velocity, v.min_velocity);
            }
        }
    }

    fn detect_close_calls(&self) -> usize {
        let mut count = 0;
        let n = self.vehicles.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let a = &self.vehicles[i];
                let b = &self.vehicles[j];
                if a.direction == b.direction {
                    continue;
                }
                let dist = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt();
                if dist < SAFE_DISTANCE * 0.5 && dist > 2.0 {
                    count += 1;
                }
            }
        }
        count / 2
    }

    fn remove_done_vehicles(&mut self, stats: &mut Statistics) {
        self.vehicles.retain(|v| {
            if v.is_out_of_bounds() && v.state == VehicleState::Exiting {
                self.total_passed += 1;
                stats.total_passed = self.total_passed;
                stats.max_vehicles = stats.max_vehicles.max(self.total_passed);
                false
            } else {
                true
            }
        });
    }
}

fn distance_to_intersection_entry(v: &Vehicle, ix: f64, iy: f64, iw: f64) -> f64 {
    match v.direction {
        Direction::South => (iy - v.y).abs(),
        Direction::North => (v.y - (iy + iw)).abs(),
        Direction::East => (ix - v.x).abs(),
        Direction::West => (v.x - (ix + iw)).abs(),
    }
}

fn is_in_intersection_box(x: f64, y: f64, ix: f64, iy: f64, iw: f64) -> bool {
    x >= ix && x <= ix + iw && y >= iy && y <= iy + iw
}

pub fn paths_conflict(d1: Direction, r1: Route, d2: Direction, r2: Route) -> bool {
    if d1 == d2 {
        return false;
    }
    if r1 == Route::Right && r2 == Route::Right {
        return false;
    }

    use Direction::*;
    use Route::*;

    let opposite = matches!(
        (d1, d2),
        (North, South) | (South, North) | (East, West) | (West, East)
    );

    if opposite {
        if r1 == Straight && r2 == Straight {
            return false;
        }
        if r1 == Right || r2 == Right {
            return false;
        }
        return true;
    }

    let perpendicular = matches!(
        (d1, d2),
        (North, East)
            | (North, West)
            | (South, East)
            | (South, West)
            | (East, North)
            | (East, South)
            | (West, North)
            | (West, South)
    );

    if perpendicular {
        if r1 == Right {
            return false;
        }
        if r2 == Right && r1 == Straight {
            return false;
        }
        return true;
    }

    false
}
