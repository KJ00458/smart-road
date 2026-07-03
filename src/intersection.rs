use std::collections::HashSet;
use std::time::Instant;

use crate::config::*;
use crate::stats::Statistics;
use crate::vehicle::{Arm, Turn, Vehicle, VehicleState};

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

        // Reserve for vehicles in intersection OR very close to entry
        let reservation_holders: Vec<(u64, Arm, Turn)> = self
            .vehicles
            .iter()
            .filter(|v| {
                v.state == VehicleState::InIntersection
                    || (v.state == VehicleState::Approaching
                        && distance_to_entry(v, ix, iy, iw) < SAFE_DISTANCE)
            })
            .map(|v| (v.id, v.arm, v.turn))
            .collect();

        for v in &mut self.vehicles {
            if v.state != VehicleState::Approaching {
                continue;
            }
            let dist = distance_to_entry(v, ix, iy, iw);
            if dist > approach_zone {
                v.target_velocity = SPEED_HIGH;
                continue;
            }
            let conflicts = reservation_holders.iter().any(|(rid, rarm, rturn)| {
                *rid != v.id && paths_conflict(v.arm, v.turn, *rarm, *rturn)
            });
            if conflicts {
                if dist < SAFE_DISTANCE * 1.2 {
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
        let snapshots: Vec<(usize, f64, f64, Arm, usize, VehicleState, f64, f64)> = self
            .vehicles
            .iter()
            .enumerate()
            .map(|(i, v)| (i, v.x, v.y, v.arm, v.lane_index, v.state, v.dx, v.dy))
            .collect();

        for i in 0..self.vehicles.len() {
            let vi = &snapshots[i];
            for j in 0..snapshots.len() {
                if i == j { continue; }
                let vj = &snapshots[j];

                // Only apply same-arm, same-lane vehicles (or both in intersection)
                if vi.3 != vj.3 { continue; }
                if vi.4 != vj.4
                    && vi.5 != VehicleState::InIntersection
                    && vj.5 != VehicleState::InIntersection
                { continue; }

                // vj is ahead if it's further along vi's travel direction
                let is_ahead = vi.6 * (vj.1 - vi.1) + vi.7 * (vj.2 - vi.2) > 0.0;
                if !is_ahead { continue; }

                let dist = ((vi.1 - vj.1).powi(2) + (vi.2 - vj.2).powi(2)).sqrt();
                if dist < SAFE_DISTANCE {
                    let target = if dist < STOP_DISTANCE { 0.0 } else { SPEED_LOW };
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
            if v.state != VehicleState::Approaching { continue; }
            if is_in_box(v.x, v.y, ix, iy, iw) {
                v.state = VehicleState::InIntersection;
                v.entry_time = Some(Instant::now());
                v.target_velocity = SPEED_MED;
            }
        }
    }

    fn check_intersection_exit(&mut self, stats: &mut Statistics) {
        let ix = INTERSECTION_X;
        let iy = INTERSECTION_Y;
        let iw = ROAD_WIDTH;
        for v in &mut self.vehicles {
            if v.state != VehicleState::InIntersection { continue; }
            if !is_in_box(v.x, v.y, ix, iy, iw) {
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
                if a.arm == b.arm { continue; }
                let dist = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt();
                if dist < SAFE_DISTANCE * 0.5 && dist > 2.0 {
                    count += 1;
                }
            }
        }
        count / 2
    }

    fn remove_done_vehicles(&mut self, stats: &mut Statistics) {
        let total = &mut self.total_passed;
        self.vehicles.retain(|v| {
            if v.is_out_of_bounds() && v.state == VehicleState::Exiting {
                *total += 1;
                stats.total_passed = *total;
                stats.max_vehicles = stats.max_vehicles.max(*total);
                false
            } else {
                true
            }
        });
    }
}

fn distance_to_entry(v: &Vehicle, ix: f64, iy: f64, iw: f64) -> f64 {
    match v.arm {
        Arm::North => (iy - v.y).max(0.0),
        Arm::South => (v.y - (iy + iw)).max(0.0),
        Arm::East  => (v.x - (ix + iw)).max(0.0),
        Arm::West  => (ix - v.x).max(0.0),
    }
}

fn is_in_box(x: f64, y: f64, ix: f64, iy: f64, iw: f64) -> bool {
    x >= ix && x <= ix + iw && y >= iy && y <= iy + iw
}

/// Conflict detection adapted for the 6-lane / West-Forward-East model.
/// Two paths conflict if their trajectories cross inside the intersection.
pub fn paths_conflict(a1: Arm, t1: Turn, a2: Arm, t2: Turn) -> bool {
    if a1 == a2 { return false; }             // same arm, no conflict
    if t1 == Turn::East && t2 == Turn::East { return false; } // both right-turn
    if t1 == Turn::East { return false; }     // right turn never conflicts
    if t2 == Turn::East { return false; }     // ditto

    use Arm::*;
    use Turn::*;

    let opposite = matches!(
        (a1, a2),
        (North, South) | (South, North) | (East, West) | (West, East)
    );

    if opposite {
        if t1 == Forward && t2 == Forward { return false; } // both straight, parallel
        return true; // one or both turning left across center
    }

    // Perpendicular arms
    // Left turn (West) always conflicts with perpendicular
    // Straight conflicts with perpendicular left turn
    if t1 == West { return true; }
    if t2 == West { return true; }
    // Both Forward from perpendicular arms: they cross
    if t1 == Forward && t2 == Forward { return true; }

    false
}
