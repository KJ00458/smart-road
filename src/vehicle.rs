use rand::Rng;
use std::f64::consts::PI;
use std::time::Instant;

use crate::config::*;
use crate::intersection::Intersection;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    North,
    South,
    East,
    West,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Route {
    Left,
    Straight,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VehicleState {
    Approaching,
    InIntersection,
    Exiting,
    Done,
}

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub direction: Direction,
    pub route: Route,
    pub x: f64,
    pub y: f64,
    pub angle: f64,
    pub velocity: f64,
    pub target_velocity: f64,
    pub state: VehicleState,
    pub lane_index: usize,
    pub entry_time: Option<Instant>,
    pub exit_time: Option<Instant>,
    pub max_velocity: f64,
    pub min_velocity: f64,
    pub turn_progress: f64,
    pub turn_center_x: f64,
    pub turn_center_y: f64,
    pub turn_radius: f64,
    pub turn_start_angle: f64,
    pub turn_total_angle: f64,
    pub past_turn_point: bool,
    pub distance_travelled: f64,
    pub color_index: usize,
}

static VEHICLE_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

impl Vehicle {
    pub fn spawn_from_direction(dir: Direction, intersection: &Intersection) -> Option<Vehicle> {
        let mut rng = rand::thread_rng();
        let route = random_route(&mut rng);
        let lane = lane_for_route(route);
        Self::create(dir, route, lane, intersection)
    }

    pub fn spawn_random(intersection: &Intersection) -> Option<Vehicle> {
        let mut rng = rand::thread_rng();
        let dir = random_direction(&mut rng);
        let route = random_route(&mut rng);
        let lane = lane_for_route(route);
        Self::create(dir, route, lane, intersection)
    }

    fn create(
        dir: Direction,
        route: Route,
        lane: usize,
        intersection: &Intersection,
    ) -> Option<Vehicle> {
        let (x, y, angle) = spawn_position(dir, lane);

        for v in &intersection.vehicles {
            if v.direction == dir && v.lane_index == lane {
                let dist = ((v.x - x).powi(2) + (v.y - y).powi(2)).sqrt();
                if dist < SAFE_DISTANCE * 1.5 {
                    return None;
                }
            }
        }

        let mut rng = rand::thread_rng();
        let color_index = rng.gen_range(0..8);

        Some(Vehicle {
            id: VEHICLE_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
            direction: dir,
            route,
            x,
            y,
            angle,
            velocity: SPEED_HIGH,
            target_velocity: SPEED_HIGH,
            state: VehicleState::Approaching,
            lane_index: lane,
            entry_time: None,
            exit_time: None,
            max_velocity: SPEED_HIGH,
            min_velocity: SPEED_HIGH,
            turn_progress: 0.0,
            turn_center_x: 0.0,
            turn_center_y: 0.0,
            turn_radius: 0.0,
            turn_start_angle: 0.0,
            turn_total_angle: 0.0,
            past_turn_point: false,
            distance_travelled: 0.0,
            color_index,
        })
    }

    pub fn update(&mut self, dt: f64) {
        self.velocity = approach_velocity(self.velocity, self.target_velocity, dt);
        self.max_velocity = self.max_velocity.max(self.velocity);
        self.min_velocity = self.min_velocity.min(self.velocity);
        self.distance_travelled += self.velocity * dt;

        match self.state {
            VehicleState::Approaching => self.move_straight(dt),
            VehicleState::InIntersection => self.move_in_intersection(dt),
            VehicleState::Exiting => self.move_straight(dt),
            VehicleState::Done => {}
        }
    }

    fn move_straight(&mut self, dt: f64) {
        let dist = self.velocity * dt;
        match self.direction {
            Direction::North => self.y -= dist,
            Direction::South => self.y += dist,
            Direction::East => self.x += dist,
            Direction::West => self.x -= dist,
        }
    }

    fn move_in_intersection(&mut self, dt: f64) {
        match self.route {
            Route::Straight => self.move_straight(dt),
            Route::Right => self.move_turn(dt, true),
            Route::Left => self.move_turn(dt, false),
        }
    }

    fn move_turn(&mut self, dt: f64, right: bool) {
        let angular_velocity = self.velocity / self.turn_radius;
        let delta = if right {
            angular_velocity * dt
        } else {
            -angular_velocity * dt
        };
        self.turn_progress += delta.abs();

        let new_angle = self.turn_start_angle + if right { self.turn_progress } else { -self.turn_progress };
        self.x = self.turn_center_x + self.turn_radius * new_angle.cos();
        self.y = self.turn_center_y + self.turn_radius * new_angle.sin();

        let tangent = if right { new_angle + PI / 2.0 } else { new_angle - PI / 2.0 };
        self.angle = tangent;

        if self.turn_progress >= self.turn_total_angle.abs() {
            self.finish_turn();
        }
    }

    fn finish_turn(&mut self) {
        self.state = VehicleState::Exiting;
        self.past_turn_point = true;
        match (self.direction, self.route) {
            (Direction::South, Route::Right) | (Direction::North, Route::Left) => {
                self.direction = Direction::East;
                self.angle = 0.0;
            }
            (Direction::South, Route::Left) | (Direction::North, Route::Right) => {
                self.direction = Direction::West;
                self.angle = PI;
            }
            (Direction::East, Route::Right) | (Direction::West, Route::Left) => {
                self.direction = Direction::South;
                self.angle = PI / 2.0;
            }
            (Direction::East, Route::Left) | (Direction::West, Route::Right) => {
                self.direction = Direction::North;
                self.angle = -PI / 2.0;
            }
            _ => {}
        }
    }

    /// Sets up the arc parameters for a turn using fixed intersection-relative
    /// anchors instead of the vehicle's live position, avoiding timing drift.
    pub fn setup_turn(&mut self, ix: f64, iy: f64, iw: f64) {
        match (self.direction, self.route) {
            (Direction::South, Route::Right) => {
                let cx = ix + (self.lane_index as f64 + 0.5) * LANE_WIDTH;
                let turn_x = ix + iw;
                let radius = (turn_x - cx).abs().max(1.0);
                self.turn_center_x = turn_x;
                self.turn_center_y = iy;  // fixed intersection top edge
                self.turn_radius = radius;
                self.turn_start_angle = PI;
                self.turn_total_angle = PI / 2.0;
            }
            (Direction::South, Route::Left) => {
                let radius = LANE_WIDTH * 1.5;
                self.turn_center_x = ix;
                self.turn_center_y = iy;  // fixed intersection top edge
                self.turn_radius = radius;
                self.turn_start_angle = 0.0;
                self.turn_total_angle = PI / 2.0;
            }
            (Direction::North, Route::Right) => {
                let radius = LANE_WIDTH * 1.5;
                self.turn_center_x = ix + iw;
                self.turn_center_y = iy + iw;  // fixed intersection bottom edge
                self.turn_radius = radius;
                self.turn_start_angle = PI;
                self.turn_total_angle = PI / 2.0;
            }
            (Direction::North, Route::Left) => {
                let radius = LANE_WIDTH * 1.5;
                self.turn_center_x = ix;
                self.turn_center_y = iy + iw;  // fixed intersection bottom edge
                self.turn_radius = radius;
                self.turn_start_angle = 0.0;
                self.turn_total_angle = PI / 2.0;
            }
            (Direction::East, Route::Right) => {
                let radius = LANE_WIDTH * 1.5;
                self.turn_center_x = ix;  // fixed intersection left edge
                self.turn_center_y = iy + iw;
                self.turn_radius = radius;
                self.turn_start_angle = -PI / 2.0;
                self.turn_total_angle = PI / 2.0;
            }
            (Direction::East, Route::Left) => {
                let radius = LANE_WIDTH * 1.5;
                self.turn_center_x = ix;  // fixed intersection left edge
                self.turn_center_y = iy;
                self.turn_radius = radius;
                self.turn_start_angle = PI / 2.0;
                self.turn_total_angle = PI / 2.0;
            }
            (Direction::West, Route::Right) => {
                let radius = LANE_WIDTH * 1.5;
                self.turn_center_x = ix + iw;  // fixed intersection right edge
                self.turn_center_y = iy;
                self.turn_radius = radius;
                self.turn_start_angle = PI / 2.0;
                self.turn_total_angle = PI / 2.0;
            }
            (Direction::West, Route::Left) => {
                let radius = LANE_WIDTH * 1.5;
                self.turn_center_x = ix + iw;  // fixed intersection right edge
                self.turn_center_y = iy + iw;
                self.turn_radius = radius;
                self.turn_start_angle = -PI / 2.0;
                self.turn_total_angle = PI / 2.0;
            }
            _ => {}
        }
    }

    pub fn is_out_of_bounds(&self) -> bool {
        let margin = VEHICLE_H * 2.0;
        self.x < -margin
            || self.x > WINDOW_W as f64 + margin
            || self.y < -margin
            || self.y > WINDOW_H as f64 + margin
    }

    pub fn elapsed_seconds(&self) -> Option<f64> {
        match (self.entry_time, self.exit_time) {
            (Some(e), Some(x)) => Some(x.duration_since(e).as_secs_f64()),
            _ => None,
        }
    }
}

fn approach_velocity(current: f64, target: f64, dt: f64) -> f64 {
    if current < target {
        (current + ACCEL * dt).min(target)
    } else if current > target {
        (current - DECEL * dt).max(target)
    } else {
        current
    }
}

pub fn lane_for_route(route: Route) -> usize {
    match route {
        Route::Right => 0,
        Route::Straight => 1,
        Route::Left => 2,
    }
}

pub fn spawn_position(dir: Direction, lane: usize) -> (f64, f64, f64) {
    let ix = INTERSECTION_X;
    let iy = INTERSECTION_Y;
    let iw = ROAD_WIDTH;
    let offset = WINDOW_H as f64 * 0.15;

    match dir {
        Direction::South => {
            let lx = ix + (lane as f64 + 0.5) * LANE_WIDTH;
            (lx, WINDOW_H as f64 + offset, -PI / 2.0)
        }
        Direction::North => {
            let lx = ix + iw - (lane as f64 + 0.5) * LANE_WIDTH;
            (lx, -offset, PI / 2.0)
        }
        Direction::East => {
            let ly = iy + (lane as f64 + 0.5) * LANE_WIDTH;
            (-offset, ly, 0.0)
        }
        Direction::West => {
            let ly = iy + iw - (lane as f64 + 0.5) * LANE_WIDTH;
            (WINDOW_W as f64 + offset, ly, PI)
        }
    }
}

fn random_direction(rng: &mut impl Rng) -> Direction {
    match rng.gen_range(0..4) {
        0 => Direction::North,
        1 => Direction::South,
        2 => Direction::East,
        _ => Direction::West,
    }
}

fn random_route(rng: &mut impl Rng) -> Route {
    match rng.gen_range(0..3) {
        0 => Route::Right,
        1 => Route::Straight,
        _ => Route::Left,
    }
}
