use rand::Rng;
use std::f64::consts::PI;
use std::time::Instant;

use crate::config::*;
use crate::intersection::Intersection;

/// Which arm of the intersection the vehicle enters from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Arm {
    North, // entering from top, moving South
    South, // entering from bottom, moving North
    East,  // entering from right, moving West
    West,  // entering from left, moving East
}

/// Turn direction — matches reference: West=left, Forward=straight, East=right
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Turn {
    West,    // turn left relative to travel direction
    Forward, // go straight
    East,    // turn right relative to travel direction
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VehicleState {
    Approaching,
    InIntersection,
    Exiting,
}

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub arm: Arm,
    pub turn: Turn,
    pub lane_index: usize, // 0=West lane, 1=Forward lane, 2=East lane
    pub x: f64,
    pub y: f64,
    pub angle: f64,       // radians, for rendering
    pub exit_arm: Arm,    // which arm it will exit from
    pub velocity: f64,
    pub target_velocity: f64,
    pub state: VehicleState,
    pub entry_time: Option<Instant>,
    pub exit_time: Option<Instant>,
    pub max_velocity: f64,
    pub min_velocity: f64,
    pub has_turned: bool,
    pub distance_travelled: f64,
    pub color_index: usize,
    // After turning, travel direction changes
    pub dx: f64, // unit direction x
    pub dy: f64, // unit direction y
    // Snap-turn destination coordinate
    pub turn_trigger: f64, // the Y (NS arms) or X (EW arms) at which to snap-turn
}

static VEHICLE_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

impl Vehicle {
    pub fn new(arm: Arm, turn: Turn, intersection: &Intersection) -> Option<Vehicle> {
        let lane_index = lane_for_turn(turn);
        let (x, y, angle, dx, dy) = spawn_position(arm, lane_index);
        let turn_trigger = turn_trigger_coord(arm, turn, lane_index);
        let exit_arm = exit_arm_for(arm, turn);

        // Prevent spawning too close to existing vehicle in same lane
        for v in &intersection.vehicles {
            if v.arm == arm && v.lane_index == lane_index && v.state == VehicleState::Approaching {
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
            arm,
            turn,
            lane_index,
            x, y, angle,
            exit_arm,
            velocity: SPEED_HIGH,
            target_velocity: SPEED_HIGH,
            state: VehicleState::Approaching,
            entry_time: None,
            exit_time: None,
            max_velocity: SPEED_HIGH,
            min_velocity: SPEED_HIGH,
            has_turned: false,
            distance_travelled: 0.0,
            color_index,
            dx, dy,
            turn_trigger,
        })
    }

    pub fn spawn_random(intersection: &Intersection) -> Option<Vehicle> {
        let mut rng = rand::thread_rng();
        let arm = random_arm(&mut rng);
        let turn = random_turn(&mut rng);
        Self::new(arm, turn, intersection)
    }

    pub fn spawn_from_arm(arm: Arm, intersection: &Intersection) -> Option<Vehicle> {
        let mut rng = rand::thread_rng();
        let turn = random_turn(&mut rng);
        Self::new(arm, turn, intersection)
    }

    pub fn update(&mut self, dt: f64) {
        self.velocity = approach_velocity(self.velocity, self.target_velocity, dt);
        self.max_velocity = self.max_velocity.max(self.velocity);
        self.min_velocity = self.min_velocity.min(self.velocity);
        self.distance_travelled += self.velocity * dt;

        let dist = self.velocity * dt;
        self.x += self.dx * dist;
        self.y += self.dy * dist;

        // Check snap-turn trigger
        if self.state == VehicleState::InIntersection && !self.has_turned && self.turn != Turn::Forward {
            let triggered = match self.arm {
                Arm::North => self.y >= self.turn_trigger,
                Arm::South => self.y <= self.turn_trigger,
                Arm::East  => self.x <= self.turn_trigger,
                Arm::West  => self.x >= self.turn_trigger,
            };
            if triggered {
                self.snap_turn();
            }
        }
    }

    fn snap_turn(&mut self) {
        self.has_turned = true;
        // Snap coordinate to exact turn point then update direction + angle
        match self.arm {
            Arm::North => {
                self.y = self.turn_trigger;
                match self.turn {
                    Turn::West => { self.dx = -1.0; self.dy = 0.0; self.angle = PI; self.arm = Arm::East; }
                    Turn::East => { self.dx =  1.0; self.dy = 0.0; self.angle = 0.0; self.arm = Arm::West; }
                    Turn::Forward => {}
                }
            }
            Arm::South => {
                self.y = self.turn_trigger;
                match self.turn {
                    Turn::West => { self.dx =  1.0; self.dy = 0.0; self.angle = 0.0; self.arm = Arm::West; }
                    Turn::East => { self.dx = -1.0; self.dy = 0.0; self.angle = PI; self.arm = Arm::East; }
                    Turn::Forward => {}
                }
            }
            Arm::East => {
                self.x = self.turn_trigger;
                match self.turn {
                    Turn::West => { self.dx = 0.0; self.dy = -1.0; self.angle = -PI/2.0; self.arm = Arm::South; }
                    Turn::East => { self.dx = 0.0; self.dy =  1.0; self.angle =  PI/2.0; self.arm = Arm::North; }
                    Turn::Forward => {}
                }
            }
            Arm::West => {
                self.x = self.turn_trigger;
                match self.turn {
                    Turn::West => { self.dx = 0.0; self.dy =  1.0; self.angle =  PI/2.0; self.arm = Arm::North; }
                    Turn::East => { self.dx = 0.0; self.dy = -1.0; self.angle = -PI/2.0; self.arm = Arm::South; }
                    Turn::Forward => {}
                }
            }
        }
        self.state = VehicleState::Exiting;
    }

    pub fn is_out_of_bounds(&self) -> bool {
        let margin = VEHICLE_H * 3.0;
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

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Lane index: 0=West(left turn), 1=Forward, 2=East(right turn)
pub fn lane_for_turn(turn: Turn) -> usize {
    match turn {
        Turn::West    => 0,
        Turn::Forward => 1,
        Turn::East    => 2,
    }
}

/// The outbound arm a vehicle exits on.
fn exit_arm_for(arm: Arm, turn: Turn) -> Arm {
    match (arm, turn) {
        (Arm::North, Turn::Forward) => Arm::North,
        (Arm::North, Turn::West)    => Arm::West,
        (Arm::North, Turn::East)    => Arm::East,
        (Arm::South, Turn::Forward) => Arm::South,
        (Arm::South, Turn::West)    => Arm::East,
        (Arm::South, Turn::East)    => Arm::West,
        (Arm::East,  Turn::Forward) => Arm::East,
        (Arm::East,  Turn::West)    => Arm::North,
        (Arm::East,  Turn::East)    => Arm::South,
        (Arm::West,  Turn::Forward) => Arm::West,
        (Arm::West,  Turn::West)    => Arm::South,
        (Arm::West,  Turn::East)    => Arm::North,
    }
}

/// Spawn position + initial direction vector + angle.
/// Inbound lanes occupy the inner 3 lanes (closer to center line).
pub fn spawn_position(arm: Arm, lane: usize) -> (f64, f64, f64, f64, f64) {
    let ix = INTERSECTION_X;
    let iy = INTERSECTION_Y;
    let iw = ROAD_WIDTH;
    let lw = LANE_WIDTH;
    let off = SPAWN_OFFSET;

    match arm {
        Arm::North => {
            // Moving South (dy=+1). Inbound = left half of vertical strip (lower X)
            let lx = ix + (lane as f64 + 0.5) * lw;
            (lx, iy - off, PI / 2.0, 0.0, 1.0)
        }
        Arm::South => {
            // Moving North (dy=-1). Inbound = right half (higher X)
            let lx = ix + iw - (lane as f64 + 0.5) * lw;
            (lx, iy + iw + off, -PI / 2.0, 0.0, -1.0)
        }
        Arm::East => {
            // Moving West (dx=-1). Inbound = top half of horizontal strip (lower Y)
            let ly = iy + (lane as f64 + 0.5) * lw;
            (ix + iw + off, ly, PI, -1.0, 0.0)
        }
        Arm::West => {
            // Moving East (dx=+1). Inbound = bottom half (higher Y)
            let ly = iy + iw - (lane as f64 + 0.5) * lw;
            (-off, ly, 0.0, 1.0, 0.0)
        }
    }
}

/// The Y coordinate (for N/S arms) or X coordinate (for E/W arms) at which
/// the vehicle snaps its direction — derived from the reference destination logic.
fn turn_trigger_coord(arm: Arm, turn: Turn, _lane: usize) -> f64 {
    let ix = INTERSECTION_X;
    let iy = INTERSECTION_Y;
    let iw = ROAD_WIDTH;
    let lw = LANE_WIDTH;

    match (arm, turn) {
        // North arm (moving South)
        (Arm::North, Turn::East)  => iy + lw * 3.0, // right turn: early
        (Arm::North, Turn::West)  => iy + lw * 5.0, // left turn: deep
        // South arm (moving North)
        (Arm::South, Turn::East)  => iy + iw - lw * 3.0,
        (Arm::South, Turn::West)  => iy + iw - lw * 5.0,
        // East arm (moving West)
        (Arm::East,  Turn::East)  => ix + iw - lw * 3.0,
        (Arm::East,  Turn::West)  => ix + iw - lw * 5.0,
        // West arm (moving East)
        (Arm::West,  Turn::East)  => ix + lw * 3.0,
        (Arm::West,  Turn::West)  => ix + lw * 5.0,
        // Forward: unused
        _ => 0.0,
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

fn random_arm(rng: &mut impl Rng) -> Arm {
    match rng.gen_range(0..4) {
        0 => Arm::North,
        1 => Arm::South,
        2 => Arm::East,
        _ => Arm::West,
    }
}

fn random_turn(rng: &mut impl Rng) -> Turn {
    match rng.gen_range(0..3) {
        0 => Turn::West,
        1 => Turn::Forward,
        _ => Turn::East,
    }
}
