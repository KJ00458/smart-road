//! Vehicle model — closely mirrors the reference repo's Lane/Direction/has_turned pattern
//! but uses our own names (Arm / Turn) and f64 pixel coords.

use std::time::Instant;
use rand::Rng;
use crate::config::*;

// ── Types ────────────────────────────────────────────────────────────────────

/// Which arm of the intersection the vehicle enters from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Arm { North, South, East, West }

/// Intended exit direction — West = turn left, Forward = straight, East = turn right.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Turn { West, Forward, East }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase { Approaching, Crossing, Exiting }

// ── Spawn / destination tables (all derived from TILE grid) ──────────────────
//
// Layout (each arm has 3 inbound lanes numbered 0=West 1=Forward 2=East):
//
//   North arm  inbound lanes go LEFT half of vertical strip (indices 0..2, i.e. col 0..2)
//   South arm  inbound lanes go RIGHT half                  (indices 3..5, col 3..5)
//   West arm   inbound lanes go BOTTOM half of horizontal   (row 3..5)
//   East arm   inbound lanes go TOP half                    (row 0..2)
//
// Within each arm, lane 0 = West (left-turn), 1 = Forward, 2 = East (right-turn)
// mirroring reference: SPAWN_UP_WEST / SPAWN_UP_FORWARD / SPAWN_UP_EAST etc.

/// Returns (x, y, vx, vy) — spawn position + initial velocity for given arm+turn.
pub fn spawn_coords(arm: Arm, turn: Turn) -> (f64, f64, f64, f64) {
    let lane = lane_idx(turn) as f64;
    let off  = SPAWN_OFF;
    match arm {
        // North arm: moving South (+vy).  Inbound uses left cols (0,1,2 → x=IX+lane*T+T/2)
        Arm::North => (
            IX + lane * TILE + TILE / 2.0,
            IY - off,
            0.0, SPD_NORMAL,
        ),
        // South arm: moving North (-vy).  Inbound uses right cols (0→col5, 1→col4, 2→col3)
        Arm::South => (
            IX + (5.0 - lane) * TILE + TILE / 2.0,
            IY + ROAD_W + off,
            0.0, -SPD_NORMAL,
        ),
        // West arm: moving East (+vx).  Inbound uses bottom rows (0→row5, 1→row4, 2→row3)
        Arm::West => (
            IX - off,
            IY + (5.0 - lane) * TILE + TILE / 2.0,
            SPD_NORMAL, 0.0,
        ),
        // East arm: moving West (-vx).  Inbound uses top rows (0→row0, 1→row1, 2→row2)
        Arm::East => (
            IX + ROAD_W + off,
            IY + lane * TILE + TILE / 2.0,
            -SPD_NORMAL, 0.0,
        ),
    }
}

/// The Y (North/South arms) or X (East/West arms) coordinate at which the
/// vehicle snaps its velocity axis — exact mirror of reference `should_turn` thresholds.
pub fn turn_threshold(arm: Arm, turn: Turn) -> f64 {
    let lane = lane_idx(turn) as f64;
    match arm {
        // North arm moving South: turn at a Y inside intersection
        Arm::North => match turn {
            Turn::East    => IY + TILE * 1.0,                // right turn: row 1
            Turn::West    => IY + TILE * 4.0,               // left  turn: row 4 (deep)
            Turn::Forward => f64::MAX,
        },
        // South arm moving North: turn at Y counting from bottom
        Arm::South => match turn {
            Turn::East    => IY + ROAD_W - TILE * 1.0,
            Turn::West    => IY + ROAD_W - TILE * 4.0,
            Turn::Forward => f64::MIN,
        },
        // East arm moving West: turn at X
        Arm::East => match turn {
            Turn::East    => IX + ROAD_W - TILE * 1.0,
            Turn::West    => IX + ROAD_W - TILE * 4.0,
            Turn::Forward => f64::MIN,
        },
        // West arm moving East: turn at X
        Arm::West => match turn {
            Turn::East    => IX + TILE * 1.0,
            Turn::West    => IX + TILE * 4.0,
            Turn::Forward => f64::MAX,
        },
    }
}

/// New velocity after turning, plus snapped coordinate on the turn axis.
/// Returns (new_vx, new_vy, snap_x_opt, snap_y_opt)
pub fn post_turn_velocity(arm: Arm, turn: Turn, cur_x: f64, cur_y: f64)
    -> (f64, f64, Option<f64>, Option<f64>)
{
    let thr = turn_threshold(arm, turn);
    match arm {
        Arm::North => match turn {
            Turn::East  => (-SPD_NORMAL, 0.0, None,    Some(thr)), // go West
            Turn::West  => ( SPD_NORMAL, 0.0, None,    Some(thr)), // go East
            Turn::Forward => unreachable!(),
        },
        Arm::South => match turn {
            Turn::East  => ( SPD_NORMAL, 0.0, None,    Some(thr)), // go East
            Turn::West  => (-SPD_NORMAL, 0.0, None,    Some(thr)), // go West
            Turn::Forward => unreachable!(),
        },
        Arm::East => match turn {
            Turn::East  => (0.0,  SPD_NORMAL, Some(thr), None),    // go South
            Turn::West  => (0.0, -SPD_NORMAL, Some(thr), None),    // go North
            Turn::Forward => unreachable!(),
        },
        Arm::West => match turn {
            Turn::East  => (0.0, -SPD_NORMAL, Some(thr), None),    // go North
            Turn::West  => (0.0,  SPD_NORMAL, Some(thr), None),    // go South
            Turn::Forward => unreachable!(),
        },
    }
}

/// Whether the vehicle has passed its exit boundary.
pub fn has_reached_dest(arm: Arm, turn: Turn, has_turned: bool, x: f64, y: f64) -> bool {
    let off = SPAWN_OFF * 1.5;
    match arm {
        Arm::North => match turn {
            Turn::Forward => y > IY + ROAD_W + off,
            Turn::East    => has_turned && x < IX - off,
            Turn::West    => has_turned && x > IX + ROAD_W + off,
        },
        Arm::South => match turn {
            Turn::Forward => y < IY - off,
            Turn::East    => has_turned && x > IX + ROAD_W + off,
            Turn::West    => has_turned && x < IX - off,
        },
        Arm::East => match turn {
            Turn::Forward => x < IX - off,
            Turn::East    => has_turned && y > IY + ROAD_W + off,
            Turn::West    => has_turned && y < IY - off,
        },
        Arm::West => match turn {
            Turn::Forward => x > IX + ROAD_W + off,
            Turn::East    => has_turned && y < IY - off,
            Turn::West    => has_turned && y > IY + ROAD_W + off,
        },
    }
}

// ── Vehicle struct ────────────────────────────────────────────────────────────

pub struct Vehicle {
    pub id: u64,
    pub arm: Arm,
    pub turn: Turn,
    pub phase: Phase,
    pub x: f64,
    pub y: f64,
    pub vx: f64,   // current velocity x (px/s)
    pub vy: f64,   // current velocity y (px/s)
    pub has_turned: bool,
    pub color_idx: usize,
    pub entry_t: Option<Instant>,
    pub exit_t:  Option<Instant>,
    pub max_spd: f64,
    pub min_spd: f64,
}

static ID_CTR: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(1);

impl Vehicle {
    pub fn new(arm: Arm, turn: Turn) -> Vehicle {
        let (x, y, vx, vy) = spawn_coords(arm, turn);
        let mut rng = rand::thread_rng();
        Vehicle {
            id: ID_CTR.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
            arm, turn,
            phase: Phase::Approaching,
            x, y, vx, vy,
            has_turned: false,
            color_idx: rng.gen_range(0..8),
            entry_t: None,
            exit_t: None,
            max_spd: SPD_NORMAL,
            min_spd: SPD_NORMAL,
        }
    }

    pub fn new_random() -> Vehicle {
        let mut rng = rand::thread_rng();
        let arm  = [Arm::North, Arm::South, Arm::East, Arm::West][rng.gen_range(0..4)];
        let turn = [Turn::West, Turn::Forward, Turn::East][rng.gen_range(0..3)];
        Self::new(arm, turn)
    }

    pub fn new_from_arm(arm: Arm) -> Vehicle {
        let mut rng = rand::thread_rng();
        let turn = [Turn::West, Turn::Forward, Turn::East][rng.gen_range(0..3)];
        Self::new(arm, turn)
    }

    /// Mirror of reference `should_turn`: check if we've reached the turning threshold.
    pub fn should_turn(&self) -> bool {
        if self.turn == Turn::Forward || self.has_turned { return false; }
        let thr = turn_threshold(self.arm, self.turn);
        match self.arm {
            Arm::North => self.y >= thr,
            Arm::South => self.y <= thr,
            Arm::East  => self.x <= thr,
            Arm::West  => self.x >= thr,
        }
    }

    /// Snap velocity to new axis — mirror of reference `turning()`.
    pub fn do_turn(&mut self) {
        self.has_turned = true;
        let (nvx, nvy, snap_x, snap_y) =
            post_turn_velocity(self.arm, self.turn, self.x, self.y);
        self.vx = nvx;
        self.vy = nvy;
        if let Some(sy) = snap_y { self.y = sy; }
        if let Some(sx) = snap_x { self.x = sx; }
    }

    /// Move by dt seconds, respecting `vehicle_ahead` safe distance.
    pub fn update(&mut self, vehicle_ahead: Option<(f64, f64)>, dt: f64) {
        // Collision-based speed adaptation (like reference adapt_velocity + update_position)
        let blocked = if let Some((ax, ay)) = vehicle_ahead {
            let dist = (ax - self.x).abs() + (ay - self.y).abs();
            dist < SAFE_DIST
        } else {
            false
        };

        if !blocked {
            self.x += self.vx * dt;
            self.y += self.vy * dt;
        }
        // else: just stop (velocity already set by set_speed)

        let spd = (self.vx * self.vx + self.vy * self.vy).sqrt();
        if spd > self.max_spd { self.max_spd = spd; }
        if spd < self.min_spd { self.min_spd = spd; }
    }

    /// Apply a new scalar speed, preserving direction sign.
    pub fn set_speed(&mut self, spd: f64) {
        if self.vx != 0.0 {
            self.vx = self.vx.signum() * spd;
        } else {
            self.vy = self.vy.signum() * spd;
        }
    }

    pub fn speed(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy).sqrt()
    }

    pub fn is_done(&self) -> bool {
        has_reached_dest(self.arm, self.turn, self.has_turned, self.x, self.y)
    }

    pub fn elapsed_secs(&self) -> Option<f64> {
        match (self.entry_t, self.exit_t) {
            (Some(a), Some(b)) => Some(b.duration_since(a).as_secs_f64()),
            _ => None,
        }
    }

    /// Render angle in radians (0 = pointing right/East screen direction)
    pub fn angle(&self) -> f64 {
        self.vy.atan2(self.vx)
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Turn → inbound lane index (0=West/left, 1=Forward, 2=East/right)
pub fn lane_idx(turn: Turn) -> usize {
    match turn {
        Turn::West    => 0,
        Turn::Forward => 1,
        Turn::East    => 2,
    }
}

/// Whether two (arm,turn) combos conflict inside the intersection.
/// Logic: right-turns never conflict; straights on opposite arms don't conflict;
/// everything else does.
pub fn paths_conflict(a1: Arm, t1: Turn, a2: Arm, t2: Turn) -> bool {
    if a1 == a2 { return false; }
    if t1 == Turn::East { return false; }  // right-turn clears early
    if t2 == Turn::East { return false; }

    let opposite = matches!(
        (a1, a2),
        (Arm::North, Arm::South) | (Arm::South, Arm::North)
        | (Arm::East, Arm::West) | (Arm::West, Arm::East)
    );
    if opposite && t1 == Turn::Forward && t2 == Turn::Forward {
        return false; // parallel straights
    }
    true
}
