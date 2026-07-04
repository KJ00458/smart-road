use rand::Rng;
use std::time::Instant;
use crate::config::*;
use crate::path::get_path;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Arm { North, South, East, West }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Turn { Right, Forward, Left }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase { Approaching, Crossing, Exiting }

/// Three discrete speed states.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Spd { Slow, Med, Fast }

impl Spd {
    pub fn px(self) -> f64 {
        match self {
            Spd::Slow => SPD_SLOW,
            Spd::Med  => SPD_MED,
            Spd::Fast => SPD_FAST,
        }
    }
}

static ID: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(1);

pub struct Vehicle {
    pub id:       u64,
    pub arm:      Arm,
    pub turn:     Turn,
    pub phase:    Phase,
    pub path:     &'static [(f64,f64)],
    pub wp:       usize,
    pub x:        f64,
    pub y:        f64,
    pub spd:      Spd,
    pub color:    usize,
    pub entry_t:  Option<Instant>,
    pub exit_t:   Option<Instant>,
    pub max_spd:  f64,
    pub min_spd:  f64,
    /// Priority: lower number = higher priority (assigned at spawn time, monotone increasing).
    pub priority: u64,
    /// Sensor cone: how far ahead (px) the car "sees" on its own lane
    pub sensor_range: f64,
}

impl Vehicle {
    pub fn new(arm: Arm, turn: Turn) -> Self {
        let path = get_path(arm, turn);
        let (x, y) = path[0];
        let id = ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        Vehicle {
            id, arm, turn,
            phase: Phase::Approaching,
            path, wp: 1,
            x, y,
            spd: Spd::Fast,
            color: rand::thread_rng().gen_range(0..8),
            entry_t: None, exit_t: None,
            max_spd: 0.0, min_spd: f64::MAX,
            priority: id,   // earlier-spawned = higher priority
            sensor_range: SENSOR_RANGE,
        }
    }

    pub fn new_random() -> Self {
        let mut r = rand::thread_rng();
        let arm  = [Arm::North,Arm::South,Arm::East,Arm::West][r.gen_range(0..4)];
        let turn = [Turn::Right,Turn::Forward,Turn::Left][r.gen_range(0..3)];
        Self::new(arm, turn)
    }

    pub fn new_from_arm(arm: Arm) -> Self {
        let turn = [Turn::Right,Turn::Forward,Turn::Left]
            [rand::thread_rng().gen_range(0..3)];
        Self::new(arm, turn)
    }

    /// Advance along waypoints by spd*dt pixels. Returns true when path is complete.
    pub fn step(&mut self, dt: f64) -> bool {
        if self.wp >= self.path.len() { return true; }
        let (tx, ty) = self.path[self.wp];
        let dx = tx - self.x;
        let dy = ty - self.y;
        let dist = (dx*dx + dy*dy).sqrt();
        let speed = self.spd.px();
        let step  = speed * dt;
        if step >= dist {
            self.x = tx;
            self.y = ty;
            self.wp += 1;
            if self.wp >= self.path.len() { return true; }
        } else {
            self.x += dx / dist * step;
            self.y += dy / dist * step;
        }
        if speed > self.max_spd { self.max_spd = speed; }
        if speed < self.min_spd { self.min_spd = speed; }
        false
    }

    /// Heading angle (radians, 0=East) toward current waypoint.
    pub fn angle(&self) -> f64 {
        if self.wp < self.path.len() {
            let (tx, ty) = self.path[self.wp];
            (ty - self.y).atan2(tx - self.x)
        } else if self.wp >= 2 {
            let (ax, ay) = self.path[self.wp - 2];
            let (bx, by) = self.path[self.wp - 1];
            (by - ay).atan2(bx - ax)
        } else {
            0.0
        }
    }

    /// Forward direction unit vector toward current waypoint.
    pub fn forward_dir(&self) -> (f64, f64) {
        let a = self.angle();
        (a.cos(), a.sin())
    }

    /// Signed distance of point (px,py) projected onto the forward axis.
    /// Positive = ahead of the vehicle.
    pub fn forward_dist_to(&self, px: f64, py: f64) -> f64 {
        let (fx, fy) = self.forward_dir();
        fx*(px - self.x) + fy*(py - self.y)
    }

    /// Lateral offset of point (px,py) from the vehicle's forward axis.
    pub fn lateral_offset_to(&self, px: f64, py: f64) -> f64 {
        let (fx, fy) = self.forward_dir();
        // perpendicular = (-fy, fx)
        -fy*(px - self.x) + fx*(py - self.y)
    }

    pub fn elapsed(&self) -> Option<f64> {
        match (self.entry_t, self.exit_t) {
            (Some(a), Some(b)) => Some(b.duration_since(a).as_secs_f64()),
            _ => None,
        }
    }

    pub fn min_spd_d(&self) -> f64 {
        if self.min_spd == f64::MAX { 0.0 } else { self.min_spd }
    }
}

// ---------------------------------------------------------------------------
// Path-conflict helper (geometrically: do the two waypoint paths share a cell?)
// ---------------------------------------------------------------------------

/// Returns true if paths for (a1,t1) and (a2,t2) share at least one waypoint
/// within CONFLICT_DIST pixels of each other — i.e. they geometrically cross.
pub fn paths_conflict(a1: Arm, t1: Turn, a2: Arm, t2: Turn) -> bool {
    if a1 == a2 { return false; }
    use crate::path::get_path;
    let p1 = get_path(a1, t1);
    let p2 = get_path(a2, t2);
    for &(x1,y1) in p1 {
        for &(x2,y2) in p2 {
            let d = ((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)).sqrt();
            if d < CONFLICT_DIST { return true; }
        }
    }
    false
}
