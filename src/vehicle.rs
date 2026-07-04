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
    /// Current speed in px/s. 0.0 = fully stopped.
    pub spd_px:   f64,
    pub color:    usize,
    pub entry_t:  Option<Instant>,
    pub exit_t:   Option<Instant>,
    pub max_spd:  f64,
    pub min_spd:  f64,
    pub priority: u64,
    pub crashed:  bool,
    pub hitbox_level: usize,
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
            spd_px: SPD_NORMAL,
            color: rand::thread_rng().gen_range(0..8),
            entry_t: None, exit_t: None,
            max_spd: 0.0, min_spd: f64::MAX,
            priority: id,
            crashed: false,
            hitbox_level: 0,
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

    /// Advance along waypoints by current speed. Returns true when done.
    pub fn step(&mut self, dt: f64) -> bool {
        if self.wp >= self.path.len() { return true; }
        let (tx, ty) = self.path[self.wp];
        let dx = tx - self.x;
        let dy = ty - self.y;
        let dist = (dx*dx + dy*dy).sqrt();
        let step  = self.spd_px * dt;
        if step >= dist {
            self.x = tx; self.y = ty;
            self.wp += 1;
            if self.wp >= self.path.len() { return true; }
        } else {
            self.x += dx / dist * step;
            self.y += dy / dist * step;
        }
        if self.spd_px > self.max_spd { self.max_spd = self.spd_px; }
        if self.spd_px < self.min_spd { self.min_spd = self.spd_px; }
        false
    }

    /// Heading angle (radians) toward current waypoint.
    pub fn angle(&self) -> f64 {
        if self.wp < self.path.len() {
            let (tx, ty) = self.path[self.wp];
            (ty - self.y).atan2(tx - self.x)
        } else if self.wp >= 2 {
            let (ax, ay) = self.path[self.wp - 2];
            let (bx, by) = self.path[self.wp - 1];
            (by - ay).atan2(bx - ax)
        } else { 0.0 }
    }

    pub fn forward_dir(&self) -> (f64, f64) {
        let a = self.angle(); (a.cos(), a.sin())
    }

    /// Signed forward distance to point (positive = ahead).
    pub fn forward_dist_to(&self, px: f64, py: f64) -> f64 {
        let (fx, fy) = self.forward_dir();
        fx*(px - self.x) + fy*(py - self.y)
    }

    /// Signed lateral offset to point.
    pub fn lateral_offset_to(&self, px: f64, py: f64) -> f64 {
        let (fx, fy) = self.forward_dir();
        -fy*(px - self.x) + fx*(py - self.y)
    }

    /// 4 corners of the forward hitbox projected `len` px ahead of the
    /// vehicle’s front face. Matches Golden76z rotated_rect approach.
    pub fn hitbox_corners(&self, len: f64) -> [(f64,f64); 4] {
        let a = self.angle();
        let (fx, fy) = (a.cos(), a.sin());
        let (px, py) = (-fy, fx); // perpendicular (left)
        let hw = HB_HALF_W;
        // Start from the front face of the vehicle
        let front_x = self.x + fx * (VH / 2.0);
        let front_y = self.y + fy * (VH / 2.0);
        [
            (front_x - px*hw,           front_y - py*hw),           // near-left
            (front_x + px*hw,           front_y + py*hw),           // near-right
            (front_x + fx*len + px*hw,  front_y + fy*len + py*hw),  // far-right
            (front_x + fx*len - px*hw,  front_y + fy*len - py*hw),  // far-left
        ]
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

// ── Geometry helpers used by intersection.rs ─────────────────────────────────

/// Axis-aligned bounding box of 4 corners.
pub fn aabb(corners: &[(f64,f64); 4]) -> (f64,f64,f64,f64) {
    let min_x = corners.iter().map(|c|c.0).fold(f64::MAX, f64::min);
    let max_x = corners.iter().map(|c|c.0).fold(f64::MIN, f64::max);
    let min_y = corners.iter().map(|c|c.1).fold(f64::MAX, f64::min);
    let max_y = corners.iter().map(|c|c.1).fold(f64::MIN, f64::max);
    (min_x, min_y, max_x, max_y)
}

/// True if two AABBs overlap.
pub fn aabb_overlap(a: (f64,f64,f64,f64), b: (f64,f64,f64,f64)) -> bool {
    a.0 < b.2 && a.2 > b.0 && a.1 < b.3 && a.3 > b.1
}
