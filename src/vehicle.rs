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
    pub id:           u64,
    pub arm:          Arm,
    pub turn:         Turn,
    pub phase:        Phase,
    pub path:         &'static [(f64,f64)],
    pub wp:           usize,
    pub x:            f64,
    pub y:            f64,
    pub spd:          Spd,
    pub color:        usize,
    pub entry_t:      Option<Instant>,
    pub exit_t:       Option<Instant>,
    pub max_spd:      f64,
    pub min_spd:      f64,
    /// Spawn order — lower = earlier = higher right-of-way
    pub priority:     u64,
    pub sensor_range: f64,
    pub crashed:      bool,
    /// Current hitbox level: 0=Big 1=Med 2=Small 3=VerySmall 4=Stop
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
            spd: Spd::Fast,
            color: rand::thread_rng().gen_range(0..8),
            entry_t: None, exit_t: None,
            max_spd: 0.0, min_spd: f64::MAX,
            priority: id,
            sensor_range: SENSOR_RANGE,
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

    /// Advance along waypoints. Returns true when final waypoint reached.
    pub fn step(&mut self, dt: f64) -> bool {
        if self.wp >= self.path.len() { return true; }
        let (tx, ty) = self.path[self.wp];
        let dx = tx - self.x;
        let dy = ty - self.y;
        let dist = (dx*dx + dy*dy).sqrt();
        let speed = self.spd.px();
        let step  = speed * dt;
        if step >= dist {
            self.x = tx; self.y = ty;
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

    /// Direction toward the current waypoint.
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

    pub fn forward_dist_to(&self, px: f64, py: f64) -> f64 {
        let (fx, fy) = self.forward_dir();
        fx*(px - self.x) + fy*(py - self.y)
    }

    pub fn lateral_offset_to(&self, px: f64, py: f64) -> f64 {
        let (fx, fy) = self.forward_dir();
        -fy*(px - self.x) + fx*(py - self.y)
    }

    /// Returns the 4 corners of this vehicle's projected forward hitbox
    /// for a given length ahead. Used for overlap testing.
    /// Returns (front_left, front_right, back_right, back_left).
    pub fn hitbox_corners(&self, len: f64) -> [(f64,f64); 4] {
        let a = self.angle();
        let (fx, fy) = (a.cos(), a.sin());
        let (px, py) = (-fy, fx); // perpendicular
        let hw = HB_HALF_W;
        // front of car
        let front_x = self.x + fx * (VH / 2.0);
        let front_y = self.y + fy * (VH / 2.0);
        [
            (front_x - px*hw,        front_y - py*hw),         // near-left
            (front_x + px*hw,        front_y + py*hw),         // near-right
            (front_x + fx*len + px*hw, front_y + fy*len + py*hw), // far-right
            (front_x + fx*len - px*hw, front_y + fy*len - py*hw), // far-left
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

/// Axis-aligned bounding box of a set of corners.
pub fn aabb(corners: &[(f64,f64); 4]) -> (f64,f64,f64,f64) {
    let min_x = corners.iter().map(|c| c.0).fold(f64::MAX, f64::min);
    let max_x = corners.iter().map(|c| c.0).fold(f64::MIN, f64::max);
    let min_y = corners.iter().map(|c| c.1).fold(f64::MAX, f64::min);
    let max_y = corners.iter().map(|c| c.1).fold(f64::MIN, f64::max);
    (min_x, min_y, max_x, max_y)
}

/// True if two AABBs overlap.
pub fn aabb_overlap(a: (f64,f64,f64,f64), b: (f64,f64,f64,f64)) -> bool {
    a.0 < b.2 && a.2 > b.0 && a.1 < b.3 && a.3 > b.1
}

/// True if paths of (a1,t1) and (a2,t2) share any waypoint segment closer
/// than CONFLICT_DIST. Same-arm paths never conflict.
pub fn paths_conflict(a1: Arm, t1: Turn, a2: Arm, t2: Turn) -> bool {
    if a1 == a2 { return false; }
    use crate::path::get_path;
    let p1 = get_path(a1, t1);
    let p2 = get_path(a2, t2);
    for i in 0..p1.len().saturating_sub(1) {
        let (a1x,a1y) = p1[i]; let (b1x,b1y) = p1[i+1];
        for j in 0..p2.len().saturating_sub(1) {
            let (a2x,a2y) = p2[j]; let (b2x,b2y) = p2[j+1];
            for k in 0..=8 {
                let t = k as f64 / 8.0;
                let x1 = a1x + t*(b1x-a1x); let y1 = a1y + t*(b1y-a1y);
                let d = pt_seg_dist(x1, y1, a2x, a2y, b2x, b2y);
                if d < CONFLICT_DIST { return true; }
            }
        }
    }
    false
}

pub fn pt_seg_dist(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let dx = bx-ax; let dy = by-ay;
    let len2 = dx*dx+dy*dy;
    if len2 < 1e-9 { return ((px-ax)*(px-ax)+(py-ay)*(py-ay)).sqrt(); }
    let t = ((px-ax)*dx+(py-ay)*dy)/len2;
    let t = t.clamp(0.0,1.0);
    let cx = ax+t*dx; let cy = ay+t*dy;
    ((px-cx)*(px-cx)+(py-cy)*(py-cy)).sqrt()
}
