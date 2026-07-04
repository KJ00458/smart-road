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
    /// priority = spawn ID; LOWER id = spawned earlier = higher priority (right of way)
    pub priority:     u64,
    pub sensor_range: f64,
    pub crashed:      bool,
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

    /// Advance along waypoints. Returns true when final (off-screen) waypoint reached.
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

    /// Direction toward the current waypoint (or last segment if done).
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

    /// Shortest distance from point (px,py) to this vehicle's remaining path segments.
    /// Used by the sensor to check if another car is on our future path.
    pub fn dist_to_future_path(&self, px: f64, py: f64) -> f64 {
        let mut best = f64::MAX;
        // start from current position segment
        let seg_start = if self.wp > 0 { self.wp - 1 } else { 0 };
        for i in seg_start..self.path.len().saturating_sub(1) {
            let (ax, ay) = if i == seg_start { (self.x, self.y) } else { self.path[i] };
            let (bx, by) = self.path[i + 1];
            let d = point_to_segment_dist(px, py, ax, ay, bx, by);
            if d < best { best = d; }
        }
        best
    }

    /// Approximate path distance from this vehicle's current position to point (px,py),
    /// only counting if the point projects *ahead* along the remaining path.
    pub fn path_dist_ahead_to(&self, px: f64, py: f64) -> Option<f64> {
        // Check if the point is near any future segment
        let seg_start = if self.wp > 0 { self.wp - 1 } else { 0 };
        let mut accumulated = 0.0f64;
        for i in seg_start..self.path.len().saturating_sub(1) {
            let (ax, ay) = if i == seg_start { (self.x, self.y) } else { self.path[i] };
            let (bx, by) = self.path[i + 1];
            let seg_len = ((bx-ax)*(bx-ax)+(by-ay)*(by-ay)).sqrt();
            let d = point_to_segment_dist(px, py, ax, ay, bx, by);
            if d < SENSOR_HALF_W * 2.0 {
                // project the point onto this segment to get how far along
                let t = if seg_len > 0.0 {
                    let dot = (px-ax)*(bx-ax)+(py-ay)*(by-ay);
                    (dot / (seg_len*seg_len)).clamp(0.0, 1.0)
                } else { 0.0 };
                return Some(accumulated + t * seg_len);
            }
            accumulated += seg_len;
        }
        None
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

/// Minimum distance from point P to segment AB.
pub fn point_to_segment_dist(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let dx = bx - ax; let dy = by - ay;
    let len2 = dx*dx + dy*dy;
    if len2 < 1e-9 { return ((px-ax)*(px-ax)+(py-ay)*(py-ay)).sqrt(); }
    let t = ((px-ax)*dx+(py-ay)*dy) / len2;
    let t = t.clamp(0.0, 1.0);
    let cx = ax + t*dx; let cy = ay + t*dy;
    ((px-cx)*(px-cx)+(py-cy)*(py-cy)).sqrt()
}

/// True if paths of (a1,t1) and (a2,t2) share any point closer than CONFLICT_DIST.
pub fn paths_conflict(a1: Arm, t1: Turn, a2: Arm, t2: Turn) -> bool {
    if a1 == a2 { return false; }
    use crate::path::get_path;
    let p1 = get_path(a1, t1);
    let p2 = get_path(a2, t2);
    // Check segment-to-segment distance for better accuracy
    for i in 0..p1.len().saturating_sub(1) {
        let (a1x,a1y) = p1[i]; let (b1x,b1y) = p1[i+1];
        for j in 0..p2.len().saturating_sub(1) {
            let (a2x,a2y) = p2[j]; let (b2x,b2y) = p2[j+1];
            // sample points along each segment
            for k in 0..=8 {
                let t = k as f64 / 8.0;
                let x1 = a1x + t*(b1x-a1x); let y1 = a1y + t*(b1y-a1y);
                let d = point_to_segment_dist(x1, y1, a2x, a2y, b2x, b2y);
                if d < CONFLICT_DIST { return true; }
            }
        }
    }
    false
}
