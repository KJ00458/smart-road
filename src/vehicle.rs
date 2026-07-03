use rand::Rng;
use std::time::Instant;
use crate::config::*;
use crate::path::get_path;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Arm { North, South, East, West }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Turn { West, Forward, East }

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
    pub id:      u64,
    pub arm:     Arm,
    pub turn:    Turn,
    pub phase:   Phase,
    pub path:    &'static [(f64,f64)],
    pub wp:      usize,
    pub x:       f64,
    pub y:       f64,
    pub spd:     Spd,
    pub color:   usize,
    pub entry_t: Option<Instant>,
    pub exit_t:  Option<Instant>,
    pub max_spd: f64,
    pub min_spd: f64,
}

impl Vehicle {
    pub fn new(arm: Arm, turn: Turn) -> Self {
        let path = get_path(arm, turn);
        let (x, y) = path[0];
        Vehicle {
            id: ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
            arm, turn,
            phase: Phase::Approaching,
            path, wp: 1,
            x, y,
            spd: Spd::Fast,
            color: rand::thread_rng().gen_range(0..8),
            entry_t: None, exit_t: None,
            max_spd: 0.0, min_spd: f64::MAX,
        }
    }

    pub fn new_random() -> Self {
        let mut r = rand::thread_rng();
        let arm  = [Arm::North,Arm::South,Arm::East,Arm::West][r.gen_range(0..4)];
        let turn = [Turn::West,Turn::Forward,Turn::East][r.gen_range(0..3)];
        Self::new(arm, turn)
    }

    pub fn new_from_arm(arm: Arm) -> Self {
        let turn = [Turn::West,Turn::Forward,Turn::East]
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

/// Does path (a1,t1) cross path (a2,t2) inside the intersection?
pub fn paths_conflict(a1: Arm, t1: Turn, a2: Arm, t2: Turn) -> bool {
    if a1 == a2 { return false; }
    if t1 == Turn::East && t2 == Turn::East { return false; }
    let opp = matches!(
        (a1,a2),
        (Arm::North,Arm::South)|(Arm::South,Arm::North)
        |(Arm::East,Arm::West)|(Arm::West,Arm::East)
    );
    if opp && t1 == Turn::Forward && t2 == Turn::Forward { return false; }
    true
}
