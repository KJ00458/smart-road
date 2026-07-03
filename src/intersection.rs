use std::time::Instant;
use crate::config::*;
use crate::stats::Statistics;
use crate::vehicle::{Arm, Phase, Spd, Turn, Vehicle, paths_conflict};

const IX_BOX_L: f64 = IX;
const IX_BOX_R: f64 = IX + ROAD;
const IX_BOX_T: f64 = IY;
const IX_BOX_B: f64 = IY + ROAD;

pub struct World {
    pub vehicles: Vec<Vehicle>,
    pub total_passed: usize,
}

impl World {
    pub fn new() -> Self { World { vehicles: Vec::new(), total_passed: 0 } }

    pub fn spawn(&mut self, v: Vehicle) {
        // Reject if spawn point is occupied
        for e in &self.vehicles {
            if e.arm == v.arm && e.turn == v.turn {
                let d = ((e.x-v.x).powi(2)+(e.y-v.y).powi(2)).sqrt();
                if d < GAP * 2.2 { return; }
            }
        }
        self.vehicles.push(v);
    }

    pub fn update(&mut self, dt: f64, stats: &mut Statistics) {
        // ── 1. Speed control ────────────────────────────────────────────────
        // Collect (id, arm, turn, phase, x, y) snapshots for read-only use
        let snap: Vec<(u64,Arm,Turn,Phase,f64,f64)> = self.vehicles.iter()
            .map(|v|(v.id,v.arm,v.turn,v.phase,v.x,v.y)).collect();

        for i in 0..self.vehicles.len() {
            let spd = self.decide_speed(i, &snap);
            self.vehicles[i].spd = spd;
        }

        // ── 2. Move ────────────────────────────────────────────────────────
        let mut done_ids: Vec<u64> = Vec::new();
        for v in &mut self.vehicles {
            let finished = v.step(dt);

            // Phase transitions
            if v.phase == Phase::Approaching && in_box(v.x, v.y) {
                v.phase = Phase::Crossing;
                v.entry_t = Some(Instant::now());
            }
            if v.phase == Phase::Crossing && !in_box(v.x, v.y) {
                v.phase = Phase::Exiting;
                v.exit_t = Some(Instant::now());
                if let Some(t) = v.elapsed() { stats.record_time(t); }
                stats.record_spd(v.max_spd, v.min_spd);
            }

            if finished { done_ids.push(v.id); }
        }

        // ── 3. Remove finished ─────────────────────────────────────────────
        self.vehicles.retain(|v| {
            if done_ids.contains(&v.id) {
                stats.total_passed += 1;
                stats.max_passed = stats.max_passed.max(stats.total_passed);
                false
            } else { true }
        });
        self.total_passed = stats.total_passed;

        // ── 4. Close-call detection ────────────────────────────────────────
        stats.close_calls += self.count_close_calls();
    }

    fn decide_speed(&self, idx: usize, snap: &[(u64,Arm,Turn,Phase,f64,f64)]) -> Spd {
        let v = &self.vehicles[idx];

        // ─ Rule 1: vehicle directly ahead on same path ────────────────────────
        // Find closest vehicle ahead on the same (arm, turn) path
        let ahead_dist = snap.iter()
            .filter(|s| s.0 != v.id && s.1 == v.arm && s.2 == v.turn)
            .map(|s| {
                // Signed distance along path direction
                let wp = v.wp.min(v.path.len().saturating_sub(1));
                let (tx,ty) = v.path[wp];
                let dir_x = tx - v.x; let dir_y = ty - v.y;
                let len = (dir_x*dir_x+dir_y*dir_y).sqrt().max(0.001);
                // dot product: positive means ahead
                let dot = dir_x*(s.4-v.x)/len + dir_y*(s.5-v.y)/len;
                if dot > 0.0 {
                    Some(((s.4-v.x).powi(2)+(s.5-v.y).powi(2)).sqrt())
                } else { None }
            })
            .flatten()
            .fold(f64::MAX, f64::min);

        if ahead_dist < STOP_GAP  { return Spd::Slow; }
        if ahead_dist < GAP       { return Spd::Med;  }

        // ─ Rule 2: intersection conflict ──────────────────────────────────
        if v.phase == Phase::Approaching {
            let dist_to_ix = dist_to_entry(v);
            if dist_to_ix < GAP * 2.5 {
                // Check if any conflicting vehicle is in or approaching intersection
                let blocked = snap.iter().any(|s| {
                    s.0 != v.id
                        && paths_conflict(v.arm, v.turn, s.1, s.2)
                        && (s.3 == Phase::Crossing
                            || (s.3 == Phase::Approaching && dist_to_entry_raw(s.4,s.5,s.1) < GAP))
                });
                if blocked {
                    return if dist_to_ix < STOP_GAP * 2.0 { Spd::Slow } else { Spd::Med };
                }
            }
        }

        Spd::Fast
    }

    fn count_close_calls(&self) -> usize {
        let n = self.vehicles.len();
        let mut c = 0;
        for i in 0..n {
            for j in (i+1)..n {
                let a = &self.vehicles[i]; let b = &self.vehicles[j];
                if a.arm == b.arm && a.turn == b.turn { continue; }
                let d = ((a.x-b.x).powi(2)+(a.y-b.y).powi(2)).sqrt();
                if d < VH * 1.2 && d > 1.0 { c += 1; }
            }
        }
        c / 2
    }
}

// ── Free functions ────────────────────────────────────────────────────────────────

fn in_box(x: f64, y: f64) -> bool {
    x >= IX_BOX_L && x <= IX_BOX_R && y >= IX_BOX_T && y <= IX_BOX_B
}

fn dist_to_entry(v: &Vehicle) -> f64 {
    dist_to_entry_raw(v.x, v.y, v.arm)
}

fn dist_to_entry_raw(x: f64, y: f64, arm: Arm) -> f64 {
    match arm {
        Arm::North => (IY - y).max(0.0),
        Arm::South => (y - (IY + ROAD)).max(0.0),
        Arm::East  => (x - (IX + ROAD)).max(0.0),
        Arm::West  => (IX - x).max(0.0),
    }
}
