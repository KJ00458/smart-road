//! Intersection simulation — Golden76z hitbox-shrink speed system.
//!
//! HOW IT WORKS (inspired by Golden76z/smart-road):
//!
//!  Each vehicle projects a forward hitbox of decreasing size:
//!    Level 0 (BIG)        → SPD_FAST  — clear road ahead
//!    Level 1 (MEDIUM)     → SPD_FAST  — comfortable gap
//!    Level 2 (SMALL)      → SPD_MED   — cautious
//!    Level 3 (VERY_SMALL) → SPD_SLOW  — someone close
//!    Level 4 (STOP)       → SPD_SLOW  — nearly stopped
//!
//!  Every frame we test each car's BIG hitbox. If it overlaps another car
//!  we shrink to MEDIUM, test again, etc. The first size that doesn't
//!  collide is the car's speed for this frame.
//!
//!  Same-lane cars (same arm+turn) use a simpler forward-distance check so
//!  they queue up cleanly without hitbox edge-cases.
//!
//!  Result: the first car through the intersection keeps BIG → FAST.
//!  Cars behind shrink → slow. No deadlock because every car is always
//!  moving at SOME speed unless physically blocked.

use std::time::Instant;
use crate::config::*;
use crate::stats::Statistics;
use crate::vehicle::{Arm, Phase, Spd, Turn, Vehicle, aabb, aabb_overlap, paths_conflict};

const IX_BOX_L: f64 = IX;
const IX_BOX_R: f64 = IX + ROAD;
const IX_BOX_T: f64 = IY;
const IX_BOX_B: f64 = IY + ROAD;

// Hitbox sizes in order (Golden76z approach)
const HB_SIZES: [f64; 5] = [HB_BIG, HB_MEDIUM, HB_SMALL, HB_VERY_SMALL, HB_STOP];
const HB_SPEEDS: [Spd; 5] = [Spd::Fast, Spd::Fast, Spd::Med, Spd::Slow, Spd::Slow];

pub struct World {
    pub vehicles:     Vec<Vehicle>,
    pub total_passed: usize,
}

impl World {
    pub fn new() -> Self { World { vehicles: Vec::new(), total_passed: 0 } }

    /// Spawn only if enough gap exists on the same spawn lane.
    pub fn spawn(&mut self, v: Vehicle) {
        for e in &self.vehicles {
            if e.arm == v.arm && e.turn == v.turn {
                if dist(e.x,e.y,v.x,v.y) < GAP * 2.2 { return; }
            }
        }
        self.vehicles.push(v);
    }

    pub fn update(&mut self, dt: f64, stats: &mut Statistics) {
        // ── 1. Build snapshot (id, arm, turn, phase, x, y, hitbox_level) ──
        let snap: Vec<(u64,Arm,Turn,Phase,f64,f64,usize)> = self.vehicles.iter()
            .map(|v|(v.id,v.arm,v.turn,v.phase,v.x,v.y,v.hitbox_level)).collect();

        // ── 2. Hitbox-shrink speed decision (Golden76z style) ─────────────
        //
        // For each vehicle, precompute the AABBs of all OTHER vehicles'
        // bodies (not their sensor boxes — their actual body box).
        // Then try shrinking our sensor hitbox until it doesn't overlap any
        // of them. That size → our speed.
        //
        // Same-lane follow uses direct forward-distance (simpler & more
        // accurate for cars queued on the approach lane).

        // Precompute body AABBs for every car (for hitbox overlap tests)
        let body_aabbs: Vec<(u64, (f64,f64,f64,f64))> = self.vehicles.iter()
            .map(|v| {
                let hw = VW/2.0; let hh = VH/2.0;
                let a = v.angle();
                let (fx,fy) = (a.cos(), a.sin());
                let (px,py) = (-fy, fx);
                let corners = [
                    (v.x - fx*hh - px*hw, v.y - fy*hh - py*hw),
                    (v.x - fx*hh + px*hw, v.y - fy*hh + py*hw),
                    (v.x + fx*hh + px*hw, v.y + fy*hh + py*hw),
                    (v.x + fx*hh - px*hw, v.y + fy*hh - py*hw),
                ];
                let min_x = corners.iter().map(|c|c.0).fold(f64::MAX,f64::min);
                let max_x = corners.iter().map(|c|c.0).fold(f64::MIN,f64::max);
                let min_y = corners.iter().map(|c|c.1).fold(f64::MAX,f64::min);
                let max_y = corners.iter().map(|c|c.1).fold(f64::MIN,f64::max);
                (v.id, (min_x,min_y,max_x,max_y))
            })
            .collect();

        for i in 0..self.vehicles.len() {
            let (spd, level) = decide_speed(i, &self.vehicles, &snap, &body_aabbs);
            self.vehicles[i].spd = spd;
            self.vehicles[i].hitbox_level = level;
        }

        // ── 3. Move & phase transitions ───────────────────────────────────
        let mut done_ids: Vec<u64> = Vec::new();
        for v in &mut self.vehicles {
            let finished = v.step(dt);
            if v.phase == Phase::Approaching && in_box(v.x, v.y) {
                v.phase   = Phase::Crossing;
                v.entry_t = Some(Instant::now());
            }
            if v.phase == Phase::Crossing && !in_box(v.x, v.y) {
                v.phase  = Phase::Exiting;
                v.exit_t = Some(Instant::now());
                if let Some(t) = v.elapsed() { stats.record_time(t); }
                stats.record_spd(v.max_spd, v.min_spd);
            }
            if finished { done_ids.push(v.id); }
        }

        // ── 4. Remove finished ────────────────────────────────────────────
        self.vehicles.retain(|v| {
            if done_ids.contains(&v.id) {
                stats.total_passed += 1;
                stats.max_passed = stats.max_passed.max(stats.total_passed);
                false
            } else { true }
        });
        self.total_passed = stats.total_passed;

        // ── 5. Crash & close-call detection ──────────────────────────────
        let n = self.vehicles.len();
        let mut crash_pairs: Vec<(usize,usize)> = Vec::new();
        for i in 0..n {
            for j in (i+1)..n {
                let ax=self.vehicles[i].x; let ay=self.vehicles[i].y;
                let bx=self.vehicles[j].x; let by=self.vehicles[j].y;
                let ai=self.vehicles[i].arm; let at=self.vehicles[i].turn;
                let bi=self.vehicles[j].arm; let bt=self.vehicles[j].turn;
                let ac=self.vehicles[i].crashed; let bc=self.vehicles[j].crashed;
                if ai==bi && at==bt { continue; }
                let d=dist(ax,ay,bx,by);
                if d < VH*1.5 && d > 1.0 { stats.close_calls += 1; }
                if d < CRASH_DIST {
                    if !ac && !bc { stats.crashes += 1; }
                    crash_pairs.push((i,j));
                }
            }
        }
        for (i,j) in crash_pairs {
            self.vehicles[i].crashed = true;
            self.vehicles[j].crashed = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Speed decision — Golden76z hitbox-shrink system
// ---------------------------------------------------------------------------
fn decide_speed(
    idx: usize,
    vehicles: &[Vehicle],
    snap: &[(u64,Arm,Turn,Phase,f64,f64,usize)],
    body_aabbs: &[(u64,(f64,f64,f64,f64))],
) -> (Spd, usize) {
    let v = &vehicles[idx];

    // ── Rule 1: Same-lane following (forward distance check) ──────────────
    // Cars on identical paths queue up with direct forward distance.
    let same_lane_ahead: f64 = snap.iter()
        .filter(|s| s.0 != v.id && s.1 == v.arm && s.2 == v.turn)
        .filter_map(|s| {
            let fwd = v.forward_dist_to(s.4, s.5);
            let lat = v.lateral_offset_to(s.4, s.5).abs();
            if fwd > 0.0 && fwd < SENSOR_RANGE && lat < VW * 0.8 {
                Some(fwd)
            } else { None }
        })
        .fold(f64::MAX, f64::min);

    if same_lane_ahead < STOP_GAP { return (Spd::Slow, 4); }
    if same_lane_ahead < GAP      { return (Spd::Med, 2);  }

    // ── Rule 2: Cross-path hitbox shrink (Golden76z approach) ─────────────
    // Only active when approaching or crossing (not after we've exited the box).
    if v.phase == Phase::Exiting {
        return (Spd::Fast, 0);
    }

    // Collect body AABBs of all OTHER vehicles whose path conflicts with ours.
    let conflict_aabbs: Vec<(f64,f64,f64,f64)> = body_aabbs.iter()
        .filter(|(id, _)| {
            if *id == v.id { return false; }
            // Find the snap entry for this id to get arm/turn
            if let Some(s) = snap.iter().find(|s| s.0 == *id) {
                // Include same-lane cars too (already handled above but
                // harmless) AND conflicting-path cars.
                // Actually: only include cars whose path conflicts OR who
                // share the lane (for safety).
                s.1 != v.arm || s.2 != v.turn // different lane = potential conflict
            } else { false }
        })
        .map(|(_, aabb)| *aabb)
        .collect();

    // Try each hitbox size from biggest to smallest.
    // The biggest that doesn't overlap anyone → our speed this frame.
    for (level, &hb_len) in HB_SIZES.iter().enumerate() {
        let corners = v.hitbox_corners(hb_len);
        let my_aabb = aabb(&corners);
        let overlaps = conflict_aabbs.iter().any(|&other| aabb_overlap(my_aabb, other));
        if !overlaps {
            return (HB_SPEEDS[level], level);
        }
    }

    // All sizes overlap — fully stopped
    (Spd::Slow, 4)
}

// ---------------------------------------------------------------------------
fn in_box(x: f64, y: f64) -> bool {
    x >= IX_BOX_L && x <= IX_BOX_R && y >= IX_BOX_T && y <= IX_BOX_B
}
#[inline]
fn dist(ax:f64,ay:f64,bx:f64,by:f64)->f64{
    ((ax-bx)*(ax-bx)+(ay-by)*(ay-by)).sqrt()
}
