//! Intersection simulation.
//!
//! Speed system — ported directly from Golden76z/smart-road:
//!
//!  Every vehicle owns a `vel_timer: Instant`. Speed may only change once
//!  every VELOCITY_COOLDOWN (20 ms). Between updates the previous speed is
//!  kept. This is the single most important detail: without the cooldown,
//!  cars re-evaluate speed every frame (~16ms) and can briefly see a clear
//!  path during the same frame another car is crossing — causing phasing /
//!  ghost collisions.
//!
//!  Each frame the vehicle projects a forward hitbox of decreasing length:
//!    BIG  (400px) → keep FAST  (200 px/s)
//!    MED  (300px) → keep FAST  (200 px/s)   <- comfortable gap, still fast
//!    SMALL(225px) → SLOW       (100 px/s)
//!    VSML (100px) → VERY_SLOW  ( 60 px/s)
//!    ASTP ( 51px) → ALMOST_STOP( 30 px/s)
//!    STOP ( 50px) → hold position (vel = 0)
//!
//!  The STOP hitbox is 1 px smaller than ALMOST_STOP. If even the 50px box
//!  overlaps, the vehicle is physically blocked — velocity set to 0.
//!
//!  Same-lane vehicles use a direct forward-distance check first (cleaner
//!  than hitbox for cars on identical paths).

use std::time::Instant;
use crate::config::*;
use crate::stats::Statistics;
use crate::vehicle::{Arm, Phase, Turn, Vehicle, aabb, aabb_overlap};

// Ordered hitbox sizes + resulting speeds (Golden76z order)
const HB_SIZES:  [f64; 6] = [HB_BIG, HB_MEDIUM, HB_SMALL, HB_VERY_SMALL, HB_ALMOST_STOP, HB_STOP];
const HB_SPEEDS: [f64; 6] = [SPD_FAST, SPD_FAST, SPD_SLOW, SPD_VERY_SLOW, SPD_ALMOST_STOP, 0.0];

pub struct World {
    pub vehicles:     Vec<Vehicle>,
    pub total_passed: usize,
    // Per-vehicle velocity cooldown timer: (vehicle_id, last_update)
    vel_timers: Vec<(u64, Instant)>,
}

impl World {
    pub fn new() -> Self {
        World {
            vehicles: Vec::new(),
            total_passed: 0,
            vel_timers: Vec::new(),
        }
    }

    /// Spawn only if there is enough gap on the same lane.
    pub fn spawn(&mut self, v: Vehicle) {
        for e in &self.vehicles {
            if e.arm == v.arm && e.turn == v.turn {
                let d = dist(e.x, e.y, v.x, v.y);
                if d < SPAWN_GAP { return; }
            }
        }
        self.vel_timers.push((v.id, Instant::now()));
        self.vehicles.push(v);
    }

    pub fn update(&mut self, dt: f64, stats: &mut Statistics) {
        let now = Instant::now();

        // ── 1. Precompute all body AABBs (used for hitbox overlap tests) ──
        let body_aabbs: Vec<(u64, (f64,f64,f64,f64))> = self.vehicles.iter()
            .map(|v| (v.id, body_aabb(v)))
            .collect();

        // ── 2. Build lightweight snapshot for same-lane distance checks ──
        let snap: Vec<(u64,Arm,Turn,f64,f64)> = self.vehicles.iter()
            .map(|v| (v.id, v.arm, v.turn, v.x, v.y))
            .collect();

        // ── 3. Per-vehicle speed decision with cooldown gate ────────────────
        for i in 0..self.vehicles.len() {
            let vid = self.vehicles[i].id;

            // Find this vehicle's cooldown timer
            let timer_idx = self.vel_timers.iter().position(|(id,_)| *id == vid);
            let can_update = if let Some(ti) = timer_idx {
                now.duration_since(self.vel_timers[ti].1) >= VELOCITY_COOLDOWN
            } else {
                true
            };

            if !can_update {
                // Hold previous speed, skip recalculation
                continue;
            }

            let new_spd = decide_speed(i, &self.vehicles, &snap, &body_aabbs);
            self.vehicles[i].spd_px = new_spd;

            // Record timer update
            if let Some(ti) = timer_idx {
                self.vel_timers[ti].1 = now;
            }
        }

        // Clean up timers for removed vehicles (run after update loop)
        // (done below after retain)

        // ── 4. Move vehicles & phase transitions ───────────────────────────
        let mut done_ids: Vec<u64> = Vec::new();
        for v in &mut self.vehicles {
            // Hold position if fully stopped
            if v.spd_px > 0.0 {
                let finished = v.step(dt);
                if finished { done_ids.push(v.id); }
            }

            // Phase: Approaching -> Crossing
            if v.phase == Phase::Approaching && in_box(v.x, v.y) {
                v.phase   = Phase::Crossing;
                v.entry_t = Some(Instant::now());
            }
            // Phase: Crossing -> Exiting
            if v.phase == Phase::Crossing && !in_box(v.x, v.y) {
                v.phase  = Phase::Exiting;
                v.exit_t = Some(Instant::now());
                if let Some(t) = v.elapsed() { stats.record_time(t); }
                stats.record_spd(v.max_spd, v.min_spd_d());
            }
        }

        // ── 5. Remove finished vehicles ───────────────────────────────────
        self.vehicles.retain(|v| {
            if done_ids.contains(&v.id) {
                stats.total_passed += 1;
                stats.max_passed = stats.max_passed.max(stats.total_passed);
                false
            } else { true }
        });
        self.total_passed = stats.total_passed;

        // Clean stale timers
        self.vel_timers.retain(|(id,_)| self.vehicles.iter().any(|v| v.id == *id));

        // ── 6. Stats: crash & close-call detection ────────────────────────
        let n = self.vehicles.len();
        for i in 0..n {
            for j in (i+1)..n {
                let ai = self.vehicles[i].arm; let at = self.vehicles[i].turn;
                let bi = self.vehicles[j].arm; let bt = self.vehicles[j].turn;
                if ai == bi && at == bt { continue; } // same lane, not a crash
                let d = dist(
                    self.vehicles[i].x, self.vehicles[i].y,
                    self.vehicles[j].x, self.vehicles[j].y,
                );
                if d < VH * 2.0 { stats.close_calls += 1; }
                if d < CRASH_DIST && !self.vehicles[i].crashed && !self.vehicles[j].crashed {
                    stats.crashes += 1;
                    self.vehicles[i].crashed = true;
                    self.vehicles[j].crashed = true;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Speed decision — ported from Golden76z create_hitbox + adapt_velocity
// ---------------------------------------------------------------------------
fn decide_speed(
    idx: usize,
    vehicles: &[Vehicle],
    snap: &[(u64,Arm,Turn,f64,f64)],
    body_aabbs: &[(u64,(f64,f64,f64,f64))],
) -> f64 {
    let v = &vehicles[idx];

    // Exiting cars: full speed, no checks needed
    if v.phase == Phase::Exiting { return SPD_FAST; }

    // ── Rule 1: Same-lane following (Golden76z: SAFE_DISTANCE check) ─────────
    let same_lane_gap: f64 = snap.iter()
        .filter(|s| s.0 != v.id && s.1 == v.arm && s.2 == v.turn)
        .filter_map(|s| {
            let fwd = v.forward_dist_to(s.3, s.4);
            let lat = v.lateral_offset_to(s.3, s.4).abs();
            if fwd > 0.0 && fwd < SENSOR_RANGE && lat < VW * 0.9 {
                Some(fwd)
            } else { None }
        })
        .fold(f64::MAX, f64::min);

    // Hard stop at STOP_GAP, slow down at SAFE_DISTANCE
    if same_lane_gap < STOP_GAP      { return 0.0; }
    if same_lane_gap < SAFE_DISTANCE { return SPD_SLOW; }

    // ── Rule 2: Cross-path hitbox shrink (Golden76z: create_hitbox loop) ────
    //
    // Collect body AABBs of every OTHER vehicle (any direction).
    // Try each hitbox size from BIG down to STOP.
    // First size whose projected box doesn’t overlap anyone -> that speed.
    // If even STOP box overlaps -> vel = 0 (physically blocked).

    let others: Vec<(f64,f64,f64,f64)> = body_aabbs.iter()
        .filter(|(id,_)| *id != v.id)
        .map(|(_,bb)| *bb)
        .collect();

    for (level, &hb_len) in HB_SIZES.iter().enumerate() {
        let corners = v.hitbox_corners(hb_len);
        let my_bb   = aabb(&corners);
        let overlaps = others.iter().any(|&o| aabb_overlap(my_bb, o));
        if !overlaps {
            return HB_SPEEDS[level];
        }
    }

    // All boxes overlap — fully stopped
    0.0
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// AABB of the vehicle body itself (not the sensor box).
pub fn body_aabb(v: &Vehicle) -> (f64,f64,f64,f64) {
    let a = v.angle();
    let (fx,fy) = (a.cos(), a.sin());
    let (px,py) = (-fy, fx);
    let hw = VW / 2.0; let hh = VH / 2.0;
    let corners = [
        (v.x - fx*hh - px*hw, v.y - fy*hh - py*hw),
        (v.x - fx*hh + px*hw, v.y - fy*hh + py*hw),
        (v.x + fx*hh + px*hw, v.y + fy*hh + py*hw),
        (v.x + fx*hh - px*hw, v.y + fy*hh - py*hw),
    ];
    let min_x = corners.iter().map(|c|c.0).fold(f64::MAX, f64::min);
    let max_x = corners.iter().map(|c|c.0).fold(f64::MIN, f64::max);
    let min_y = corners.iter().map(|c|c.1).fold(f64::MAX, f64::min);
    let max_y = corners.iter().map(|c|c.1).fold(f64::MIN, f64::max);
    (min_x, min_y, max_x, max_y)
}

#[inline]
fn in_box(x: f64, y: f64) -> bool {
    x >= IX_L && x <= IX_R && y >= IX_T && y <= IX_B
}
#[inline]
fn dist(ax:f64,ay:f64,bx:f64,by:f64)->f64{
    ((ax-bx)*(ax-bx)+(ay-by)*(ay-by)).sqrt()
}
