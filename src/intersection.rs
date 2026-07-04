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
        // Reject if spawn point is already occupied
        for e in &self.vehicles {
            if e.arm == v.arm && e.turn == v.turn {
                let d = dist(e.x,e.y,v.x,v.y);
                if d < GAP * 2.2 { return; }
            }
        }
        self.vehicles.push(v);
    }

    pub fn update(&mut self, dt: f64, stats: &mut Statistics) {
        // ── 1. Snapshot for read-only decisions ───────────────────────────
        let snap: Vec<(u64,Arm,Turn,Phase,f64,f64,u64)> = self.vehicles.iter()
            .map(|v|(v.id,v.arm,v.turn,v.phase,v.x,v.y,v.priority)).collect();

        // ── 2. Speed control (lane sensor + priority) ─────────────────────
        for i in 0..self.vehicles.len() {
            let spd = decide_speed(i, &self.vehicles, &snap);
            self.vehicles[i].spd = spd;
        }

        // ── 3. Move ────────────────────────────────────────────────────────
        let mut done_ids: Vec<u64> = Vec::new();
        for v in &mut self.vehicles {
            let finished = v.step(dt);
            if v.phase == Phase::Approaching && in_box(v.x, v.y) {
                v.phase = Phase::Crossing;
                v.entry_t = Some(Instant::now());
            }
            if v.phase == Phase::Crossing && !in_box(v.x, v.y) {
                v.phase = Phase::Exiting;
                v.exit_t  = Some(Instant::now());
                if let Some(t) = v.elapsed() { stats.record_time(t); }
                stats.record_spd(v.max_spd, v.min_spd);
            }
            if finished { done_ids.push(v.id); }
        }

        // ── 4. Remove finished ─────────────────────────────────────────────
        self.vehicles.retain(|v| {
            if done_ids.contains(&v.id) {
                stats.total_passed += 1;
                stats.max_passed = stats.max_passed.max(stats.total_passed);
                false
            } else { true }
        });
        self.total_passed = stats.total_passed;

        // ── 5. Close-call detection ────────────────────────────────────────
        stats.close_calls += count_close_calls(&self.vehicles);
    }
}

// ---------------------------------------------------------------------------
// Speed decision logic
// ---------------------------------------------------------------------------

fn decide_speed(
    idx: usize,
    vehicles: &[Vehicle],
    snap: &[(u64,Arm,Turn,Phase,f64,f64,u64)],
) -> Spd {
    let v = &vehicles[idx];

    // ── Rule 1: Lane sensor — same arm+turn, car directly ahead ───────────
    // Scan a forward cone (SENSOR_RANGE long, SENSOR_HALF_W wide)
    let min_ahead = snap.iter()
        .filter(|s| s.0 != v.id && s.1 == v.arm && s.2 == v.turn)
        .filter_map(|s| {
            let fwd = v.forward_dist_to(s.4, s.5);
            let lat = v.lateral_offset_to(s.4, s.5).abs();
            if fwd > 0.0 && fwd < v.sensor_range && lat < SENSOR_HALF_W {
                Some(fwd)
            } else {
                None
            }
        })
        .fold(f64::MAX, f64::min);

    if min_ahead < STOP_GAP  { return Spd::Slow; }
    if min_ahead < GAP       { return Spd::Med;  }

    // ── Rule 2: Cross-path sensor — future path collision detection ───────
    // Look at every vehicle on a conflicting path within PRIORITY_DIST.
    // The vehicle with the LOWER priority number (spawned first) has priority
    // and keeps FAST; the higher number (newer) slows down.
    if v.phase != Phase::Exiting {
        let intruder = snap.iter().find(|s| {
            if s.0 == v.id { return false; }
            if !paths_conflict(v.arm, v.turn, s.1, s.2) { return false; }
            // Is the other vehicle's current position inside MY sensor cone
            // OR are they approaching the same intersection zone?
            let fwd  = v.forward_dist_to(s.4, s.5);
            let lat  = v.lateral_offset_to(s.4, s.5).abs();
            let close_enough = dist(v.x,v.y,s.4,s.5) < PRIORITY_DIST;
            let in_cone = fwd > -VH && fwd < v.sensor_range && lat < SENSOR_HALF_W * 3.0;
            close_enough && (in_cone || s.3 == Phase::Crossing)
        });

        if let Some(other) = intruder {
            // Lower priority number = spawned earlier = has right-of-way
            if v.priority > other.6 {
                // We yield: slow down until they are no longer in our path
                let d = dist(v.x,v.y,other.4,other.5);
                return if d < PRIORITY_DIST * 0.5 { Spd::Slow } else { Spd::Med };
            }
            // We have priority: continue at full speed (fall through to Fast)
        }
    }

    Spd::Fast
}

// ---------------------------------------------------------------------------
// Close-call counter
// ---------------------------------------------------------------------------
fn count_close_calls(vehicles: &[Vehicle]) -> usize {
    let n = vehicles.len();
    let mut c = 0;
    for i in 0..n {
        for j in (i+1)..n {
            let a = &vehicles[i]; let b = &vehicles[j];
            if a.arm == b.arm && a.turn == b.turn { continue; }
            let d = dist(a.x,a.y,b.x,b.y);
            if d < VH * 1.2 && d > 1.0 { c += 1; }
        }
    }
    c / 2
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
fn in_box(x: f64, y: f64) -> bool {
    x >= IX_BOX_L && x <= IX_BOX_R && y >= IX_BOX_T && y <= IX_BOX_B
}

#[inline]
fn dist(ax:f64,ay:f64,bx:f64,by:f64)->f64{
    ((ax-bx)*(ax-bx)+(ay-by)*(ay-by)).sqrt()
}
