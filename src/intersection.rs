use std::time::Instant;
use crate::config::*;
use crate::stats::Statistics;
use crate::vehicle::{Arm, Phase, Spd, Turn, Vehicle, paths_conflict};

// Intersection box boundaries
const IX_BOX_L: f64 = IX;
const IX_BOX_R: f64 = IX + ROAD;
const IX_BOX_T: f64 = IY;
const IX_BOX_B: f64 = IY + ROAD;

pub struct World {
    pub vehicles:     Vec<Vehicle>,
    pub total_passed: usize,
}

impl World {
    pub fn new() -> Self { World { vehicles: Vec::new(), total_passed: 0 } }

    /// Spawn a vehicle only if there is enough gap on its spawn lane.
    pub fn spawn(&mut self, v: Vehicle) {
        for e in &self.vehicles {
            // Same spawn lane = same arm AND same turn (same waypoint path)
            if e.arm == v.arm && e.turn == v.turn {
                if dist(e.x, e.y, v.x, v.y) < GAP * 2.2 { return; }
            }
        }
        self.vehicles.push(v);
    }

    pub fn update(&mut self, dt: f64, stats: &mut Statistics) {
        // ── 1. Snapshot positions/metadata for speed decisions ──────────────
        // (id, arm, turn, phase, x, y, priority)
        let snap: Vec<(u64,Arm,Turn,Phase,f64,f64,u64)> = self.vehicles.iter()
            .map(|v|(v.id,v.arm,v.turn,v.phase,v.x,v.y,v.priority)).collect();

        // ── 2. Speed control ───────────────────────────────────────────────
        for i in 0..self.vehicles.len() {
            let spd = decide_speed(i, &self.vehicles, &snap);
            self.vehicles[i].spd = spd;
        }

        // ── 3. Move & phase transitions ────────────────────────────────────
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

        // ── 4. Remove finished ─────────────────────────────────────────────
        self.vehicles.retain(|v| {
            if done_ids.contains(&v.id) {
                stats.total_passed += 1;
                stats.max_passed = stats.max_passed.max(stats.total_passed);
                false
            } else { true }
        });
        self.total_passed = stats.total_passed;

        // ── 5. Crash & close-call detection ───────────────────────────────
        let n = self.vehicles.len();
        let mut crash_pairs: Vec<(usize, usize)> = Vec::new();
        for i in 0..n {
            for j in (i+1)..n {
                let ax = self.vehicles[i].x; let ay = self.vehicles[i].y;
                let bx = self.vehicles[j].x; let by = self.vehicles[j].y;
                let ai = self.vehicles[i].arm; let at = self.vehicles[i].turn;
                let bi = self.vehicles[j].arm; let bt = self.vehicles[j].turn;
                let ac = self.vehicles[i].crashed;
                let bc = self.vehicles[j].crashed;
                if ai == bi && at == bt { continue; }
                let d = dist(ax,ay,bx,by);
                if d < VH * 1.5 && d > 1.0 { stats.close_calls += 1; }
                if d < CRASH_DIST {
                    if !ac && !bc { stats.crashes += 1; }
                    crash_pairs.push((i, j));
                }
            }
        }
        for (i, j) in crash_pairs {
            self.vehicles[i].crashed = true;
            self.vehicles[j].crashed = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Speed decision — called each frame for every vehicle
// ---------------------------------------------------------------------------
fn decide_speed(
    idx: usize,
    vehicles: &[Vehicle],
    snap: &[(u64,Arm,Turn,Phase,f64,f64,u64)],
) -> Spd {
    let v = &vehicles[idx];

    // ── Rule 1: Same-lane sensor (follow the car ahead on my exact path) ──
    //
    // We look at all vehicles with the same arm+turn (same lane) and find
    // the closest one that is AHEAD of us along the shared path.
    let same_lane_ahead = snap.iter()
        .filter(|s| s.0 != v.id && s.1 == v.arm && s.2 == v.turn)
        .filter_map(|s| {
            // Use path-distance so we handle turns correctly
            let fwd = v.forward_dist_to(s.4, s.5);
            let lat = v.lateral_offset_to(s.4, s.5).abs();
            if fwd > 0.0 && fwd < v.sensor_range && lat < SENSOR_HALF_W {
                Some(fwd)
            } else { None }
        })
        .fold(f64::MAX, f64::min);

    if same_lane_ahead < STOP_GAP { return Spd::Slow; }
    if same_lane_ahead < GAP      { return Spd::Med;  }

    // ── Rule 2: Cross-path / priority sensor ──────────────────────────────
    //
    // Only applies when Approaching or Crossing (not after we've exited).
    // We scan all other vehicles whose path conflicts with ours AND whose
    // current position falls on our future path within sensor range.
    //
    // Priority rule:
    //   - Lower ID = spawned earlier = RIGHT OF WAY → keep Fast
    //   - Higher ID = spawned later  = must YIELD  → slow/stop
    //
    // Additionally: a car already IN the intersection box always has
    // priority over one still approaching, regardless of ID.
    if v.phase != Phase::Exiting {
        // Distance from this vehicle to the intersection box edge
        let v_dist_to_box = dist_to_box(v.x, v.y);

        let threatening = snap.iter().find(|s| {
            if s.0 == v.id { return false; }
            // Only care about conflicting paths
            if !paths_conflict(v.arm, v.turn, s.1, s.2) { return false; }

            // Is the other car within our sensor range?
            let total_dist = dist(v.x, v.y, s.4, s.5);
            if total_dist > PRIORITY_DIST { return false; }

            // Is the other car on (or near) our future path?
            let on_our_path = v.dist_to_future_path(s.4, s.5) < SENSOR_HALF_W * 2.5;
            // Or is it already in the intersection box crossing our path?
            let in_box_crossing = s.3 == Phase::Crossing;

            if !on_our_path && !in_box_crossing { return false; }

            // Determine who has priority:
            //   - Car already Crossing always beats Approaching
            //   - Otherwise lower ID wins
            let other_priority = s.6; // lower = better
            let other_in_box   = s.3 == Phase::Crossing;
            let i_am_in_box    = v.phase == Phase::Crossing;

            // If I am already crossing and the other is only approaching → I have priority, no yield
            if i_am_in_box && !other_in_box { return false; }
            // If other is crossing and I am approaching → I must yield
            if other_in_box && !i_am_in_box { return true; }
            // Both approaching or both crossing → lower ID wins
            // This vehicle must yield if its ID is higher (came later)
            v.priority > other_priority
        });

        if let Some(other) = threatening {
            let d = dist(v.x, v.y, other.4, other.5);
            // Stop completely if very close, medium if moderately close
            return if d < NEAR_BOX_DIST * 0.4 || v_dist_to_box < STOP_GAP {
                Spd::Slow
            } else {
                Spd::Med
            };
        }
    }

    // ── Default: open road, go full speed ────────────────────────────────
    Spd::Fast
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// True if (x,y) is inside the intersection box.
fn in_box(x: f64, y: f64) -> bool {
    x >= IX_BOX_L && x <= IX_BOX_R && y >= IX_BOX_T && y <= IX_BOX_B
}

/// Approximate distance from point (x,y) to the nearest edge of the intersection box.
/// Returns 0 if already inside.
fn dist_to_box(x: f64, y: f64) -> f64 {
    if in_box(x, y) { return 0.0; }
    let dx = (IX_BOX_L - x).max(0.0).max(x - IX_BOX_R);
    let dy = (IX_BOX_T - y).max(0.0).max(y - IX_BOX_B);
    (dx*dx + dy*dy).sqrt()
}

#[inline]
fn dist(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ((ax-bx)*(ax-bx)+(ay-by)*(ay-by)).sqrt()
}
