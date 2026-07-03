use std::time::Instant;
use crate::config::*;
use crate::stats::Statistics;
use crate::vehicle::{Arm, Phase, Turn, Vehicle, paths_conflict};

pub struct World {
    pub vehicles: Vec<Vehicle>,
    pub total_passed: usize,
}

impl World {
    pub fn new() -> Self { World { vehicles: Vec::new(), total_passed: 0 } }

    pub fn spawn(&mut self, v: Vehicle) {
        // Don't spawn if too close to another vehicle in same arm+lane
        for existing in &self.vehicles {
            if existing.arm == v.arm && existing.turn == v.turn
                && existing.phase == Phase::Approaching
            {
                let dist = (existing.x - v.x).abs() + (existing.y - v.y).abs();
                if dist < SAFE_DIST * 2.0 { return; }
            }
        }
        self.vehicles.push(v);
    }

    pub fn update(&mut self, dt: f64, stats: &mut Statistics) {
        // 1. Smart intersection control — set speed targets
        self.control_speeds();

        // 2. Snapshot positions for safe-distance checks
        let snaps: Vec<(usize, f64, f64, Arm, Turn, bool)> = self.vehicles.iter()
            .enumerate()
            .map(|(i,v)| (i, v.x, v.y, v.arm, v.turn, v.has_turned))
            .collect();

        // 3. Move each vehicle, checking vehicle-ahead safe distance
        for i in 0..self.vehicles.len() {
            // Find closest vehicle ahead in same arm+lane+direction-axis
            let ahead = self.find_ahead(i, &snaps);
            let v = &mut self.vehicles[i];

            // Mirror: should_turn -> turning
            if v.should_turn() && !v.has_turned {
                v.do_turn();
            }

            v.update(ahead, dt);

            // Mark entry into intersection box
            if v.phase == Phase::Approaching && in_box(v.x, v.y) {
                v.phase = Phase::Crossing;
                v.entry_t = Some(Instant::now());
            }
            // Mark exit from intersection box
            if v.phase == Phase::Crossing && !in_box(v.x, v.y) {
                v.phase = Phase::Exiting;
                v.exit_t  = Some(Instant::now());
                if let Some(t) = v.elapsed_secs() { stats.record_time(t); }
                stats.record_spd(v.max_spd, v.min_spd);
            }
        }

        // 4. Close-call detection
        stats.close_calls += self.count_close_calls();

        // 5. Remove done vehicles
        self.vehicles.retain(|v| {
            if v.is_done() {
                stats.total_passed += 1;
                stats.max_passed = stats.max_passed.max(stats.total_passed);
                false
            } else { true }
        });
        self.total_passed = stats.total_passed;
    }

    fn control_speeds(&mut self) {
        // Collect who is inside or right at the entry of the intersection
        let holders: Vec<(usize, Arm, Turn)> = self.vehicles.iter()
            .enumerate()
            .filter(|(_,v)| {
                v.phase == Phase::Crossing
                    || (v.phase == Phase::Approaching && dist_to_entry(v) < SAFE_DIST)
            })
            .map(|(i,v)| (i, v.arm, v.turn))
            .collect();

        for i in 0..self.vehicles.len() {
            let v = &self.vehicles[i];
            if v.phase != Phase::Approaching { continue; }
            let d = dist_to_entry(v);
            let zone = ROAD_W * 0.9;
            if d > zone {
                let spd = SPD_NORMAL;
                self.vehicles[i].set_speed(spd);
                continue;
            }
            let conflict = holders.iter().any(|(j, ha, ht)| {
                *j != i && paths_conflict(v.arm, v.turn, *ha, *ht)
            });
            if conflict {
                let spd = if d < SAFE_DIST * 1.1 { SPD_STOP } else { SPD_VSLOW };
                self.vehicles[i].set_speed(spd);
            } else {
                let spd = SPD_NORMAL;
                self.vehicles[i].set_speed(spd);
            }
        }
    }

    fn find_ahead(&self, idx: usize, snaps: &[(usize, f64, f64, Arm, Turn, bool)]) -> Option<(f64,f64)> {
        let vi = &snaps[idx];
        // Ahead = same arm, same turn (or same exit direction after turn), further along travel axis
        let mut best_dist = f64::MAX;
        let mut best: Option<(f64,f64)> = None;
        let (_, vx, vy, varm, vturn, _) = *vi;
        for sn in snaps {
            if sn.0 == idx { continue; }
            if sn.3 != varm || sn.4 != vturn { continue; }
            // Is sn ahead of vi in travel direction?
            let v = &self.vehicles[vi.0];
            let ahead = v.vx * (sn.1 - vx) + v.vy * (sn.2 - vy);
            if ahead > 0.0 {
                let d = (sn.1 - vx).powi(2) + (sn.2 - vy).powi(2);
                if d < best_dist { best_dist = d; best = Some((sn.1, sn.2)); }
            }
        }
        best
    }

    fn count_close_calls(&self) -> usize {
        let n = self.vehicles.len();
        let mut c = 0;
        for i in 0..n {
            for j in (i+1)..n {
                let a = &self.vehicles[i];
                let b = &self.vehicles[j];
                if a.arm == b.arm { continue; }
                let d = ((a.x-b.x).powi(2)+(a.y-b.y).powi(2)).sqrt();
                if d < SAFE_DIST * 0.4 && d > 1.0 { c += 1; }
            }
        }
        c / 2
    }
}

fn in_box(x: f64, y: f64) -> bool {
    x >= IX && x <= IX + ROAD_W && y >= IY && y <= IY + ROAD_W
}

fn dist_to_entry(v: &Vehicle) -> f64 {
    match v.arm {
        Arm::North => (IY - v.y).max(0.0),
        Arm::South => (v.y - (IY + ROAD_W)).max(0.0),
        Arm::East  => (v.x - (IX + ROAD_W)).max(0.0),
        Arm::West  => (IX - v.x).max(0.0),
    }
}
