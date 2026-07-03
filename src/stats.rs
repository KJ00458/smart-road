use std::time::Instant;

pub struct Statistics {
    pub total_passed: usize,
    pub max_passed:   usize,
    pub close_calls:  usize,
    pub max_spd:      f64,
    pub min_spd:      f64,
    pub max_time:     f64,
    pub min_time:     f64,
    pub vel_sum:      f64,
    pub vel_count:    usize,
    pub session_start: Instant,
}

impl Statistics {
    pub fn new() -> Self {
        Statistics {
            total_passed: 0,
            max_passed: 0,
            close_calls: 0,
            max_spd: 0.0,
            min_spd: f64::MAX,
            max_time: 0.0,
            min_time: f64::MAX,
            vel_sum: 0.0,
            vel_count: 0,
            session_start: Instant::now(),
        }
    }

    pub fn record_spd(&mut self, mx: f64, mn: f64) {
        if mx > self.max_spd { self.max_spd = mx; }
        if mn < self.min_spd { self.min_spd = mn; }
        self.vel_sum   += mx;
        self.vel_count += 1;
    }

    pub fn record_time(&mut self, t: f64) {
        if t > self.max_time { self.max_time = t; }
        if t < self.min_time { self.min_time = t; }
    }

    pub fn avg_spd(&self) -> f64 {
        if self.vel_count == 0 { 0.0 } else { self.vel_sum / self.vel_count as f64 }
    }

    pub fn min_spd_disp(&self) -> f64 {
        if self.min_spd == f64::MAX { 0.0 } else { self.min_spd }
    }

    pub fn min_time_disp(&self) -> f64 {
        if self.min_time == f64::MAX { 0.0 } else { self.min_time }
    }

    pub fn session_secs(&self) -> f64 {
        self.session_start.elapsed().as_secs_f64()
    }
}
