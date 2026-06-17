use std::time::Instant;

#[derive(Debug, Default)]
pub struct Statistics {
    pub total_passed: usize,
    pub max_vehicles: usize,
    pub max_velocity: f64,
    pub min_velocity: f64,
    pub max_time: f64,
    pub min_time: f64,
    pub close_calls: usize,
    pub start_time: Option<Instant>,
    pub vehicle_count_over_time: Vec<(f64, usize)>,
    pub avg_velocity_samples: Vec<f64>,
}

impl Statistics {
    pub fn new() -> Self {
        Statistics {
            min_velocity: f64::MAX,
            min_time: f64::MAX,
            start_time: Some(Instant::now()),
            ..Default::default()
        }
    }

    pub fn record_time(&mut self, elapsed: f64) {
        if elapsed > self.max_time {
            self.max_time = elapsed;
        }
        if elapsed < self.min_time {
            self.min_time = elapsed;
        }
    }

    pub fn record_velocity(&mut self, max_v: f64, min_v: f64) {
        if max_v > self.max_velocity {
            self.max_velocity = max_v;
        }
        if min_v < self.min_velocity {
            self.min_velocity = min_v;
        }
        self.avg_velocity_samples.push((max_v + min_v) / 2.0);
    }

    pub fn avg_velocity(&self) -> f64 {
        if self.avg_velocity_samples.is_empty() {
            return 0.0;
        }
        self.avg_velocity_samples.iter().sum::<f64>() / self.avg_velocity_samples.len() as f64
    }

    pub fn session_duration(&self) -> f64 {
        self.start_time
            .map(|t| t.elapsed().as_secs_f64())
            .unwrap_or(0.0)
    }

    pub fn min_time_display(&self) -> f64 {
        if self.min_time == f64::MAX {
            0.0
        } else {
            self.min_time
        }
    }

    pub fn min_velocity_display(&self) -> f64 {
        if self.min_velocity == f64::MAX {
            0.0
        } else {
            self.min_velocity
        }
    }
}
