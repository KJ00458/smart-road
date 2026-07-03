mod config;
mod intersection;
mod stats;
mod vehicle;
mod renderer;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::{Duration, Instant};

use config::*;
use intersection::Intersection;
use renderer::Renderer;
use stats::Statistics;
use vehicle::{Arm, Vehicle};

fn main() {
    let sdl = sdl2::init().expect("SDL2 init failed");
    let video = sdl.video().expect("SDL2 video failed");
    let ttf = sdl2::ttf::init().expect("TTF init failed");

    let window = video
        .window("Smart Road — Autonomous Intersection", WINDOW_W, WINDOW_H)
        .position_centered()
        .build()
        .expect("Window creation failed");

    let canvas = window
        .into_canvas()
        .accelerated()
        .present_vsync()
        .build()
        .expect("Canvas creation failed");

    let texture_creator = canvas.texture_creator();
    let mut renderer = Renderer::new(canvas, &texture_creator, &ttf);

    let mut event_pump = sdl.event_pump().expect("Event pump failed");
    let mut intersection = Intersection::new();
    let mut stats = Statistics::new();

    let mut random_mode = false;
    let mut last_random_spawn = Instant::now();
    let mut key_spawn_cooldown: std::collections::HashMap<Keycode, Instant> =
        std::collections::HashMap::new();

    let target_frame = Duration::from_secs_f64(1.0 / TARGET_FPS);

    'running: loop {
        let frame_start = Instant::now();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => break 'running,
                Event::KeyDown {
                    keycode: Some(kc),
                    repeat: false,
                    ..
                } => match kc {
                    Keycode::Escape => {
                        renderer.show_statistics(&stats);
                        break 'running;
                    }
                    Keycode::R => {
                        random_mode = !random_mode;
                    }
                    Keycode::Up | Keycode::Down | Keycode::Left | Keycode::Right => {
                        let now = Instant::now();
                        let ready = key_spawn_cooldown
                            .get(&kc)
                            .map(|t| now.duration_since(*t) >= KEY_COOLDOWN)
                            .unwrap_or(true);
                        if ready {
                            let arm = arm_from_keycode(kc);
                            if let Some(v) = Vehicle::spawn_from_arm(arm, &intersection) {
                                intersection.add_vehicle(v);
                            }
                            key_spawn_cooldown.insert(kc, now);
                        }
                    }
                    _ => {}
                },
                _ => {}
            }
        }

        if random_mode && last_random_spawn.elapsed() >= RANDOM_SPAWN_INTERVAL {
            if let Some(v) = Vehicle::spawn_random(&intersection) {
                intersection.add_vehicle(v);
            }
            last_random_spawn = Instant::now();
        }

        let dt = target_frame.as_secs_f64();
        intersection.update(dt, &mut stats);
        renderer.draw_frame(&intersection, &stats, random_mode);

        let elapsed = frame_start.elapsed();
        if elapsed < target_frame {
            std::thread::sleep(target_frame - elapsed);
        }
    }
}

fn arm_from_keycode(kc: Keycode) -> Arm {
    match kc {
        Keycode::Up    => Arm::North,
        Keycode::Down  => Arm::South,
        Keycode::Right => Arm::East,
        Keycode::Left  => Arm::West,
        _ => unreachable!(),
    }
}
