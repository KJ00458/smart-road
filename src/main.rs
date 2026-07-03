mod config;
mod intersection;
mod path;
mod renderer;
mod stats;
mod vehicle;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::collections::HashMap;
use std::time::{Duration, Instant};

use config::*;
use intersection::World;
use renderer::Renderer;
use stats::Statistics;
use vehicle::{Arm, Vehicle};

fn main() {
    let sdl    = sdl2::init().expect("SDL2");
    let video  = sdl.video().expect("video");
    let ttf    = sdl2::ttf::init().expect("TTF");
    let win    = video.window("Smart Road", WINDOW_W, WINDOW_H)
        .position_centered().build().expect("window");
    let canvas = win.into_canvas().accelerated().present_vsync().build().expect("canvas");
    let tc     = canvas.texture_creator();

    let mut renderer = Renderer::new(canvas, &tc, &ttf);
    let mut events   = sdl.event_pump().expect("events");
    let mut world    = World::new();
    let mut stats    = Statistics::new();

    let mut rng_mode  = false;
    let mut last_rand = Instant::now();
    let mut key_cd: HashMap<Keycode, Instant> = HashMap::new();
    let frame = Duration::from_secs_f64(1.0 / FPS);

    'main: loop {
        let t0 = Instant::now();
        for ev in events.poll_iter() {
            match ev {
                Event::Quit {..} => break 'main,
                Event::KeyDown { keycode: Some(k), repeat: false, .. } => match k {
                    Keycode::Escape => { renderer.show_stats(&stats); break 'main; }
                    Keycode::R      => rng_mode = !rng_mode,
                    Keycode::Up | Keycode::Down | Keycode::Left | Keycode::Right => {
                        let now = Instant::now();
                        let ok  = key_cd.get(&k).map(|t| now.duration_since(*t) >= KEY_CD).unwrap_or(true);
                        if ok {
                            world.spawn(Vehicle::new_from_arm(key_to_arm(k)));
                            key_cd.insert(k, now);
                        }
                    }
                    _ => {}
                },
                _ => {}
            }
        }
        if rng_mode && last_rand.elapsed() >= RAND_CD {
            world.spawn(Vehicle::new_random());
            last_rand = Instant::now();
        }
        world.update(frame.as_secs_f64(), &mut stats);
        renderer.draw(&world, &stats, rng_mode);
        let e = t0.elapsed();
        if e < frame { std::thread::sleep(frame - e); }
    }
}

fn key_to_arm(k: Keycode) -> Arm {
    match k {
        Keycode::Up    => Arm::North,
        Keycode::Down  => Arm::South,
        Keycode::Left  => Arm::West,
        Keycode::Right => Arm::East,
        _ => unreachable!(),
    }
}
