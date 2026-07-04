mod config;
mod intersection;
mod path;
mod renderer;
mod stats;
mod vehicle;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::{Duration, Instant};

use config::*;
use intersection::World;
use renderer::Renderer;
use stats::Statistics;
use vehicle::{Arm, Turn, Vehicle};

fn main() {
    let sdl    = sdl2::init().expect("SDL2");
    let video  = sdl.video().expect("video");
    let ttf    = sdl2::ttf::init().expect("TTF");
    let win    = video.window("Smart Road", WINDOW_W, WINDOW_H)
        .position_centered().build().expect("window");
    let canvas = win.into_canvas().accelerated().present_vsync().build().expect("canvas");
    let tc     = canvas.texture_creator();

    let mut renderer  = Renderer::new(canvas, &tc, &ttf);
    let mut events    = sdl.event_pump().expect("events");
    let mut world     = World::new();
    let mut stats     = Statistics::new();

    let mut rng_mode  = false;
    let mut manual    = false;
    let mut sel_arm   = SelArm::None;
    let mut last_rand = Instant::now();
    let mut last_key  = Instant::now();
    let frame = Duration::from_secs_f64(1.0 / FPS);

    'main: loop {
        let t0 = Instant::now();

        for ev in events.poll_iter() {
            match ev {
                Event::Quit {..} => break 'main,
                Event::KeyDown { keycode: Some(k), repeat: false, .. } => match k {

                    Keycode::Escape => { renderer.show_stats(&stats); break 'main; }

                    // [R] = auto/random mode
                    Keycode::R => {
                        manual   = false;
                        rng_mode = true;
                        sel_arm  = SelArm::None;
                    }

                    // [M] = manual two-step mode
                    Keycode::M => {
                        manual   = true;
                        rng_mode = false;
                        sel_arm  = SelArm::None;
                    }

                    // Arrow keys
                    Keycode::Up | Keycode::Down | Keycode::Left | Keycode::Right => {
                        if manual {
                            // Step 1: select arm, show in HUD
                            sel_arm = match k {
                                Keycode::Up    => SelArm::North,
                                Keycode::Down  => SelArm::South,
                                Keycode::Left  => SelArm::West,
                                Keycode::Right => SelArm::East,
                                _ => unreachable!(),
                            };
                        } else {
                            // Auto: instant random-turn spawn with cooldown
                            if last_key.elapsed() >= KEY_CD {
                                let arm = match k {
                                    Keycode::Up    => Arm::North,
                                    Keycode::Down  => Arm::South,
                                    Keycode::Left  => Arm::West,
                                    Keycode::Right => Arm::East,
                                    _ => unreachable!(),
                                };
                                world.spawn(Vehicle::new_from_arm(arm));
                                last_key = Instant::now();
                            }
                        }
                    }

                    // Number keys: Step 2 of manual spawn
                    Keycode::Num1 | Keycode::Num2 | Keycode::Num3 => {
                        if manual && sel_arm != SelArm::None {
                            if last_key.elapsed() >= KEY_CD {
                                let arm = match sel_arm {
                                    SelArm::North => Arm::North,
                                    SelArm::South => Arm::South,
                                    SelArm::East  => Arm::East,
                                    SelArm::West  => Arm::West,
                                    SelArm::None  => unreachable!(),
                                };
                                let turn = match k {
                                    Keycode::Num1 => Turn::Right,
                                    Keycode::Num2 => Turn::Forward,
                                    Keycode::Num3 => Turn::Left,
                                    _ => unreachable!(),
                                };
                                world.spawn(Vehicle::new(arm, turn));
                                last_key = Instant::now();
                            }
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
        renderer.draw(&world, &stats, rng_mode, manual, sel_arm);

        let e = t0.elapsed();
        if e < frame { std::thread::sleep(frame - e); }
    }
}
