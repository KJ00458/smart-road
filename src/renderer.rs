use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{Canvas, TextureCreator};
use sdl2::ttf::Sdl2TtfContext;
use sdl2::video::{Window, WindowContext};
use std::f64::consts::PI;

use crate::config::*;
use crate::intersection::Intersection;
use crate::stats::Statistics;
use crate::vehicle::{Direction, Route, Vehicle, VehicleState};

const VEHICLE_COLORS: [(u8, u8, u8); 8] = [
    (220, 50, 50),
    (50, 150, 220),
    (50, 200, 100),
    (240, 180, 30),
    (180, 80, 220),
    (240, 120, 40),
    (40, 220, 220),
    (220, 220, 220),
];

pub struct Renderer<'ttf, 'tc> {
    canvas: Canvas<Window>,
    font: sdl2::ttf::Font<'ttf, 'tc>,
    font_large: sdl2::ttf::Font<'ttf, 'tc>,
    _texture_creator: &'tc TextureCreator<WindowContext>,
}

impl<'ttf, 'tc> Renderer<'ttf, 'tc> {
    pub fn new(
        canvas: Canvas<Window>,
        texture_creator: &'tc TextureCreator<WindowContext>,
        ttf: &'ttf Sdl2TtfContext,
    ) -> Self {
        let font_path = find_system_font();
        let font = ttf
            .load_font(&font_path, 15)
            .expect("Could not load font. Please install a system TTF font.");
        let font_large = ttf
            .load_font(&font_path, 22)
            .expect("Could not load font large");

        Renderer {
            canvas,
            font,
            font_large,
            _texture_creator: texture_creator,
        }
    }

    pub fn draw_frame(&mut self, intersection: &Intersection) {
        let c = Color::RGB(
            GRASS_COLOR.0,
            GRASS_COLOR.1,
            GRASS_COLOR.2,
        );
        self.canvas.set_draw_color(c);
        self.canvas.clear();

        self.draw_roads();
        self.draw_lane_markings();
        self.draw_intersection_box();
        self.draw_route_labels();

        for v in &intersection.vehicles {
            self.draw_vehicle(v);
        }

        self.canvas.present();
    }

    fn draw_roads(&mut self) {
        let ix = INTERSECTION_X as i32;
        let iy = INTERSECTION_Y as i32;
        let iw = ROAD_WIDTH as i32;

        let road_color = Color::RGB(ROAD_COLOR.0, ROAD_COLOR.1, ROAD_COLOR.2);
        self.canvas.set_draw_color(road_color);

        self.canvas
            .fill_rect(Rect::new(ix, 0, iw as u32, WINDOW_H))
            .unwrap();
        self.canvas
            .fill_rect(Rect::new(0, iy, WINDOW_W, iw as u32))
            .unwrap();
    }

    fn draw_lane_markings(&mut self) {
        let ix = INTERSECTION_X as i32;
        let iy = INTERSECTION_Y as i32;
        let iw = ROAD_WIDTH as i32;
        let lw = LANE_WIDTH as i32;
        let dash_len = 20i32;
        let gap_len = 20i32;
        let line_color = Color::RGB(LANE_LINE_COLOR.0, LANE_LINE_COLOR.1, LANE_LINE_COLOR.2);
        self.canvas.set_draw_color(line_color);

        for lane in 1..LANE_COUNT {
            let lx = ix + lane as i32 * lw;
            let mut y = 0i32;
            while y < iy {
                let end_y = (y + dash_len).min(iy);
                self.canvas
                    .fill_rect(Rect::new(lx - 1, y, 2, (end_y - y) as u32))
                    .unwrap();
                y += dash_len + gap_len;
            }
            let mut y = iy + iw;
            while y < WINDOW_H as i32 {
                let end_y = (y + dash_len).min(WINDOW_H as i32);
                self.canvas
                    .fill_rect(Rect::new(lx - 1, y, 2, (end_y - y) as u32))
                    .unwrap();
                y += dash_len + gap_len;
            }
        }

        for lane in 1..LANE_COUNT {
            let ly = iy + lane as i32 * lw;
            let mut x = 0i32;
            while x < ix {
                let end_x = (x + dash_len).min(ix);
                self.canvas
                    .fill_rect(Rect::new(x, ly - 1, (end_x - x) as u32, 2))
                    .unwrap();
                x += dash_len + gap_len;
            }
            let mut x = ix + iw;
            while x < WINDOW_W as i32 {
                let end_x = (x + dash_len).min(WINDOW_W as i32);
                self.canvas
                    .fill_rect(Rect::new(x, ly - 1, (end_x - x) as u32, 2))
                    .unwrap();
                x += dash_len + gap_len;
            }
        }
    }

    fn draw_intersection_box(&mut self) {
        let ix = INTERSECTION_X as i32;
        let iy = INTERSECTION_Y as i32;
        let iw = ROAD_WIDTH as i32;
        let color = Color::RGB(60, 60, 60);
        self.canvas.set_draw_color(color);
        self.canvas
            .fill_rect(Rect::new(ix, iy, iw as u32, iw as u32))
            .unwrap();

        let border_color = Color::RGB(100, 100, 100);
        self.canvas.set_draw_color(border_color);
        self.canvas
            .draw_rect(Rect::new(ix, iy, iw as u32, iw as u32))
            .unwrap();
    }

    fn draw_route_labels(&mut self) {
        let ix = INTERSECTION_X as i32;
        let iy = INTERSECTION_Y as i32;
        let iw = ROAD_WIDTH as i32;
        let lw = LANE_WIDTH as i32;

        let label_color = Color::RGB(255, 255, 100);
        self.canvas.set_draw_color(label_color);

        let north_labels = ["← r", "↓ s", "→ l"];
        for (i, _lbl) in north_labels.iter().enumerate() {
            let lx = ix + i as i32 * lw + lw / 2 - 8;
            let ly = iy - 22;
            let _ = (lx, ly);
        }

        let south_labels = ["← l", "↑ s", "→ r"];
        for (i, _lbl) in south_labels.iter().enumerate() {
            let _lx = ix + i as i32 * lw + lw / 2 - 8;
            let _ly = iy + iw + 8;
        }

        let _ = south_labels;
    }

    fn draw_vehicle(&mut self, v: &Vehicle) {
        let c = VEHICLE_COLORS[v.color_index % VEHICLE_COLORS.len()];
        let body_color = Color::RGB(c.0, c.1, c.2);
        let window_color = Color::RGB(
            (c.0 as u16 * 6 / 10) as u8,
            (c.1 as u16 * 6 / 10) as u8,
            (c.2 as u16 * 6 / 10) as u8,
        );

        let hw = VEHICLE_W / 2.0;
        let hh = VEHICLE_H / 2.0;
        let angle = v.angle;

        let corners = [
            rotate_point(-hw, -hh, angle),
            rotate_point(hw, -hh, angle),
            rotate_point(hw, hh, angle),
            rotate_point(-hw, hh, angle),
        ];

        let points: Vec<sdl2::rect::Point> = corners
            .iter()
            .map(|(dx, dy)| {
                sdl2::rect::Point::new(
                    (v.x + dx) as i32,
                    (v.y + dy) as i32,
                )
            })
            .collect();

        self.canvas.set_draw_color(body_color);
        fill_polygon(&mut self.canvas, &points);

        let ww = hw * 0.55;
        let wh = hh * 0.35;
        let front_offset = hh * 0.35;

        let fw_corners = [
            rotate_point(-ww, -front_offset - wh, angle),
            rotate_point(ww, -front_offset - wh, angle),
            rotate_point(ww, -front_offset, angle),
            rotate_point(-ww, -front_offset, angle),
        ];
        let fw_points: Vec<sdl2::rect::Point> = fw_corners
            .iter()
            .map(|(dx, dy)| {
                sdl2::rect::Point::new((v.x + dx) as i32, (v.y + dy) as i32)
            })
            .collect();
        self.canvas.set_draw_color(window_color);
        fill_polygon(&mut self.canvas, &fw_points);

        let headlight_color = Color::RGB(255, 255, 200);
        self.canvas.set_draw_color(headlight_color);
        let hl_y = -hh + 4.0;
        for hx in &[-hw * 0.55, hw * 0.55] {
            let (dx, dy) = rotate_point(*hx, hl_y, angle);
            self.canvas
                .fill_rect(Rect::new(
                    (v.x + dx - 3.0) as i32,
                    (v.y + dy - 3.0) as i32,
                    6,
                    6,
                ))
                .unwrap();
        }

        let taillight_color = Color::RGB(200, 30, 30);
        self.canvas.set_draw_color(taillight_color);
        let tl_y = hh - 4.0;
        for tx in &[-hw * 0.55, hw * 0.55] {
            let (dx, dy) = rotate_point(*tx, tl_y, angle);
            self.canvas
                .fill_rect(Rect::new(
                    (v.x + dx - 3.0) as i32,
                    (v.y + dy - 3.0) as i32,
                    6,
                    6,
                ))
                .unwrap();
        }
    }

    pub fn show_statistics(&mut self, stats: &Statistics) {
        let tc = self._texture_creator;
        let mut running = true;
        let sdl_context = unsafe {
            sdl2::init().unwrap()
        };
        let mut ep = sdl_context.event_pump().unwrap_or_else(|_| {
            panic!("Cannot get event pump for stats window")
        });

        loop {
            if !running {
                break;
            }
            for event in ep.poll_iter() {
                use sdl2::event::Event;
                match event {
                    Event::Quit { .. } => running = false,
                    Event::KeyDown {
                        keycode: Some(sdl2::keyboard::Keycode::Escape),
                        ..
                    }
                    | Event::KeyDown {
                        keycode: Some(sdl2::keyboard::Keycode::Return),
                        ..
                    } => running = false,
                    _ => {}
                }
            }

            self.canvas
                .set_draw_color(Color::RGB(STATS_BG_COLOR.0, STATS_BG_COLOR.1, STATS_BG_COLOR.2));
            self.canvas.clear();

            self.draw_stats_panel(stats, tc);
            self.canvas.present();
            std::thread::sleep(std::time::Duration::from_millis(16));
        }
    }

    fn draw_stats_panel(
        &mut self,
        stats: &Statistics,
        tc: &'tc TextureCreator<WindowContext>,
    ) {
        let title_color = Color::RGB(
            STATS_TITLE_COLOR.0,
            STATS_TITLE_COLOR.1,
            STATS_TITLE_COLOR.2,
        );
        let text_color = Color::RGB(
            STATS_TEXT_COLOR.0,
            STATS_TEXT_COLOR.1,
            STATS_TEXT_COLOR.2,
        );
        let accent_color = Color::RGB(100, 255, 150);
        let warn_color = Color::RGB(255, 180, 60);

        let lines: Vec<(&str, String, Color)> = vec![
            (
                "SMART ROAD — SIMULATION STATISTICS",
                String::new(),
                title_color,
            ),
            ("─────────────────────────────────────", String::new(), text_color),
            (
                "Vehicles passed intersection",
                format!("{}", stats.max_vehicles),
                accent_color,
            ),
            (
                "Session duration",
                format!("{:.1}s", stats.session_duration()),
                text_color,
            ),
            ("─────────────────────────────────────", String::new(), text_color),
            (
                "Max velocity",
                format!("{:.1} px/s", stats.max_velocity),
                accent_color,
            ),
            (
                "Min velocity",
                format!("{:.1} px/s", stats.min_velocity_display()),
                text_color,
            ),
            (
                "Avg velocity",
                format!("{:.1} px/s", stats.avg_velocity()),
                text_color,
            ),
            ("─────────────────────────────────────", String::new(), text_color),
            (
                "Max intersection transit time",
                format!("{:.3}s", stats.max_time),
                accent_color,
            ),
            (
                "Min intersection transit time",
                format!("{:.3}s", stats.min_time_display()),
                text_color,
            ),
            ("─────────────────────────────────────", String::new(), text_color),
            (
                "Close calls",
                format!("{}", stats.close_calls),
                if stats.close_calls > 0 { warn_color } else { accent_color },
            ),
            ("─────────────────────────────────────", String::new(), text_color),
            (
                "Press ESC or ENTER to exit",
                String::new(),
                Color::RGB(150, 150, 180),
            ),
        ];

        let mut y = 60i32;
        let x = 80i32;

        for (label, value, color) in &lines {
            let display = if value.is_empty() {
                label.to_string()
            } else {
                format!("{:<38} {}", label, value)
            };

            let surface = if label.contains("SMART ROAD") {
                self.font_large
                    .render(&display)
                    .blended(*color)
                    .unwrap_or_else(|_| {
                        self.font_large.render(" ").blended(*color).unwrap()
                    })
            } else {
                self.font
                    .render(&display)
                    .blended(*color)
                    .unwrap_or_else(|_| self.font.render(" ").blended(*color).unwrap())
            };

            let texture = tc.create_texture_from_surface(&surface).unwrap();
            let sdl2::render::TextureQuery { width, height, .. } = texture.query();
            self.canvas
                .copy(
                    &texture,
                    None,
                    Some(Rect::new(x, y, width, height)),
                )
                .unwrap();

            y += height as i32 + 8;
        }
    }
}

fn rotate_point(x: f64, y: f64, angle: f64) -> (f64, f64) {
    let (sin, cos) = angle.sin_cos();
    (x * cos - y * sin, x * sin + y * cos)
}

fn fill_polygon(canvas: &mut Canvas<Window>, points: &[sdl2::rect::Point]) {
    if points.len() < 3 {
        return;
    }
    let min_y = points.iter().map(|p| p.y).min().unwrap_or(0);
    let max_y = points.iter().map(|p| p.y).max().unwrap_or(0);

    for y in min_y..=max_y {
        let mut intersections: Vec<i32> = Vec::new();
        let n = points.len();
        for i in 0..n {
            let p1 = points[i];
            let p2 = points[(i + 1) % n];
            if (p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y) {
                let x = p1.x
                    + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
                intersections.push(x);
            }
        }
        intersections.sort_unstable();
        let mut i = 0;
        while i + 1 < intersections.len() {
            let x0 = intersections[i];
            let x1 = intersections[i + 1];
            if x1 > x0 {
                canvas
                    .fill_rect(Rect::new(x0, y, (x1 - x0) as u32, 1))
                    .ok();
            }
            i += 2;
        }
    }
}

fn find_system_font() -> std::path::PathBuf {
    let candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/System/Library/Fonts/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "C:\\Windows\\Fonts\\arial.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    ];
    for path in &candidates {
        let p = std::path::PathBuf::from(path);
        if p.exists() {
            return p;
        }
    }
    panic!(
        "No system TTF font found. Please install dejavu-fonts or liberation-fonts.\n\
         Ubuntu: sudo apt-get install fonts-dejavu\n\
         Arch:   sudo pacman -S ttf-dejavu"
    );
}
