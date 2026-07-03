use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{Canvas, TextureCreator};
use sdl2::ttf::Sdl2TtfContext;
use sdl2::video::{Window, WindowContext};
use std::f64::consts::PI;

use crate::config::*;
use crate::intersection::Intersection;
use crate::stats::Statistics;
use crate::vehicle::{Arm, Vehicle, VehicleState};

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

const HUD_W: u32 = 235;
const HUD_PAD: i32 = 10;
const HUD_LINE_H: i32 = 18;

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
        let font = ttf.load_font(&font_path, 14).expect("font load failed");
        let font_large = ttf.load_font(&font_path, 22).expect("font_large load failed");
        Renderer { canvas, font, font_large, _texture_creator: texture_creator }
    }

    pub fn draw_frame(&mut self, intersection: &Intersection, stats: &Statistics, random_mode: bool) {
        self.canvas.set_draw_color(Color::RGB(GRASS_COLOR.0, GRASS_COLOR.1, GRASS_COLOR.2));
        self.canvas.clear();
        self.draw_roads();
        self.draw_lane_markings();
        self.draw_intersection_box();
        for v in &intersection.vehicles {
            self.draw_vehicle(v);
        }
        self.draw_hud(stats, intersection, random_mode);
        self.canvas.present();
    }

    fn draw_roads(&mut self) {
        let ix = INTERSECTION_X as i32;
        let iy = INTERSECTION_Y as i32;
        let iw = ROAD_WIDTH as i32;
        self.canvas.set_draw_color(Color::RGB(ROAD_COLOR.0, ROAD_COLOR.1, ROAD_COLOR.2));
        self.canvas.fill_rect(Rect::new(ix, 0, iw as u32, WINDOW_H)).unwrap();
        self.canvas.fill_rect(Rect::new(0, iy, WINDOW_W, iw as u32)).unwrap();
    }

    fn draw_lane_markings(&mut self) {
        let ix = INTERSECTION_X as i32;
        let iy = INTERSECTION_Y as i32;
        let iw = ROAD_WIDTH as i32;
        let lw = LANE_WIDTH as i32;
        let dash = 20i32;
        let gap  = 20i32;

        // Dashed lane dividers (all 5 internal lines)
        self.canvas.set_draw_color(Color::RGB(LANE_LINE_COLOR.0, LANE_LINE_COLOR.1, LANE_LINE_COLOR.2));
        for lane in 1..LANE_COUNT {
            if lane == LANE_COUNT / 2 { continue; } // skip center — drawn solid below
            let lx = ix + lane as i32 * lw;
            self.draw_dashed_vertical(lx, 0, iy, dash, gap);
            self.draw_dashed_vertical(lx, iy + iw, WINDOW_H as i32, dash, gap);
        }
        for lane in 1..LANE_COUNT {
            if lane == LANE_COUNT / 2 { continue; }
            let ly = iy + lane as i32 * lw;
            self.draw_dashed_horizontal(ly, 0, ix, dash, gap);
            self.draw_dashed_horizontal(ly, ix + iw, WINDOW_W as i32, dash, gap);
        }

        // Solid white center divider
        let cx = ix + (LANE_COUNT / 2) as i32 * lw;
        let cy = iy + (LANE_COUNT / 2) as i32 * lw;
        self.canvas.set_draw_color(Color::RGB(CENTER_LINE_COLOR.0, CENTER_LINE_COLOR.1, CENTER_LINE_COLOR.2));
        self.canvas.fill_rect(Rect::new(cx - 2, 0, 4, iy as u32)).unwrap();
        self.canvas.fill_rect(Rect::new(cx - 2, (iy + iw), 4, (WINDOW_H as i32 - iy - iw) as u32)).unwrap();
        self.canvas.fill_rect(Rect::new(0, cy - 2, ix as u32, 4)).unwrap();
        self.canvas.fill_rect(Rect::new((ix + iw), cy - 2, (WINDOW_W as i32 - ix - iw) as u32, 4)).unwrap();
    }

    fn draw_dashed_vertical(&mut self, x: i32, y_start: i32, y_end: i32, dash: i32, gap: i32) {
        let mut y = y_start;
        while y < y_end {
            let end = (y + dash).min(y_end);
            self.canvas.fill_rect(Rect::new(x - 1, y, 2, (end - y) as u32)).unwrap();
            y += dash + gap;
        }
    }

    fn draw_dashed_horizontal(&mut self, y: i32, x_start: i32, x_end: i32, dash: i32, gap: i32) {
        let mut x = x_start;
        while x < x_end {
            let end = (x + dash).min(x_end);
            self.canvas.fill_rect(Rect::new(x, y - 1, (end - x) as u32, 2)).unwrap();
            x += dash + gap;
        }
    }

    fn draw_intersection_box(&mut self) {
        let ix = INTERSECTION_X as i32;
        let iy = INTERSECTION_Y as i32;
        let iw = ROAD_WIDTH as i32;
        self.canvas.set_draw_color(Color::RGB(60, 60, 60));
        self.canvas.fill_rect(Rect::new(ix, iy, iw as u32, iw as u32)).unwrap();
        self.canvas.set_draw_color(Color::RGB(100, 100, 100));
        self.canvas.draw_rect(Rect::new(ix, iy, iw as u32, iw as u32)).unwrap();
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
            rotate_point(-hw, -hh, angle), rotate_point(hw, -hh, angle),
            rotate_point(hw,  hh, angle), rotate_point(-hw,  hh, angle),
        ];
        let body_pts: Vec<sdl2::rect::Point> = corners.iter()
            .map(|(dx, dy)| sdl2::rect::Point::new((v.x + dx) as i32, (v.y + dy) as i32))
            .collect();
        self.canvas.set_draw_color(body_color);
        fill_polygon(&mut self.canvas, &body_pts);

        // Windscreen
        let ww = hw * 0.55; let wh = hh * 0.35; let foff = hh * 0.35;
        let fw: Vec<sdl2::rect::Point> = [
            rotate_point(-ww, -foff - wh, angle), rotate_point(ww, -foff - wh, angle),
            rotate_point(ww,  -foff,      angle), rotate_point(-ww, -foff,     angle),
        ].iter().map(|(dx, dy)| sdl2::rect::Point::new((v.x + dx) as i32, (v.y + dy) as i32)).collect();
        self.canvas.set_draw_color(window_color);
        fill_polygon(&mut self.canvas, &fw);

        // Headlights
        self.canvas.set_draw_color(Color::RGB(255, 255, 200));
        for hx in &[-hw * 0.55, hw * 0.55] {
            let (dx, dy) = rotate_point(*hx, -hh + 4.0, angle);
            self.canvas.fill_rect(Rect::new((v.x+dx-3.0) as i32, (v.y+dy-3.0) as i32, 6, 6)).unwrap();
        }
        // Taillights
        self.canvas.set_draw_color(Color::RGB(200, 30, 30));
        for tx in &[-hw * 0.55, hw * 0.55] {
            let (dx, dy) = rotate_point(*tx, hh - 4.0, angle);
            self.canvas.fill_rect(Rect::new((v.x+dx-3.0) as i32, (v.y+dy-3.0) as i32, 6, 6)).unwrap();
        }
    }

    fn draw_hud(&mut self, stats: &Statistics, intersection: &Intersection, random_mode: bool) {
        let tc = self._texture_creator;
        let title_col = Color::RGB(100, 200, 255);
        let head_col  = Color::RGB(180, 180, 220);
        let val_col   = Color::RGB(100, 255, 150);
        let warn_col  = Color::RGB(255, 180, 60);
        let dim_col   = Color::RGB(140, 140, 160);
        let white     = Color::RGB(220, 220, 255);
        let on_col    = Color::RGB(80, 255, 120);
        let off_col   = Color::RGB(200, 80, 80);

        let lines: Vec<(String, Color)> = vec![
            ("◈ SMART ROAD".into(),                                 title_col),
            ("─────────────────────".into(),                        dim_col),
            (format!("Passed      {:>6}", stats.max_vehicles),      val_col),
            (format!("On screen   {:>6}", intersection.vehicles.len()), white),
            (format!("Close calls {:>6}", stats.close_calls),
                if stats.close_calls > 0 { warn_col } else { val_col }),
            ("─────────────────────".into(),                        dim_col),
            (format!("Max spd  {:>7.1} px/s", stats.max_velocity),  val_col),
            (format!("Min spd  {:>7.1} px/s", stats.min_velocity_display()), white),
            (format!("Avg spd  {:>7.1} px/s", stats.avg_velocity()), white),
            ("─────────────────────".into(),                        dim_col),
            (format!("Max transit {:>5.3}s", stats.max_time),       val_col),
            (format!("Min transit {:>5.3}s", stats.min_time_display()), white),
            ("─────────────────────".into(),                        dim_col),
            ("CONTROLS".into(),                                     head_col),
            (format!("[R] Auto-spawn  {}", if random_mode { "ON " } else { "OFF" }),
                if random_mode { on_col } else { off_col }),
            ("[↑↓←→] Spawn car".into(),                            white),
            ("[ESC]  Stats & quit".into(),                          dim_col),
        ];

        let hud_h = lines.len() as i32 * HUD_LINE_H + HUD_PAD * 2;
        let hud_x = WINDOW_W as i32 - HUD_W as i32 - HUD_PAD;
        let hud_y = HUD_PAD;

        self.canvas.set_draw_color(Color::RGBA(10, 10, 25, 200));
        self.canvas.fill_rect(Rect::new(hud_x, hud_y, HUD_W, hud_h as u32)).unwrap();
        self.canvas.set_draw_color(Color::RGB(60, 80, 120));
        self.canvas.draw_rect(Rect::new(hud_x, hud_y, HUD_W, hud_h as u32)).unwrap();

        let mut ty = hud_y + HUD_PAD;
        for (text, color) in &lines {
            let surf = self.font.render(text).blended(*color)
                .unwrap_or_else(|_| self.font.render(" ").blended(*color).unwrap());
            let tex = tc.create_texture_from_surface(&surf).unwrap();
            let sdl2::render::TextureQuery { width, height, .. } = tex.query();
            self.canvas.copy(&tex, None, Some(Rect::new(hud_x + HUD_PAD, ty, width, height))).unwrap();
            ty += HUD_LINE_H;
        }
    }

    pub fn show_statistics(&mut self, stats: &Statistics) {
        let tc = self._texture_creator;
        let mut running = true;
        let sdl_context = unsafe { sdl2::init().unwrap() };
        let mut ep = sdl_context.event_pump()
            .unwrap_or_else(|_| panic!("Cannot get event pump"));
        loop {
            if !running { break; }
            for event in ep.poll_iter() {
                use sdl2::event::Event;
                match event {
                    Event::Quit { .. } => running = false,
                    Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::Escape), .. }
                    | Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::Return), .. }
                        => running = false,
                    _ => {}
                }
            }
            self.canvas.set_draw_color(Color::RGB(STATS_BG_COLOR.0, STATS_BG_COLOR.1, STATS_BG_COLOR.2));
            self.canvas.clear();
            self.draw_stats_panel(stats, tc);
            self.canvas.present();
            std::thread::sleep(std::time::Duration::from_millis(16));
        }
    }

    fn draw_stats_panel(&mut self, stats: &Statistics, tc: &'tc TextureCreator<WindowContext>) {
        let title_color  = Color::RGB(STATS_TITLE_COLOR.0, STATS_TITLE_COLOR.1, STATS_TITLE_COLOR.2);
        let text_color   = Color::RGB(STATS_TEXT_COLOR.0,  STATS_TEXT_COLOR.1,  STATS_TEXT_COLOR.2);
        let accent_color = Color::RGB(100, 255, 150);
        let warn_color   = Color::RGB(255, 180, 60);
        let div          = "─────────────────────────────────────".to_string();

        let lines: Vec<(String, Color)> = vec![
            ("SMART ROAD — SIMULATION STATISTICS".into(), title_color),
            (div.clone(), text_color),
            (format!("Vehicles passed intersection    {}", stats.max_vehicles), accent_color),
            (format!("Session duration               {:.1}s", stats.session_duration()), text_color),
            (div.clone(), text_color),
            (format!("Max velocity                   {:.1} px/s", stats.max_velocity), accent_color),
            (format!("Min velocity                   {:.1} px/s", stats.min_velocity_display()), text_color),
            (format!("Avg velocity                   {:.1} px/s", stats.avg_velocity()), text_color),
            (div.clone(), text_color),
            (format!("Max intersection transit       {:.3}s", stats.max_time), accent_color),
            (format!("Min intersection transit       {:.3}s", stats.min_time_display()), text_color),
            (div.clone(), text_color),
            (format!("Close calls                    {}", stats.close_calls),
                if stats.close_calls > 0 { warn_color } else { accent_color }),
            (div.clone(), text_color),
            ("Press ESC or ENTER to exit".into(), Color::RGB(150, 150, 180)),
        ];

        let mut y = 60i32;
        for (text, color) in &lines {
            let surf = if text.starts_with("SMART ROAD") {
                self.font_large.render(text).blended(*color)
                    .unwrap_or_else(|_| self.font_large.render(" ").blended(*color).unwrap())
            } else {
                self.font.render(text).blended(*color)
                    .unwrap_or_else(|_| self.font.render(" ").blended(*color).unwrap())
            };
            let tex = tc.create_texture_from_surface(&surf).unwrap();
            let sdl2::render::TextureQuery { width, height, .. } = tex.query();
            self.canvas.copy(&tex, None, Some(Rect::new(80, y, width, height))).unwrap();
            y += height as i32 + 8;
        }
    }
}

fn rotate_point(x: f64, y: f64, angle: f64) -> (f64, f64) {
    let (sin, cos) = angle.sin_cos();
    (x * cos - y * sin, x * sin + y * cos)
}

fn fill_polygon(canvas: &mut Canvas<Window>, points: &[sdl2::rect::Point]) {
    if points.len() < 3 { return; }
    let min_y = points.iter().map(|p| p.y).min().unwrap_or(0);
    let max_y = points.iter().map(|p| p.y).max().unwrap_or(0);
    for y in min_y..=max_y {
        let mut xs: Vec<i32> = Vec::new();
        let n = points.len();
        for i in 0..n {
            let p1 = points[i]; let p2 = points[(i + 1) % n];
            if (p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y) {
                xs.push(p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y));
            }
        }
        xs.sort_unstable();
        let mut i = 0;
        while i + 1 < xs.len() {
            if xs[i+1] > xs[i] {
                canvas.fill_rect(Rect::new(xs[i], y, (xs[i+1]-xs[i]) as u32, 1)).ok();
            }
            i += 2;
        }
    }
}

fn find_system_font() -> std::path::PathBuf {
    let candidates = [
        "C:\\Windows\\Fonts\\arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    ];
    for path in &candidates {
        let p = std::path::PathBuf::from(path);
        if p.exists() { return p; }
    }
    panic!("No system TTF font found.");
}
