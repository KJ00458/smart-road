use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{Canvas, TextureCreator};
use sdl2::ttf::Sdl2TtfContext;
use sdl2::video::{Window, WindowContext};

use crate::config::*;
use crate::intersection::World;
use crate::stats::Statistics;
use crate::vehicle::Vehicle;

const COLORS: [(u8,u8,u8); 8] = [
    (210,  50,  50), ( 50, 140, 220), ( 50, 195,  90),
    (240, 175,  30), (175,  70, 215), (235, 115,  40),
    ( 40, 215, 215), (215, 215, 215),
];

const HUD_W:  u32 = 240;
const HUD_X:  i32 = (900 - 240 - 8) as i32;
const HUD_Y:  i32 = 8;
const LINE_H: i32 = 19;
const PAD:    i32 = 9;

pub struct Renderer<'f,'tc> {
    canvas: Canvas<Window>,
    font:   sdl2::ttf::Font<'f,'tc>,
    fontlg: sdl2::ttf::Font<'f,'tc>,
    tc: &'tc TextureCreator<WindowContext>,
}

impl<'f,'tc> Renderer<'f,'tc> {
    pub fn new(
        canvas: Canvas<Window>,
        tc: &'tc TextureCreator<WindowContext>,
        ttf: &'f Sdl2TtfContext,
    ) -> Self {
        let fp = font_path();
        let font   = ttf.load_font(&fp, 14).expect("font");
        let fontlg = ttf.load_font(&fp, 22).expect("fontlg");
        Renderer { canvas, font, fontlg, tc }
    }

    pub fn draw(&mut self, world: &World, stats: &Statistics, rng_mode: bool) {
        self.canvas.set_draw_color(rgb(COL_GRASS));
        self.canvas.clear();
        self.draw_road();
        self.draw_lane_marks();
        for v in &world.vehicles { self.draw_vehicle(v); }
        self.draw_hud(stats, world.vehicles.len(), rng_mode);
        self.canvas.present();
    }

    // ── Road ─────────────────────────────────────────────────────────────────

    fn draw_road(&mut self) {
        let ix = IX as i32; let iy = IY as i32; let rw = ROAD_W as i32;
        self.canvas.set_draw_color(rgb(COL_ROAD));
        self.canvas.fill_rect(Rect::new(ix, 0,       rw as u32, WINDOW_H)).ok();
        self.canvas.fill_rect(Rect::new(0,  iy, WINDOW_W,       rw as u32)).ok();
        // Intersection box slightly different shade
        self.canvas.set_draw_color(rgb(COL_INTER));
        self.canvas.fill_rect(Rect::new(ix, iy, rw as u32, rw as u32)).ok();
    }

    fn draw_lane_marks(&mut self) {
        let ix = IX as i32; let iy = IY as i32;
        let rw = ROAD_W as i32; let t = TILE as i32;
        let mid = (LANE_COUNT / 2) as i32;

        // Dashed yellow lane dividers
        self.canvas.set_draw_color(rgb(COL_YELLOW));
        for i in 1..LANE_COUNT as i32 {
            if i == mid { continue; }
            // Vertical strip — above intersection
            self.dashes_v(ix + i*t, 0,       iy,       20, 18);
            // Vertical strip — below intersection
            self.dashes_v(ix + i*t, iy + rw, WINDOW_H as i32, 20, 18);
            // Horizontal strip — left of intersection
            self.dashes_h(iy + i*t, 0,       ix,       20, 18);
            // Horizontal strip — right of intersection
            self.dashes_h(iy + i*t, ix + rw, WINDOW_W as i32, 20, 18);
        }

        // Solid white centre divider
        self.canvas.set_draw_color(rgb(COL_WHITE));
        let cx = ix + mid * t;
        let cy = iy + mid * t;
        self.canvas.fill_rect(Rect::new(cx-2, 0,       4, iy as u32)).ok();
        self.canvas.fill_rect(Rect::new(cx-2, iy+rw,   4, (WINDOW_H as i32 - iy - rw) as u32)).ok();
        self.canvas.fill_rect(Rect::new(0,       cy-2, ix as u32, 4)).ok();
        self.canvas.fill_rect(Rect::new(ix+rw, cy-2, (WINDOW_W as i32 - ix - rw) as u32, 4)).ok();
    }

    fn dashes_v(&mut self, x: i32, y0: i32, y1: i32, dash: i32, gap: i32) {
        let mut y = y0;
        while y < y1 {
            let e = (y + dash).min(y1);
            self.canvas.fill_rect(Rect::new(x-1, y, 2, (e-y) as u32)).ok();
            y += dash + gap;
        }
    }

    fn dashes_h(&mut self, y: i32, x0: i32, x1: i32, dash: i32, gap: i32) {
        let mut x = x0;
        while x < x1 {
            let e = (x + dash).min(x1);
            self.canvas.fill_rect(Rect::new(x, y-1, (e-x) as u32, 2)).ok();
            x += dash + gap;
        }
    }

    // ── Vehicle ───────────────────────────────────────────────────────────────

    fn draw_vehicle(&mut self, v: &Vehicle) {
        let (r,g,b) = COLORS[v.color_idx % 8];
        let ang = v.angle(); // radians, 0=East

        let hw = V_W / 2.0; let hh = V_H / 2.0;
        // Rotate corners
        let corners = [
            rot(-hw, -hh, ang), rot( hw, -hh, ang),
            rot( hw,  hh, ang), rot(-hw,  hh, ang),
        ];
        let pts: Vec<sdl2::rect::Point> = corners.iter()
            .map(|(dx,dy)| pt(v.x + dx, v.y + dy))
            .collect();
        self.canvas.set_draw_color(Color::RGB(r,g,b));
        fill_poly(&mut self.canvas, &pts);

        // Dark windscreen
        let wr = 0.55; let fwd_off = -hh * 0.4; let ws = hh * 0.3;
        let wp: Vec<sdl2::rect::Point> = [
            rot(-hw*wr, fwd_off - ws, ang), rot(hw*wr, fwd_off - ws, ang),
            rot( hw*wr, fwd_off,      ang), rot(-hw*wr, fwd_off,     ang),
        ].iter().map(|(dx,dy)| pt(v.x+dx, v.y+dy)).collect();
        self.canvas.set_draw_color(Color::RGB(
            (r as u16 * 5/10) as u8,
            (g as u16 * 5/10) as u8,
            (b as u16 * 5/10) as u8,
        ));
        fill_poly(&mut self.canvas, &wp);

        // Headlights
        self.canvas.set_draw_color(Color::RGB(255, 255, 180));
        for sx in &[-hw*0.5, hw*0.5] {
            let (dx,dy) = rot(*sx, -hh+3.0, ang);
            self.canvas.fill_rect(Rect::new((v.x+dx-3.0) as i32, (v.y+dy-3.0) as i32, 6,6)).ok();
        }
        // Taillights
        self.canvas.set_draw_color(Color::RGB(190, 20, 20));
        for sx in &[-hw*0.5, hw*0.5] {
            let (dx,dy) = rot(*sx, hh-3.0, ang);
            self.canvas.fill_rect(Rect::new((v.x+dx-3.0) as i32, (v.y+dy-3.0) as i32, 6,6)).ok();
        }
    }

    // ── HUD ───────────────────────────────────────────────────────────────────

    fn draw_hud(&mut self, s: &Statistics, on_screen: usize, rng: bool) {
        let div = "─────────────────────";
        let lines: Vec<(String, Color)> = vec![
            ("◈ SMART ROAD".into(),                               rgb2(COL_HUD_TITLE)),
            (div.into(),                                          rgb2(COL_HUD_DIM)),
            (format!("Passed      {:>6}", s.total_passed),       rgb2(COL_HUD_VAL)),
            (format!("On screen   {:>6}", on_screen),            Color::WHITE),
            (format!("Close calls {:>6}", s.close_calls),
                if s.close_calls > 0 { rgb2(COL_HUD_WARN) } else { rgb2(COL_HUD_VAL) }),
            (div.into(),                                          rgb2(COL_HUD_DIM)),
            (format!("Max spd  {:>7.1} px/s", s.max_spd),       rgb2(COL_HUD_VAL)),
            (format!("Min spd  {:>7.1} px/s", s.min_spd_disp()),Color::WHITE),
            (format!("Avg spd  {:>7.1} px/s", s.avg_spd()),     Color::WHITE),
            (div.into(),                                          rgb2(COL_HUD_DIM)),
            (format!("Max transit {:>5.3}s",  s.max_time),      rgb2(COL_HUD_VAL)),
            (format!("Min transit {:>5.3}s",  s.min_time_disp()),Color::WHITE),
            (div.into(),                                          rgb2(COL_HUD_DIM)),
            ("CONTROLS".into(),                                  Color::RGB(180,180,220)),
            (format!("[R] Auto  {}", if rng {"ON "} else {"OFF"}),
                if rng { rgb2(COL_HUD_ON) } else { rgb2(COL_HUD_OFF) }),
            ("[↑↓←→] Spawn".into(),                              Color::WHITE),
            ("[ESC]  Quit+stats".into(),                         rgb2(COL_HUD_DIM)),
        ];

        let hh = lines.len() as i32 * LINE_H + PAD * 2;
        self.canvas.set_draw_color(Color::RGBA(
            COL_HUD_BG.0, COL_HUD_BG.1, COL_HUD_BG.2, COL_HUD_BG.3));
        self.canvas.fill_rect(Rect::new(HUD_X, HUD_Y, HUD_W, hh as u32)).ok();
        self.canvas.set_draw_color(Color::RGB(55, 80, 120));
        self.canvas.draw_rect(Rect::new(HUD_X, HUD_Y, HUD_W, hh as u32)).ok();

        let mut ty = HUD_Y + PAD;
        for (txt, col) in &lines {
            self.blit_text(txt, *col, HUD_X + PAD, ty);
            ty += LINE_H;
        }
    }

    fn blit_text(&mut self, text: &str, col: Color, x: i32, y: i32) {
        let surf = self.font.render(text).blended(col)
            .unwrap_or_else(|_| self.font.render(" ").blended(col).unwrap());
        let tex  = self.tc.create_texture_from_surface(&surf).unwrap();
        let sdl2::render::TextureQuery { width, height, .. } = tex.query();
        self.canvas.copy(&tex, None, Some(Rect::new(x, y, width, height))).ok();
    }

    // ── Stats screen ─────────────────────────────────────────────────────────

    pub fn show_stats(&mut self, s: &Statistics) {
        loop {
            // We can't re-init SDL, just render the panel and wait for a key
            self.canvas.set_draw_color(Color::RGB(10, 10, 25));
            self.canvas.clear();
            self.stats_panel(s);
            self.canvas.present();
            std::thread::sleep(std::time::Duration::from_millis(16));
            // We broke out of the main loop already, so just show for ~3 s then quit
            static mut FRAMES: u32 = 0;
            unsafe {
                FRAMES += 1;
                if FRAMES > 180 { break; }
            }
        }
    }

    fn stats_panel(&mut self, s: &Statistics) {
        let div = "────────────────────────────────────────";
        let lines: Vec<(String, Color)> = vec![
            ("SMART ROAD — SESSION STATISTICS".into(), rgb2(COL_HUD_TITLE)),
            (div.into(),                               Color::WHITE),
            (format!("Vehicles passed     {}",  s.total_passed),        rgb2(COL_HUD_VAL)),
            (format!("Session duration    {:.1}s", s.session_secs()),   Color::WHITE),
            (div.into(),                               Color::WHITE),
            (format!("Max velocity        {:.1} px/s", s.max_spd),      rgb2(COL_HUD_VAL)),
            (format!("Min velocity        {:.1} px/s", s.min_spd_disp()),Color::WHITE),
            (format!("Avg velocity        {:.1} px/s", s.avg_spd()),    Color::WHITE),
            (div.into(),                               Color::WHITE),
            (format!("Max transit         {:.3}s", s.max_time),         rgb2(COL_HUD_VAL)),
            (format!("Min transit         {:.3}s", s.min_time_disp()),  Color::WHITE),
            (div.into(),                               Color::WHITE),
            (format!("Close calls         {}",  s.close_calls),
                if s.close_calls > 0 { rgb2(COL_HUD_WARN) } else { rgb2(COL_HUD_VAL) }),
            (div.into(),                               Color::WHITE),
            ("(window closes automatically)".into(),   Color::RGB(120,120,140)),
        ];
        let mut y = 70i32;
        for (txt, col) in &lines {
            if txt.starts_with("SMART ROAD") {
                let surf = self.fontlg.render(txt).blended(*col)
                    .unwrap_or_else(|_| self.fontlg.render(" ").blended(*col).unwrap());
                let tex = self.tc.create_texture_from_surface(&surf).unwrap();
                let sdl2::render::TextureQuery { width, height, .. } = tex.query();
                self.canvas.copy(&tex, None, Some(Rect::new(70, y, width, height))).ok();
                y += height as i32 + 10;
            } else {
                self.blit_text(txt, *col, 70, y);
                y += LINE_H + 4;
            }
        }
    }
}

// ── Utilities ─────────────────────────────────────────────────────────────────

fn rot(x: f64, y: f64, a: f64) -> (f64,f64) {
    let (s,c) = a.sin_cos();
    (x*c - y*s, x*s + y*c)
}

fn pt(x: f64, y: f64) -> sdl2::rect::Point {
    sdl2::rect::Point::new(x as i32, y as i32)
}

fn rgb(c: (u8,u8,u8)) -> Color { Color::RGB(c.0,c.1,c.2) }
fn rgb2(c: (u8,u8,u8)) -> Color { Color::RGB(c.0,c.1,c.2) }

fn fill_poly(canvas: &mut Canvas<Window>, pts: &[sdl2::rect::Point]) {
    if pts.len() < 3 { return; }
    let y0 = pts.iter().map(|p| p.y).min().unwrap();
    let y1 = pts.iter().map(|p| p.y).max().unwrap();
    let n  = pts.len();
    for y in y0..=y1 {
        let mut xs: Vec<i32> = Vec::new();
        for i in 0..n {
            let a = pts[i]; let b = pts[(i+1)%n];
            if (a.y <= y && b.y > y) || (b.y <= y && a.y > y) {
                xs.push(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
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

fn font_path() -> std::path::PathBuf {
    for p in &[
        "C:\\Windows\\Fonts\\arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
    ] {
        let pb = std::path::PathBuf::from(p);
        if pb.exists() { return pb; }
    }
    panic!("No TTF font found");
}
