use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{Canvas, TextureCreator};
use sdl2::ttf::Sdl2TtfContext;
use sdl2::video::{Window, WindowContext};

use crate::config::*;
use crate::intersection::World;
use crate::stats::Statistics;
use crate::vehicle::{Vehicle, Phase};

const COLORS: [(u8,u8,u8);8] = [
    (210,55,55),(55,145,220),(55,200,90),(240,175,30),
    (175,70,215),(235,115,40),(40,215,215),(215,215,215),
];
const HUD_W:  u32 = 270;
const HUD_X:  i32 = (WINDOW_W - HUD_W - 10) as i32;
const HUD_Y:  i32 = 10;
const LINE_H: i32 = 21;
const PAD:    i32 = 10;

const SENSOR_HALF_W: f64 = HB_HALF_W;
const SENSOR_LEN:    f64 = HB_BIG;

pub struct Renderer<'f,'tc> {
    canvas: Canvas<Window>,
    font:   sdl2::ttf::Font<'f,'tc>,
    fontlg: sdl2::ttf::Font<'f,'tc>,
    tc:     &'tc TextureCreator<WindowContext>,
}

impl<'f,'tc> Renderer<'f,'tc> {
    pub fn new(canvas: Canvas<Window>, tc: &'tc TextureCreator<WindowContext>,
               ttf: &'f Sdl2TtfContext) -> Self {
        let fp = font_path();
        Renderer {
            canvas,
            font:   ttf.load_font(&fp, 15).expect("font"),
            fontlg: ttf.load_font(&fp, 24).expect("fontlg"),
            tc,
        }
    }

    pub fn draw(&mut self, world: &World, stats: &Statistics,
                rng: bool, manual: bool, sel: SelArm) {
        self.canvas.set_draw_color(col(C_GRASS));
        self.canvas.clear();
        self.road();
        self.lane_marks();
        for v in &world.vehicles { self.sensor_beam(v); }
        for v in &world.vehicles { self.vehicle(v); }
        self.hud(stats, world.vehicles.len(), rng, manual, sel);
        self.canvas.present();
    }

    fn road(&mut self) {
        let (ix,iy,rw) = (IX as i32, IY as i32, ROAD as i32);
        self.canvas.set_draw_color(col(C_ROAD));
        self.canvas.fill_rect(Rect::new(ix, 0, rw as u32, WINDOW_H)).ok();
        self.canvas.fill_rect(Rect::new(0, iy, WINDOW_W, rw as u32)).ok();
        self.canvas.set_draw_color(col(C_INTER));
        self.canvas.fill_rect(Rect::new(ix, iy, rw as u32, rw as u32)).ok();
    }

    fn lane_marks(&mut self) {
        let (ix,iy) = (IX as i32, IY as i32);
        let (rw,t)  = (ROAD as i32, TILE as i32);
        let mid     = (LANES/2) as i32;
        self.canvas.set_draw_color(col(C_YELLOW));
        for i in 1..LANES as i32 {
            if i == mid { continue; }
            self.dv(ix+i*t, 0,     iy,    22, 18);
            self.dv(ix+i*t, iy+rw, WINDOW_H as i32, 22, 18);
            self.dh(iy+i*t, 0,     ix,    22, 18);
            self.dh(iy+i*t, ix+rw, WINDOW_W as i32, 22, 18);
        }
        self.canvas.set_draw_color(col(C_WHITE));
        let (cx,cy) = (ix+mid*t, iy+mid*t);
        self.canvas.fill_rect(Rect::new(cx-2, 0,    4, iy as u32)).ok();
        self.canvas.fill_rect(Rect::new(cx-2, iy+rw,4, (WINDOW_H as i32-iy-rw) as u32)).ok();
        self.canvas.fill_rect(Rect::new(0,    cy-2, ix as u32, 4)).ok();
        self.canvas.fill_rect(Rect::new(ix+rw,cy-2, (WINDOW_W as i32-ix-rw) as u32, 4)).ok();
    }

    fn dv(&mut self, x:i32, y0:i32, y1:i32, d:i32, g:i32) {
        let mut y=y0; while y<y1 { let e=(y+d).min(y1);
            self.canvas.fill_rect(Rect::new(x-1,y,2,(e-y) as u32)).ok(); y+=d+g; } }
    fn dh(&mut self, y:i32, x0:i32, x1:i32, d:i32, g:i32) {
        let mut x=x0; while x<x1 { let e=(x+d).min(x1);
            self.canvas.fill_rect(Rect::new(x,y-1,(e-x) as u32,2)).ok(); x+=d+g; } }

    fn sensor_beam(&mut self, v: &Vehicle) {
        if v.phase == Phase::Exiting { return; }
        let a = v.angle();
        let (fx,fy) = (a.cos(), a.sin());
        let (px,py) = (-fy, fx);
        let hw  = SENSOR_HALF_W;
        let len = SENSOR_LEN;
        let front_x = v.x + fx*(VH/2.0);
        let front_y = v.y + fy*(VH/2.0);
        let tl = (front_x - px*hw, front_y - py*hw);
        let tr = (front_x + px*hw, front_y + py*hw);
        let br = (front_x + fx*len + px*hw, front_y + fy*len + py*hw);
        let bl = (front_x + fx*len - px*hw, front_y + fy*len - py*hw);
        let beam_col = if v.spd_px >= SPD_NORMAL {
            Color::RGBA(  0, 200, 255, 180)
        } else if v.spd_px >= SPD_SLOW {
            Color::RGBA(255, 195,   0, 180)
        } else {
            Color::RGBA(255,  50,  50, 180)
        };
        self.canvas.set_draw_color(beam_col);
        dot_line(&mut self.canvas, tl, tr, 5, 4);
        dot_line(&mut self.canvas, bl, br, 5, 4);
        dot_line(&mut self.canvas, tl, bl, 5, 4);
        dot_line(&mut self.canvas, tr, br, 5, 4);
    }

    fn vehicle(&mut self, v: &Vehicle) {
        let base = if v.crashed { (180u8,20u8,20u8) } else { COLORS[v.color%8] };
        let (r,g,b) = base;
        let a  = v.angle();
        let hw = VW/2.0; let hh = VH/2.0;
        let corners = [rot(-hw,-hh,a),rot(hw,-hh,a),rot(hw,hh,a),rot(-hw,hh,a)];
        let pts: Vec<sdl2::rect::Point> = corners.iter()
            .map(|(dx,dy)| spt(v.x+dx,v.y+dy)).collect();
        self.canvas.set_draw_color(Color::RGB(r,g,b));
        fill_poly(&mut self.canvas, &pts);
        let wr=0.55; let fo=-hh*0.4; let ws=hh*0.3;
        let wp: Vec<sdl2::rect::Point> = [
            rot(-hw*wr,fo-ws,a),rot(hw*wr,fo-ws,a),rot(hw*wr,fo,a),rot(-hw*wr,fo,a),
        ].iter().map(|(dx,dy)| spt(v.x+dx,v.y+dy)).collect();
        self.canvas.set_draw_color(Color::RGB(
            (r as u16*5/10) as u8,(g as u16*5/10) as u8,(b as u16*5/10) as u8));
        fill_poly(&mut self.canvas, &wp);
        self.canvas.set_draw_color(Color::RGB(255,255,180));
        for sx in &[-hw*0.5, hw*0.5] {
            let (dx,dy)=rot(*sx,-hh+4.0,a);
            self.canvas.fill_rect(Rect::new((v.x+dx-3.0) as i32,(v.y+dy-3.0) as i32,6,6)).ok();
        }
        let tl_col = if v.spd_px < SPD_NORMAL { Color::RGB(255,20,20) } else { Color::RGB(110,10,10) };
        self.canvas.set_draw_color(tl_col);
        for sx in &[-hw*0.5, hw*0.5] {
            let (dx,dy)=rot(*sx,hh-4.0,a);
            self.canvas.fill_rect(Rect::new((v.x+dx-3.0) as i32,(v.y+dy-3.0) as i32,6,6)).ok();
        }
    }

    fn hud(&mut self, s: &Statistics, on: usize,
           rng: bool, manual: bool, sel: SelArm) {
        let div = "───────────────────────────";

        let mode_str = if manual { "MANUAL" } else if rng { "AUTO" } else { "IDLE" };
        let mode_col = if manual { Color::RGB(255,200,0) } else if rng { c(C_HUD_ON) } else { c(C_HUD_DIM) };

        let arm_str = match sel {
            SelArm::North => "↑ NORTH selected",
            SelArm::South => "↓ SOUTH selected",
            SelArm::East  => "→ EAST  selected",
            SelArm::West  => "← WEST  selected",
            SelArm::None  => "  (no arm chosen)",
        };
        let arm_col = if sel == SelArm::None { c(C_HUD_DIM) } else { Color::RGB(255,230,80) };

        let mut lines: Vec<(String,Color)> = vec![
            ("◈  SMART ROAD".into(),                            c(C_HUD_TITLE)),
            (div.into(),                                        c(C_HUD_DIM)),
            (format!("Passed      {:>6}",  s.total_passed),    c(C_HUD_VAL)),
            (format!("On screen   {:>6}",  on),                Color::WHITE),
            (format!("Crashes     {:>6}",  s.crashes),
                if s.crashes>0 {c(C_HUD_CRASH)} else {c(C_HUD_VAL)}),
            (format!("Close calls {:>6}",  s.close_calls/60),
                if s.close_calls>60 {c(C_HUD_WARN)} else {c(C_HUD_VAL)}),
            (div.into(),                                        c(C_HUD_DIM)),
            (format!("Max spd {:>7.1} px/s", s.max_spd),      c(C_HUD_VAL)),
            (format!("Min spd {:>7.1} px/s", s.min_spd_d()),  Color::WHITE),
            (format!("Avg spd {:>7.1} px/s", s.avg_spd()),    Color::WHITE),
            (div.into(),                                        c(C_HUD_DIM)),
            (format!("Max transit {:>5.2}s", s.max_time),     c(C_HUD_VAL)),
            (format!("Min transit {:>5.2}s", s.min_time_d()), Color::WHITE),
            (div.into(),                                        c(C_HUD_DIM)),
            (format!("Mode: {}", mode_str),                    mode_col),
        ];

        if manual {
            lines.push((arm_str.into(), arm_col));
        }

        lines.push((div.into(), c(C_HUD_DIM)));
        lines.push(("  CONTROLS".into(), Color::RGB(170,170,215)));
        lines.push(("  [R]  Auto mode".into(),     c(C_HUD_DIM)));
        lines.push(("  [M]  Manual mode".into(),   c(C_HUD_DIM)));
        lines.push(("  [ESC] Stats & quit".into(), c(C_HUD_DIM)));
        lines.push((div.into(), c(C_HUD_DIM)));

        if manual {
            lines.push(("  MANUAL SPAWN".into(),               Color::RGB(255,200,0)));
            lines.push(("  Step 1: Arrow key".into(),          Color::WHITE));
            lines.push(("    ↑↓←→ = pick arm (N/S/W/E)".into(), Color::WHITE));
            lines.push(("  Step 2: Number key".into(),         Color::WHITE));
            lines.push(("    [1] = Turn right".into(),         Color::WHITE));
            lines.push(("    [2] = Straight".into(),           Color::WHITE));
            lines.push(("    [3] = Turn left".into(),          Color::WHITE));
        } else {
            lines.push(("  AUTO SPAWN".into(),                  c(C_HUD_ON)));
            lines.push(("  Arrow keys spawn a random".into(),   Color::WHITE));
            lines.push(("  car from that direction.".into(),    Color::WHITE));
            lines.push(("".into(),                              Color::WHITE));
            lines.push(("  Switch to [M]anual to".into(),       c(C_HUD_DIM)));
            lines.push(("  choose the turn type.".into(),       c(C_HUD_DIM)));
        }

        let box_h = lines.len() as i32 * LINE_H + PAD*2;
        self.canvas.set_draw_color(Color::RGBA(C_HUD_BG.0,C_HUD_BG.1,C_HUD_BG.2,C_HUD_BG.3));
        self.canvas.fill_rect(Rect::new(HUD_X,HUD_Y,HUD_W,box_h as u32)).ok();
        self.canvas.set_draw_color(Color::RGB(40,65,105));
        self.canvas.draw_rect(Rect::new(HUD_X,HUD_Y,HUD_W,box_h as u32)).ok();
        let mut ty = HUD_Y + PAD;
        for (txt,color) in &lines { self.blit(txt,*color,HUD_X+PAD,ty); ty+=LINE_H; }
    }

    fn blit(&mut self, t:&str, col:Color, x:i32, y:i32) {
        let s = self.font.render(t).blended(col)
            .unwrap_or_else(|_| self.font.render(" ").blended(col).unwrap());
        let tx = self.tc.create_texture_from_surface(&s).unwrap();
        let sdl2::render::TextureQuery{width,height,..} = tx.query();
        self.canvas.copy(&tx,None,Some(Rect::new(x,y,width,height))).ok();
    }

    pub fn show_stats(&mut self, s: &Statistics) {
        let div = "────────────────────────────────────────────";
        let lines: Vec<(String,Color)> = vec![
            ("SMART ROAD — STATISTICS".into(),                c(C_HUD_TITLE)),
            (div.into(),                                       Color::WHITE),
            (format!("Vehicles passed   {}",  s.total_passed), c(C_HUD_VAL)),
            (format!("Session           {:.1}s",s.session_secs()), Color::WHITE),
            (div.into(),                                       Color::WHITE),
            (format!("Crashes           {}",  s.crashes),
                if s.crashes>0{c(C_HUD_CRASH)}else{c(C_HUD_VAL)}),
            (format!("Close calls       {}",  s.close_calls/60), c(C_HUD_WARN)),
            (div.into(),                                       Color::WHITE),
            (format!("Max velocity      {:.1} px/s",s.max_spd),    c(C_HUD_VAL)),
            (format!("Min velocity      {:.1} px/s",s.min_spd_d()),Color::WHITE),
            (format!("Avg velocity      {:.1} px/s",s.avg_spd()),  Color::WHITE),
            (div.into(),                                       Color::WHITE),
            (format!("Max transit       {:.2}s",s.max_time),   c(C_HUD_VAL)),
            (format!("Min transit       {:.2}s",s.min_time_d()),Color::WHITE),
            (div.into(),                                       Color::WHITE),
            ("(closes in 3s)".into(),                         c(C_HUD_DIM)),
        ];
        for _frame in 0..180u32 {
            self.canvas.set_draw_color(Color::RGB(8,8,18));
            self.canvas.clear();
            let mut y = 80i32;
            for (txt,col) in &lines {
                if txt.starts_with("SMART") {
                    let s2 = self.fontlg.render(txt).blended(*col)
                        .unwrap_or_else(|_| self.fontlg.render(" ").blended(*col).unwrap());
                    let tx = self.tc.create_texture_from_surface(&s2).unwrap();
                    let sdl2::render::TextureQuery{width,height,..} = tx.query();
                    self.canvas.copy(&tx,None,Some(Rect::new(80,y,width,height))).ok();
                    y += height as i32 + 12;
                } else {
                    self.blit(txt,*col,80,y); y += LINE_H+4;
                }
            }
            self.canvas.present();
            std::thread::sleep(std::time::Duration::from_millis(16));
        }
    }
}

fn dot_line(canvas:&mut Canvas<Window>,(x0,y0):(f64,f64),(x1,y1):(f64,f64),dot_len:i32,gap_len:i32){
    let dx=x1-x0; let dy=y1-y0;
    let total=(dx*dx+dy*dy).sqrt();
    if total<1.0{return;}
    let ux=dx/total; let uy=dy/total;
    let step=(dot_len+gap_len) as f64;
    let mut t=0.0f64;
    while t<total {
        let t_end=(t+dot_len as f64).min(total);
        let ax=(x0+ux*t) as i32; let ay=(y0+uy*t) as i32;
        let bx=(x0+ux*t_end) as i32; let by=(y0+uy*t_end) as i32;
        canvas.draw_line(sdl2::rect::Point::new(ax,ay),sdl2::rect::Point::new(bx,by)).ok();
        t+=step;
    }
}

fn rot(x:f64,y:f64,a:f64)->(f64,f64){let(s,cc)=a.sin_cos();(x*cc-y*s,x*s+y*cc)}
fn spt(x:f64,y:f64)->sdl2::rect::Point{sdl2::rect::Point::new(x as i32,y as i32)}
fn col(c:(u8,u8,u8))->Color{Color::RGB(c.0,c.1,c.2)}
fn c(c:(u8,u8,u8))->Color{Color::RGB(c.0,c.1,c.2)}

fn fill_poly(canvas:&mut Canvas<Window>,pts:&[sdl2::rect::Point]){
    if pts.len()<3{return;}
    let y0=pts.iter().map(|p|p.y).min().unwrap();
    let y1=pts.iter().map(|p|p.y).max().unwrap();
    let n=pts.len();
    for y in y0..=y1 {
        let mut xs:Vec<i32>=Vec::new();
        for i in 0..n {
            let a=pts[i];let b=pts[(i+1)%n];
            if(a.y<=y&&b.y>y)||(b.y<=y&&a.y>y){
                xs.push(a.x+(y-a.y)*(b.x-a.x)/(b.y-a.y));
            }
        }
        xs.sort_unstable();
        let mut i=0;
        while i+1<xs.len(){
            if xs[i+1]>xs[i]{canvas.fill_rect(Rect::new(xs[i],y,(xs[i+1]-xs[i]) as u32,1)).ok();}
            i+=2;
        }
    }
}

fn font_path()->std::path::PathBuf{
    for p in &[
        "C:\\Windows\\Fonts\\arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
    ]{
        let pb=std::path::PathBuf::from(p);
        if pb.exists(){return pb;}
    }
    panic!("No TTF font found");
}
