//! Hard-coded waypoint paths — RIGHT-HAND driving, 6-lane grid.
//!
//! Window 1100×1100.
//! Road strip: IX=370..730 (horiz), IY=370..730 (vert), width=360px.
//! 6 lanes of 60px each. Lane centres (left→right / top→bottom):
//!   col/row 0→400, 1→460, 2→520, 3→580, 4→640, 5→700
//!
//! Screen orientation:
//!   Top    = North arm   (cars spawn at top,    travel SOUTH ↓)
//!   Bottom = South arm   (cars spawn at bottom, travel NORTH ↑)
//!   Left   = West arm    (cars spawn at left,   travel EAST  →)
//!   Right  = East arm    (cars spawn at right,  travel WEST  ←)
//!
//! RIGHT-HAND TRAFFIC: each arm uses the RIGHT half of its road as INBOUND.
//!
//!  ┌──────────────────────────────────────────────────────────────────┐
//!  │  NORTH arm (top, moving ↓ SOUTH):                               │
//!  │    Inbound = right side of downward travel = HIGH x cols 3,4,5  │
//!  │    col3 x=580 = right-turn lane  (turns East →)                 │
//!  │    col4 x=640 = straight lane    (exits bottom)                 │
//!  │    col5 x=700 = left-turn lane   (turns West ←)                 │
//!  │                                                                  │
//!  │  SOUTH arm (bottom, moving ↑ NORTH):                           │
//!  │    Inbound = right side of upward travel = LOW x cols 2,1,0     │
//!  │    col2 x=520 = right-turn lane  (turns West ←)                 │
//!  │    col1 x=460 = straight lane    (exits top)                    │
//!  │    col0 x=400 = left-turn lane   (turns East →)                 │
//!  │                                                                  │
//!  │  WEST arm (left, moving → EAST):                               │
//!  │    Inbound = right side of eastward travel = HIGH y rows 3,4,5  │
//!  │    row3 y=580 = right-turn lane  (turns North ↑)                │
//!  │    row4 y=640 = straight lane    (exits right)                  │
//!  │    row5 y=700 = left-turn lane   (turns South ↓)                │
//!  │                                                                  │
//!  │  EAST arm (right, moving ← WEST):                              │
//!  │    Inbound = right side of westward travel = LOW y rows 2,1,0   │
//!  │    row2 y=520 = right-turn lane  (turns South ↓)                │
//!  │    row1 y=460 = straight lane    (exits left)                   │
//!  │    row0 y=400 = left-turn lane   (turns North ↑)                │
//!  └──────────────────────────────────────────────────────────────────┘
//!
//! Outbound (exit) lanes are the LEFT half of each arm:
//!   North outbound (going ↑):  cols 0,1,2  x=400,460,520
//!   South outbound (going ↓):  cols 3,4,5  x=580,640,700
//!   West  outbound (going ←):  rows 0,1,2  y=400,460,520
//!   East  outbound (going →):  rows 3,4,5  y=580,640,700
//!
//! Spawn points (off-screen by OFF=80):
//!   North spawn: y = IY - OFF = 290       (top edge - 80)
//!   South spawn: y = IY + ROAD + OFF = 810 (bottom edge + 80)
//!   West  spawn: x = IX - OFF = 290
//!   East  spawn: x = IX + ROAD + OFF = 810
//!
//! Exit points also go off-screen by 80px past the road edge.

use crate::vehicle::{Arm, Turn};

// ─── NORTH arm (spawn top y=290, travel South ↓) ────────────────────────────
// Right turn  → col3 x=580, pivot at outbound East row y=520, exit x=810
const PATH_N_R: &[(f64,f64)] = &[(580.0, 290.0), (580.0, 520.0), (810.0, 520.0)];
// Straight    → col4 x=640, straight south, exit y=810
const PATH_N_F: &[(f64,f64)] = &[(640.0, 290.0), (640.0, 810.0)];
// Left turn   → col5 x=700, pivot at outbound West row y=700, exit x=290
const PATH_N_L: &[(f64,f64)] = &[(700.0, 290.0), (700.0, 700.0), (290.0, 700.0)];

// ─── SOUTH arm (spawn bottom y=810, travel North ↑) ─────────────────────────
// Right turn  → col2 x=520, pivot at outbound West row y=580, exit x=290
const PATH_S_R: &[(f64,f64)] = &[(520.0, 810.0), (520.0, 580.0), (290.0, 580.0)];
// Straight    → col1 x=460, straight north, exit y=290
const PATH_S_F: &[(f64,f64)] = &[(460.0, 810.0), (460.0, 290.0)];
// Left turn   → col0 x=400, pivot at outbound East row y=400, exit x=810
const PATH_S_L: &[(f64,f64)] = &[(400.0, 810.0), (400.0, 400.0), (810.0, 400.0)];

// ─── WEST arm (spawn left x=290, travel East →) ─────────────────────────────
// Right turn  → row3 y=580, pivot at outbound North col x=580, exit y=290
const PATH_W_R: &[(f64,f64)] = &[(290.0, 580.0), (580.0, 580.0), (580.0, 290.0)];
// Straight    → row4 y=640, straight east, exit x=810
const PATH_W_F: &[(f64,f64)] = &[(290.0, 640.0), (810.0, 640.0)];
// Left turn   → row5 y=700, pivot at outbound South col x=700, exit y=810
const PATH_W_L: &[(f64,f64)] = &[(290.0, 700.0), (700.0, 700.0), (700.0, 810.0)];

// ─── EAST arm (spawn right x=810, travel West ←) ────────────────────────────
// Right turn  → row2 y=520, pivot at outbound South col x=520, exit y=810
const PATH_E_R: &[(f64,f64)] = &[(810.0, 520.0), (520.0, 520.0), (520.0, 810.0)];
// Straight    → row1 y=460, straight west, exit x=290
const PATH_E_F: &[(f64,f64)] = &[(810.0, 460.0), (290.0, 460.0)];
// Left turn   → row0 y=400, pivot at outbound North col x=400, exit y=290
const PATH_E_L: &[(f64,f64)] = &[(810.0, 400.0), (400.0, 400.0), (400.0, 290.0)];

// ─── Selector ────────────────────────────────────────────────────────────────
pub fn get_path(arm: Arm, turn: Turn) -> &'static [(f64,f64)] {
    match (arm, turn) {
        (Arm::North, Turn::Right)   => PATH_N_R,
        (Arm::North, Turn::Forward) => PATH_N_F,
        (Arm::North, Turn::Left)    => PATH_N_L,
        (Arm::South, Turn::Right)   => PATH_S_R,
        (Arm::South, Turn::Forward) => PATH_S_F,
        (Arm::South, Turn::Left)    => PATH_S_L,
        (Arm::West,  Turn::Right)   => PATH_W_R,
        (Arm::West,  Turn::Forward) => PATH_W_F,
        (Arm::West,  Turn::Left)    => PATH_W_L,
        (Arm::East,  Turn::Right)   => PATH_E_R,
        (Arm::East,  Turn::Forward) => PATH_E_F,
        (Arm::East,  Turn::Left)    => PATH_E_L,
    }
}
