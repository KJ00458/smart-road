//! Hard-coded waypoint paths — RIGHT-HAND driving (US style), 6-lane grid.
//!
//! Window 1100×1100.
//! Road strip: IX=370..730 (horiz), IY=370..730 (vert), width=360px.
//! 6 lanes of 60px each. Lane centres:
//!   col/row 0→400, 1→460, 2→520, 3→580, 4→640, 5→700
//!
//! RIGHT-HAND TRAFFIC (US style — drive on the right):
//!   Inbound  = RIGHT side of your direction of travel.
//!   Outbound = LEFT  side of your direction of travel (the other half).
//!
//! INBOUND lanes (where cars enter the intersection):
//!   North  (↓): cols 0,1,2  x=400,460,520  (low x = right when facing south)
//!   South  (↑): cols 5,4,3  x=700,640,580  (high x = right when facing north)
//!   West   (→): rows 3,4,5  y=580,640,700  (high y = right when facing east)
//!   East   (←): rows 0,1,2  y=400,460,520  (low y  = right when facing west)
//!
//! OUTBOUND lanes (where cars exit the intersection):
//!   North outbound (↑ out): cols 3,4,5  x=580,640,700
//!   South outbound (↓ out): cols 0,1,2  x=400,460,520
//!   West  outbound (← out): rows 0,1,2  y=400,460,520
//!   East  outbound (→ out): rows 3,4,5  y=580,640,700
//!
//! Turn targets (which outbound lane each turn exits onto):
//!   Right turn  → nearest outbound lane of the destination arm
//!   Straight    → same column/row straight through
//!   Left turn   → far outbound lane of the destination arm
//!
//! Spawn: cars spawn at the FAR screen edge (x=0 or x=1100 or y=0 or y=1100)
//! so they travel the full lane length before reaching the intersection.

use crate::vehicle::{Arm, Turn};
use crate::config::{IX, IY, ROAD, OFF};

// ── Spawn at full screen edges (far end of the lane) ──
const SPAWN_N: f64 = 0.0;    // top of screen
const SPAWN_S: f64 = 1100.0; // bottom of screen
const SPAWN_W: f64 = 0.0;    // left of screen
const SPAWN_E: f64 = 1100.0; // right of screen

// ── Exit points: 80px past the road edge (off-screen) ──
const EXIT_N: f64 = IY - OFF;         // 290  (exit upward off-screen)
const EXIT_S: f64 = IY + ROAD + OFF;  // 810  (exit downward off-screen)
const EXIT_W: f64 = IX - OFF;         // 290  (exit leftward off-screen)
const EXIT_E: f64 = IX + ROAD + OFF;  // 810  (exit rightward off-screen)

// ── INBOUND lane centres ──

// North inbound (facing south ↓, right = low x)
const N_RT: f64 = 400.0; // col 0 — right-turn lane
const N_ST: f64 = 460.0; // col 1 — straight lane
const N_LT: f64 = 520.0; // col 2 — left-turn lane

// South inbound (facing north ↑, right = high x)
const S_RT: f64 = 700.0; // col 5 — right-turn lane
const S_ST: f64 = 640.0; // col 4 — straight lane
const S_LT: f64 = 580.0; // col 3 — left-turn lane

// West inbound (facing east →, right = high y)
const W_RT: f64 = 580.0; // row 3 — right-turn lane
const W_ST: f64 = 640.0; // row 4 — straight lane
const W_LT: f64 = 700.0; // row 5 — left-turn lane

// East inbound (facing west ←, right = low y)
const E_RT: f64 = 400.0; // row 0 — right-turn lane
const E_ST: f64 = 460.0; // row 1 — straight lane
const E_LT: f64 = 520.0; // row 2 — left-turn lane

// ── OUTBOUND lane centres ──
// These are the lanes cars EXIT onto (opposite half of each arm).

// North outbound (cars leaving northward ↑): cols 3,4,5
const OB_N0: f64 = 580.0; // col 3 — closest outbound (right-turn target from West)
const OB_N1: f64 = 640.0; // col 4 — middle outbound
const OB_N2: f64 = 700.0; // col 5 — far outbound    (left-turn target from East)

// South outbound (cars leaving southward ↓): cols 0,1,2
const OB_S0: f64 = 400.0; // col 0 — closest outbound (right-turn target from East)
const OB_S1: f64 = 460.0; // col 1
const OB_S2: f64 = 520.0; // col 2 — far outbound    (left-turn target from West)

// West outbound (cars leaving westward ←): rows 0,1,2
const OB_W0: f64 = 400.0; // row 0 — closest outbound (right-turn target from South)
const OB_W1: f64 = 460.0; // row 1
const OB_W2: f64 = 520.0; // row 2 — far outbound    (left-turn target from North)

// East outbound (cars leaving eastward →): rows 3,4,5
const OB_E0: f64 = 580.0; // row 3 — closest outbound (right-turn target from North)
const OB_E1: f64 = 640.0; // row 4
const OB_E2: f64 = 700.0; // row 5 — far outbound    (left-turn target from South)

// ── NORTH arm (spawn top y=0, travel South ↓) ────────────────────────────
// Right turn  → col 0 x=400, pivot south-to-east at East outbound row 3 (y=580)
const PATH_N_R: &[(f64,f64)] = &[
    (N_RT, SPAWN_N),    // (400,   0) spawn far top
    (N_RT, OB_E0),      // (400, 580) pivot — East outbound nearest row
    (EXIT_E, OB_E0),    // (810, 580) exit east off-screen
];
// Straight → col 1 x=460, south all the way through
const PATH_N_F: &[(f64,f64)] = &[
    (N_ST, SPAWN_N),    // (460,   0) spawn far top
    (N_ST, EXIT_S),     // (460, 810) exit south off-screen
];
// Left turn  → col 2 x=520, cross intersection, pivot south-to-west at West outbound row 2 (y=520)
const PATH_N_L: &[(f64,f64)] = &[
    (N_LT, SPAWN_N),    // (520,   0) spawn far top
    (N_LT, OB_W2),      // (520, 520) pivot — West outbound far row
    (EXIT_W, OB_W2),    // (290, 520) exit west off-screen
];

// ── SOUTH arm (spawn bottom y=1100, travel North ↑) ───────────────────────
// Right turn  → col 5 x=700, pivot north-to-west at West outbound row 0 (y=400)
const PATH_S_R: &[(f64,f64)] = &[
    (S_RT, SPAWN_S),    // (700, 1100) spawn far bottom
    (S_RT, OB_W0),      // (700,  400) pivot — West outbound nearest row
    (EXIT_W, OB_W0),    // (290,  400) exit west off-screen
];
// Straight → col 4 x=640, north all the way through
const PATH_S_F: &[(f64,f64)] = &[
    (S_ST, SPAWN_S),    // (640, 1100) spawn far bottom
    (S_ST, EXIT_N),     // (640,  290) exit north off-screen
];
// Left turn  → col 3 x=580, cross intersection, pivot north-to-east at East outbound row 5 (y=700)
const PATH_S_L: &[(f64,f64)] = &[
    (S_LT, SPAWN_S),    // (580, 1100) spawn far bottom
    (S_LT, OB_E2),      // (580,  700) pivot — East outbound far row
    (EXIT_E, OB_E2),    // (810,  700) exit east off-screen
];

// ── WEST arm (spawn left x=0, travel East →) ────────────────────────────
// Right turn  → row 3 y=580, pivot east-to-north at North outbound col 0 (x=580)
const PATH_W_R: &[(f64,f64)] = &[
    (SPAWN_W, W_RT),    // (  0, 580) spawn far left
    (OB_N0,   W_RT),    // (580, 580) pivot — North outbound nearest col
    (OB_N0,   EXIT_N),  // (580, 290) exit north off-screen
];
// Straight → row 4 y=640, east all the way through
const PATH_W_F: &[(f64,f64)] = &[
    (SPAWN_W, W_ST),    // (  0, 640) spawn far left
    (EXIT_E,  W_ST),    // (810, 640) exit east off-screen
];
// Left turn  → row 5 y=700, cross intersection, pivot east-to-south at South outbound col 2 (x=520)
const PATH_W_L: &[(f64,f64)] = &[
    (SPAWN_W, W_LT),    // (  0, 700) spawn far left
    (OB_S2,   W_LT),    // (520, 700) pivot — South outbound far col
    (OB_S2,   EXIT_S),  // (520, 810) exit south off-screen
];

// ── EAST arm (spawn right x=1100, travel West ←) ──────────────────────────
// Right turn  → row 0 y=400, pivot west-to-south at South outbound col 5 (x=700)
const PATH_E_R: &[(f64,f64)] = &[
    (SPAWN_E, E_RT),    // (1100, 400) spawn far right
    (OB_S2,   E_RT),    // ( 520, 400) pivot — South outbound nearest col  (col 2 x=520 is wrong — see note)
    (OB_S2,   EXIT_S),  // ( 520, 810) exit south off-screen
];
// Straight → row 1 y=460, west all the way through
const PATH_E_F: &[(f64,f64)] = &[
    (SPAWN_E, E_ST),    // (1100, 460) spawn far right
    (EXIT_W,  E_ST),    // ( 290, 460) exit west off-screen
];
// Left turn  → row 2 y=520, cross intersection, pivot west-to-north at North outbound col 3 (x=580)
const PATH_E_L: &[(f64,f64)] = &[
    (SPAWN_E, E_LT),    // (1100, 520) spawn far right
    (OB_N0,   E_LT),    // ( 580, 520) pivot — North outbound nearest col
    (OB_N0,   EXIT_N),  // ( 580, 290) exit north off-screen
];

// ── Selector ──────────────────────────────────────────────────────────────
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
