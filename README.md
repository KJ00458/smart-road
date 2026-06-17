# Smart Road

Autonomous vehicle intersection management simulation in Rust using SDL2.

## Prerequisites

```bash
# Ubuntu / Debian
sudo apt-get install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev

# macOS
brew install sdl2 sdl2_image sdl2_ttf

# Arch
sudo pacman -S sdl2 sdl2_image sdl2_ttf
```

## Build & Run

```bash
cargo run --release
```

## Controls

| Key | Action |
|-----|--------|
| ↑ Arrow Up | Spawn vehicle from South (northbound) |
| ↓ Arrow Down | Spawn vehicle from North (southbound) |
| → Arrow Right | Spawn vehicle from West (eastbound) |
| ← Arrow Left | Spawn vehicle from East (westbound) |
| R | Continuously spawn random vehicles |
| Esc | Stop simulation and show statistics |

## Algorithm

The smart intersection uses a **reservation-based protocol**:

1. Each vehicle approaching the intersection requests a time-slot reservation.
2. The intersection manager checks if the requested slot conflicts with any existing reservation (using a conflict matrix of crossing paths).
3. If no conflict exists, the reservation is granted and the vehicle proceeds at the assigned velocity.
4. If a conflict exists, the vehicle slows down or waits, then re-requests a later slot.
5. Vehicles with right-of-way proceed unimpeded; others adjust speed from three available velocity tiers.

This eliminates collisions without traffic lights and minimises congestion.
