# 🐦 Boid Flocking Simulation

A real-time flocking simulation built with **C++** and **SFML 3.0**, inspired by Craig Reynolds' classic Boids algorithm. Hundreds of agents move together in fluid, organic flocks — just like birds in a murmuration — using only three simple local rules.

![Boid Flocking Preview](https://raw.githubusercontent.com/snehal-thombare08/Boid-Flocking-/main/PASTE_YOUR_SCREENSHOT_FILENAME_HERE.png)

## ✨ Features

- **Emergent flocking behavior** from three local rules — Separation, Alignment, and Cohesion
- **Priority-weighted steering** — boids smoothly blend desired directions instead of jittering
- **Turn-rate limited motion** — fluid, natural-looking turns instead of instant snapping
- **Predator mode** — right-click to scare the flock and watch it scatter and reform
- **Glowing motion trails** that fade smoothly over time
- **Speed-based color gradient** — calmer blue tones at cruising speed, hotter red/orange when boids are crowded or fleeing
- **Live flock size control** — add or remove boids on the fly
- **Toroidal world** — boids wrap around screen edges seamlessly

## 🎮 Controls

| Key / Mouse        | Action                          |
|---------------------|----------------------------------|
| Right Mouse Button  | Toggle predator mode            |
| Space               | Scatter burst                   |
| `+`                 | Add boids (up to 500)           |
| `-`                 | Remove boids (down to 20)       |
| R                   | Reset the flock                 |
| Esc                 | Quit                             |

## 🧠 How It Works

Each boid follows three simple steering rules, evaluated every frame against nearby neighbors only:

1. **Separation** — steer away from boids that are too close, to avoid collisions
2. **Alignment** — steer toward the average heading of nearby boids
3. **Cohesion** — steer toward the average position of nearby boids

These three desired directions are combined as a priority-weighted blend (separation first, then alignment, then cohesion), and each boid turns gradually toward that blended direction rather than snapping instantly — which is what produces the smooth, organic group motion seen in real bird flocks.

## 🛠️ Built With

- **C++17**
- **SFML 3.0** — windowing, rendering, and input
- **CMake** — build configuration
- **MinGW** — compiler toolchain (Windows)

## 🚀 Building from Source

### Prerequisites
- CMake 3.16+
- MinGW-w64 (or any C++17 compiler)
- SFML 3.0 (via vcpkg recommended)

### Build Steps

```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg.cmake -G "MinGW Makefiles"
mingw32-make
./BoidFlocking.exe
```
## screenshot
![Boid Flocking Preview](https://raw.githubusercontent.com/snehal-thombare08/Boid-Flocking-/main/Screenshot%202026-06-17%20115544.png)

## 📦 Download

Prebuilt Windows binary available under [Releases](../../releases) — just download, extract, and run `BoidFlocking.exe`.

## 📄 License

This project is open source and available for learning and experimentation.

---

⭐ If you found this interesting, consider starring the repo!
