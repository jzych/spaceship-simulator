# Task: Client Interpolation Buffer (TASK-14) — COMPLETE

## Progress
- [x] All implementation done, code review applied, 251/251 tests green
- [x] See git history for details

---

# Task: Camera-Relative Render Transform (TASK-16) — PLANNED

## Plan
- Subtraction in double, narrow to float at the end
- Axis remap: Sim (X-fwd, Y-up, Z-right, RH) → UE5 (X-fwd, Y-right, Z-up, LH): `ue = (sim_x, -sim_z, sim_y)`
- Separate float types (`RenderVec3`, `RenderQuat`)
- Fallback render origin = `{0,0,0}`

## Progress
- [ ] render_transform.hpp / .cpp / tests
- [ ] CMakeLists.txt
- [ ] Build + all tests green

---

# Task: UE5 Project Scaffold (TASK-15) — IN PROGRESS

## Goal
Running UE5 app showing Sun/Earth/Moon orbiting with placeholder spheres.
Camera follows Earth at 0.005 AU distance so Moon orbit is visible.
Time scale 100,000x (Earth orbits Sun in ~5 min).

## Sub-Tasks

### 15a: Simulation Runner (pure C++, no UE5)
- Simulation loop class that runs server + feeds snapshots to ClientSnapshotBuffer
- Configurable time scale factor (default 100,000x)
- Runs multiple sim ticks per call to advance at scaled speed
- Tests for runner
- **Files**: `src/client/simulation_runner.hpp/.cpp`, tests

### 15b: UE5 Project Setup (user follows step-by-step guide)
- Create C++ project via UE5 editor (Third Person template → strip content)
- Configure Build.cs to link spaceship-shared + spaceship-client-stub + spaceship-server
- BuildSimLibs script for CMake pre-build
- Verify compilation
- **Files**: `ue5/SpaceshipClient/` scaffold

### 15c: Actors & Bridge (C++ code → user compiles in UE5)
- `APlanetActor` — sphere mesh, color per body (yellow Sun, blue Earth, gray Moon)
- `USimulationSubsystem` — owns SimulationRunner, spawns/updates planet actors per frame
- Camera-relative rendering: Earth as render origin, positions offset in double then → float → UE cm
- Earth-follow camera at 0.005 AU distance
- Planet visual radii exaggerated to be visible (min display radius)
- **Files**: UE5 Source/ actor + subsystem classes

### 15d: Camera & Integration (user compiles and runs)
- Default camera placement: 0.005 AU from Earth, looking at Earth
- Verify: Moon orbits Earth visibly, Earth+Moon orbit Sun
- Time scale via console variable

## Progress
- [x] 15a: Simulation runner (10/10 tests, 261/261 total)
- [x] 15b: UE5 project setup guide (`ue5/SETUP_GUIDE.md`)
- [x] 15c: Actors & bridge code (PlanetActor, SimulationSubsystem)
- [ ] 15d: User compiles and runs in UE5 editor
