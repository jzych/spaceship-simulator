---
name: cmake-cpp-workflow
description: CMake and C++ workflow guidance for this repository when using Codex from WSL.
origin: local
---

# CMake C++ Workflow

Use this skill when working on build logic, C++ source files, or project verification.

## Priorities

1. Inspect `CMakeLists.txt` before changing target structure.
2. Keep compiler assumptions explicit and minimal.
3. Prefer WSL commands over Windows-native build commands.

## Standard Commands

Configure:

```bash
cmake -S . -B build
```

Build:

```bash
cmake --build build
```

Run:

```bash
./build/spaceship-simulator
```

## WSL Workspace Path

```bash
cd /mnt/m/CodingProjects/spaceship-simulator
```

## Change Discipline

- If you add source files, update `CMakeLists.txt` in the same change.
- If you add compile-time requirements, document them in `docs/codex-wsl-setup.md`.
- Avoid introducing build-time dependencies unless the task requires them.
