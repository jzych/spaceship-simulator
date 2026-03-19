# Agent Operating Guide

This repository is configured for Codex with a lean, ECC-inspired workflow.

## Objectives

- Keep changes small, testable, and easy to review.
- Prefer evidence before edits: inspect code, confirm assumptions, then change.
- Use WSL paths and Linux-native tooling when building or running the project.

## Project Facts

- Language: C++20
- Build system: CMake
- Primary target: `spaceship-simulator`
- Source root: `src/`

## Working Rules

1. Read the touched files before editing.
2. Keep `CMakeLists.txt` and source changes aligned.
3. Prefer simple implementations over speculative abstractions.
4. Prefer narrow, non-owning interfaces when passing existing data; use `std::span` or `std::string_view` whenever practical.
5. Preserve a clean command path for WSL users.
6. After meaningful changes, run the smallest relevant verification step.

## Verification Order

When tool availability permits, prefer this order:

1. Configure: `cmake -S . -B build`
2. Build: `cmake --build build`
3. Run: `./build/spaceship-simulator`

Inside WSL, use the Linux workspace path:

`/mnt/m/CodingProjects/spaceship-simulator`

## Codex-Specific Notes

- Project-local Codex settings live in `.codex/`.
- Repo skills live in `.agents/skills/`.
- Multi-agent roles are defined in `.codex/agents/`.
- If a task needs web verification, prefer official docs or primary sources.

## When To Use Skills

- Use `cmake-cpp-workflow` for build, refactor, or feature work in this repo.
- Use `verification-loop` before handing off substantial changes.
