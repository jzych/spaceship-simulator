# Codex Supplement

This file supplements the root `AGENTS.md` for Codex sessions in this repository.

## Runtime Defaults

- Prefer the project-local `.codex/config.toml`.
- Keep instructions in `AGENTS.md`; do not replace them with a global override unless required.
- Use repo-local agents and skills before inventing new workflow scaffolding.

## Model Guidance

- Routine edits, builds, and reviews: use the default Codex model.
- Broad refactors or ambiguous debugging: increase reasoning effort instead of over-customizing config.

## WSL Guidance

- Launch Codex from WSL when possible so CMake, compilers, and paths stay Unix-native.
- Preferred Linux path for this repo: `/mnt/m/CodingProjects/spaceship-simulator`
- If working from Windows PowerShell, use `scripts/codex-wsl.ps1`.

## Skills

Codex auto-loads skills from `.agents/skills/`.

- `cmake-cpp-workflow`: build/update workflow for this project
- `verification-loop`: repo-specific verification checklist
