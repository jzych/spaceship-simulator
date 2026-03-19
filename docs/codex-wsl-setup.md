# Codex + WSL Setup

This repository is prepared for a lean, Everything-Claude-Code-style Codex workflow without copying the full upstream repository.

## What Was Added

- `AGENTS.md`: repo operating instructions
- `.codex/`: project-local Codex config and multi-agent roles
- `.agents/skills/`: repo-local skills that Codex can auto-load
- `scripts/codex-wsl.ps1`: launch helper for opening this repo in WSL

## Recommended Usage

1. Install Codex CLI in your WSL distro.
2. Install the C++ build tools in WSL if they are not already present:

   ```bash
   sudo apt update
   sudo apt install -y build-essential cmake
   ```

3. Open a shell in WSL.
4. Change into the repo:

   ```bash
   cd /mnt/m/CodingProjects/spaceship-simulator
   ```

5. Start Codex:

   ```bash
   codex
   ```

## PowerShell Launcher

From Windows PowerShell:

```powershell
.\scripts\codex-wsl.ps1
```

If PowerShell blocks local scripts with an execution policy error, use either:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\codex-wsl.ps1
```

or the wrapper command:

```powershell
.\scripts\codex-wsl.cmd
```

By default it targets the `Ubuntu` distro and runs `codex` in:

`/mnt/m/CodingProjects/spaceship-simulator`

Override the distro if needed:

```powershell
.\scripts\codex-wsl.ps1 -Distro Debian
```

## Optional Global Config

If you want these defaults globally, copy `.codex/config.toml` into `~/.codex/config.toml` inside WSL. Keep the project-local files if you want this repo to remain self-contained.

## Notes

- The project-local config keeps MCP servers commented out by default so the repo works even if `node` or `npx` are not installed in WSL.
- Multi-agent support is enabled in `.codex/config.toml`.
- The current setup is intentionally minimal and tailored to this CMake project.
- Verified on this machine: WSL resolves the repo path and `codex` is installed, but `cmake` was not present in the Ubuntu environment at setup time.
