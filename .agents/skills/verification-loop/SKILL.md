---
name: verification-loop
description: Repo-specific verification workflow for CMake/C++ changes.
origin: local
---

# Verification Loop

Use this before handing off meaningful changes.

## Phase 1: Configure

```bash
cmake -S . -B build
```

Stop if configure fails.

## Phase 2: Build

```bash
cmake --build build
```

Stop if the target fails to compile or link.

## Phase 3: Run

```bash
./build/spaceship-simulator
```

Confirm the binary starts and exits successfully.

## Phase 4: Diff Review

```bash
git diff --stat
git diff --name-only
```

Check for:

- unintended build file drift
- dead includes or warnings introduced by new code
- path assumptions that only work on Windows

## Output

Summarize:

- Configure: pass or fail
- Build: pass or fail
- Run: pass or fail
- Diff review: notable risks or none
