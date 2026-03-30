# UE5 Project Setup Guide (Task 15b)

## Prerequisites

- Unreal Engine 5.4+ installed via Epic Games Launcher
- CMake 3.20+ and a C++ compiler (MSVC on Windows, or Clang/GCC on Linux)
- This repo cloned at some path (referred to as `REPO_ROOT` below)

## Step 1: Build the simulation libraries

Open a terminal at `REPO_ROOT` and run:

```bash
# Windows (Developer Command Prompt or PowerShell)
cmake -S . -B build-ue5 -DBUILD_TESTING=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build build-ue5 --config Release

# Linux
cmake -S . -B build-ue5 -DBUILD_TESTING=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-ue5
```

After this, you should have these library files:

- **Windows**: `build-ue5/Release/spaceship-server.lib` and `build-ue5/Release/spaceship-client-stub.lib`
- **Linux**: `build-ue5/libspaceship-server.a` and `build-ue5/libspaceship-client-stub.a`

Verify they exist before continuing.

## Step 2: Create the UE5 project

1. Open **Epic Games Launcher** -> **Unreal Engine** -> **Launch** (5.4+)
2. In the Project Browser:
   - Select **Games** -> **Blank**
   - Choose **C++** (not Blueprint)
   - Project Name: `SpaceshipClient`
   - Location: `REPO_ROOT/ue5/` (so the project lands in `ue5/SpaceshipClient/`)
3. Click **Create**
4. Wait for the editor to open and initial compilation to finish
5. **Close the UE5 editor** completely

## Step 3: Replace source files

Copy the source files from this repo into the UE5 project. The files I've provided are:

```
ue5/SpaceshipClient/Source/SpaceshipClient/
  SpaceshipClient.Build.cs        <- REPLACE the auto-generated one
  Actors/
    PlanetActor.h
    PlanetActor.cpp
  Subsystems/
    SimulationSubsystem.h
    SimulationSubsystem.cpp
```

These files are already in the repo at the paths above. You need to:

1. **Replace `SpaceshipClient.Build.cs`** — overwrite the auto-generated one with the version from this repo
2. **Copy the `Actors/` and `Subsystems/` folders** into `Source/SpaceshipClient/`

The auto-generated files (`SpaceshipClient.h`, `.cpp`, `SpaceshipClientGameModeBase.h/.cpp`) can stay — they don't conflict.

## Step 4: Update Build.cs paths

Open `ue5/SpaceshipClient/Source/SpaceshipClient/SpaceshipClient.Build.cs` and verify the paths are correct for your system:

- `SimSourceRoot` should point to `REPO_ROOT/src`
- `SimLibDir` should point to where the `.lib`/`.a` files were built

The default Build.cs assumes the repo root is 4 directories up from the Build.cs file (`../../../../`). If your layout differs, update the paths.

## Step 5: Regenerate project files and compile from Visual Studio

UE5 does NOT auto-detect new source files — you must regenerate the project files
after adding `Actors/` and `Subsystems/` subdirectories.

### 5a: Close UE5 editor completely

Make sure the editor is fully closed.

### 5b: Delete stale build artifacts in `ue5/SpaceshipClient/`

Delete these folders if they exist (they will be regenerated):
- `Binaries/`
- `Intermediate/`
- `.vs/`

### 5c: Regenerate Visual Studio project files

**Option A (easiest):** Right-click `SpaceshipClient.uproject` in Windows Explorer
and choose **"Generate Visual Studio project files"**.

**Option B (command line):** Open a terminal and run:
```
"C:\Program Files\Epic Games\UE_5.4\Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.exe" -projectfiles -project="FULL_PATH\ue5\SpaceshipClient\SpaceshipClient.uproject" -game -rocket -progress
```
Replace `FULL_PATH` with the absolute path to this repo.

This creates `SpaceshipClient.sln` that includes all `.cpp` files in subdirectories.

### 5d: Verify lib files exist

Before building, confirm these files exist from Step 1:
- `build-ue5/Release/spaceship-server.lib`
- `build-ue5/Release/spaceship-client-stub.lib`

If missing, re-run Step 1.

### 5e: Build from Visual Studio

1. Open `SpaceshipClient.sln` in Visual Studio 2022
2. Set configuration dropdown to **Development Editor** and platform to **Win64**
3. Right-click `SpaceshipClient` in Solution Explorer → **Build**
4. Watch the Output window — first compile links the sim libraries and takes ~1-2 minutes

If you see **linker errors** about missing symbols:
- Confirm both `.lib` files exist in `build-ue5/Release/`
- Some MSVC configs place outputs in `build-ue5/` directly (not `build-ue5/Release/`).
  If so, edit `SimLibDir` in `SpaceshipClient.Build.cs` to remove the `"Release"` segment.

### 5f: Launch the editor from Visual Studio

Press **F5** or click **Local Windows Debugger** to launch the UE5 editor with
the compiled module.

## Step 6: Run the simulation

1. In the UE5 editor, click **Play** (the green triangle) or press **Alt+P**
2. You should see:
   - A yellow sphere (Sun) far away
   - A blue sphere (Earth) near the center of the view
   - A gray sphere (Moon) orbiting Earth
3. The simulation runs at 100,000x real time — Moon should visibly orbit Earth

## Troubleshooting

### "Module not found" or linker errors
- Rebuild the sim libraries (Step 1) and verify the `.lib`/`.a` files exist
- Check that Build.cs paths match your actual file locations

### No planets visible
- Check Output Log for errors from `USimulationSubsystem`
- The subsystem logs planet positions each frame — look for `SimBridge` log entries
- Planets may be spawned but too far away — check the camera position

### UHT error: "#include found after .generated.h file"
The `.generated.h` include must always be the **last** `#include` in a header.
This is already fixed in the current source files — ensure you have the latest version.

### Compilation errors about missing headers
- Verify `SimSourceRoot` in Build.cs points to the `src/` directory
- Ensure `THIRD_PARTY_INCLUDES_START`/`END` macros are wrapping the sim includes

### Moon doesn't appear to move
- The time scale might be too low. Default is 100,000x.
- At 100,000x, Moon completes an orbit in ~23 seconds of wall time
  (real period: 27.3 days / 100,000 = 23.6 seconds)
