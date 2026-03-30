using UnrealBuildTool;
using System.IO;

public class SpaceshipClient : ModuleRules
{
    public SpaceshipClient(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp20;

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "InputCore",
        });

        // ---------- Simulation library integration ----------

        // Repo root is 4 levels up from this Build.cs file:
        //   ue5/SpaceshipClient/Source/SpaceshipClient/SpaceshipClient.Build.cs
        //   -> ../../../../ = REPO_ROOT
        string RepoRoot = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..", "..", ".."));
        string SimSourceRoot = Path.Combine(RepoRoot, "src");

        // Headers: the sim code uses #include "shared/sim_types.hpp" etc.,
        // so the include root must be src/.
        PublicIncludePaths.Add(SimSourceRoot);

        // Wrap sim headers to suppress UE macro conflicts (check, verify, TEXT, etc.)
        bEnableUndefinedIdentifierWarnings = false;

        // Static libraries built by CMake (see SETUP_GUIDE.md Step 1).
        // Adjust SimLibDir if your CMake build directory differs.
        string SimLibDir;
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            SimLibDir = Path.Combine(RepoRoot, "build-ue5", "Release");
            PublicAdditionalLibraries.Add(Path.Combine(SimLibDir, "spaceship-server.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(SimLibDir, "spaceship-client-stub.lib"));
        }
        else
        {
            SimLibDir = Path.Combine(RepoRoot, "build-ue5");
            PublicAdditionalLibraries.Add(Path.Combine(SimLibDir, "libspaceship-server.a"));
            PublicAdditionalLibraries.Add(Path.Combine(SimLibDir, "libspaceship-client-stub.a"));
        }
    }
}
