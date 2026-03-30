using UnrealBuildTool;
using System.IO;

public class SpaceshipSim : ModuleRules
{
    public SpaceshipSim(ReadOnlyTargetRules Target) : base(Target)
    {
        // UBT compiles all .cpp files it finds under this module's directory tree.
        // Simulation source lives in Private/ — no pre-compiled lib step required.
        PCHUsage = PCHUsageMode.NoPCHs;
        CppStandard = CppStandardVersion.Cpp20;

        // Include root: #include "shared/sim_types.hpp", "server/...", "client/..." all resolve.
        PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));

        // Suppress warnings from UE macros that shadow standard C++ identifiers
        // (e.g. check, verify, TEXT) present in simulation headers.
        bEnableUndefinedIdentifierWarnings = false;

        PublicDependencyModuleNames.AddRange(new string[] { "Core" });
    }
}
