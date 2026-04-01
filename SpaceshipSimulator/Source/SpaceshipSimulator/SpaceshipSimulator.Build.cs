// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class SpaceshipSimulator : ModuleRules
{
	public SpaceshipSimulator(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Cpp20;

		// Module root: allows cross-subdirectory includes (e.g. "Pawns/SimSpectatorPawn.h"
		// from within GameModes/). UBT does not add the module root by default when
		// subdirectories contain source files.
		PrivateIncludePaths.Add(ModuleDirectory);

		// Simulation source root: all spaceship:: headers resolve from here.
		// UBT auto-discovers and compiles all .cpp files under Sim/ as part of
		// this module — no plugin DLL boundary, no __declspec(dllexport) needed.
		string SimRoot = Path.Combine(ModuleDirectory, "Sim");
		PrivateIncludePaths.Add(SimRoot);
		PublicIncludePaths.Add(SimRoot);

		// Suppress warnings from UE macros (check, verify, TEXT) that shadow
		// standard C++ identifiers in simulation headers.
		bEnableUndefinedIdentifierWarnings = false;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput"
		});
	}
}
