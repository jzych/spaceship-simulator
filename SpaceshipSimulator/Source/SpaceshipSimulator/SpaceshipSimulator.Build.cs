// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class SpaceshipSimulator : ModuleRules
{
	public SpaceshipSimulator(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		CppStandard = CppStandardVersion.Cpp20;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput"
		});

		// Simulation plugin — provides all spaceship:: namespaced headers and code.
		// No pre-compiled lib step required; UBT compiles sim sources directly.
		PrivateDependencyModuleNames.Add("SpaceshipSim");

		// Suppress warnings from UE macros that shadow standard C++ identifiers
		// in simulation headers included transitively via SpaceshipSim.
		bEnableUndefinedIdentifierWarnings = false;
	}
}
