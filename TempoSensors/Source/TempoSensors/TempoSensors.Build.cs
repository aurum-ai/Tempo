﻿// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class TempoSensors : TempoModuleRules
{
	public TempoSensors(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				// Unreal
				"Core",
				// Tempo
				"TempoCoreShared",
				"TempoScripting",
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				// Unreal
				"CoreUObject",
				"Engine",
				"RenderCore",
				"RHI",
				"Slate",
				"SlateCore",
				// Tempo
				"TempoSensorsShared",
				"TempoCamera",
				"TempoLabels",
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
			}
			);
	}
}
