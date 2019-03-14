// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class PCLPlugin : ModuleRules
{
    public PCLPlugin(ReadOnlyTargetRules Target) : base(Target)
    {

        PublicIncludePaths.AddRange(
        new string[] {
                "PCLPlugin/Public"
            // ... add public include paths required here ...
        }
        );


        PrivateIncludePaths.AddRange(
            new string[] {
                "PCLPlugin/Private",
                // ... add other private include paths required here ...
            }
            );


        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine",
                "InputCore",
                "RHI",
                "RenderCore",
                "PCL",
                "Projects"
                // ... add other public dependencies that you statically link with here ...
            }
            );


        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                // ... add private dependencies that you statically link with here ...	
            }
            );

        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
                // ... add any modules that your module loads dynamically here ...
            }
            );
        
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;



        bUseRTTI = true;
        bEnableExceptions = true;
        bEnableUndefinedIdentifierWarnings = false;
    }
}
