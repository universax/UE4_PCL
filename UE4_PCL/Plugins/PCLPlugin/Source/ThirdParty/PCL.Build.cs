// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class PCL : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    public PCL(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        LoadLibraries(Target);
    }

    public bool LoadLibraries(ReadOnlyTargetRules Target)
    {
        bool isLibrarySupported = false;
        //bool bDebug = (Target.Configuration == UnrealTargetConfiguration.Debug && BuildConfiguration.bDebugBuildsActuallyUseDebugCRT);

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            isLibrarySupported = true;

            // Add the import library
            PublicLibraryPaths.Add(Path.Combine(ModulePath, "Boost", "lib", "vc140"));
            PublicLibraryPaths.Add(Path.Combine(ModulePath, "Boost", "lib", "vc141"));

            PublicLibraryPaths.Add(Path.Combine(ModulePath, "PCL", "lib"));
            PublicAdditionalLibraries.AddRange(
                    new string[]
                    {
                        "pcl_common_release.lib",
                        "pcl_features_release.lib",
                        "pcl_filters_release.lib",
                        "pcl_io_ply_release.lib",
                        "pcl_io_release.lib",
                        "pcl_kdtree_release.lib",
                        "pcl_keypoints_release.lib",
                        "pcl_ml_release.lib",
                        "pcl_outofcore_release.lib",
                        "pcl_people_release.lib",
                        "pcl_recognition_release.lib",
                        "pcl_registration_release.lib",
                        "pcl_sample_consensus_release.lib",
                        "pcl_search_release.lib",
                        "pcl_segmentation_release.lib",
                        "pcl_tracking_release.lib",
                    }
                );
            PublicLibraryPaths.Add(Path.Combine(ModulePath, "FLANN", "lib"));
            PublicAdditionalLibraries.AddRange(
                    new string[]
                    {
                        "flann_cpp_s.lib",
                        "flann_s.lib",
                    }
                );

            PublicLibraryPaths.Add(Path.Combine(ModulePath, "WpdPack", "lib"));
            PublicAdditionalLibraries.AddRange(
                    new string[]
                    {
                         "Packet.lib",
                         "wpcap.lib",
                    }
                );

        }

        if (isLibrarySupported)
        {
            PublicIncludePaths.Add(Path.Combine(ModulePath, "PCL/include/pcl-1.8"));
            PublicIncludePaths.Add(Path.Combine(ModulePath, "Boost/include/boost-1_64"));
            PublicIncludePaths.Add(Path.Combine(ModulePath, "Eigen/eigen3"));
            PublicIncludePaths.Add(Path.Combine(ModulePath, "FLANN/include"));
            PublicIncludePaths.Add(Path.Combine(ModulePath, "WpdPack/include"));

            // Not sure if needed
            PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
            PublicDefinitions.Add("BOOST_DISABLE_ABI_HEADERS=1");

            // Needed configurations in order to run Boost
            bUseRTTI = true;
            bEnableExceptions = true;
            bEnableUndefinedIdentifierWarnings = false;
        }

        PublicDefinitions.Add(string.Format("WITH_PCL_BINDING={0}", isLibrarySupported ? 1 : 0));
        PublicDefinitions.Add(string.Format("WITH_BOOST_BINDING={0}", isLibrarySupported ? 1 : 0));
        PublicDefinitions.Add(string.Format("WITH_FLANN_BINDING={0}", isLibrarySupported ? 1 : 0));
        PublicDefinitions.Add(string.Format("WITH_WpdPack_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}
