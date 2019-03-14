#include "PCLPlugin.h"
#include "CoreUObject.h"
#include "Engine.h"
#include "Core.h"
#include "ModuleManager.h"
//#include "IPluginManager.h"

#pragma warning ( disable: 4503 )

#define LOCTEXT_NAMESPACE "FPCLPluginModule"

void FPCLPluginModule::StartupModule()
{

}

void FPCLPluginModule::ShutdownModule()
{

}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FPCLPluginModule, PCLPlugin)