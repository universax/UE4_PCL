// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.
#pragma once

#include "ModuleManager.h"

DECLARE_LOG_CATEGORY_EXTERN(PCLPlugin, Log, All);

class FPCLPluginModule : public IModuleInterface
{
public:
	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};