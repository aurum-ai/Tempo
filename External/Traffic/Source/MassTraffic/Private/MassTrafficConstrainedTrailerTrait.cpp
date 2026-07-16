// Copyright Epic Games, Inc. All Rights Reserved.

#include "MassTrafficConstrainedTrailerTrait.h"
#include "MassTrafficFragments.h"

#include "MassEntityTemplateRegistry.h"
#include "MassSimulationLOD.h"
#include "MassTrafficSubsystem.h"
#include "MassEntityUtils.h"


void UMassTrafficConstrainedTrailerTrait::BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const
{
	FMassEntityManager& EntityManager = UE::Mass::Utils::GetEntityManagerChecked(World);

	UMassTrafficSubsystem* MassTrafficSubsystem = UWorld::GetSubsystem<UMassTrafficSubsystem>(&World);
	if (!MassTrafficSubsystem)
	{
		// Editor-world template builds (e.g. the MassGameplayEditor trait
		// repository initializing during entity-config validation) have no
		// MassTrafficSubsystem. Building an incomplete template here is
		// harmless; asserting takes the whole editor down.
		return;
	}

	// Add Params as shared fragment
	const FConstSharedStruct ParamsSharedFragment = EntityManager.GetOrCreateConstSharedFragment(Params);
	BuildContext.AddConstSharedFragment(ParamsSharedFragment);

	// Constrained trailer ref fragment
	BuildContext.AddFragment<FMassTrafficConstrainedTrailerFragment>();

	IF_MASSTRAFFIC_ENABLE_DEBUG(BuildContext.RequireFragment<FMassTrafficDebugFragment>());
}
