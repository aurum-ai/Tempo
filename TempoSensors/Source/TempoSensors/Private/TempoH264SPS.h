// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"

namespace TempoH264
{
	// Adds an H.264 VUI bitstream restriction to SPS NAL units that do not already carry VUI.
	// Returns true when at least one SPS was rewritten. The restriction declares zero frame
	// reordering and bounds the decoded-picture buffer to the SPS reference-frame requirement.
	bool AddLowLatencyVUI(TArray<uint8>& AnnexBAccessUnit);
}
