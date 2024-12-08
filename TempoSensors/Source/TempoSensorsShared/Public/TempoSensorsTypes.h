// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"

#include "TempoSensorsTypes.generated.h"

UENUM(BlueprintType)
enum EMeasurementType : uint8
{
	COLOR_IMAGE = 0 UMETA(DisplayName = "ColorImage"),
	DEPTH_IMAGE = 1 UMETA(DisplayName = "DepthImage"),
	LABEL_IMAGE = 2 UMETA(DisplayName = "LabelImage"),
	BOUNDING_BOXES = 3 UMETA(DisplayName = "BoundingBoxes"),
};
