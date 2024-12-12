// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "ActorClassificationInterface.h"

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Subsystems/WorldSubsystem.h"
#include "HAL/CriticalSection.h"

#include "TempoActorLabeler.generated.h"

/**
 * Tags all meshes on all Actors in the world with the appropriate label.
 */
UCLASS()
class TEMPOLABELS_API UTempoActorLabeler : public UWorldSubsystem, public IActorClassificationInterface
{
	GENERATED_BODY()

public:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;

	virtual FName GetActorClassification(const AActor* Actor) const override;

	const TMap<uint8, int32>& GetInstanceToLabelMap() const { return InstanceToLabel; }

protected:
	void BuildLabelMaps();
	
	void LabelAllActors();
	
	void LabelActor(AActor* Actor);

	void LabelAllComponents(const AActor* Actor, int32 ActorLabelId);

	void LabelComponent(UActorComponent* Component);
	
	void LabelComponent(UPrimitiveComponent* Component, int32 ActorLabelId);
	
	UPROPERTY(VisibleAnywhere)
	UDataTable* SemanticLabelTable;

	UPROPERTY(VisibleAnywhere)
	TMap<TSubclassOf<AActor>, FName> ActorLabels;

	UPROPERTY(VisibleAnywhere)
	TMap<FString, FName> StaticMeshLabels;

	UPROPERTY(VisibleAnywhere)
	TMap<FName, int32> LabelIds;

	UPROPERTY(VisibleAnywhere)
	FName NoLabelName = TEXT("NoLabel");

	UPROPERTY(VisibleAnywhere)
	int32 NoLabelId = 0;

	// Cache to avoid re-searching the label table for Actors we've already labeled
	UPROPERTY()
	TMap<const AActor*, int32> LabeledActors;

	// Cache to avoid re-labeling Components we've already labeled
	UPROPERTY()
	TMap<const UPrimitiveComponent*, int32> LabeledComponents;

	// Maps unique instance IDs to their associated labels
	UPROPERTY()
	TMap<uint8, int32> InstanceToLabel;

	// Maps parent actors to their assigned unique instance IDs
	UPROPERTY()
	TMap<const AActor*, uint8> ParentActorInstanceIds;

	uint8 nextUniqueInstanceId = 1;

	// Critical section for thread-safe access to instance ID maps
	FCriticalSection CriticalSection;
};
