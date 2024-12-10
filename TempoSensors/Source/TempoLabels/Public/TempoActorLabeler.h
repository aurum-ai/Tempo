// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "ActorClassificationInterface.h"

#include "CoreMinimal.h"
#include "Engine/DataTable.h"
#include "Subsystems/WorldSubsystem.h"

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

	virtual void Deinitialize() override;

protected:
	// Handle cleanup when an actor is destroyed
	UFUNCTION()
	void OnActorDestroyed(AActor* DestroyedActor);

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

	// Store instance IDs for actors
	UPROPERTY()
	TMap<const AActor*, uint32> ActorInstanceIds;

	// Next available instance ID
	UPROPERTY()
	uint32 NextInstanceId = 1;

	// Handle for the actor destroyed delegate
	FDelegateHandle ActorDestroyedHandle;

	// Get or create instance ID for an actor
	uint32 GetOrCreateInstanceId(const AActor* Actor);
};
