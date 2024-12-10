// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoCamera.h"

#include "TempoCameraModule.h"

#include "TempoSensorsSettings.h"

#include "TempoLabelTypes.h"

#include "Engine/TextureRenderTarget2D.h"

namespace
{
	// This is the largest float less than the largest uint32 (2^32 - 1).
	// We use it to discretize the depth buffer into a uint32 pixel.
	constexpr float kMaxDiscreteDepth = 4294967040.0;
} // namespace

FTempoCameraIntrinsics::FTempoCameraIntrinsics(const FIntPoint& SizeXY, float HorizontalFOV)
	: Fx(SizeXY.X / 2.0 / FMath::Tan(FMath::DegreesToRadians(HorizontalFOV) / 2.0)), Fy(Fx), // Fx == Fy means the sensor's pixels are square, not that the FOV is symmetric.
	Cx(SizeXY.X / 2.0)
	, Cy(SizeXY.Y / 2.0)
{
}

template <typename PixelType>
void RespondToColorRequests(const TTextureRead<PixelType>* TextureRead, const TArray<FColorImageRequest>& Requests, float TransmissionTime)
{
	TempoCamera::ColorImage ColorImage;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraDecodeColor);
		ColorImage.set_width(TextureRead->ImageSize.X);
		ColorImage.set_height(TextureRead->ImageSize.Y);
		std::vector<char> ImageData;
		ImageData.reserve(TextureRead->Image.Num() * 3);
		for (const auto& Pixel : TextureRead->Image)
		{
			ImageData.push_back(Pixel.B());
			ImageData.push_back(Pixel.G());
			ImageData.push_back(Pixel.R());
		}
		ColorImage.mutable_data()->assign(ImageData.begin(), ImageData.end());
		ColorImage.mutable_header()->set_sequence_id(TextureRead->SequenceId);
		ColorImage.mutable_header()->set_capture_time(TextureRead->CaptureTime);
		ColorImage.mutable_header()->set_transmission_time(TransmissionTime);
		ColorImage.mutable_header()->set_sensor_name(TCHAR_TO_UTF8(*FString::Printf(TEXT("%s/%s"), *TextureRead->OwnerName, *TextureRead->SensorName)));
	}

	TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraRespondColor);
	for (auto ColorImageRequestIt = Requests.CreateConstIterator(); ColorImageRequestIt; ++ColorImageRequestIt)
	{
		ColorImageRequestIt->ResponseContinuation.ExecuteIfBound(ColorImage, grpc::Status_OK);
	}
}

template <typename PixelType>
void RespondToLabelRequests(const TTextureRead<PixelType>* TextureRead, const TArray<FLabelImageRequest>& Requests, float TransmissionTime)
{
	TempoCamera::LabelImage LabelImage;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraDecodeLabel);
		LabelImage.set_width(TextureRead->ImageSize.X);
		LabelImage.set_height(TextureRead->ImageSize.Y);
		std::vector<char> ImageData;
		ImageData.reserve(TextureRead->Image.Num());
		for (const PixelType& Pixel : TextureRead->Image)
		{
			ImageData.push_back(Pixel.Label());
		}
		LabelImage.mutable_data()->assign(ImageData.begin(), ImageData.end());
		LabelImage.mutable_header()->set_sequence_id(TextureRead->SequenceId);
		LabelImage.mutable_header()->set_capture_time(TextureRead->CaptureTime);
		LabelImage.mutable_header()->set_transmission_time(TransmissionTime);
		LabelImage.mutable_header()->set_sensor_name(TCHAR_TO_UTF8(*FString::Printf(TEXT("%s/%s"), *TextureRead->OwnerName, *TextureRead->SensorName)));
	}

	TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraRespondLabel);
	for (auto LabelImageRequestIt = Requests.CreateConstIterator(); LabelImageRequestIt; ++LabelImageRequestIt)
	{
		LabelImageRequestIt->ResponseContinuation.ExecuteIfBound(LabelImage, grpc::Status_OK);
	}
}

template <typename PixelType>
void RespondToBoundingBoxRequests(const TTextureRead<PixelType>* TextureRead, const TArray<FBoundingBoxesRequest>& Requests, float TransmissionTime)
{
	TempoCamera::BoundingBoxes BoundingBoxes;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraDecodeBoundingBoxes);

		for (const auto& Pixel : TextureRead->Image) {
			if (Pixel.InstanceId() > 0) {
				UE_LOG(LogTempoCamera, Display, TEXT("RespondToBoundingBoxes() saw InstanceId: %u"), static_cast<uint32>(Pixel.InstanceId()));
				break;
			}
		}
		
		TMap<uint8, FBox2D> LabelBounds;
		TMap<uint8, TSet<uint32>> LabelToInstanceIds;  // Map of Label -> Set of unique Instance IDs

		for (int32 Y = 0; Y < TextureRead->ImageSize.Y; Y++)
		{
			for (int32 X = 0; X < TextureRead->ImageSize.X; X++)
			{
				uint8 Label = TextureRead->Image[Y * TextureRead->ImageSize.X + X].Label();
				if (Label <= 0)
				{
					continue;
				}
				uint32 InstanceId = static_cast<uint32>(TextureRead->Image[Y * TextureRead->ImageSize.X + X].InstanceId());
				
        // Add instance ID to the set for this label
        if (!LabelToInstanceIds.Contains(Label))
        {
            LabelToInstanceIds.Add(Label, TSet<uint32>());
        }
        LabelToInstanceIds[Label].Add(InstanceId);

				FVector2D PixelPos(X, Y);
				if (auto* Bounds = LabelBounds.Find(Label))
				{
					*Bounds += PixelPos;
				}
				else
				{
					FBox2D NewBox(PixelPos, PixelPos);
					NewBox.bIsValid = true;
					LabelBounds.Add(Label, NewBox);
				}
			}
		}

		// Log the unique instance IDs for each label
		for (const auto& Pair : LabelToInstanceIds)
		{
				FString InstanceIdsStr;
				for (uint32 InstanceId : Pair.Value)
				{
						if (!InstanceIdsStr.IsEmpty())
						{
								InstanceIdsStr += TEXT(", ");
						}
						InstanceIdsStr += FString::Printf(TEXT("%lu"), InstanceId);
				}
				UE_LOG(LogTempoCamera, Display, TEXT("Label %lu has instance IDs: [%s]"), 
						Pair.Key, *InstanceIdsStr);
		}

		// we can re-activate this once the data packing is working
		for (const auto& Pair : LabelBounds)
		{
			auto* BoundingBox = BoundingBoxes.add_boxes();
			BoundingBox->set_label(Pair.Key);
			BoundingBox->set_x_min(Pair.Value.Min.X / TextureRead->ImageSize.X);
			BoundingBox->set_y_min(Pair.Value.Min.Y / TextureRead->ImageSize.Y);
			BoundingBox->set_x_max(Pair.Value.Max.X / TextureRead->ImageSize.X);
			BoundingBox->set_y_max(Pair.Value.Max.Y / TextureRead->ImageSize.Y);
		}
		BoundingBoxes.mutable_header()->set_sequence_id(TextureRead->SequenceId);
		BoundingBoxes.mutable_header()->set_capture_time(TextureRead->CaptureTime);
		BoundingBoxes.mutable_header()->set_transmission_time(TransmissionTime);
		BoundingBoxes.mutable_header()->set_sensor_name(TCHAR_TO_UTF8(*FString::Printf(TEXT("%s/%s"), *TextureRead->OwnerName, *TextureRead->SensorName)));
	}

	TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraRespondBoundingBoxes);
	for (auto RequestIt = Requests.CreateConstIterator(); RequestIt; ++RequestIt)
	{
		RequestIt->ResponseContinuation.ExecuteIfBound(BoundingBoxes, grpc::Status_OK);
	}
}

void TTextureRead<FCameraPixelNoDepth>::RespondToRequests(const TArray<FColorImageRequest>& Requests, float TransmissionTime) const
{
	RespondToColorRequests(this, Requests, TransmissionTime);
}

void TTextureRead<FCameraPixelNoDepth>::RespondToRequests(const TArray<FLabelImageRequest>& Requests, float TransmissionTime) const
{
	RespondToLabelRequests(this, Requests, TransmissionTime);
}

void TTextureRead<FCameraPixelWithDepth>::RespondToRequests(const TArray<FColorImageRequest>& Requests, float TransmissionTime) const
{
	RespondToColorRequests(this, Requests, TransmissionTime);
}

void TTextureRead<FCameraPixelWithDepth>::RespondToRequests(const TArray<FLabelImageRequest>& Requests, float TransmissionTime) const
{
	RespondToLabelRequests(this, Requests, TransmissionTime);
}

void TTextureRead<FCameraPixelWithDepth>::RespondToRequests(const TArray<FDepthImageRequest>& Requests, float TransmissionTime) const
{
	TempoCamera::DepthImage DepthImage;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraDecodeDepth);
		DepthImage.set_width(ImageSize.X);
		DepthImage.set_height(ImageSize.Y);
		DepthImage.mutable_depths()->Reserve(ImageSize.X * ImageSize.Y);
		for (const FCameraPixelWithDepth& Pixel : Image)
		{
			DepthImage.add_depths(Pixel.Depth(MinDepth, MaxDepth, kMaxDiscreteDepth));
		}
		DepthImage.mutable_header()->set_sequence_id(SequenceId);
		DepthImage.mutable_header()->set_capture_time(CaptureTime);
		DepthImage.mutable_header()->set_transmission_time(TransmissionTime);
		DepthImage.mutable_header()->set_sensor_name(TCHAR_TO_UTF8(*FString::Printf(TEXT("%s/%s"), *OwnerName, *SensorName)));
	}

	TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraRespondDepth);
	for (auto DepthImageRequestIt = Requests.CreateConstIterator(); DepthImageRequestIt; ++DepthImageRequestIt)
	{
		DepthImageRequestIt->ResponseContinuation.ExecuteIfBound(DepthImage, grpc::Status_OK);
	}
}

void TTextureRead<FCameraPixelWithInstances>::RespondToRequests(const TArray<FColorImageRequest>& Requests, float TransmissionTime) const
{
	RespondToColorRequests(this, Requests, TransmissionTime);
}

void TTextureRead<FCameraPixelWithInstances>::RespondToRequests(const TArray<FLabelImageRequest>& Requests, float TransmissionTime) const
{
	RespondToLabelRequests(this, Requests, TransmissionTime);
}

void TTextureRead<FCameraPixelWithInstances>::RespondToRequests(const TArray<FBoundingBoxesRequest>& Requests, float TransmissionTime) const
{
	RespondToBoundingBoxRequests(this, Requests, TransmissionTime);
}

UTempoCamera::UTempoCamera()
{
	MeasurementTypes = { EMeasurementType::COLOR_IMAGE, EMeasurementType::LABEL_IMAGE, EMeasurementType::DEPTH_IMAGE, EMeasurementType::BOUNDING_BOXES };
	CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	PostProcessSettings.AutoExposureMethod = AEM_Basic;
	PostProcessSettings.AutoExposureSpeedUp = 20.0;
	PostProcessSettings.AutoExposureSpeedDown = 20.0;
	// Auto exposure percentages chosen to better match their own recommended settings (see Scene.h).
	PostProcessSettings.AutoExposureLowPercent = 75.0;
	PostProcessSettings.AutoExposureHighPercent = 85.0;
	PostProcessSettings.MotionBlurAmount = 0.0;
	ShowFlags.SetAntiAliasing(true);
	ShowFlags.SetTemporalAA(true);
	ShowFlags.SetMotionBlur(false);

	// Start with no-depth settings
	RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8; // Corresponds to PF_B8G8R8A8
	PixelFormatOverride = EPixelFormat::PF_Unknown;
}

void UTempoCamera::BeginPlay()
{
	Super::BeginPlay();

	ApplyDepthEnabled();
}

void UTempoCamera::UpdateSceneCaptureContents(FSceneInterface* Scene)
{
	if (!bDepthEnabled && !PendingDepthImageRequests.IsEmpty())
	{
		if (!PendingBoundingBoxRequests.IsEmpty())
		{
			UE_LOG(LogTempoCamera, Error, TEXT("Bounding boxes are enabled, cannot enable depth"));
			return;
		}
		// If a client is requesting depth, start rendering it.
		SetDepthEnabled(true);
	}

	if (bDepthEnabled && PendingDepthImageRequests.IsEmpty())
	{
		// If no client is requesting depth, stop rendering it.
		SetDepthEnabled(false);
	}

	if (!bBoundingBoxEnabled && !PendingBoundingBoxRequests.IsEmpty())
	{
		if (!PendingDepthImageRequests.IsEmpty())
		{
			UE_LOG(LogTempoCamera, Error, TEXT("Depth is enabled, cannot enable bounding boxes"));
			return;
		}
		SetBoundingBoxEnabled(true);
	}

	if (bBoundingBoxEnabled && PendingBoundingBoxRequests.IsEmpty())
	{
		// If no client is requesting bounding boxes, stop rendering them.
		SetBoundingBoxEnabled(false);
	}

	Super::UpdateSceneCaptureContents(Scene);
}

void UTempoCamera::RequestMeasurement(const TempoCamera::ColorImageRequest& Request, const TResponseDelegate<TempoCamera::ColorImage>& ResponseContinuation)
{
	PendingColorImageRequests.Add({ Request, ResponseContinuation });
}

void UTempoCamera::RequestMeasurement(const TempoCamera::LabelImageRequest& Request, const TResponseDelegate<TempoCamera::LabelImage>& ResponseContinuation)
{
	PendingLabelImageRequests.Add({ Request, ResponseContinuation });
}

void UTempoCamera::RequestMeasurement(const TempoCamera::DepthImageRequest& Request, const TResponseDelegate<TempoCamera::DepthImage>& ResponseContinuation)
{
	PendingDepthImageRequests.Add({ Request, ResponseContinuation });
}

void UTempoCamera::RequestMeasurement(const TempoCamera::BoundingBoxesRequest& Request, const TResponseDelegate<TempoCamera::BoundingBoxes>& ResponseContinuation)
{
	PendingBoundingBoxRequests.Add({ Request, ResponseContinuation });
}

FTempoCameraIntrinsics UTempoCamera::GetIntrinsics() const
{
	return FTempoCameraIntrinsics(SizeXY, FOVAngle);
}

bool UTempoCamera::HasPendingRequests() const
{
	return !PendingColorImageRequests.IsEmpty() || !PendingLabelImageRequests.IsEmpty() || !PendingDepthImageRequests.IsEmpty() || !PendingBoundingBoxRequests.IsEmpty();
}

FTextureRead* UTempoCamera::MakeTextureRead() const
{
	check(GetWorld());

	if (bBoundingBoxEnabled && bDepthEnabled)
	{
		// What to do here?
		UE_LOG(LogTempoCamera, Error, TEXT("Depth and bounding boxes are both enabled, which is not supported"));
		return nullptr;
	}

	if (bBoundingBoxEnabled)
	{
		return static_cast<FTextureRead*>(new TTextureRead<FCameraPixelWithInstances>(
			SizeXY,
			SequenceId,
			GetWorld()->GetTimeSeconds(),
			GetOwnerName(),
			GetSensorName()));
	}
	else if (bDepthEnabled)
	{
		return static_cast<FTextureRead*>(new TTextureRead<FCameraPixelWithDepth>(
			SizeXY,
			SequenceId,
			GetWorld()->GetTimeSeconds(),
			GetOwnerName(),
			GetSensorName(),
			MinDepth,
			MaxDepth));
	}
	else
	{
		return static_cast<FTextureRead*>(new TTextureRead<FCameraPixelNoDepth>(
			SizeXY,
			SequenceId,
			GetWorld()->GetTimeSeconds(),
			GetOwnerName(),
			GetSensorName()));
	}
}

TFuture<void> UTempoCamera::DecodeAndRespond(TUniquePtr<FTextureRead> TextureRead)
{
	const double TransmissionTime = GetWorld()->GetTimeSeconds();

	const bool	  bSupportsDepth = TextureRead->GetType() == TEXT("WithDepth");
	const bool	  bSupportsBoundingBoxes = TextureRead->GetType() == TEXT("WithInstances");
	TFuture<void> Future = Async(EAsyncExecution::TaskGraph, [TextureRead = MoveTemp(TextureRead), ColorImageRequests = PendingColorImageRequests, LabelImageRequests = PendingLabelImageRequests, DepthImageRequests = PendingDepthImageRequests, BoundingBoxRequests = PendingBoundingBoxRequests, TransmissionTimeCpy = TransmissionTime] {
		TRACE_CPUPROFILER_EVENT_SCOPE(TempoCameraDecodeAndRespond);

		if (TextureRead->GetType() == TEXT("WithDepth"))
		{
			static_cast<TTextureRead<FCameraPixelWithDepth>*>(TextureRead.Get())->RespondToRequests(ColorImageRequests, TransmissionTimeCpy);
			static_cast<TTextureRead<FCameraPixelWithDepth>*>(TextureRead.Get())->RespondToRequests(LabelImageRequests, TransmissionTimeCpy);
			static_cast<TTextureRead<FCameraPixelWithDepth>*>(TextureRead.Get())->RespondToRequests(DepthImageRequests, TransmissionTimeCpy);
		}
		else if (TextureRead->GetType() == TEXT("NoDepth"))
		{
			static_cast<TTextureRead<FCameraPixelNoDepth>*>(TextureRead.Get())->RespondToRequests(ColorImageRequests, TransmissionTimeCpy);
			static_cast<TTextureRead<FCameraPixelNoDepth>*>(TextureRead.Get())->RespondToRequests(LabelImageRequests, TransmissionTimeCpy);
		}
		else if (TextureRead->GetType() == TEXT("WithInstances"))
		{
			static_cast<TTextureRead<FCameraPixelWithInstances>*>(TextureRead.Get())->RespondToRequests(ColorImageRequests, TransmissionTimeCpy);
			static_cast<TTextureRead<FCameraPixelWithInstances>*>(TextureRead.Get())->RespondToRequests(LabelImageRequests, TransmissionTimeCpy);
			static_cast<TTextureRead<FCameraPixelWithInstances>*>(TextureRead.Get())->RespondToRequests(BoundingBoxRequests, TransmissionTimeCpy);
		}
	});

	PendingColorImageRequests.Empty();
	PendingLabelImageRequests.Empty();
	PendingBoundingBoxRequests.Empty();
	if (bSupportsDepth)
	{
		PendingDepthImageRequests.Empty();
	}
	if (bSupportsBoundingBoxes)
	{
		PendingBoundingBoxRequests.Empty();
	}

	return Future;
}

int32 UTempoCamera::GetMaxTextureQueueSize() const
{
	return GetDefault<UTempoSensorsSettings>()->GetMaxCameraRenderBufferSize();
}

void UTempoCamera::SetDepthEnabled(bool bDepthEnabledIn)
{
	if (bDepthEnabled == bDepthEnabledIn)
	{
		return;
	}

	UE_LOG(LogTempoCamera, Display, TEXT("Setting owner: %s camera: %s depth enabled: %d"), *GetOwnerName(), *GetSensorName(), bDepthEnabledIn);

	bDepthEnabled = bDepthEnabledIn;
	ApplyDepthEnabled();
}

void UTempoCamera::ApplyDepthEnabled()
{
	const UTempoSensorsSettings* TempoSensorsSettings = GetDefault<UTempoSensorsSettings>();
	check(TempoSensorsSettings);

	if (bDepthEnabled)
	{
		RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
		PixelFormatOverride = EPixelFormat::PF_A16B16G16R16;

		if (const TObjectPtr<UMaterialInterface> PostProcessMaterialWithDepth = GetDefault<UTempoSensorsSettings>()->GetCameraPostProcessMaterialWithDepth())
		{
			PostProcessMaterialInstance = UMaterialInstanceDynamic::Create(PostProcessMaterialWithDepth.Get(), this);
			MinDepth = GEngine->NearClipPlane;
			MaxDepth = TempoSensorsSettings->GetMaxCameraDepth();
			PostProcessMaterialInstance->SetScalarParameterValue(TEXT("MinDepth"), MinDepth);
			PostProcessMaterialInstance->SetScalarParameterValue(TEXT("MaxDepth"), MaxDepth);
			PostProcessMaterialInstance->SetScalarParameterValue(TEXT("MaxDiscreteDepth"), kMaxDiscreteDepth);
		}
		else
		{
			UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialWithDepth is not set in TempoSensors settings"));
		}
	}
	else
	{
		if (const TObjectPtr<UMaterialInterface> PostProcessMaterialNoDepth = GetDefault<UTempoSensorsSettings>()->GetCameraPostProcessMaterialNoDepth())
		{
			PostProcessMaterialInstance = UMaterialInstanceDynamic::Create(PostProcessMaterialNoDepth.Get(), this);
		}
		else
		{
			UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialWithDepth is not set in TempoSensors settings"));
		}

		RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8; // Corresponds to PF_B8G8R8A8
		PixelFormatOverride = EPixelFormat::PF_Unknown;
	}

	UDataTable*		 SemanticLabelTable = GetDefault<UTempoSensorsSettings>()->GetSemanticLabelTable();
	FName			 OverridableLabelRowName = TempoSensorsSettings->GetOverridableLabelRowName();
	FName			 OverridingLabelRowName = TempoSensorsSettings->GetOverridingLabelRowName();
	TOptional<int32> OverridableLabel;
	TOptional<int32> OverridingLabel;
	if (!OverridableLabelRowName.IsNone())
	{
		SemanticLabelTable->ForeachRow<FSemanticLabel>(TEXT(""),
			[&OverridableLabelRowName,
				&OverridingLabelRowName,
				&OverridableLabel,
				&OverridingLabel](const FName& Key, const FSemanticLabel& Value) {
				if (Key == OverridableLabelRowName)
				{
					OverridableLabel = Value.Label;
				}
				if (Key == OverridingLabelRowName)
				{
					OverridingLabel = Value.Label;
				}
			});
	}

	if (PostProcessMaterialInstance)
	{
		if (OverridableLabel.IsSet() && OverridingLabel.IsSet())
		{
			PostProcessMaterialInstance->SetScalarParameterValue(TEXT("OverridableLabel"), OverridableLabel.GetValue());
			PostProcessMaterialInstance->SetScalarParameterValue(TEXT("OverridingLabel"), OverridingLabel.GetValue());
		}
		else
		{
			PostProcessMaterialInstance->SetScalarParameterValue(TEXT("OverridingLabel"), 0.0);
		}
		PostProcessSettings.WeightedBlendables.Array.Empty();
		PostProcessSettings.WeightedBlendables.Array.Init(FWeightedBlendable(1.0, PostProcessMaterialInstance), 1);
		PostProcessMaterialInstance->EnsureIsComplete();
	}
	else
	{
		UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialInstance is not set."));
	}

	InitRenderTarget();
}

void UTempoCamera::SetBoundingBoxEnabled(bool bBoundingBoxEnabledIn)
{
	if (bBoundingBoxEnabled == bBoundingBoxEnabledIn)
	{
		return;
	}

	UE_LOG(LogTempoCamera, Display, TEXT("Setting owner: %s camera: %s bounding box enabled: %d"),
		*GetOwnerName(), *GetSensorName(), bBoundingBoxEnabledIn);

	bBoundingBoxEnabled = bBoundingBoxEnabledIn;
	ApplyBoundingBoxEnabled();
}

// void UTempoCamera::ApplyBoundingBoxEnabled()
// {
// 	// Implement setting the correct render target format and pixel format and material
// 	const UTempoSensorsSettings* TempoSensorsSettings = GetDefault<UTempoSensorsSettings>();
// 	check(TempoSensorsSettings);

// 	if (!bBoundingBoxEnabled)
// 	{
// 		UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialWithInstances is not set in TempoSensors settings"));
// 		return;
// 	}

// 	if (bDepthEnabled)
// 	{
// 		UE_LOG(LogTempoCamera, Error, TEXT("Depth is already enabled, cannot enable bounding boxes"));
// 		return;
// 	}

// 	const TObjectPtr<UMaterialInterface> PostProcessMaterialWithInstances = GetDefault<UTempoSensorsSettings>()->GetCameraPostProcessMaterialWithInstances();
// 	if (!PostProcessMaterialWithInstances)
// 	{
// 		UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialWithInstances is not set in TempoSensors settings"));
// 		return;
// 	}

// 	// // Create and set up the instance ID render target
// 	// if (!InstanceIdRenderTarget)
// 	// {
// 	// 	InstanceIdRenderTarget = NewObject<UTextureRenderTarget2D>(this);
// 	// 	check(InstanceIdRenderTarget);

// 	// 	UE_LOG(LogTempoCamera, Display, TEXT("Creating 16-bit instance ID render target for owner: %s camera: %s bounding box enabled: %d size: %d x %d"),
// 	// 		*GetOwnerName(), *GetSensorName(), bBoundingBoxEnabled, SizeXY.X, SizeXY.Y);

// 	// 	// Use compatible format for 16-bit instance IDs
// 	// 	InstanceIdRenderTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
// 	// 	InstanceIdRenderTarget->InitCustomFormat(SizeXY.X, SizeXY.Y, PF_A16B16G16R16, false);
// 	// 	InstanceIdRenderTarget->ClearColor = FLinearColor::Black;
// 	// 	InstanceIdRenderTarget->bAutoGenerateMips = false;
// 	// 	InstanceIdRenderTarget->bGPUSharedFlag = true;
// 	// 	InstanceIdRenderTarget->UpdateResource();

// 	// 	InstanceIdRenderTarget->AddToRoot();
// 	// }

// 	// Set up the post process material
// 	PostProcessMaterialInstance = UMaterialInstanceDynamic::Create(PostProcessMaterialWithInstances.Get(), this);
// 	check(PostProcessMaterialInstance);

// 	// Initialize the render target with the right format to handle 16-bit opacity
// 	RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
// 	PixelFormatOverride = EPixelFormat::PF_A16B16G16R16;

// 	// Pass the instance ID render target to the material
// 	// UE_LOG(LogTempoCamera, Display, TEXT("Calling SetTextureParameterValue for instance ID render target to %s for owner: %s camera: %s bounding box enabled: %d"),
// 	// 	*InstanceIdRenderTarget->GetName(), *GetOwnerName(), *GetSensorName(), bBoundingBoxEnabled);
// 	// PostProcessMaterialInstance->SetTextureParameterValue(TEXT("InstanceIdTarget"), InstanceIdRenderTarget);

// 	PostProcessSettings.WeightedBlendables.Array.Empty();
// 	PostProcessSettings.WeightedBlendables.Array.Init(FWeightedBlendable(1.0, PostProcessMaterialInstance), 1);
// 	PostProcessMaterialInstance->EnsureIsComplete();

// 	UE_LOG(LogTempoCamera, Display, TEXT("Applied bounding box instances material for owner: %s camera: %s bounding box enabled: %d"),
// 		*GetOwnerName(), *GetSensorName(), bBoundingBoxEnabled);
// 	InitRenderTarget();
// }

void UTempoCamera::ApplyBoundingBoxEnabled()
{
    const UTempoSensorsSettings* TempoSensorsSettings = GetDefault<UTempoSensorsSettings>();
    check(TempoSensorsSettings);

    if (bBoundingBoxEnabled)
    {
        // Set formats first, just like in ApplyDepthEnabled
        RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
        PixelFormatOverride = EPixelFormat::PF_A16B16G16R16;

        const TObjectPtr<UMaterialInterface> PostProcessMaterialWithInstances = GetDefault<UTempoSensorsSettings>()->GetCameraPostProcessMaterialWithInstances();
        if (!PostProcessMaterialWithInstances)
        {
            UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialWithInstances is not set in TempoSensors settings"));
            return;
        }

        PostProcessMaterialInstance = UMaterialInstanceDynamic::Create(PostProcessMaterialWithInstances.Get(), this);
        check(PostProcessMaterialInstance);
    }
    else
    {
        // Match the no-depth case from ApplyDepthEnabled
        if (const TObjectPtr<UMaterialInterface> PostProcessMaterialNoDepth = GetDefault<UTempoSensorsSettings>()->GetCameraPostProcessMaterialNoDepth())
        {
            PostProcessMaterialInstance = UMaterialInstanceDynamic::Create(PostProcessMaterialNoDepth.Get(), this);
        }
        else
        {
            UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialNoDepth is not set in TempoSensors settings"));
        }

        RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        PixelFormatOverride = EPixelFormat::PF_Unknown;
    }

    // Apply the same semantic label handling as in ApplyDepthEnabled
    UDataTable* SemanticLabelTable = GetDefault<UTempoSensorsSettings>()->GetSemanticLabelTable();
    FName OverridableLabelRowName = TempoSensorsSettings->GetOverridableLabelRowName();
    FName OverridingLabelRowName = TempoSensorsSettings->GetOverridingLabelRowName();
    TOptional<int32> OverridableLabel;
    TOptional<int32> OverridingLabel;
    if (!OverridableLabelRowName.IsNone())
    {
        SemanticLabelTable->ForeachRow<FSemanticLabel>(TEXT(""),
            [&OverridableLabelRowName,
                &OverridingLabelRowName,
                &OverridableLabel,
                &OverridingLabel](const FName& Key, const FSemanticLabel& Value) {
                if (Key == OverridableLabelRowName)
                {
                    OverridableLabel = Value.Label;
                }
                if (Key == OverridingLabelRowName)
                {
                    OverridingLabel = Value.Label;
                }
            });
    }

    if (PostProcessMaterialInstance)
    {
        if (OverridableLabel.IsSet() && OverridingLabel.IsSet())
        {
            PostProcessMaterialInstance->SetScalarParameterValue(TEXT("OverridableLabel"), OverridableLabel.GetValue());
            PostProcessMaterialInstance->SetScalarParameterValue(TEXT("OverridingLabel"), OverridingLabel.GetValue());
        }
        else
        {
            PostProcessMaterialInstance->SetScalarParameterValue(TEXT("OverridingLabel"), 0.0);
        }

        PostProcessSettings.WeightedBlendables.Array.Empty();
        PostProcessSettings.WeightedBlendables.Array.Init(FWeightedBlendable(1.0, PostProcessMaterialInstance), 1);
        PostProcessMaterialInstance->EnsureIsComplete();
    }
    else
    {
        UE_LOG(LogTempoCamera, Error, TEXT("PostProcessMaterialInstance is not set."));
    }

    InitRenderTarget();
} 
