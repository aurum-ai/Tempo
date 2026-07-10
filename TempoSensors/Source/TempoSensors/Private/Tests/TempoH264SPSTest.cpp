// Copyright Tempo Simulation, LLC. All Rights Reserved

#if WITH_DEV_AUTOMATION_TESTS

#include "TempoH264SPS.h"

#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FTempoH264AddsLowLatencyVUITest,
	"Tempo.Sensors.Video.H264AddsLowLatencyVUI",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTempoH264AddsLowLatencyVUITest::RunTest(const FString& Parameters)
{
	TArray<uint8> AccessUnit = {
		0x00, 0x00, 0x00, 0x01,
		0x27, 0x42, 0x00, 0x1f, 0xab, 0x40, 0x78, 0x08, 0xbf, 0x68,
		0x00, 0x00, 0x00, 0x01,
		0x28, 0xce, 0x3c, 0x80,
	};
	const TArray<uint8> Expected = {
		0x00, 0x00, 0x00, 0x01,
		0x27, 0x42, 0x00, 0x1f, 0xab, 0x40, 0x78, 0x08, 0xbf, 0x70, 0x0f, 0x08, 0x84, 0x6a,
		0x00, 0x00, 0x00, 0x01,
		0x28, 0xce, 0x3c, 0x80,
	};

	TestTrue(TEXT("SPS without VUI is rewritten"), TempoH264::AddLowLatencyVUI(AccessUnit));
	TestTrue(TEXT("Access unit contains the low-latency VUI and preserves the PPS"), AccessUnit == Expected);
	TestFalse(TEXT("Already-rewritten SPS is left unchanged"), TempoH264::AddLowLatencyVUI(AccessUnit));
	TestTrue(TEXT("Second pass is byte-stable"), AccessUnit == Expected);

	TArray<uint8> ThreeByteStartCodes = {
		0x00, 0x00, 0x01,
		0x27, 0x42, 0x00, 0x1f, 0xab, 0x40, 0x78, 0x08, 0xbf, 0x68,
		0x00, 0x00, 0x01,
		0x28, 0xce, 0x3c, 0x80,
	};
	const TArray<uint8> ExpectedThreeByteStartCodes = {
		0x00, 0x00, 0x01,
		0x27, 0x42, 0x00, 0x1f, 0xab, 0x40, 0x78, 0x08, 0xbf, 0x70, 0x0f, 0x08, 0x84, 0x6a,
		0x00, 0x00, 0x01,
		0x28, 0xce, 0x3c, 0x80,
	};
	TestTrue(TEXT("Three-byte Annex-B start codes are supported"), TempoH264::AddLowLatencyVUI(ThreeByteStartCodes));
	TestTrue(TEXT("Three-byte start codes and PPS are preserved"), ThreeByteStartCodes == ExpectedThreeByteStartCodes);

	TArray<uint8> DeltaFrame = {0x00, 0x00, 0x00, 0x01, 0x21, 0x9a, 0x20};
	const TArray<uint8> OriginalDeltaFrame = DeltaFrame;
	TestFalse(TEXT("Access units without an SPS are not rewritten"), TempoH264::AddLowLatencyVUI(DeltaFrame));
	TestTrue(TEXT("Access units without an SPS remain byte-stable"), DeltaFrame == OriginalDeltaFrame);

	TArray<uint8> TruncatedSPS = {0x00, 0x00, 0x00, 0x01, 0x27, 0x42};
	const TArray<uint8> OriginalTruncatedSPS = TruncatedSPS;
	TestFalse(TEXT("A truncated SPS is rejected"), TempoH264::AddLowLatencyVUI(TruncatedSPS));
	TestTrue(TEXT("A truncated SPS remains byte-stable"), TruncatedSPS == OriginalTruncatedSPS);
	return true;
}

#endif
