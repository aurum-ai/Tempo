// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoH264SPS.h"

namespace
{
	class FBitReader
	{
	public:
		explicit FBitReader(const TArray<uint8>& InData)
			: Data(InData)
		{
		}

		bool ReadBit(bool& Out)
		{
			uint32 Value = 0;
			if (!ReadBits(1, Value))
			{
				return false;
			}
			Out = Value != 0;
			return true;
		}

		bool ReadBits(uint32 Count, uint32& Out)
		{
			if (Count > 32 || BitPosition + Count > static_cast<uint64>(Data.Num()) * 8)
			{
				return false;
			}
			Out = 0;
			for (uint32 Index = 0; Index < Count; ++Index)
			{
				Out = (Out << 1) | ((Data[BitPosition / 8] >> (7 - BitPosition % 8)) & 1);
				++BitPosition;
			}
			return true;
		}

		bool ReadUE(uint32& Out)
		{
			uint32 LeadingZeroBits = 0;
			bool Bit = false;
			while (true)
			{
				if (!ReadBit(Bit))
				{
					return false;
				}
				if (Bit)
				{
					break;
				}
				if (++LeadingZeroBits > 31)
				{
					return false;
				}
			}

			uint32 Suffix = 0;
			if (LeadingZeroBits > 0 && !ReadBits(LeadingZeroBits, Suffix))
			{
				return false;
			}
			Out = ((1u << LeadingZeroBits) - 1) + Suffix;
			return true;
		}

		bool ReadSE(int32& Out)
		{
			uint32 CodeNum = 0;
			if (!ReadUE(CodeNum))
			{
				return false;
			}
			Out = (CodeNum & 1) != 0 ? static_cast<int32>((CodeNum + 1) / 2) : -static_cast<int32>(CodeNum / 2);
			return true;
		}

		uint64 GetBitPosition() const
		{
			return BitPosition;
		}

	private:
		const TArray<uint8>& Data;
		uint64 BitPosition = 0;
	};

	class FBitWriter
	{
	public:
		void WriteBit(bool Value)
		{
			if (BitPosition % 8 == 0)
			{
				Data.Add(0);
			}
			if (Value)
			{
				Data.Last() |= static_cast<uint8>(1u << (7 - BitPosition % 8));
			}
			++BitPosition;
		}

		void WriteUE(uint32 Value)
		{
			const uint32 CodeNum = Value + 1;
			uint32 NumBits = 0;
			for (uint32 Remaining = CodeNum; Remaining > 0; Remaining >>= 1)
			{
				++NumBits;
			}
			for (uint32 Index = 1; Index < NumBits; ++Index)
			{
				WriteBit(false);
			}
			for (int32 Index = static_cast<int32>(NumBits) - 1; Index >= 0; --Index)
			{
				WriteBit(((CodeNum >> Index) & 1) != 0);
			}
		}

		const TArray<uint8>& GetData() const
		{
			return Data;
		}

	private:
		TArray<uint8> Data;
		uint64 BitPosition = 0;
	};

	bool SkipScalingList(FBitReader& Reader, uint32 Size)
	{
		int32 LastScale = 8;
		int32 NextScale = 8;
		for (uint32 Index = 0; Index < Size; ++Index)
		{
			if (NextScale != 0)
			{
				int32 DeltaScale = 0;
				if (!Reader.ReadSE(DeltaScale))
				{
					return false;
				}
				NextScale = (LastScale + DeltaScale + 256) % 256;
			}
			LastScale = NextScale == 0 ? LastScale : NextScale;
		}
		return true;
	}

	bool FindVUIFlag(const TArray<uint8>& RBSP, uint64& OutBitPosition, uint32& OutMaxNumRefFrames, bool& OutVUIAlreadyPresent)
	{
		FBitReader Reader(RBSP);
		uint32 ProfileIdc = 0;
		uint32 Ignored = 0;
		if (!Reader.ReadBits(8, ProfileIdc) || !Reader.ReadBits(8, Ignored) || !Reader.ReadBits(8, Ignored) || !Reader.ReadUE(Ignored))
		{
			return false;
		}

		if (ProfileIdc == 100 || ProfileIdc == 110 || ProfileIdc == 122 || ProfileIdc == 244 ||
			ProfileIdc == 44 || ProfileIdc == 83 || ProfileIdc == 86 || ProfileIdc == 118 ||
			ProfileIdc == 128 || ProfileIdc == 138 || ProfileIdc == 139 || ProfileIdc == 134 || ProfileIdc == 135)
		{
			uint32 ChromaFormatIdc = 0;
			if (!Reader.ReadUE(ChromaFormatIdc))
			{
				return false;
			}
			bool Flag = false;
			if (ChromaFormatIdc == 3 && !Reader.ReadBit(Flag))
			{
				return false;
			}
			if (!Reader.ReadUE(Ignored) || !Reader.ReadUE(Ignored) || !Reader.ReadBit(Flag) || !Reader.ReadBit(Flag))
			{
				return false;
			}
			if (Flag)
			{
				const uint32 ScalingListCount = ChromaFormatIdc == 3 ? 12 : 8;
				for (uint32 Index = 0; Index < ScalingListCount; ++Index)
				{
					if (!Reader.ReadBit(Flag))
					{
						return false;
					}
					if (Flag && !SkipScalingList(Reader, Index < 6 ? 16 : 64))
					{
						return false;
					}
				}
			}
		}

		uint32 PicOrderCntType = 0;
		if (!Reader.ReadUE(Ignored) || !Reader.ReadUE(PicOrderCntType))
		{
			return false;
		}
		if (PicOrderCntType == 0)
		{
			if (!Reader.ReadUE(Ignored))
			{
				return false;
			}
		}
		else if (PicOrderCntType == 1)
		{
			bool Flag = false;
			int32 SignedIgnored = 0;
			uint32 CycleLength = 0;
			if (!Reader.ReadBit(Flag) || !Reader.ReadSE(SignedIgnored) || !Reader.ReadSE(SignedIgnored) || !Reader.ReadUE(CycleLength))
			{
				return false;
			}
			for (uint32 Index = 0; Index < CycleLength; ++Index)
			{
				if (!Reader.ReadSE(SignedIgnored))
				{
					return false;
				}
			}
		}

		bool Flag = false;
		if (!Reader.ReadUE(OutMaxNumRefFrames) || !Reader.ReadBit(Flag) || !Reader.ReadUE(Ignored) || !Reader.ReadUE(Ignored) || !Reader.ReadBit(Flag))
		{
			return false;
		}
		if (!Flag && !Reader.ReadBit(Flag))
		{
			return false;
		}
		if (!Reader.ReadBit(Flag) || !Reader.ReadBit(Flag))
		{
			return false;
		}
		if (Flag)
		{
			for (uint32 Index = 0; Index < 4; ++Index)
			{
				if (!Reader.ReadUE(Ignored))
				{
					return false;
				}
			}
		}

		OutBitPosition = Reader.GetBitPosition();
		return Reader.ReadBit(OutVUIAlreadyPresent);
	}

	TArray<uint8> EBSPToRBSP(const uint8* Data, int32 Size)
	{
		TArray<uint8> Result;
		Result.Reserve(Size);
		for (int32 Index = 0; Index < Size; ++Index)
		{
			if (Index >= 2 && Index + 1 < Size
				&& Data[Index] == 3 && Data[Index - 1] == 0 && Data[Index - 2] == 0
				&& Data[Index + 1] <= 3)
			{
				continue;
			}
			Result.Add(Data[Index]);
		}
		return Result;
	}

	TArray<uint8> RBSPToEBSP(const TArray<uint8>& RBSP)
	{
		TArray<uint8> Result;
		Result.Reserve(RBSP.Num() + RBSP.Num() / 16);
		for (uint8 Byte : RBSP)
		{
			if (Result.Num() >= 2 && Result[Result.Num() - 1] == 0 && Result[Result.Num() - 2] == 0 && Byte <= 3)
			{
				Result.Add(3);
			}
			Result.Add(Byte);
		}
		return Result;
	}

	bool RewriteSPS(const uint8* Data, int32 Size, TArray<uint8>& Out)
	{
		if (Size < 2 || (Data[0] & 0x1f) != 7)
		{
			return false;
		}

		const TArray<uint8> RBSP = EBSPToRBSP(Data + 1, Size - 1);
		uint64 VUIFlagPosition = 0;
		uint32 MaxNumRefFrames = 0;
		bool bVUIAlreadyPresent = false;
		if (!FindVUIFlag(RBSP, VUIFlagPosition, MaxNumRefFrames, bVUIAlreadyPresent) || bVUIAlreadyPresent)
		{
			return false;
		}

		FBitWriter Writer;
		for (uint64 BitIndex = 0; BitIndex < VUIFlagPosition; ++BitIndex)
		{
			Writer.WriteBit(((RBSP[BitIndex / 8] >> (7 - BitIndex % 8)) & 1) != 0);
		}

		Writer.WriteBit(true);  // vui_parameters_present_flag
		Writer.WriteBit(false); // aspect_ratio_info_present_flag
		Writer.WriteBit(false); // overscan_info_present_flag
		Writer.WriteBit(false); // video_signal_type_present_flag
		Writer.WriteBit(false); // chroma_loc_info_present_flag
		Writer.WriteBit(false); // timing_info_present_flag
		Writer.WriteBit(false); // nal_hrd_parameters_present_flag
		Writer.WriteBit(false); // vcl_hrd_parameters_present_flag
		Writer.WriteBit(false); // pic_struct_present_flag
		Writer.WriteBit(true);  // bitstream_restriction_flag
		Writer.WriteBit(true);  // motion_vectors_over_pic_boundaries_flag
		Writer.WriteUE(0);      // max_bytes_per_pic_denom
		Writer.WriteUE(0);      // max_bits_per_mb_denom
		Writer.WriteUE(16);     // log2_max_mv_length_horizontal
		Writer.WriteUE(16);     // log2_max_mv_length_vertical
		Writer.WriteUE(0);      // max_num_reorder_frames
		Writer.WriteUE(FMath::Max(1u, MaxNumRefFrames)); // max_dec_frame_buffering
		Writer.WriteBit(true);  // rbsp_stop_one_bit; zero padding is already present

		Out.Reset();
		Out.Add(Data[0]);
		Out.Append(RBSPToEBSP(Writer.GetData()));
		return true;
	}

	int32 FindStartCode(const TArray<uint8>& Data, int32 From, int32& OutPrefixSize)
	{
		for (int32 Index = From; Index + 3 <= Data.Num(); ++Index)
		{
			if (Data[Index] != 0 || Data[Index + 1] != 0)
			{
				continue;
			}
			if (Data[Index + 2] == 1)
			{
				OutPrefixSize = 3;
				return Index;
			}
			if (Index + 3 < Data.Num() && Data[Index + 2] == 0 && Data[Index + 3] == 1)
			{
				OutPrefixSize = 4;
				return Index;
			}
		}
		return INDEX_NONE;
	}
}

bool TempoH264::AddLowLatencyVUI(TArray<uint8>& AnnexBAccessUnit)
{
	TArray<uint8> Result;
	int32 Cursor = 0;
	bool bRewritten = false;
	while (Cursor < AnnexBAccessUnit.Num())
	{
		int32 PrefixSize = 0;
		const int32 StartCode = FindStartCode(AnnexBAccessUnit, Cursor, PrefixSize);
		if (StartCode == INDEX_NONE)
		{
			Result.Append(AnnexBAccessUnit.GetData() + Cursor, AnnexBAccessUnit.Num() - Cursor);
			break;
		}

		const int32 NALStart = StartCode + PrefixSize;
		int32 NextPrefixSize = 0;
		const int32 NextStartCode = FindStartCode(AnnexBAccessUnit, NALStart, NextPrefixSize);
		const int32 NALEnd = NextStartCode == INDEX_NONE ? AnnexBAccessUnit.Num() : NextStartCode;
		Result.Append(AnnexBAccessUnit.GetData() + Cursor, NALStart - Cursor);

		TArray<uint8> RewrittenSPS;
		if (RewriteSPS(AnnexBAccessUnit.GetData() + NALStart, NALEnd - NALStart, RewrittenSPS))
		{
			Result.Append(RewrittenSPS);
			bRewritten = true;
		}
		else
		{
			Result.Append(AnnexBAccessUnit.GetData() + NALStart, NALEnd - NALStart);
		}
		Cursor = NALEnd;
	}

	if (bRewritten)
	{
		AnnexBAccessUnit = MoveTemp(Result);
	}
	return bRewritten;
}
