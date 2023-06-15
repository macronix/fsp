/**
  ******************************************************************************
  * @file    randomizer.c
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   This file provides data randomizer encode and decode.
  ******************************************************************************
  */

#include "../spi_nand_driver/mx_define.h"

u16 RanInitSeedReg[16]={0x11a3,0x20b3,0x30a4,0x4434,0x55a2,0x6176,0x7ee7,0x828a,0x1939,0x4a7a,0xb2ba,0xc0ac,0x2dd8,0xe2e7,0x1565,};
u32 RanPolyReg[4] = {0x8003,0x8453,0x8113,0x8723};

typedef union INPUT_DATA
{
	u32 InputData_32;
	struct
	{
		unsigned int Bit0 : 1;
		unsigned int Bit1 : 1;
		unsigned int Bit2 : 1;
		unsigned int Bit3 : 1;
		unsigned int Bit4 : 1;
		unsigned int Bit5 : 1;
		unsigned int Bit6 : 1;
		unsigned int Bit7 : 1;
		unsigned int Bit8 : 1;
		unsigned int Bit9 : 1;
		unsigned int Bit10 : 1;
		unsigned int Bit11 : 1;
		unsigned int Bit12 : 1;
		unsigned int Bit13 : 1;
		unsigned int Bit14 : 1;
		unsigned int Bit15 : 1;
		unsigned int Bit16 : 1;
		unsigned int Bit17 : 1;
		unsigned int Bit18 : 1;
		unsigned int Bit19 : 1;
		unsigned int Bit20 : 1;
		unsigned int Bit21 : 1;
		unsigned int Bit22 : 1;
		unsigned int Bit23 : 1;
		unsigned int Bit24 : 1;
		unsigned int Bit25 : 1;
		unsigned int Bit26 : 1;
		unsigned int Bit27 : 1;
		unsigned int Bit28 : 1;
		unsigned int Bit29 : 1;
		unsigned int Bit30 : 1;
		unsigned int Bit31 : 1;
	}Bit;
}InputData;

typedef union
{
	u32 Lfsr_16;
	struct
	{
		unsigned int Bit0 : 1;
		unsigned int Bit1 : 1;
		unsigned int Bit2 : 1;
		unsigned int Bit3 : 1;
		unsigned int Bit4 : 1;
		unsigned int Bit5 : 1;
		unsigned int Bit6 : 1;
		unsigned int Bit7 : 1;
		unsigned int Bit8 : 1;
		unsigned int Bit9 : 1;
		unsigned int Bit10 : 1;
		unsigned int Bit11 : 1;
		unsigned int Bit12 : 1;
		unsigned int Bit13 : 1;
		unsigned int Bit14 : 1;
		unsigned int Bit15 : 1;
	}Bit;
}Lfsr;

u16 GetSeed(u8 PageAddr, u8 ChunkNum)
{
	u8 SelectSeedNum;
	u16 Seed;
	/*
	 * Select seeds according to the register PAGE_ADDR (PA[3:0]) and chunk number (CN).
	 */
	SelectSeedNum = ((PageAddr & 0x0F) + ChunkNum) % 16;

	/*
	 * Rotate seeds by the register PAGE_ADDR (PA[7:4])
	 */
	Seed = RanInitSeedReg[SelectSeedNum] >> ((PageAddr & 0xF0) >> 4);

	return Seed;
}

int RandomGenerate(u16 LfsrCur,u16 *LfsrNxt, u32 RanPolyReg, u32 BitCount)
{
	u8 j;
	u32 RandomBit = 0;
	Lfsr Lfsr_Cur,Lfsr_Nxt;

	Lfsr_Cur.Lfsr_16 = LfsrCur;

	for(j = 0; j < BitCount; j++)
	{
		Lfsr_Nxt.Bit.Bit0  = Lfsr_Cur.Bit.Bit15;
		Lfsr_Nxt.Bit.Bit2  = ((RanPolyReg >>  2) & 0x01) ? Lfsr_Cur.Bit.Bit1  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit1 ;
		Lfsr_Nxt.Bit.Bit3  = ((RanPolyReg >>  3) & 0x01) ? Lfsr_Cur.Bit.Bit2  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit2 ;
		Lfsr_Nxt.Bit.Bit4  = ((RanPolyReg >>  4) & 0x01) ? Lfsr_Cur.Bit.Bit3  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit3 ;
		Lfsr_Nxt.Bit.Bit5  = ((RanPolyReg >>  5) & 0x01) ? Lfsr_Cur.Bit.Bit4  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit4 ;
		Lfsr_Nxt.Bit.Bit6  = ((RanPolyReg >>  6) & 0x01) ? Lfsr_Cur.Bit.Bit5  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit5 ;
		Lfsr_Nxt.Bit.Bit7  = ((RanPolyReg >>  7) & 0x01) ? Lfsr_Cur.Bit.Bit6  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit6 ;
		Lfsr_Nxt.Bit.Bit8  = ((RanPolyReg >>  8) & 0x01) ? Lfsr_Cur.Bit.Bit7  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit7 ;
		Lfsr_Nxt.Bit.Bit9  = ((RanPolyReg >>  9) & 0x01) ? Lfsr_Cur.Bit.Bit8  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit8 ;
		Lfsr_Nxt.Bit.Bit10 = ((RanPolyReg >> 10) & 0x01) ? Lfsr_Cur.Bit.Bit9  ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit9 ;
		Lfsr_Nxt.Bit.Bit11 = ((RanPolyReg >> 11) & 0x01) ? Lfsr_Cur.Bit.Bit10 ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit10;
		Lfsr_Nxt.Bit.Bit12 = ((RanPolyReg >> 12) & 0x01) ? Lfsr_Cur.Bit.Bit11 ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit11;
		Lfsr_Nxt.Bit.Bit13 = ((RanPolyReg >> 13) & 0x01) ? Lfsr_Cur.Bit.Bit12 ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit12;
		Lfsr_Nxt.Bit.Bit14 = ((RanPolyReg >> 14) & 0x01) ? Lfsr_Cur.Bit.Bit13 ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit13;
		Lfsr_Nxt.Bit.Bit15 = ((RanPolyReg >> 15) & 0x01) ? Lfsr_Cur.Bit.Bit14 ^ Lfsr_Cur.Bit.Bit15 : Lfsr_Cur.Bit.Bit14;

		RandomBit |= ((Lfsr_Nxt.Bit.Bit15 & 0x01) ? 1 : 0) << j;
		Lfsr_Cur.Lfsr_16 = Lfsr_Nxt.Lfsr_16;
	}
	*LfsrNxt = Lfsr_Cur.Lfsr_16;

	return RandomBit ;
}

void RandomizerEncodeDecode(u8 PageAddr, u8 ChunkNum, u8 *Input, u8 *Output, u32 DataSize)
{
	InputData Input_Data;
	u16 Seed;
	u16 LfsrCur;
	u16 LfsrNxt;
	u32 RandomBit_0, RandomBit_1, RandomBit_2, RandomBit_3;
	u32 i=0,j=0;
	u32 DataSizeTmp;

	Seed = GetSeed(PageAddr, ChunkNum);
	LfsrCur = Seed;
u32 kk=((DataSize % 128 == 0) ?  DataSize/128 : (DataSize/128 + 1));

	for(j = 0; j < kk; j++)
	{
		if(DataSize >= 128 )
			DataSizeTmp = 128;
		else
			DataSizeTmp = DataSize;

		RandomBit_0 = RandomGenerate(LfsrCur, &LfsrNxt, RanPolyReg[0], DataSizeTmp/4);
		RandomBit_1 = RandomGenerate(LfsrCur, &LfsrNxt, RanPolyReg[1], DataSizeTmp/4);
		RandomBit_2 = RandomGenerate(LfsrCur, &LfsrNxt, RanPolyReg[2], DataSizeTmp/4);
		RandomBit_3 = RandomGenerate(LfsrCur, &LfsrNxt, RanPolyReg[3], DataSizeTmp/4);

		LfsrCur = LfsrNxt;
		for(i = 0; i < DataSizeTmp/4; i++)
		{
			Input_Data.InputData_32 = (Input[4*i+128*j+3] << 24) | (Input[4*i+128*j+2] << 16)
					                 | (Input[4*i+128*j+1] << 8) | (Input[4*i+128*j+0] << 0);
			Input_Data.Bit.Bit0  ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit4  ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit10 ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit14 ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit19 ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit23 ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit25 ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit29 ^= (RandomBit_0 >> i) & 0x01 ? 1 : 0;

			Input_Data.Bit.Bit1  ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit5  ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit11 ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit15 ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit18 ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit22 ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit24 ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit28 ^= (RandomBit_1 >> i) & 0x01 ? 1 : 0;

			Input_Data.Bit.Bit2  ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit6  ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit8  ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit12 ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit17 ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit21 ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit27 ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit31 ^= (RandomBit_2 >> i) & 0x01 ? 1 : 0;

			Input_Data.Bit.Bit3  ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit7  ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit9  ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit13 ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit16 ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit20 ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit26 ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;
			Input_Data.Bit.Bit30 ^= (RandomBit_3 >> i) & 0x01 ? 1 : 0;

			Output[4*i+128*j+3] = (Input_Data.InputData_32) >> 24;
			Output[4*i+128*j+2] = (Input_Data.InputData_32) >> 16;
			Output[4*i+128*j+1] = (Input_Data.InputData_32) >> 8;
			Output[4*i+128*j+0] = (Input_Data.InputData_32) >> 0;
		}
		DataSize -=128;
	}
}
