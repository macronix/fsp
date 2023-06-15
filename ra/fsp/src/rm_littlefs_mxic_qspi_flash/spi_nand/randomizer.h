/**
  ******************************************************************************
  * @file    randomizer.h
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   Header for randomizer.c module.
  ******************************************************************************
  */

#ifndef RANDOMIZER_H_
#define RANDOMIZER_H_

void RandomizerEncodeDecode(u8 PageAddr, u8 ChunkNum, u8 *Input, u8 *Output, u32 DataSize);

#endif /* RANDOMIZER_H_ */
