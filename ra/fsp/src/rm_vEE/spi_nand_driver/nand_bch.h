/**
  ******************************************************************************
  * @file    nand_bch.h
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   Header for nand_bch.c module.
  ******************************************************************************
  */

#ifndef NAND_BCH_H__
#define NAND_BCH_H__

#include "../spi_nand_driver/spi_nand_cmd.h"

struct mtd_info;
struct nand_bch_control;

#define CONFIG_NAND_ECC_BCH

#if defined(CONFIG_NAND_ECC_BCH)

static inline int mtd_nand_has_bch(void) { return 1; }

/*
 * Calculate BCH ecc code
 */
int nand_bch_calculate_ecc(MxChip *Mxic, const unsigned char *dat,
		unsigned char *ecc_code);

/*
 * Detect and correct bit errors
 */
int nand_bch_correct_data(MxChip *Mxic, unsigned char *dat, unsigned char *read_ecc,
		unsigned char *calc_ecc);
/*
 * Initialize BCH encoder/decoder
 */
struct nand_bch_control *
nand_bch_init(MxChip *Mxic, unsigned int eccsize,
	      unsigned int eccbytes, struct NandEccLayout **ecclayout);
/*
 * Release BCH encoder/decoder resources
 */
void nand_bch_free(struct nand_bch_control *nbc);

#else /* !CONFIG_NAND_ECC_BCH */

static inline int mtd_nand_has_bch(void) { return 0; }

static inline int
nand_bch_calculate_ecc(struct mtd_info *mtd, const unsigned char *dat,
		unsigned char *ecc_code)
{
	return -1;
}

static inline int
nand_bch_correct_data(struct mtd_info *mtd, unsigned char *buf,
		      unsigned char *read_ecc, unsigned char *calc_ecc)
{
	return -1;
}

static inline struct nand_bch_control *
nand_bch_init(MxChip *Mxic, unsigned int eccsize,
	      unsigned int eccbytes, struct NandEccLayout **ecclayout)
{
	return NULL;
}

static inline void nand_bch_free(struct nand_bch_control *nbc) {}

#endif /* CONFIG_NAND_ECC_BCH */

#endif /* __MTD_NAND_BCH_H__ */
