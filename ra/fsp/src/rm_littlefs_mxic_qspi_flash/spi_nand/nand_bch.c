/**
  ******************************************************************************
  * @file    nand_bch.c
  * @author  MX Application Team
  * @date    10-July-2018
  * @brief   This file provides software BCH ECC function.
  ******************************************************************************
  */

#include "../spi_nand/nand_bch.h"

#include "malloc.h"

#include "../spi_nand/bch.h"
#include "../spi_nand/bitops.h"
#include "../spi_nand/mx_define.h"
#include "../spi_nand/spi_nand_cmd.h"

/**
 * struct nand_bch_control - private NAND BCH control structure
 * @bch:       BCH control structure
 * @ecclayout: private ecc layout for this BCH configuration
 * @errloc:    error location array
 * @eccmask:   XOR ecc mask, allows erased pages to be decoded as valid
 */
struct nand_bch_control {
	struct bch_control   *bch;
	struct NandEccLayout ecclayout;
	unsigned int         *errloc;
	unsigned char        *eccmask;
};

/**
 * nand_bch_calculate_ecc - [NAND Interface] Calculate ECC for data block
 * @mtd:	MTD block structure
 * @buf:	input buffer with raw data
 * @code:	output buffer with ECC
 */
int nand_bch_calculate_ecc(MxChip *Mxic, const unsigned char *buf,
			   unsigned char *code)
{
	struct nand_bch_control *nbc = Mxic->EccCtrl.Priv;
	unsigned int i;

	memset(code, 0, Mxic->EccCtrl.EccBytes);
	encode_bch(nbc->bch, buf, Mxic->EccCtrl.EccSize, code);

	/* apply mask so that an erased page is a valid codeword */
	for (i = 0; i < Mxic->EccCtrl.EccBytes; i++)
		code[i] ^= nbc->eccmask[i];

	return 0;
}

/**
 * nand_bch_correct_data - [NAND Interface] Detect and correct bit error(s)
 * @mtd:	MTD block structure
 * @buf:	raw data read from the chip
 * @read_ecc:	ECC from the chip
 * @calc_ecc:	the ECC calculated from raw data
 *
 * Detect and correct bit errors for a data byte block
 */
int nand_bch_correct_data(MxChip *Mxic, unsigned char *buf,
			  unsigned char *read_ecc, unsigned char *calc_ecc)
{
	struct nand_bch_control *nbc = Mxic->EccCtrl.Priv;
	unsigned int *errloc = nbc->errloc;
	int i, count;

	count = decode_bch(nbc->bch, NULL, Mxic->EccCtrl.EccSize, read_ecc, calc_ecc,
			   NULL, errloc);
	if (count > 0) {
		for (i = 0; i < count; i++) {
			if (errloc[i] < (Mxic->EccCtrl.EccSize*8))
				/* error is located in data, correct it */
				buf[errloc[i] >> 3] ^= (1 << (errloc[i] & 7));
			/* else error in ecc, no action needed */

		}
	} else if (count < 0) {
		count = -1;
	}
	return count;
}

/**
 * nand_bch_init - [NAND Interface] Initialize NAND BCH error correction
 * @mtd:	MTD block structure
 * @eccsize:	ecc block size in bytes
 * @EccBytes:	ecc Length in bytes
 * @ecclayout:	output default layout
 *
 * Returns:
 *  a pointer to a new NAND BCH control structure, or NULL upon failure
 *
 * Initialize NAND BCH error correction. Parameters @eccsize and @EccBytes
 * are used to compute BCH parameters m (Galois field order) and t (error
 * correction capability). @EccBytes should be equal to the number of bytes
 * required to store m*t bits, where m is such that 2^m-1 > @eccsize*8.
 *
 * Example: to configure 4 bit correction per 512 bytes, you should pass
 * @eccsize = 512  (thus, m=13 is the smallest integer such that 2^m-1 > 512*8)
 * @EccBytes = 7   (7 bytes are required to store m*t = 13*4 = 52 bits)
 */
struct nand_bch_control *
nand_bch_init(MxChip *Mxic, unsigned int eccsize, unsigned int EccBytes,
	      struct NandEccLayout **ecclayout)
{
	unsigned int m, t, eccsteps, i;
	struct NandEccLayout *layout;
	struct nand_bch_control *nbc = NULL;
	unsigned char *erased_page;


	m = fls(1+8*eccsize);
	t = (EccBytes*8)/m;

	nbc = malloc(sizeof(*nbc));
	if (!nbc)
		goto fail;

	memset(nbc, 0, sizeof(*nbc));

	nbc->bch = init_bch(m, t, 0);
	if (!nbc->bch)
		goto fail;

	/* verify that EccBytes has the expected value */
	if (nbc->bch->ecc_bytes != EccBytes) {
		Mx_printf( "invalid EccBytes %u, should be %u\n",
		       EccBytes, nbc->bch->ecc_bytes);
		goto fail;
	}

	eccsteps = Mxic->EccCtrl.EccSteps;

	/* if no ecc placement scheme was provided, build one */
	if (!*ecclayout) {

		/* handle large page devices only */
		if (Mxic->OobSize < 64) {
			Mx_printf( "must provide an oob scheme for "
			       "oobsize %d\n", Mxic->OobSize);
			goto fail;
		}

		layout = &nbc->ecclayout;
		layout->EccBytes = eccsteps*EccBytes;

		/* reserve 2 bytes for bad block marker */
		if (layout->EccBytes+2 > Mxic->OobSize) {
			Mx_printf( "no suitable oob scheme available "
			       "for oobsize %d EccBytes %u\n", Mxic->OobSize,
			       EccBytes);
			goto fail;
		}
		/* put ecc bytes at oob tail */
		for (i = 0; i < layout->EccBytes; i++)
			layout->EccPos[i] = Mxic->OobSize-layout->EccBytes+i;

		layout->OobFree[0].Offset = 2;
		layout->OobFree[0].Length = Mxic->OobSize-2-layout->EccBytes;

		*ecclayout = layout;
	}

	/* sanity checks */
	if (8*(eccsize+EccBytes) >= (1 << m)) {
		goto fail;
	}
	if ((*ecclayout)->EccBytes != (eccsteps*EccBytes)) {
		Mx_printf( "invalid ecc layout\n");
		goto fail;
	}

	nbc->eccmask = malloc(EccBytes);
	nbc->errloc = malloc(t*sizeof(*nbc->errloc));
	if (!nbc->eccmask || !nbc->errloc)
		goto fail;
	/*
	 * compute and store the inverted ecc of an erased ecc block
	 */
	erased_page = malloc(eccsize);
	if (!erased_page)
		goto fail;

	memset(erased_page, 0xff, eccsize);
	memset(nbc->eccmask, 0, EccBytes);
	encode_bch(nbc->bch, erased_page, eccsize, nbc->eccmask);
	free(erased_page);

	for (i = 0; i < EccBytes; i++)
		nbc->eccmask[i] ^= 0xff;

	return nbc;
fail:
	nand_bch_free(nbc);
	return NULL;
}

/**
 * nand_bch_free - [NAND Interface] Release NAND BCH ECC resources
 * @nbc:	NAND BCH control structure
 */
void nand_bch_free(struct nand_bch_control *nbc)
{
	if (nbc) {
		free_bch(nbc->bch);
		free(nbc->errloc);
		free(nbc->eccmask);
		free(nbc);
	}
}
