/***********************************************************************************************************************
 * Copyright [2020-2023] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics America Inc. and may only be used with products
 * of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.  Renesas products are
 * sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for the selection and use
 * of Renesas products and Renesas assumes no liability.  No license, express or implied, to any intellectual property
 * right is granted by Renesas. This software is protected under all applicable laws, including copyright laws. Renesas
 * reserves the right to change or discontinue this software and/or this documentation. THE SOFTWARE AND DOCUMENTATION
 * IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND TO THE FULLEST EXTENT
 * PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY, INCLUDING WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE SOFTWARE OR
 * DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.  TO THE MAXIMUM
 * EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR DOCUMENTATION
 * (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER, INCLUDING,
 * WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY LOST PROFITS,
 * OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY
 * OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/
#ifndef MBEDTLS_SHA512_ALT_H
 #define MBEDTLS_SHA512_ALT_H

 #include "common.h"
 #include "r_sce_if.h"

 #include <stddef.h>
 #include <stdint.h>

 #ifdef __cplusplus
extern "C" {
 #endif

 #define SIZE_MBEDTLS_SHA512_PROCESS_BUFFER_BYTES    128U

/**
 * \brief          The SHA-512 context structure.
 *
 *                 The structure is used both for SHA-512 and for SHA-224
 *                 checksum calculations. The choice between these two is
 *                 made in the call to mbedtls_sha512_starts_ret().
 */
typedef struct mbedtls_sha512_context
{
    uint64_t      total[2];                                              /*!< The number of Bytes processed.  */
    uint64_t      state[8];                                              /*!< The intermediate digest state.  */
    unsigned char buffer[SIZE_MBEDTLS_SHA512_PROCESS_BUFFER_BYTES];      /*!< The data block being processed. */
    int           is384;                                                 /*!< Determines which function to use:
                                                                          * 0: Use SHA-512, or 1: Use SHA-384. */
    uint32_t       used;                                                 // Used to indicate if the final block has user data or only padding
    sce_hash_cmd_t sce_operation_state;
 #if BSP_FEATURE_CRYPTO_HAS_RSIP7
    uint32_t      rsip_internal_state[20];                               // RSIP specific state array
    unsigned char rsip_buffer[SIZE_MBEDTLS_SHA512_PROCESS_BUFFER_BYTES]; /*!< buffered data for RSIP procedure. */
    uint32_t      use_rsip_buffer;                                       // Used to indicate if the rsip_buffer is to be used or not
    uint32_t      rsip_buffer_processed;                                 // Used to indicate if the rsip_buffer data is processed or not
 #endif
} mbedtls_sha512_context;

int mbedtls_internal_sha512_process_ext(mbedtls_sha512_context * ctx,
                                        const unsigned char      data[SIZE_MBEDTLS_SHA512_PROCESS_BUFFER_BYTES],
                                        uint32_t                 len);

 #ifdef __cplusplus
}
 #endif

#endif                                 /* mbedtls_sha512.h */
