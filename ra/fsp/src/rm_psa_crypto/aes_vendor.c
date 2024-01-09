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

#include "common.h"


#if defined(MBEDTLS_PSA_CRYPTO_ACCEL_DRV_C)

 #include "aes_vendor.h"

/* Auto-generated values depending on which drivers are registered.
 * ID 0 is reserved for unallocated operations.
 * ID 1 is reserved for the Mbed TLS software driver. */
#define PSA_CRYPTO_MBED_TLS_DRIVER_ID (1)

/** Determine standard key size in bits for a vendor type key bit size associated with an elliptic curve.
 *  THis function is invoked during key generation and the user specifies the bits which will be the
 * standard bit size (SIZE_AES_128BIT_KEYLEN_BITS), but the wrapped key has a different size (SIZE_AES_128BIT_KEYLEN_BYTES_WRAPPED)
 * so that is returned.
 * This function is also invoked during key import. Since the wrapped key to be imported has a non-standard size,
 * when this key is imported, the bit length with be non standard. This function will account for that case as well.
 *
 * \param[in] type     Key type
 * \param[in] bits     Vendor key size in bits
 * \param[out] raw     Equivalent standard key size in bits
 */

psa_status_t prepare_raw_data_slot_vendor (psa_key_type_t type, size_t bits, struct key_data * key)
{
    (void) type;
    (void) bits;
    (void) key;
    uint32_t   * p_temp_keydata;
    psa_status_t ret = PSA_SUCCESS;

 #if defined(MBEDTLS_AES_ALT)
  #if defined(MBEDTLS_CHECK_PARAMS)
    if (bits % 8 != 0)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }
  #endif                               // defined(MBEDTLS_CHECK_PARAMS)

    /* Check if the key is of AES type */
    if (PSA_KEY_TYPE_IS_AES(type))
    {
        /* Check that the bit size is acceptable for the key type */
        switch (bits)
        {
            case SIZE_AES_128BIT_KEYLEN_BITS:
            case SIZE_AES_128BIT_KEYLEN_BITS_WRAPPED:
            {
                key->bytes = SIZE_AES_128BIT_KEYLEN_BYTES_WRAPPED;
                break;
            }

            case SIZE_AES_192BIT_KEYLEN_BITS:
            case SIZE_AES_192BIT_KEYLEN_BITS_WRAPPED:
            {
                key->bytes = SIZE_AES_192BIT_KEYLEN_BYTES_WRAPPED;
                break;
            }

            case SIZE_AES_256BIT_KEYLEN_BITS:
            case SIZE_AES_256BIT_KEYLEN_BITS_WRAPPED:
            {
                key->bytes = SIZE_AES_256BIT_KEYLEN_BYTES_WRAPPED;
                break;
            }

            default:
            {
                ret = PSA_ERROR_NOT_SUPPORTED;
            }
        }
    }
    else
 #endif
    {
        ret = PSA_ERROR_NOT_SUPPORTED;
    }

    if (!ret)
    {
/* Allocate memory for the key */
        p_temp_keydata = mbedtls_calloc((key->bytes / 4), sizeof(uint32_t));
        key->data      = (uint8_t *) p_temp_keydata;
        if (key->data == NULL)
        {
            key->bytes = 0;

            ret = PSA_ERROR_INSUFFICIENT_MEMORY;
        }
    }

    return ret;
}

/*************crypto_accel_driver.h implementations follow***************************/

psa_status_t psa_generate_symmetric_vendor (psa_key_type_t type, size_t bits, uint8_t * output, size_t output_size)
{
    fsp_err_t err = FSP_SUCCESS;
    int       ret = PSA_SUCCESS;

    (void) type;
    (void) bits;
    (void) output;
    (void) output_size;

 #if defined(MBEDTLS_AES_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT)))

    /* Check if the key is of AES type */
    if (PSA_KEY_TYPE_IS_AES(type))
    {
        /* Check that the bit size is acceptable for the key type */
        switch (bits)
        {
            case SIZE_AES_128BIT_KEYLEN_BITS:
            {
                if (output_size != SIZE_AES_128BIT_KEYLEN_BYTES_WRAPPED)
                {
                    ret = PSA_ERROR_BUFFER_TOO_SMALL;
                }

                if (!ret)
                {
                    err = HW_SCE_GenerateAes128RandomKeyIndexSub((uint32_t *) output);
                }

                break;
            }

  #if BSP_FEATURE_CRYPTO_HAS_SCE9 || BSP_FEATURE_CRYPTO_HAS_SCE7 || BSP_FEATURE_CRYPTO_HAS_RSIP7
            case SIZE_AES_192BIT_KEYLEN_BITS:
            {
                if (output_size != SIZE_AES_192BIT_KEYLEN_BYTES_WRAPPED)
                {
                    ret = PSA_ERROR_BUFFER_TOO_SMALL;
                }

                if (!ret)
                {
                    err = HW_SCE_GenerateAes192RandomKeyIndexSub((uint32_t *) output);
                }

                break;
            }
  #endif
            case SIZE_AES_256BIT_KEYLEN_BITS:
            {
                if (output_size != SIZE_AES_256BIT_KEYLEN_BYTES_WRAPPED)
                {
                    ret = PSA_ERROR_BUFFER_TOO_SMALL;
                }

                if (!ret)
                {
                    err = HW_SCE_GenerateAes256RandomKeyIndexSub((uint32_t *) output);
                }

                break;
            }

            default:
            {
                ret = PSA_ERROR_NOT_SUPPORTED;
            }
        }
    }
    else
 #endif                                /* defined(MBEDTLS_AES_ALT) && ((PSA_CRYPTO_IS_WRAPPED_SUPPORT_REQUIRED(PSA_CRYPTO_CFG_AES_FORMAT))) */
    {
        ret = PSA_ERROR_NOT_SUPPORTED;
    }

    if ((FSP_SUCCESS != err) && (ret == PSA_SUCCESS))
    {
        ret = PSA_ERROR_HARDWARE_FAILURE;
    }

    return ret;
}

void psa_aead_setup_vendor (void * ctx)
{
    (void) ctx;
#if defined(MBEDTLS_AES_ALT)	
    mbedtls_aes_context *context = (mbedtls_aes_context *)ctx;
    context->vendor_ctx = (bool *) true;
#endif	
}

#endif                                 /* MBEDTLS_PSA_CRYPTO_ACCEL_DRV_C */
