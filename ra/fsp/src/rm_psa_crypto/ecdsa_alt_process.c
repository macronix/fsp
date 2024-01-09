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
#include "hw_sce_ra_private.h"
#include "hw_sce_private.h"
#include "hw_sce_ecc_private.h"

#if (defined(MBEDTLS_ECDSA_SIGN_ALT) || defined(MBEDTLS_ECDSA_VERIFY_ALT) || defined(MBEDTLS_ECP_ALT))

fsp_err_t HW_SCE_ECC_256GenerateSign (const uint32_t * InData_CurveType,
                                      const uint32_t * InData_G,
                                      const uint32_t * InData_PrivKey,
                                      const uint32_t * InData_MsgDgst,
                                      uint32_t       * OutData_R,
                                      uint32_t       * OutData_S)
{
    uint32_t      signature[HW_SCE_ECDSA_DATA_BYTE_SIZE / 4U] = {0};
    uint32_t      wrapped_private_key[13U];
    sce_oem_cmd_t key_command;
    const uint32_t * p_domain_param;

    /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P256_PRIVATE;
        p_domain_param = DomainParam_NIST_P256;
    }
    /* Brainpool Curve */
    else if (SCE_ECC_CURVE_TYPE_BRAINPOOL == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P256R1_PRIVATE;
        p_domain_param = DomainParam_Brainpool_256r1;
    }
    /* Koblitz curve */
    else
    {
        key_command = SCE_OEM_CMD_ECC_SECP256K1_PRIVATE;
        p_domain_param = DomainParam_Koblitz_secp256k1;
    }

    /* Install the plaintext private key to get the wrapped key */
    fsp_err_t err = HW_SCE_GenerateOemKeyIndexPrivate(SCE_OEM_KEY_TYPE_PLAIN,
                                                      key_command,
                                                      NULL,
                                                      NULL,
                                                      (const uint8_t *) InData_PrivKey,
                                                      wrapped_private_key);
    if (FSP_SUCCESS == err)
    {
        err = HW_SCE_EcdsaSignatureGenerateSubAdaptor(InData_CurveType,
                                               InData_G,
                                               wrapped_private_key,
                                               InData_MsgDgst,
                                               p_domain_param,
                                               signature);
    }

    if (FSP_SUCCESS == err)
    {
        memcpy(OutData_R, signature, (HW_SCE_ECDSA_DATA_BYTE_SIZE / 2U));
        memcpy(OutData_S, &signature[(HW_SCE_ECDSA_DATA_BYTE_SIZE / 4U) / 2U], (HW_SCE_ECDSA_DATA_BYTE_SIZE / 2U));
    }

    return err;
}

fsp_err_t HW_SCE_ECC_256HrkGenerateSign (const uint32_t * InData_CurveType,
                                         const uint32_t * InData_G,
                                         const uint32_t * InData_KeyIndex,
                                         const uint32_t * InData_MsgDgst,
                                         uint32_t       * OutData_R,
                                         uint32_t       * OutData_S)
{
     const uint32_t * p_domain_param;

    /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        p_domain_param = DomainParam_NIST_P256;
    }
    /* Brainpool Curve */
    else if (SCE_ECC_CURVE_TYPE_BRAINPOOL == *InData_CurveType)
    {
        p_domain_param = DomainParam_Brainpool_256r1;
    }
    /* Koblitz curve */
    else
    {
        p_domain_param = DomainParam_Koblitz_secp256k1;
    }
    uint32_t  signature[(HW_SCE_ECDSA_DATA_BYTE_SIZE / 4U)] = {0};
    fsp_err_t err = HW_SCE_EcdsaSignatureGenerateSubAdaptor(InData_CurveType,
                                                     InData_G,
                                                     InData_KeyIndex,
                                                     InData_MsgDgst,
                                                     p_domain_param,
                                                     signature);
    if (FSP_SUCCESS == err)
    {
        memcpy(OutData_R, signature, (HW_SCE_ECDSA_DATA_BYTE_SIZE / 2U));
        memcpy(OutData_S, &signature[(HW_SCE_ECDSA_DATA_BYTE_SIZE / 4U) / 2U], (HW_SCE_ECDSA_DATA_BYTE_SIZE / 2U));
    }

    return err;
}

fsp_err_t HW_SCE_ECC_384GenerateSign (const uint32_t * InData_CurveType,
                                      const uint32_t * InData_G,
                                      const uint32_t * InData_PrivKey,
                                      const uint32_t * InData_MsgDgst,
                                      uint32_t       * OutData_R,
                                      uint32_t       * OutData_S)
{
    FSP_PARAMETER_NOT_USED(InData_G);
    uint32_t      signature[HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 4U] = {0};
    uint32_t      wrapped_private_key[17U];
    sce_oem_cmd_t key_command;
    const uint32_t * p_domain_param;

    /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P384_PRIVATE;
        p_domain_param = DomainParam_NIST_P384;
    }
    /* Brainpool Curve */
    else if (SCE_ECC_CURVE_TYPE_BRAINPOOL == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P384R1_PRIVATE;
        p_domain_param = DomainParam_Brainpool_384r1;
    }
    /* Koblitz curve unsupported */
    else
    {
        return FSP_ERR_UNSUPPORTED;
    }

    /* Install the plaintext private key to get the wrapped key */
    fsp_err_t err = HW_SCE_GenerateOemKeyIndexPrivate(SCE_OEM_KEY_TYPE_PLAIN,
                                                      key_command,
                                                      NULL,
                                                      NULL,
                                                      (const uint8_t *) InData_PrivKey,
                                                      wrapped_private_key);
    if (FSP_SUCCESS == err)
    {
        err =
            HW_SCE_EcdsaP384SignatureGenerateSubAdaptor((uint32_t *)InData_CurveType, wrapped_private_key, InData_MsgDgst, p_domain_param, signature);
    }

    if (FSP_SUCCESS == err)
    {
        memcpy(OutData_R, signature, (HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 2U));
        memcpy(OutData_S, &signature[(HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 4U) / 2U],
               (HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 2U));
    }

    return err;
}

fsp_err_t HW_SCE_ECC_384HrkGenerateSign (const uint32_t * InData_CurveType,
                                         const uint32_t * InData_G,
                                         const uint32_t * InData_KeyIndex,
                                         const uint32_t * InData_MsgDgst,
                                         uint32_t       * OutData_R,
                                         uint32_t       * OutData_S)
{
    FSP_PARAMETER_NOT_USED(InData_G);
    const uint32_t * p_domain_param;
        /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        p_domain_param = DomainParam_NIST_P384;
    }
    /* Brainpool Curve */
    else if (SCE_ECC_CURVE_TYPE_BRAINPOOL == *InData_CurveType)
    {
        p_domain_param = DomainParam_Brainpool_384r1;
    }
    /* Koblitz curve unsupported */
    else
    {
        return FSP_ERR_UNSUPPORTED;
    }

    uint32_t  signature[HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 4U] = {0};
    fsp_err_t err =
        HW_SCE_EcdsaP384SignatureGenerateSubAdaptor((uint32_t *)InData_CurveType, InData_KeyIndex, InData_MsgDgst, p_domain_param, signature);
    if (FSP_SUCCESS == err)
    {
        memcpy(OutData_R, signature, (HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 2U));
        memcpy(OutData_S, &signature[(HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 4U) / 2U],
               (HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 2U));
    }

    return err;
}

#if BSP_FEATURE_CRYPTO_HAS_RSIP7
fsp_err_t HW_SCE_ECC_521GenerateSign (const uint32_t * InData_CurveType,
                                      const uint32_t * InData_G,
                                      const uint32_t * InData_PrivKey,
                                      const uint32_t * InData_MsgDgst,
                                      uint32_t       * OutData_R,
                                      uint32_t       * OutData_S)
{
    FSP_PARAMETER_NOT_USED(InData_G);
    uint8_t       signature[HW_SCE_ECDSA_P521_DATA_BYTE_SIZE] = {0};
    uint32_t      wrapped_private_key[25U];
    sce_oem_cmd_t key_command;
    const uint32_t * p_domain_param;
    uint8_t padding = 14U;

    /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P521_PRIVATE;
        p_domain_param = DomainParam_NIST_P521;
    }
    /* Koblitz and Brainpool curve unsupported */
    else
    {
        return FSP_ERR_UNSUPPORTED;
    }

    /* Install the plaintext private key to get the wrapped key */
    fsp_err_t err = HW_SCE_GenerateOemKeyIndexPrivate(SCE_OEM_KEY_TYPE_PLAIN,
                                                      key_command,
                                                      NULL,
                                                      NULL,
                                                      (const uint8_t *) InData_PrivKey,
                                                      wrapped_private_key);
    if (FSP_SUCCESS == err)
    {
        err =
            HW_SCE_EcdsaP521SignatureGenerateSubAdaptor((uint32_t *)InData_CurveType, wrapped_private_key, InData_MsgDgst, p_domain_param, (uint32_t *)signature);
    }

    if (FSP_SUCCESS == err)
    {
        memcpy(OutData_R, &signature[padding], ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding));
        memcpy(OutData_S, &signature[((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U)) + padding],
                  ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding));
    }

    return err;
}

fsp_err_t HW_SCE_ECC_521HrkGenerateSign (const uint32_t * InData_CurveType,
                                         const uint32_t * InData_G,
                                         const uint32_t * InData_KeyIndex,
                                         const uint32_t * InData_MsgDgst,
                                         uint32_t       * OutData_R,
                                         uint32_t       * OutData_S)
{
    FSP_PARAMETER_NOT_USED(InData_G);
    const uint32_t * p_domain_param;
    uint8_t padding = 14U;

        /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        p_domain_param = DomainParam_NIST_P521;
    }
    /* Koblitz and Brainpool curve unsupported */
    else
    {
        return FSP_ERR_UNSUPPORTED;
    }

    uint8_t  signature[HW_SCE_ECDSA_P521_DATA_BYTE_SIZE] = {0};
    fsp_err_t err =
        HW_SCE_EcdsaP521SignatureGenerateSubAdaptor((uint32_t *)InData_CurveType, InData_KeyIndex, InData_MsgDgst, p_domain_param, (uint32_t *) signature);
    if (FSP_SUCCESS == err)
    {
        memcpy(OutData_R, &signature[padding], ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding));
        memcpy(OutData_S, &signature[((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U)) + padding],
        		((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding));
    }

    return err;
}
#endif

fsp_err_t HW_SCE_ECC_256VerifySign (const uint32_t * InData_CurveType,
                                    const uint32_t * InData_G,
                                    const uint32_t * InData_PubKey,
                                    const uint32_t * InData_MsgDgst,
                                    const uint32_t * InData_R,
                                    const uint32_t * InData_S)
{
    uint32_t signature[HW_SCE_ECDSA_DATA_BYTE_SIZE / 4U] = {0};
    uint32_t formatted_public_key[ECC_256_FORMATTED_PUBLIC_KEY_LENGTH_WORDS];
    memcpy(signature, InData_R, (HW_SCE_ECDSA_DATA_BYTE_SIZE / 2U));
    memcpy(&signature[(HW_SCE_ECDSA_DATA_BYTE_SIZE / 4U) / 2U], InData_S, (HW_SCE_ECDSA_DATA_BYTE_SIZE / 2U));
    sce_oem_cmd_t key_command;
    const uint32_t * p_domain_param;

    /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P256_PUBLIC;
        p_domain_param = DomainParam_NIST_P256;
    }
    /* Brainpool Curve */
    else if (SCE_ECC_CURVE_TYPE_BRAINPOOL == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P256R1_PUBLIC;
        p_domain_param = DomainParam_Brainpool_256r1;
    }
    /* Koblitz curve */
    else
    {
        key_command = SCE_OEM_CMD_ECC_SECP256K1_PUBLIC;
        p_domain_param = DomainParam_Koblitz_secp256k1;
    }

    /* Install the plaintext public key to get the formatted public key */
    fsp_err_t err = HW_SCE_GenerateOemKeyIndexPrivate(SCE_OEM_KEY_TYPE_PLAIN,
                                                      key_command,
                                                      NULL,
                                                      NULL,
                                                      (const uint8_t *) InData_PubKey,
                                                      formatted_public_key);
    if (FSP_SUCCESS == err)
    {
        /* InData_CurveType = curve type; InData_G = command */
        err = HW_SCE_EcdsaSignatureVerificationSubAdaptor(InData_CurveType,
                                                   InData_G,
                                                   formatted_public_key,
                                                   InData_MsgDgst,
                                                   signature,
                                                   p_domain_param);
    }

    return err;
}

fsp_err_t HW_SCE_ECC_384VerifySign (const uint32_t * InData_CurveType,
                                    const uint32_t * InData_G,
                                    const uint32_t * InData_PubKey,
                                    const uint32_t * InData_MsgDgst,
                                    const uint32_t * InData_R,
                                    const uint32_t * InData_S)
{
    FSP_PARAMETER_NOT_USED(InData_G);
    uint32_t signature[HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 4U] = {0};
    uint32_t formatted_public_key[ECC_384_FORMATTED_PUBLIC_KEY_LENGTH_WORDS];
    memcpy(signature, InData_R, (HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 2U));
    memcpy(&signature[(HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 4U) / 2U], InData_S, (HW_SCE_ECDSA_P384_DATA_BYTE_SIZE / 2U));

    sce_oem_cmd_t key_command;
    const uint32_t * p_domain_param;

    /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P384_PUBLIC;
        p_domain_param = DomainParam_NIST_P384;
    }
    /* Brainpool Curve */
    else if (SCE_ECC_CURVE_TYPE_BRAINPOOL == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P384R1_PUBLIC;
        p_domain_param = DomainParam_Brainpool_384r1;
    }
    /* Koblitz curve unsupported */
    else
    {
        return FSP_ERR_UNSUPPORTED;
    }

    /* Install the plaintext public key to get the formatted public key */
    fsp_err_t err = HW_SCE_GenerateOemKeyIndexPrivate(SCE_OEM_KEY_TYPE_PLAIN,
                                                      key_command,
                                                      NULL,
                                                      NULL,
                                                      (const uint8_t *) InData_PubKey,
                                                      formatted_public_key);
    if (FSP_SUCCESS == err)
    {
        err = HW_SCE_EcdsaP384SignatureVerificationSubAdaptor((uint32_t *)InData_CurveType,
                                                       formatted_public_key,
                                                       InData_MsgDgst,
                                                       signature,
                                                       p_domain_param);
    }

    return err;
}

#if BSP_FEATURE_CRYPTO_HAS_RSIP7
fsp_err_t HW_SCE_ECC_521VerifySign (const uint32_t * InData_CurveType,
                                    const uint32_t * InData_G,
                                    const uint32_t * InData_PubKey,
                                    const uint32_t * InData_MsgDgst,
                                    const uint32_t * InData_R,
                                    const uint32_t * InData_S)
{
    FSP_PARAMETER_NOT_USED(InData_G);
    uint8_t signature[HW_SCE_ECDSA_P521_DATA_BYTE_SIZE] = {0};
    uint32_t formatted_public_key[ECC_521_FORMATTED_PUBLIC_KEY_LENGTH_WORDS];
    uint8_t padding = 14U;

    memcpy(&signature[padding], InData_R, ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding)); /* 119bit 0 padding + R + 119bit 0 padding + S */
    memcpy(&signature[((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U)) + padding], InData_S, ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding));

    uint8_t Pubkey[HW_SCE_ECDSA_P521_DATA_BYTE_SIZE + 16] = {0}; /* OEM key install expects 44words of Public key as input */
    uint8_t * IndataPublickey  = (uint8_t *)InData_PubKey;
    memcpy(&Pubkey[padding], IndataPublickey, ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding)); /* 119bit 0 padding + Px + 119bit 0 padding + Py */
    memcpy(&Pubkey[((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U)) + padding], (IndataPublickey + ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding)),
           ((HW_SCE_ECDSA_P521_DATA_BYTE_SIZE / 2U) - padding));

    sce_oem_cmd_t key_command;
    const uint32_t * p_domain_param;

    /* NIST curve */
    if (SCE_ECC_CURVE_TYPE_NIST == *InData_CurveType)
    {
        key_command = SCE_OEM_CMD_ECC_P521_PUBLIC;
        p_domain_param = DomainParam_NIST_P521;
    }
    /* Koblitz and Brainpool curve unsupported */
    else
    {
        return FSP_ERR_UNSUPPORTED;
    }

    /* Install the plaintext public key to get the formatted public key */
    fsp_err_t err = HW_SCE_GenerateOemKeyIndexPrivate(SCE_OEM_KEY_TYPE_PLAIN,
                                                      key_command,
                                                      NULL,
                                                      NULL,
                                                      (const uint8_t *) Pubkey,
                                                      formatted_public_key);
    if (FSP_SUCCESS == err)
    {
        err = HW_SCE_EcdsaP521SignatureVerificationSubAdaptor((uint32_t *)InData_CurveType,
                                                             formatted_public_key,
                                                             InData_MsgDgst,
                                                             (uint32_t *)signature,
                                                             p_domain_param);
    }

    return err;
}
#endif
#endif                                 /*  #if (defined(MBEDTLS_ECDSA_SIGN_ALT) || defined(MBEDTLS_ECDSA_VERIFY_ALT) || defined(MBEDTLS_ECP_ALT)) */
