/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * Tiny-AES-128-CBC Implementation Header
 * Zero dynamic memory allocation - strictly static / stack storage.
 * ============================================================================
 */

#ifndef OBC_TINY_AES_H_
#define OBC_TINY_AES_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AES_BLOCKLEN 16
#define AES_KEYLEN   16
#define AES_KEYEXPSIZE 176

typedef struct {
    uint8_t roundKey[AES_KEYEXPSIZE];
    uint8_t iv[AES_BLOCKLEN];
} TinyAesContext;

/**
 * @brief Initialize TinyAesContext with a 128-bit key and 128-bit IV
 */
void TinyAes_Init(TinyAesContext* ctx, const uint8_t* key, const uint8_t* iv);

/**
 * @brief Set the IV for CBC operations
 */
void TinyAes_SetIv(TinyAesContext* ctx, const uint8_t* iv);

/**
 * @brief Encrypt a buffer in CBC mode (length MUST be a multiple of 16)
 */
void TinyAes_EncryptCbc(TinyAesContext* ctx, uint8_t* buf, size_t length);

/**
 * @brief Decrypt a buffer in CBC mode (length MUST be a multiple of 16)
 */
void TinyAes_DecryptCbc(TinyAesContext* ctx, uint8_t* buf, size_t length);

/**
 * @brief Apply PKCS#7 padding to data in static buffer
 * @return Total padded length (multiple of 16) or 0 if buffer too small
 */
size_t TinyAes_ApplyPkcs7Padding(uint8_t* buf, size_t dataLen, size_t maxBufLen);

/**
 * @brief Validate and strip PKCS#7 padding
 * @return Unpadded data length or 0 if padding invalid
 */
size_t TinyAes_RemovePkcs7Padding(const uint8_t* buf, size_t paddedLen);

/**
 * @brief Compute CRC-16-CCITT checksum (Poly: 0x1021, Init: 0xFFFF)
 */
uint16_t TinyAes_ComputeCrc16(const uint8_t* data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* OBC_TINY_AES_H_ */
