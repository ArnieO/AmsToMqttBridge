#ifndef _ENCRYPTION_H
#define _ENCRYPTION_H

#include <stdio.h>

#define MBEDTLS_SHA512_C

#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED

uint8_t* ams_key_exchange_init();
uint8_t* ams_key_exchange_get(unsigned char peer_key[32]);
int ams_encrypt(uint8_t* key, uint8_t* iv, uint8_t* in, uint8_t* out, int len, uint8_t* tag);
int ams_decrypt(uint8_t* key, uint8_t* iv, uint8_t* in, uint8_t* out, int len, uint8_t* tag);

#endif