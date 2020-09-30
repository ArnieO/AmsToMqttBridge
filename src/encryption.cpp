#include <stdint.h>
#include "encryption.h"

#include "esp_log.h"
#define TAG "AMSNOW"

//#include "mbedtls_config.h"
#include "mbedtls/esp_config.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/cipher.h"
#include "mbedtls/gcm.h"

mbedtls_ecdh_context ecdh_context;
mbedtls_entropy_context entropy_context;
mbedtls_ctr_drbg_context ctr_drbg;
uint8_t ecdh_public[32];

// https://github.com/ARMmbed/mbedtls/blob/development/programs/pkey/ecdh_curve25519.c
uint8_t* ams_key_exchange_init() {
    const char pers[] = "ecdh";
    int ret = 0;
    mbedtls_ecdh_init( &ecdh_context );
    fflush( stdout );
    mbedtls_entropy_init( &entropy_context );
    if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy_context,
                               (const unsigned char *) pers,
                               sizeof pers ) ) != 0 ) {
        ESP_LOGE(TAG,  "mbedtls_ctr_drbg_seed returned %d\n", ret );
        return NULL;
    }
    fflush( stdout );

    if( (ret = mbedtls_ecp_group_load( &ecdh_context.grp, MBEDTLS_ECP_DP_CURVE25519 )) != 0 ) {
        ESP_LOGE(TAG,  "mbedtls_ecp_group_load returned %d\n", ret );
        return NULL;
    }

    if( (ret = mbedtls_ecdh_gen_public( &ecdh_context.grp, &ecdh_context.d, &ecdh_context.Q,
                                   mbedtls_ctr_drbg_random, &ctr_drbg )) != 0 ) {
        ESP_LOGE(TAG,  "mbedtls_ecdh_gen_public returned %d\n", ret );
        return NULL;
    }

    if( (ret = mbedtls_mpi_write_binary( &ecdh_context.Q.X, ecdh_public, 32 )) != 0 ) {
        ESP_LOGE(TAG,  "mbedtls_mpi_write_binary returned %d\n", ret );
        return NULL;
    }
    return ecdh_public;
}

uint8_t* ams_key_exchange_get(unsigned char peer_key[32]) {
    int ret = 0;
    fflush( stdout );
    if( (ret = mbedtls_mpi_lset( &ecdh_context.Qp.Z, 1 )) != 0 ) {
        ESP_LOGE(TAG,  "mbedtls_mpi_lset returned %d\n", ret );
        return NULL;
    }

    if( (ret = mbedtls_mpi_read_binary( &ecdh_context.Qp.X, peer_key, 32 )) != 0 ) {
        ESP_LOGE(TAG,  "mbedtls_mpi_read_binary returned %d\n", ret );
        return NULL;
    }

    if( (ret = mbedtls_ecdh_compute_shared( &ecdh_context.grp, &ecdh_context.z,
                                       &ecdh_context.Qp, &ecdh_context.d,
                                       mbedtls_ctr_drbg_random, &ctr_drbg )) != 0 ) {
        ESP_LOGE(TAG,  "mbedtls_ecdh_compute_shared returned %d\n", ret );
        return NULL;
    }
    return (uint8_t*) ecdh_context.z.p;
}

// https://gist.github.com/unprovable/892a677d672990f46bca97194ae549bc
int ams_encrypt(uint8_t* key, uint8_t* iv, uint8_t* in, uint8_t* out, int len, uint8_t* tag) {
    int ret = 0;
    if((ret = mbedtls_ctr_drbg_random(&ctr_drbg, iv, 16)) != 0) {
        ESP_LOGE(TAG,  "mbedtls_ctr_drbg_random returned %d\n", ret );
    }

    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);

    if((ret = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128)) != 0) {
        ESP_LOGE(TAG,  "mbedtls_gcm_setkey returned %d\n", ret );
    }
    if((ret = mbedtls_gcm_starts(&ctx, MBEDTLS_GCM_ENCRYPT, iv, 16, NULL, 0)) != 0) {
        ESP_LOGE(TAG,  "mbedtls_gcm_starts returned %d\n", ret );
    }
    if((ret = mbedtls_gcm_update(&ctx, len, in, out)) != 0) {
        ESP_LOGE(TAG,  "mbedtls_gcm_update returned %d\n", ret );
    }
    if((ret = mbedtls_gcm_finish(&ctx, tag, 16)) != 0) {
        ESP_LOGE(TAG,  "mbedtls_gcm_finish returned %d\n", ret );
    }
    mbedtls_gcm_free(&ctx);
    return 0;
}

int ams_decrypt(uint8_t* key, uint8_t* iv, uint8_t* in, uint8_t* out, int len, uint8_t* tag) {
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, 128);
    mbedtls_gcm_starts(&ctx, MBEDTLS_GCM_DECRYPT, iv, 16, NULL, 0);
    mbedtls_gcm_update(&ctx, 192, in, out);
    mbedtls_gcm_finish(&ctx, tag, 16); 
    mbedtls_gcm_free(&ctx);
    return 0;
}
