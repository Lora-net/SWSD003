#ifndef LR11XX_CRYPTO_ENGINE_TYPES_STR_H
#define LR11XX_CRYPTO_ENGINE_TYPES_STR_H
#include "lr11xx_crypto_engine_types.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* lr11xx_crypto_element_to_str( const lr11xx_crypto_element_t value );
const char* lr11xx_crypto_status_to_str( const lr11xx_crypto_status_t value );
const char* lr11xx_crypto_lorawan_version_to_str( const lr11xx_crypto_lorawan_version_t value );
const char* lr11xx_crypto_keys_idx_to_str( const lr11xx_crypto_keys_idx_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_CRYPTO_ENGINE_TYPES_STR_H
