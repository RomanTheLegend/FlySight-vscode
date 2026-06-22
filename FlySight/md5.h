#ifndef MD5_H_
#define MD5_H_

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t state[4];
    uint32_t count[2];
    uint8_t  buffer[64];
} MD5_Context;

void MD5_Init(MD5_Context *ctx);
void MD5_Update(MD5_Context *ctx, const uint8_t *data, size_t len);
void MD5_Final(uint8_t digest[16], MD5_Context *ctx);

#endif /* MD5_H_ */
