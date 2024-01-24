#ifndef CSAN_CALLOC_H
#define CSAN_CALLOC_H

#include <stddef.h>
#include <stdint.h>

void __csan_calloc(uint64_t call_id,
                   uint64_t func_id,
                   unsigned MAAP_count, uint64_t prop,
                   void *result, size_t num, size_t size);

void __cilksan_record_alloc(void*, size_t);

#endif // CSAN_CALLOC_H