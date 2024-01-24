#include "csan_calloc.h"
#include <stddef.h>
#include <stdint.h>

void __csan_calloc(uint64_t call_id,
                   uint64_t func_id,
                   unsigned MAAP_count, uint64_t prop,
                   void *result, size_t num, size_t size) {
  __cilksan_record_alloc(result, num * size);
}