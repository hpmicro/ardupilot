/*
  wrappers around core memory allocation functions from libc
*/

#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

void *__wrap_malloc(size_t size)
{
    // fc_heap_alloc guarantees zero filled memory
    if (size <= 0) {
        return NULL;
    }
    void *ret = pvPortMalloc(size);
    if (ret == NULL) {
        /* Allocation failed - return NULL and let caller handle it */
        return NULL;
    }
    return (void*)(ret);
}

void *__wrap__malloc_r(struct _reent *r, size_t size)
{
    if (r == NULL) {
        return NULL;
    }
    return pvPortMalloc(size);
}

void __wrap_free(void *ptr)
{
    if (ptr == NULL) {
        return;
    }
    vPortFree((void*)ptr);
}

void *__wrap_realloc(void *ptr, size_t size)
{
    /* realloc semantics:
       - ptr == NULL -> behave like malloc
       - size == 0 -> free and return NULL
       Note: FreeRTOS heap API doesn't provide the original allocation size,
       so we allocate a new block of 'size' and copy up to 'size' bytes from
       the old block (this matches other HAL implementations).
    */
    if (ptr == NULL) {
        return __wrap_malloc(size);
    }

    if (size == 0) {
        __wrap_free(ptr);
        return NULL;
    }

    void *newptr = pvPortMalloc(size);
    if (newptr == NULL) {
        /* Allocation failed - return NULL and let caller handle it */
        return NULL;
    }

    /* Zero the new block and copy bytes from the old block. */
    memset(newptr, 0, size);
    memcpy(newptr, ptr, size);

    __wrap_free(ptr);
    return newptr;

}

void * __wrap__realloc_r(struct _reent *reent_ptr, void *ptr, size_t new_size)
{
    return __wrap_realloc(ptr, new_size);
}

void *__wrap_calloc(size_t nmemb, size_t size)
{
    void *ret = __wrap_malloc(nmemb*size);
    if (ret != NULL) {
        memset(ret, 0, nmemb*size);
    }
    return ret;
}

/*
  return true if a memory region is safe for a DMA operation
*/
#include <stdint.h>
#include "hpm_soc_feature.h"

/* Linker provided symbols for non-cacheable region */
extern uint8_t __noncacheable_start__[];
extern uint8_t __noncacheable_end__[];

bool mem_is_dma_safe(const void *addr, uint32_t size, bool filesystem_op)
{
    (void)filesystem_op;

    uintptr_t a = (uintptr_t)addr;
    uintptr_t end = a + (uintptr_t)size;
    uintptr_t nc_start = (uintptr_t)__noncacheable_start__;
    uintptr_t nc_end = (uintptr_t)__noncacheable_end__;

    /*
      For filesystem operations we only consider non-cacheable AXI SRAM
      as safe (matching behavior of other HALs which require special
      region for SD DMA). For generic DMA ops we allow either non-
      cacheable region or cacheline aligned buffers of cacheline-sized
      multiples.
    */
    if (filesystem_op) {
        if (a >= nc_start && end <= nc_end) {
            return true;
        }
        return false;
    }

    /* If the buffer is in the non-cacheable region it's safe */
    if (a >= nc_start && end <= nc_end) {
        return true;
    }

    /* Otherwise require cacheline alignment and size to be a multiple of cacheline */
#ifndef HPM_L1C_CACHELINE_SIZE
#define HPM_L1C_CACHELINE_SIZE 64
#endif
    if ((a & (HPM_L1C_CACHELINE_SIZE - 1)) != 0) {
        return false;
    }
    if ((size & (HPM_L1C_CACHELINE_SIZE - 1)) != 0) {
        return false;
    }

    return true;
}

void *malloc(size_t size) __attribute__((error("Use pvPortMalloc instead!")));