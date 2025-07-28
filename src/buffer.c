#include "buffer.h"
#include "allocator.h"


/**
 * @details The function will use the given allocator to allocate memory for the struct Buffer and for the raw data.
 * Allocation errors will be handled by freeing the allocated memory and returning NULL.
 * If the allocation of the struct Buffer fails, NULL will be returned.
 * If the allocation of the raw data fails, the allocated memory for the struct Buffer will be freed and NULL will be returned.
 */
Buffer* bufferAllocate(Allocator* allocator, uint16_t size) {
    Buffer* buf = (Buffer*)allocate(allocator, sizeof(Buffer));
    if (!buf) {
        return NULL; // Allocation failed
    }
    buf->raw = (uint8_t*)allocate(allocator, size * sizeof(uint8_t) + 1);
    if (!(buf->raw)) {
        deallocate(allocator, buf);
        return NULL; // Allocation failed
    }
    buf->size = size;
    buf->head = 0;
    buf->tail = 0;
    return buf;
}

/**
 * @details The function will deallocate the raw data and the struct Buffer associated with the given Buffer pointer.
 * If the buffer pointer is NULL or the buffer is NULL, the function does nothing.
 * The allocator is used to deallocate the memory.
 * In case of error during deallocation, the function does not report any error but rather just frees the allocated memory.
 */
void bufferDeallocate(Allocator* allocator, Buffer** buffer) {
    if (buffer && (*buffer)) {
        deallocate(allocator, (*buffer)->raw);
        deallocate(allocator, *buffer);
        *buffer = NULL;
    }
}

void __bufferMoveTail(Buffer* buffer, uint16_t offset) {
    if (buffer && offset > 0) {
        buffer->tail = (uint16_t)((buffer->tail + offset) % (buffer->size + 1));
    }  
}

void __bufferMoveHead(Buffer* buffer, uint16_t offset) {
    if (buffer && offset >= 0) {
        buffer->head = (uint16_t)((buffer->head + offset) % (buffer->size + 1));
    }  
}

void bufferClear(Buffer* buffer) {
    if (buffer) {
        buffer->head = 0;
        buffer->tail = 0;
    }
}
bool bufferIsEmpty(const Buffer* buffer) {
    return buffer->head == buffer->tail;
}
bool bufferIsFull(const Buffer* buffer) {
    return ((buffer->head + 1) % (buffer->size + 1)) == buffer->tail;
}

bool bufferWrite(Buffer* buffer, const uint8_t data) {
    if (!buffer) {
        return false; // Invalid buffer
    }
    if (bufferIsFull(buffer)) {
        return false; // Buffer is full, cannot write
    }
    buffer->raw[buffer->head] = data;
    __bufferMoveHead(buffer, 1);
    return true;

}

bool bufferRead(Buffer* buffer, uint8_t* data) {
    if (!buffer || !data || bufferIsEmpty(buffer)) {
        return false; // Invalid buffer or empty buffer
    }
    *data = buffer->raw[buffer->tail];
    __bufferMoveTail(buffer, 1);
    return true;
}