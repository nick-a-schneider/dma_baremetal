#include "buffer.h"

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