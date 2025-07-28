#pragma once

#include "allocator.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t* raw;
    uint16_t size;
    uint16_t head;
    uint16_t tail;
} Buffer;

#define CREATE_BUFFER(...)  \
{                           \
    .raw = NULL,            \
    .size = 0,              \
    .head = 0,              \
    .tail = 0,              \
    __VA_ARGS__             \
}

Buffer* bufferAllocate(Allocator* allocator, uint16_t size);
void bufferDeallocate(Allocator* allocator, Buffer** buffer);

void __bufferMoveTail(Buffer* buffer, uint16_t offset);
void __bufferMoveHead(Buffer* buffer, uint16_t offset);

void bufferClear(Buffer* buffer);

bool bufferIsEmpty(const Buffer* buffer);
bool bufferIsFull(const Buffer* buffer);

bool bufferWrite(Buffer* buffer, const uint8_t data);
bool bufferRead(Buffer* buffer, uint8_t* data);
