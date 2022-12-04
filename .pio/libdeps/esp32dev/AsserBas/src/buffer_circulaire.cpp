#include "buffer_circulaire.h"

int buf_circ_push(buf_circ_t *c, float data)
{
    int next;

    next = c->head + 1;  // next is where head will point to after this write.
    if (next >= c->maxlen)
        next = 0;

    // if the head + 1 == tail, circular buffer is full. Notice that one slot
    // is always left empty to differentiate empty vs full condition
    if (next == c->tail)
        return -1;

    c->buffer[c->head] = data;  // Load data and then move
    c->head = next;             // head to next data offset.
    return 0;  // return success to indicate successful push.
}

int buf_circ_pop(buf_circ_t *c, float *data)
{
    int next;

    if (c->head == c->tail)  // if the head == tail, we don't have any data
        return -1;

    next = c->tail + 1;  // next is where tail will point to after this read.
    if(next >= c->maxlen)
        next = 0;

    *data = c->buffer[c->tail];  // Read data and then move
    c->tail = next;              // tail to next offset.
    return 0;  // return success to indicate successful push.
}

int buf_circ_free_space(buf_circ_t *c)
{
    int freeSpace;
    freeSpace = c->tail - c->head;
    if (freeSpace <= 0)
        freeSpace += c->maxlen;
    return freeSpace - 1; // -1 to account for the always-empty slot.
}