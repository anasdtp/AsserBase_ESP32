
#ifndef BUFFER_CIRCULAIRE_H
#define BUFFER_CIRCULAIRE_H

#include <stdint.h>

typedef struct {
    float * buffer;
    int head;
    int tail;
    int maxlen;
} buf_circ_t;

#define BUF_CIRC_DEF(x,y)                 \
    float x##_data_space[y+1];            \
    buf_circ_t x = {                      \
        .buffer = x##_data_space,         \
        .head = 0,                        \
        .tail = 0,                        \
        .maxlen = y+1                     \
    }


/*
 * Method: circ_buf_pop
 * Returns:
 *  0 - Success
 * -1 - Empty
 */
int buf_circ_pop(buf_circ_t *c, float *data);

/*
 * Method: circ_buf_push
 * Returns:
 *  0 - Success
 * -1 - Out of space
 */
int buf_circ_push(buf_circ_t *c, float data);

/*
 * Method: circ_bbuf_free_space
 * Returns: number of bytes available
 */
int buf_circ_free_space(buf_circ_t *c);

#endif /// BUFFER_CIRCULAIRE_H