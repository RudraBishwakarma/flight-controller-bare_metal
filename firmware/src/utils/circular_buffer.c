#include "circular_buffer.h"
#include <string.h>


void cb_init(CircularBuffer *cb, float *buffer_array, int size) {
    cb->buffer = buffer_array;
    cb->size = size;
    cb->head = 0;     //start writing at position 0
    cb->count = 0;    //buffer is empty

    for (int i = 0; i < size; i++) {
        cb->buffer[i] = 0.0f;
    }
}

void cb_push(CircularBuffer *cb, float value) {
    cb->buffer[cb->head] = value; //write the new value at the head position
    cb->head = cb->head + 1; //move head forward by 1

    //if head reaches the end, wrap around to position 0
    if (cb->head >= cb->size) {
        cb->head = 0;
    }

    //increment count, but don't exceed size
    if (cb->count < cb->size) {
        cb->count = cb->count + 1;
    }
    //once full count stay at 'size' forever
}

float cb_average(const CircularBuffer *cb) {
    //if buffer is empty return 0 to avoid division by 0
    if (cb->count == 0) {
        return 0.0f;
    }

    //sum all elements
    float sum = 0.0f;
    for (int i = 0; i < cb->count; i++) {
        sum += cb->buffer[i];
    }

    //divide by count
    return sum / (float)cb->count;
}

float cb_get(const CircularBuffer *cb, int index) {
    //safety check: index must be valid
    if (index < 0 || index >= cb->count) {
        return 0.0f;
    }

    //calculate the actual position in the array
    //the oldest element might not be at position 0
    //if buffer was wrapped, we need to calculate the current position

    int actual_index;
    if (cb->count < cb->size) {
        //buffer hasn't been wrapped yet
        //oldest position is at position 0
        actual_index = index;
    }
    else {
        //buffer has been wrapped
        //oldest position is at 'head' position (about to be overwritten)
        actual_index = (cb->head + index) % cb->size;
    }

    return cb->buffer[actual_index];
}

bool cb_is_full(const CircularBuffer *cb) {
    return cb->count >= cb->size;
}


int cb_count(const CircularBuffer *cb) {
    return cb->count;
}

void cb_reset(CircularBuffer *cb) {
    cb->head = 0;
    cb->count = 0;
    //clear the buffer
    for (int i = 0; i < cb->size; i++) {
        cb->buffer[i] = 0.0f;
    }
}

float cb_max(const CircularBuffer *cb) {
    if (cb->count == 0) {
        return 0.0f;
    }

    float max_val = cb->buffer[0];
    for (int i = 1; i < cb->count; i++) {
        if (cb->buffer[i] > max_val) {
            max_val = cb->buffer[i];
        }
    }
    return max_val;
}

float cb_min(const CircularBuffer *cb) {
    if (cb->count == 0) {
        return 0.0f;
    }
    
    float min_val = cb->buffer[0];
    for (int i = 1; i < cb->count; i++) {
        if (cb->buffer[i] < min_val) {
            min_val = cb->buffer[i];
        }
    }
    return min_val;
}