#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float *buffer;    //pointer to the actual data
    int size;         //maximum number of elements
    int head;         //where to write next
    int count;        //how many element currently stored
} CircularBuffer;

void cb_init(CircularBuffer *cb, float *buffer_array, int size);
//initialize the buffer 
//must be called before using any other function
// buffer_array is a pre allocated float array of size element

void cb_push(CircularBuffer *cb, float value);
// add a new value to the buffer
//if buffer is full overwrite the oldest value 

float cb_average(const CircularBuffer *cb);
//get the average of all values currently in the buffer 

float cb_get(const CircularBuffer *cb, int index);
//get a specific element from the buffer
//index 0 = oldest, index(count-1)=newest

bool cb_is_full(const CircularBuffer *cb);
//check circular buffer is full

int cb_count(const CircularBuffer *cb);
//get current number of elements

void cb_reset(CircularBuffer *cb);
//reset circular buffer

float cb_max(const CircularBuffer *cb);
//get the max value from the circular buffer

float cb_min(const CircularBuffer *cb);
//get the min value from the buffer 

#endif   //CIRCULAR_BUFFER_H