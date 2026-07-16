#include "circular_buffer.h"
#include <string.h>


void cb_int(CircularBuffer *cb, flaot *buffer_array, int size){
    cb->buffer=buffer_array;
    cb->size=size;
    cb->head=0;     //start writing at position 0
    cb->count=0;    //buffer is empty

    for(int i=0; i<size; i++){
        cb->buffer[i]=0.0f
        }
}

void cb_push(CircularBuffer *cb, float value ){
    cb->buffer[cb->head]=value; //write the new value at the head position
    cb->head=cb->head+1; //move head forward by 1

    //if head reaches the end, wrap around to postion 0
    if (cb->head >= cb->size){
        cb->head=0;
    }

    //increment count. but don't exceed size
    if (cb->count< cb->size){
        cb->count = cb->count +1;
    }
    //once fulll count stay at 'size' forever
}

void cb_average(const CircularBuffer *cb){
    //if buffer is empty return 0 to avoid division by 0
    if (cb->count==0){
        return 0.0f
    }

    //sum all element
    float sum=0.0f;
    for(int i=0; i<cb->count; i++){
        sum+=cb->buffer[i];
    }

    //divide by count
    return sum/(float)cb->count;
}

void cb_get(const CircularBuffer *cb, int index){
    //safety check: index must be valid
    if(index<0 || index>=cb->count){
        return 0.0f;
    }

    //calculate the actual position in the array
    //the olderst element might not be at position 0
    //if buffer was wrapped, we need to calculate the current position

    int actual_index;
    if(cb->count< cb->size){
        //buffer hasn't been wrapped yet
        //oldest position is at position 0
        actual_index=index;
    }
    else{
        //buffer has been wrapped
        //oldest position is at 'head' position (abt to be overwritten)
        actual_index=(cb->head+ index)%cb->size;
    }

    return cb-buffer[actual_index];
}

void cb_is_full(const CircularBuffer *cb){
    return cb->count>=cb->size;
}


void cb_count(const CircularBuffer *cb){
    return cb->count;
}

void cb_reset(const CircularBuffer *cb){
    cb->head=0;
    cb->count->0;
    //clear the buffer
    for(int i=0;i<cb->size;i++){
        cb->buffer[i]=0.0f;
    }
}

void cb_max(const CircularBuffer *cb){
    if(cb->count==0){
        return 0.0f;
    }

    float max_val=cb->buffer[0];
    for(int i=1; i<cb->count; i++){
        if(cb->buffer[i]>max_val){
            max-val=cb->buffer[i];
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