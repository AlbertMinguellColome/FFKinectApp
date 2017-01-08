#pragma once
#include "ofMain.h"

class parallelise {
public:
    template<class c_l_a_s_s>
    void for_all(void (c_l_a_s_s::*otherFunction)(int,ofShortPixels &pix), int start, int end,ofShortPixels &pix) {
                void (c_l_a_s_s::*with)(int,ofShortPixels &pix) ;
                c_l_a_s_s * doTask;
                with = otherFunction;
                dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
                dispatch_apply(end-start, queue, ^(size_t blockIdx) {
                    int i = start+blockIdx;
                    (doTask->*with)(i,std::ref(pix));
                });}
    
    template<class c_l_a_s_s>
    void for_batch(void (c_l_a_s_s::*otherFunction)(int), int start, int end, int batchSize) {
        void (c_l_a_s_s::*with)(int) ;
        c_l_a_s_s * doTask;
        with = otherFunction;
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
        const int workSize = end-start;
        dispatch_apply(workSize/batchSize, queue, ^(size_t blockIdx){
            const int start_i = start + blockIdx*batchSize;
            const int end_i = start_i + batchSize;
            for(int i = start_i; i<end_i; i++){
                (doTask->*with)(i);
            }});
        const int start_e = end - workSize%batchSize;
        const int end_e = end;
        if (start_e < end_e){
            for(int i = start_e; i<end_e; i++){
                (doTask->*with)(i);
            }}}
};








