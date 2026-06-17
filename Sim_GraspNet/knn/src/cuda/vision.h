#pragma once
#include <torch/extension.h>
//#include <THC/THC.h>
#include <cstdint>

void knn_device(float* ref_dev, int ref_width,
    float* query_dev, int query_width,
    int height, int k, float* dist_dev, int64_t* ind_dev, cudaStream_t stream);
