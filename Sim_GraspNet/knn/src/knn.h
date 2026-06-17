
#ifdef WITH_CUDA
#include <cuda_runtime.h> // Include this header for CUDA functions
#include "cuda/vision.h"
#include <ATen/ATen.h>
#include <ATen/cuda/CUDAContext.h>
#endif

#include "cpu/vision.h"

// ...

#ifdef WITH_CUDA
at::cuda::CUDAStream current_stream = at::cuda::getCurrentCUDAStream();
#endif

int knn(at::Tensor& ref, at::Tensor& query, at::Tensor& idx) {
    // TODO check dimensions
    int64_t batch, ref_nb, query_nb, dim, k;
    batch = ref.size(0);
    dim = ref.size(1);
    k = idx.size(1);
    ref_nb = ref.size(2);
    query_nb = query.size(2);

    float* ref_dev = ref.data_ptr<float>();
    float* query_dev = query.data_ptr<float>();
    int64_t* idx_dev = idx.data_ptr<int64_t>();

    if (ref.is_cuda()) {
#ifdef WITH_CUDA
        void* dist_dev_ptr; // Declare as void*
        cudaMalloc(&dist_dev_ptr, ref_nb * query_nb * sizeof(float)); // Use proper CUDA function
        float* dist_dev = static_cast<float*>(dist_dev_ptr); // Cast to float* when using
        at::cuda::CUDAStream current_stream = at::cuda::getCurrentCUDAStream();

        for (int b = 0; b < batch; b++) {
            knn_device(ref_dev + b * dim * ref_nb, ref_nb, query_dev + b * dim * query_nb, query_nb, dim, k,
                       dist_dev, idx_dev + b * k * query_nb, c10::cuda::getCurrentCUDAStream().stream());
        }
        cudaFree(dist_dev_ptr); // Free using the void* pointer
        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            printf("error in knn: %s\n", cudaGetErrorString(err));
            return 0; // Change to an appropriate error handling mechanism
        }
        return 1;
#else
        AT_ERROR("Not compiled with GPU support");
#endif
    }

    float* dist_dev = (float*)malloc(ref_nb * query_nb * sizeof(float));
    int64_t* ind_buf = (int64_t*)malloc(ref_nb * sizeof(int64_t));
    for (int b = 0; b < batch; b++) {
        knn_cpu(ref_dev + b * dim * ref_nb, ref_nb, query_dev + b * dim * query_nb, query_nb, dim, k,
                dist_dev, idx_dev + b * k * query_nb, ind_buf);
    }

    free(dist_dev);
    free(ind_buf);

    return 1;
}

