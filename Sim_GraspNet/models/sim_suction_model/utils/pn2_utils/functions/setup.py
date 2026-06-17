from setuptools import setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension
import os

# 设置 CUDA 架构，避免 PyTorch 自动检测时的解析错误
# 如果环境变量已设置，则使用环境变量的值；否则使用默认值
if 'TORCH_CUDA_ARCH_LIST' not in os.environ:
    # Support NVIDIA Ampere (e.g. RTX 3080 Ti -> sm_86)
    os.environ['TORCH_CUDA_ARCH_LIST'] = '8.6'

extra_compile_args = {'cxx': ['-g'],
                      'nvcc': ['-O2']}

setup(
    name='dgcnn_ext',
    ext_modules=[
        CUDAExtension(
            name='dgcnn_ext',
            sources=[
                'csrc/main.cpp',
                'csrc/gather_knn_kernel.cu',
            ],
            extra_compile_args=extra_compile_args
        ),
    ],
    cmdclass={
        'build_ext': BuildExtension
    })
