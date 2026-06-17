import os
import torch
from setuptools import setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension

# --- 第一步：处理 GPU 架构兼容性 ---
# 自动检测当前 PC 的显卡架构，不再死守 Jetson Orin 的 8.7
if 'TORCH_CUDA_ARCH_LIST' not in os.environ:
    if torch.cuda.is_available():
        # 动态获取当前显卡的计算能力 (例如 3090 会返回 8.6, 4090 会返回 8.9)
        major, minor = torch.cuda.get_device_capability()
        os.environ['TORCH_CUDA_ARCH_LIST'] = f"{major}.{minor}"
    else:
        # 如果没检测到显卡，默认一个常用的架构
        os.environ['TORCH_CUDA_ARCH_LIST'] = "8.0"

# --- 第二步：配置编译参数 ---
# 删除了导致报错的 ARM/Jetson 专用参数 (-march=armv8-a, -mtune=cortex-a78ae)
extra_compile_args = {
    'cxx': ['-g', '-std=c++17', '-O3'],
    'nvcc': [
        '-O3',
        '--ptxas-options=-v',
        '--use_fast_math',
        '-m64',
        '--expt-relaxed-constexpr',
        # 在 PC 端通常不需要禁用这些半精度操作，除非你的代码有特殊要求
        # '-D__CUDA_NO_HALF_OPERATORS__',
        # '-D__CUDA_NO_HALF_CONVERSIONS__',
        # '-D__CUDA_NO_HALF2_OPERATORS__'
    ]
}

# --- 第三步：执行安装配置 ---
setup(
    name='pn2_ext',
    ext_modules=[
        CUDAExtension(
            name='pn2_ext',
            sources=[
                'csrc/main.cpp',
                'csrc/ball_query_kernel.cu',
                'csrc/grouping_kernel.cu',
                'csrc/sampling_kernel.cu',
                'csrc/interpolate_kernel.cu',
            ],
            extra_compile_args=extra_compile_args
        ),
    ],
    cmdclass={
        'build_ext': BuildExtension
    }
)