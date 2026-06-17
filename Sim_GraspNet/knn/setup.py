#!/usr/bin/env python

import glob
import os

import torch
from setuptools import find_packages
from setuptools import setup
from torch.utils.cpp_extension import CUDA_HOME
from torch.utils.cpp_extension import CppExtension
from torch.utils.cpp_extension import CUDAExtension

# 设置 CUDA 架构，避免 PyTorch 自动检测时的解析错误
# 如果环境变量已设置，则使用环境变量的值；否则使用默认值
if 'TORCH_CUDA_ARCH_LIST' not in os.environ:
    # Support NVIDIA Ampere (e.g. RTX 3080 Ti -> sm_86) and Ada (sm_87)
    # Use a semicolon-separated list to build for both architectures.
    os.environ['TORCH_CUDA_ARCH_LIST'] = '8.6;8.7'

requirements = ["torch", "torchvision"]


def get_extensions():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    extensions_dir = os.path.join(this_dir, "src")

    main_file = glob.glob(os.path.join(extensions_dir, "*.cpp"))
    source_cpu = glob.glob(os.path.join(extensions_dir, "cpu", "*.cpp"))
    source_cuda = glob.glob(os.path.join(extensions_dir, "cuda", "*.cu"))

    sources = main_file + source_cpu
    extension = CppExtension

    extra_compile_args = {"cxx": []}
    define_macros = []

    sources = [os.path.join(extensions_dir, s) for s in sources]

    include_dirs = [extensions_dir]

    ext_modules = [
        extension(
            "knn_pytorch.knn_pytorch",
            sources,
            include_dirs=include_dirs,
            define_macros=define_macros,
            extra_compile_args=extra_compile_args,
        )
    ]

    return ext_modules


setup(
    name="knn_pytorch",
    version="0.1",
    author="foolyc",
    url="https://github.com/foolyc/torchKNN",
    description="KNN implement in Pytorch 1.0 including both cpu version and gpu version",
    ext_modules=get_extensions(),
    cmdclass={"build_ext": torch.utils.cpp_extension.BuildExtension},
)
