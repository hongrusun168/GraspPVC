# KNN PyTorch Extension
# This package provides KNN (K-Nearest Neighbors) implementation for PyTorch

try:
    from . import knn_pytorch
    __all__ = ['knn_pytorch']
except ImportError:
    # Extension not yet compiled
    pass
