"""
Grasp gRPC Server
使用方法:
    python main.py --port 50051
"""

import argparse
import grpc
from concurrent import futures
import logging

from generated import grasp_pb2_grpc
from server.grasp_service import GraspServiceServicer

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def serve(port: int = 50051):
    """启动 gRPC Server"""
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    grasp_pb2_grpc.add_GraspServiceServicer_to_server(
        GraspServiceServicer(), server
    )
    server.add_insecure_port(f'0.0.0.0:{port}')
    server.start()
    logger.info(f"Grasp gRPC Server 已启动，监听端口: {port}")
    logger.info("按 Ctrl+C 停止服务器")
    server.wait_for_termination()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Grasp gRPC Server')
    parser.add_argument('--port', type=int, default=50052, help='监听端口 (默认: 50051)')
    args = parser.parse_args()
    serve(args.port)
