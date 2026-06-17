"""
Grasp gRPC Server
使用方法:
    python main.py --port 50051
"""
import os
import sys
import time  # 必须导入 time 模块

# 配置路径
current_dir = os.path.abspath(os.path.dirname(__file__))
desired_dir = os.path.abspath(os.path.join(current_dir, "grasp_py"))
sys.path.insert(0, desired_dir)

import argparse
import grpc
from concurrent import futures
import logging

# 导入你的 generated 协议和业务逻辑
from generated import grasp_pb2_grpc
from server.grasp_service import GraspServiceServicer

# 日志配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def serve(port: int = 50051):
    """启动 gRPC Server"""
    # 1. 创建服务器并配置线程池
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    
    # 2. 注册业务逻辑
    grasp_pb2_grpc.add_GraspServiceServicer_to_server(
        GraspServiceServicer(), server
    )
    
    # 3. 绑定端口
    server.add_insecure_port(f'0.0.0.0:{port}')
    server.start()
    
    logger.info(f"Grasp gRPC Server 已启动，监听端口: {port}")
    logger.info("按 Ctrl+C 停止服务器")
    
    # 4. 解决 Windows 卡死的核心逻辑
    try:
        while True:
            # 使用 time.sleep 代替 wait_for_termination
            # 这样主线程是“活”的，可以瞬间捕捉到 Ctrl+C 信号
            time.sleep(1)
    except KeyboardInterrupt:
        # 捕获到 Ctrl+C 后的清理动作
        logger.info("\n接收到停止信号 (Ctrl+C)，正在关闭服务...")
        
        # 立即停止 gRPC 接收新请求
        server.stop(0)
        
        logger.info("Grasp gRPC Server 已安全关闭。")
        
        # 强制退出，防止 VS Code 终端因残留线程而无法返回输入状态
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Grasp gRPC Server')
    parser.add_argument('--port', type=int, default=50052, help='监听端口 (默认: 50051)')
    args = parser.parse_args()
    
    # 启动
    serve(args.port)