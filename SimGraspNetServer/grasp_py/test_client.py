"""测试 gRPC 客户端 - 完整功能版"""
import os
import sys
import grpc
import time

# 路径配置
current_dir = os.path.abspath(os.path.dirname(__file__))
desired_dir = os.path.abspath(os.path.join(current_dir, "grasp_py"))
sys.path.insert(0, desired_dir)

# 导入生成的协议代码
try:
    from generated import grasp_pb2, grasp_pb2_grpc
except ImportError:
    print("错误：无法找到 generated 文件夹中的 proto 生成文件，请检查路径。")
    sys.exit(1)


def test_client():
    # 使用 with 语句，确保无论发生什么，channel 都会被优雅关闭
    with grpc.insecure_channel('localhost:50052') as channel:
        stub = grasp_pb2_grpc.GraspServiceStub(channel)

        try:
            # start_time = time.time()
            # print("测试 GraspPVC...")
            # response = stub.GraspPVC(grasp_pb2.DefaultRequest(defaultReq=1))
            # print(f'  Response: errorcode={response.errorcode}, message={response.message}')
            # end_time = time.time()
            # print("[INFO]: PVC 抓取耗时 ", end_time - start_time, "s")

            # start_time = time.time()
            # print("测试 GraspEVA...")
            # response = stub.GraspEVA(grasp_pb2.DefaultRequest(defaultReq=1))
            # print(f'  Response: errorcode={response.errorcode}, message={response.message}')
            # end_time = time.time()
            # print("[INFO]: EVA 抓取耗时 ", end_time - start_time, "s")

            # start_time = time.time()
            # print("测试 GraspPVCandEVA...")
            # response = stub.GraspPVCandEVA(grasp_pb2.DefaultRequest(defaultReq=1))
            # print(f'  Response: errorcode={response.errorcode}, message={response.message}')
            # end_time = time.time()
            # print("[INFO]: 抓取 EVA 和 PVC 管共花费 ", end_time - start_time, "s")
            # # ------------------------------------

            # print("\n所有指令执行完毕!")
            # response = stub.ZeroBack(grasp_pb2.DefaultRequest(defaultReq=1))

            # response = stub.EmergencyStop(grasp_pb2.DefaultRequest(defaultReq=1))


        except grpc.RpcError as e:
            print(f"\n[RPC 错误] 无法连接到服务器或调用失败: {e.code()}")
            print(f"详情: {e.details()}")
        except Exception as e:
            print(f"\n[意外错误]: {e}")


if __name__ == '__main__':
    try:
        for i in range(5):
            test_client()
    except KeyboardInterrupt:
        # 优雅处理 Ctrl+C
        print("\n[用户中断] 正在强行停止客户端测试 ... ...")
        # 直接退出，with 块会处理剩下的清理工作
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)