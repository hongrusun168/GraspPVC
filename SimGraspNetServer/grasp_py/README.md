# Grasp gRPC Server

## 环境搭建

```bash
conda env create -f conda_env.yml
```

## 生成 gRPC 代码

```bash
conda run -n grasp_grpc python -m grpc_tools.protoc -I./proto --python_out=./generated --grpc_python_out=./generated ./proto/grasp.proto
```

然后手动修复 import：
- 编辑 `generated/grasp_pb2_grpc.py`
- 将 `import grasp_pb2` 改为 `from generated import grasp_pb2`

## 启动服务器

```bash
call conda activate grasp_grpc
python main.py --port 50051
```

## 测试

```bash
call conda activate grasp_grpc
python test_client.py
```

## 算法人员修改

只需修改 `server/grasp_service.py` 中的三个业务函数：
- `grasp_pvc_and_eva()` - 抓取 PVC 和泡棉
- `grasp_pvc()` - 单抓 PVC
- `grasp_eva()` - 单抓泡棉
