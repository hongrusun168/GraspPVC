import requests
import json

class GraspClient:
    def __init__(self, host="127.0.0.1", port=5302):
        self.base_url = f"http://{host}:{port}"

    def request_inner_grasp(self, data=None):
        """
        请求执行 INNER 抓取
        对应服务端端点: /process_inner
        """
        url = f"{self.base_url}/process_inner"
        return self._send_request(url, data, "Inner Grasp")

    def request_outer_grasp(self, data=None):
        """
        请求执行 OUTER 抓取
        对应服务端端点: /process_outer
        """
        url = f"{self.base_url}/process_outer"
        return self._send_request(url, data, "Outer Grasp")

    def _send_request(self, url, data, label):
        print(f"\n[INFO] 正在向 {url} 发送 {label} 请求...")
        try:
            # 使用 POST 方式发送数据
            response = requests.post(url, json=data, timeout=60)
            
            # 检查 HTTP 状态码
            if response.status_code == 200:
                result = response.json()
                print(f"[SUCCESS] {label} 调用成功!")
                print(f"响应内容: {json.dumps(result, indent=2, ensure_ascii=False)}")
                return result
            elif response.status_code == 500:
                result = response.json()
                print(f"[FAILED] {label} 逻辑执行失败")
                print(f"错误消息: {result.get('message')}")
                return result
            else:
                print(f"[ERROR] 服务器返回异常状态码: {response.status_code}")
                return None

        except requests.exceptions.RequestException as e:
            print(f"[ERROR] 连接服务器失败: {e}")
            return None


# --- 运行测试 ---
if __name__ == "__main__":
    client = GraspClient()

    # 模拟发送一些业务参数
    payload = {"user": "operator_1", "mode": "auto"}

    # 1. 测试 Inner 抓取
    client.request_inner_grasp(payload)

    # 2. 测试 Outer 抓取
    client.request_outer_grasp(payload)