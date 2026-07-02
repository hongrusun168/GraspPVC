import time
import pyaubo_sdk


class AuboRobotController:                                                              # 用于机械臂控制的类
    def __init__(self, robot_ip, robot_port):
        self._M_PI = 3.14159265358979323846
        self._robot_ip = robot_ip
        self._robot_port = robot_port
        self.robot_rpc_client = None
        self._Connect_to_AuboArm()
        self._robot_name = self.robot_rpc_client.getRobotNames()[0]
        self._robot = self.robot_rpc_client.getRobotInterface(self._robot_name)
        self._robot.getMotionControl().setSpeedFraction(0.50)
        
    
    def _Connect_to_AuboArm(self):
        """
            连接到机械臂，建立 RPC 客户端
        """
        self.robot_rpc_client = pyaubo_sdk.RpcClient()
        self.robot_rpc_client.connect(self._robot_ip, self._robot_port)
        if self.robot_rpc_client.hasConnected():
            print ("1. ----------------------连接到机械臂成功----------------------")
            self._Load_to_AuboArm()
        else:
            print ("1. ----------------------连接到机械臂失败----------------------")
    
    def _Load_to_AuboArm(self):
        """
            登陆到机械臂，获取机械臂状态
        """
        self.robot_rpc_client.login("aubo", "123456")
        if self.robot_rpc_client.hasLogined():
            print ("2. ----------------------登陆到机械臂成功----------------------")
        else:
            print ("2. ----------------------登陆到机械臂失败----------------------")
    
    def _waitArrival(self):
        cnt = 0
        while self._robot.getMotionControl().getExecId() == -1:
            cnt += 1
            if cnt > 5:
                print ("Motion fail!")
                return -1
            time.sleep(0.05)
            # print("getExecId: ", self._robot.getMotionControl().getExecId())
        
        id = self._robot.getMotionControl().getExecId()
        while True:
            idl = self._robot.getMotionControl().getExecId()
            if id != idl:
                break
            time.sleep(0.05)
        
    
    def Move_to_Pose(self, pose):
        """
            控制机械臂移动到指定位姿
        """
        # print ("Moving to Grasp ... ...")
        # time.sleep(0.05)
        self._robot.getMotionControl() \
            .moveLine(pose, 60 * (self._M_PI / 180), 1000 * (self._M_PI / 180), 0, 0)
        self._waitArrival()
    

    def MoveC_to_Pose(self, pose1, pose2):
        ret = self._robot.getMotionControl()\
            .moveCircle(pose1, pose2, 180 * (self._M_PI / 180), 1000000 * (self._M_PI / 180), 0, 0) # 接口调用: 圆弧运动
        self._waitArrival()
    

    def Stop_Move(self) -> int:
        self._robot.getMotionControl().stopMove(True, True)
        self._robot.getMotionControl().clearPath()
        time.sleep(2)
        self._robot.getMotionControl().startMove()
        time.sleep(1)
        self._robot.getRobotManage().setUnlockProtectiveStop()
        return 0
    