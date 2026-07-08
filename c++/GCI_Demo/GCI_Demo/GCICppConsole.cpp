#include <iostream> 
#include <gc3dAlgorithm.h>

using namespace gc3d;

//打印硬件设备信息
void printDeviceInformation(DeviceInformation& inform) {
	std::cout << "产品型号：" << inform.productType << std::endl;
	std::cout << "传感器Y分辨率：" << inform.sensorHeight << std::endl;
	std::cout << "传感器X分辨率：" << inform.sensorWidth << std::endl;
	std::cout << "传感器序列号：" << inform.serialNum << std::endl;
	std::cout << "传感器类型：" << inform.setupType << std::endl;

}

int main() {
	//----------------初始化--------------------//
	//初始化时必要的传入的设备信息指针，初始化完成后会根据设备数量，分配对应个数的设备信息
	DeviceInformation* infos = nullptr;
	//初始化时必要的传入的设备数量
	size_t devNum = 0;
	//调用设备初始化函数，查找相机，若相机数量为0，则返回不成功，否则返回成功
	//devNum返回查找到的设备数量
	uint32_t status = initialDevice(infos, devNum);


	//如果查找到相机，则进行下面的打开相机操作
	//首先声明一个设备
	GC3DDevice device;
	if (GC3D_SUCCESS == status) {
		//打开第一台相机为例，打开时会默认加载“GData”目录下的对应ini配置文件
		if (GC3D_SUCCESS == device.openDeviceByIndex(0)) {
			DeviceInformation info;
			device.getDeviceInfo(info);
			printDeviceInformation(info);
		}
		else {
			std::cout << "open device error!" << std::endl;
			system("pause");
			return -1;
		}
	}
	else {
		std::cout << gc3d::GC3DDevice::getErrMsg(status) << std::endl;
		system("pause");
		return -1;
	}

	//-------声明必要的数据-------//
	GC3DMetaData data;  //3D原始数据
	GC3DGridData gdata;	//规则化数据
	gc3d::GC3DImageData imgData;//图像数据

PrintOptions:
	//show options
	int choice = -1;
	printf("Enter your choice to continue:\n");
	printf("1: Single snap 3D.\n");
	printf("2: Single snap 2D.\n");
	printf("3: Single snap 3D for gridData.\n");
	printf("0: Exit.\n");

	scanf_s("%d", &choice);


	switch (choice)
	{
	case 1:
		status = device.snapShot3D();
		if (GC3D_SUCCESS != status) {
			printf("snap 3D failed.\n");
			system("pause");
			return -1;
		}
		status = device.getGC3DMetaData(data);//得到3D数据，具体见GC3DMetaData定义
		if (GC3D_SUCCESS != status) {
			printf("get 3D data failed.\n");
			system("pause");
			return -1;
		}
		printf("snap 3D success,validPointsNum:%d.\n", data.validPointsNum);

		goto PrintOptions;
		break;
	case 2:
		device.setCamParam2D(50000, 5);//设置2D采图的参数，曝光时间和增益
		status = device.snapShot2D(imgData);
		if (GC3D_SUCCESS != status) {
			printf("snap 2D failed.\n");
			system("pause");
			return -1;
		}
		else {
			printf("snap 2D success.\n");
		}
		goto PrintOptions;
		break;
	case 3:
		status = device.setNeedGridData(true);
		status = device.snapShot3D();
		if (GC3D_SUCCESS != status) {
			printf("snap 3D failed.\n");
			system("pause");
			return -1;
		}
		status = device.getGC3DGridData(gdata);
		if (GC3D_SUCCESS != status) {
			printf("get 3D failed.\n");
			system("pause");
			return -1;
		}
		printf("snap 3D success\nvalidPointsNum:%d.\nxscale:%f.\nyscale:%f.\nzscale:%f.\n", gdata.validPointsNum,
			gdata.dx,gdata.dy,gdata.dz);
		device.setNeedGridData(false);
		goto PrintOptions;
		break;
	case 0:
		device.closeDevice();
		break;
	default:
		printf("Wrong input, please enter again. \n\n");
		goto PrintOptions;
		break;
	}
	unInitialDevice();
	system("pause");
	return 0;


}

