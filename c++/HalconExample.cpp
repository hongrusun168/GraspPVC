#include <iostream> 
#include <gc3d.h>
#include <gc3dAlgorithm.h>
#include <halconcpp\HalconCpp.h>

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

	
	//----------------初始化函数--------------------//
	DeviceInformation* infos = nullptr;
	size_t devNum = 0;			//连接相机函数测试
	uint32_t status = initialDevice(infos, devNum);


	GC3DDevice device;  //声明设备 
						//-----相关重建默认参数-------//
	GC3DCameraParameters params;//声明参数相机
	params.exposureTime = 1000;  //默认1000us-----（根据实际调整）
	int fmr = 3;					//设置降噪参数-------(根据实际调整)
	float den1 = 60;				//设置降噪参数-------(根据实际调整)
	float den2 = 5;				//设置降噪参数-------(根据实际调整)
	float den3 = 5;				//设置降噪参数-------(根据实际调整)
	float minHeight = 100;		//设置重建高度范围---(根据实际调整)
	float maxHeight = 3000;			//设置重建高度范围---(根据实际调整)
	bool needGridData = false;		//不需要网格数据-----(根据实际调整)
	int recMinThre = 0;				//设置重建灰度范围---(根据实际调整)
	int recMaxThre = 256;			//设置重建灰度范围---(根据实际调整)
	int smoothParam = 0;			//设置平滑参数-------(根据实际调整)

	if (GC3D_SUCCESS == status) {
		if (GC3D_SUCCESS == device.openDeviceByIndex(0)) {		//打开第一台相机
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


	////-------开始设置参数-------//
	//device.setCameraParameters(params);
	//device.setDenoiseParameters(fmr, den1, den2, den3);		//设置降噪参数-------(根据实际调整)
	//device.setHeightRange(minHeight, maxHeight);			//设置重建高度范围---(根据实际调整)
	//device.setNeedGridData(needGridData);					//不需要网格数据-----(根据实际调整)
	//device.setReconThreshold(recMinThre, recMaxThre);		//设置重建灰度范围---(根据实际调整)
	//device.setSmoothParam(smoothParam);						//设置平滑参数-------(根据实际调整)


	//-------声明必要的数据-------//
	GC3DMetaData data;  //3D原始数据
	GC3DGridData gdata;	//规则化数据
	gc3d::GC3DImageData imgData;//图像数据
	HalconCpp::HObject ximage, yimage, zimage,depthImage,image2d;

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
		if (GC3D_SUCCESS != status){
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

		HalconCpp::GenImage1(&ximage, "real", data.imgW, data.imgH, (Hlong)data.x);
		HalconCpp::GenImage1(&yimage, "real", data.imgW, data.imgH, (Hlong)data.y);
		HalconCpp::GenImage1(&zimage, "real", data.imgW, data.imgH, (Hlong)data.z);

		//HalconCpp::WriteImage(ximage, "tiff", 0, "C:/Users/Administrator/Desktop/ximage.tif");
		//HalconCpp::WriteImage(yimage, "tiff", 0, "C:/Users/Administrator/Desktop/yimage.tif");
		//HalconCpp::WriteImage(zimage, "tiff", 0, "C:/Users/Administrator/Desktop/zimage.tif");


		goto PrintOptions;
		break;
	case 2:
		device.setCamParam2D(50000, 5);//设置2D采图的参数，曝光时间和增益
		status = device.snapShot2D(imgData);
		if (GC3D_SUCCESS != status) {
			printf("snap 2D failed.\n");
			system("pause");
			return -1;
		}else{
			printf("snap 2D success.\n");
		}
		HalconCpp::GenImage1(&image2d, "byte", imgData.width, imgData.height, (Hlong)imgData.data);
		//HalconCpp::WriteImage(image2d, "bmp", 0, "C:/Users/Administrator/Desktop/image2d.bmp");
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
		printf("snap 3D success,validPointsNum:%d.\n", gdata.validPointsNum);
		HalconCpp::GenImage1(&depthImage, "real", gdata.width, gdata.height, (Hlong)gdata.depthImageData);
		//HalconCpp::WriteImage(depthImage, "tiff", 0, "C:/Users/Administrator/Desktop/depthImage.tif");
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

