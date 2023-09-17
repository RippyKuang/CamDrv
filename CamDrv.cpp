#include <iostream>
#include "CamDrv.hpp"
#include "config.h"
using namespace std;

bool MVCamera::open() {

	if (CameraEnumerateDevice(pCameraInfo, &CamNum) == CAMERA_STATUS_SUCCESS
			&& CamNum > 0) {
		int status = CameraInit(pCameraInfo, -1, -1, &pCameraHandle);
		if (status == CAMERA_STATUS_SUCCESS) {
			tSdkImageResolution imageResolution;
			CameraGetImageResolution(pCameraHandle, &imageResolution);
			imageResolution.iIndex = 0XFF;
			imageResolution.iWidth = frame_width;
			imageResolution.iHeight = frame_height;

			CameraSetImageResolution(pCameraHandle, &imageResolution);

			CameraSetTriggerMode(pCameraHandle,1);
			m_pFrameBuffer = CameraAlignMalloc(1280 * 1024 * 3, 16);
			CameraSetCallbackFunction(pCameraHandle,MVCamera::GrabImageCallback,
					m_pFrameBuffer, NULL);
		    CameraPlay(pCameraHandle);
		    std::cout << "open camera successfully" << std::endl;
			return true;
		}
	}
	std::cout << "fail to open camera" << std::endl;
	return false;

}
void MVCamera::get_Mat(cv::Mat& img){
	 CameraSoftTrigger(pCameraHandle);
	 img = cv::Mat(cv::Size(frame_width,frame_height), CV_8UC3);
	 img.data=m_pFrameBuffer;

}
void MVCamera::GrabImageCallback(CameraHandle hCamera, BYTE *pFrameBuffer,
		tSdkFrameHead *pFrameHead, PVOID pContext) {
	//cout<<"被调用了"<<endl;
	CameraImageProcess(hCamera, pFrameBuffer, (unsigned char*)pContext, pFrameHead);

}
int main(int argc, char **argv) {
	cout << "Version " << CamDrv_VERSION_MAJOR << "." << CamDrv_VERSION_MINOR
			<< endl;
	MVCamera *c = new MVCamera();
	c->open();
	cv::Mat a;
while(1){
	c->get_Mat(a);
	cv::imshow("src",a);
	cv::waitKey(10);
}


	delete c;
	return 0;
}
