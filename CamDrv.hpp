/*
 * CamDrv.hpp
 *
 *  Created on: 17 Sept 2023
 *      Author: Kuang
 */

#ifndef CAMDRV_HPP_
#define CAMDRV_HPP_

#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <iostream>

class MVCamera {
private:
	CameraHandle pCameraHandle;
	tSdkCameraDevInfo *pCameraInfo;
	unsigned char* m_pFrameBuffer;
	int CamNum;
	int frame_width;
	int frame_height;


public:
	MVCamera() {
		CameraSdkInit(0);
		CamNum = 1;
		m_pFrameBuffer=NULL;
		pCameraHandle = -1;
		pCameraInfo = (tSdkCameraDevInfo*) malloc(sizeof(tSdkCameraDevInfo));
		frame_height=480;
		frame_width=640;
	}
	~MVCamera() {
		CameraUnInit(pCameraHandle);
		delete pCameraInfo;
	}
	void static GrabImageCallback(CameraHandle, BYTE*, tSdkFrameHead*, PVOID);
	bool open();
	void get_Mat(cv::Mat&);

};

#endif /* CAMDRV_HPP_ */
