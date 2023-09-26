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
#include "config.h"
#include <cstring>
#include <iostream>


class MVCamera {
private:

	unsigned char *m_pFrameBuffer;
	int CamNum;


	int blue_channel_gain;
	int green_channel_gain;
	int red_channel_gain;
	int saturation;
	int contrast;
	int gamma;
	int sharpness;
	int exposure_mode = 0;
	int wb_mode = 0;
	int exposure_time;
	int analog_gain;
	int auto_exp_target_brightness;
	double m_fExpLineTime;
	void static CB_EXP_MODE(int x, void *p);
	void static CB_SET_TARGET(int x, void *p);
	void static CB_SET_EXP_TIME(int x, void *p);
	void static CB_SET_ANA_GAIN(int x, void *p);
	void static CB_SET_SHARP(int x, void *p);
	void static CB_WB_MODE(int x, void *p);
	void static CB_SET_BGAIN(int x, void *p);
	void static CB_SET_GGAIN(int x, void *p);
	void static CB_SET_RGAIN(int x, void *p);
	void static CB_SET_GAMMA(int x, void *p);
	void static CB_SET_SAT(int x, void *p);
	void static CB_SET_CONSTAST(int x, void *p);

public:
	CameraHandle pCameraHandle;
	tSdkCameraDevInfo *pCameraInfo;
	tSdkCameraCapbility tCapability{};
    int frame_width;
    int frame_height;
	MVCamera() {
		CameraSdkInit(0);
		m_pFrameBuffer = nullptr;
		pCameraHandle = -1;
		pCameraInfo = (tSdkCameraDevInfo*) malloc(sizeof(tSdkCameraDevInfo));
		CamNum = 1;
		frame_width = 640;
		m_fExpLineTime = 0;
		frame_height = 480;
		exposure_time = 0;
		blue_channel_gain = 0;
		green_channel_gain = 0;
		red_channel_gain = 0;
		saturation = 0;
		contrast = 0;
		gamma = 0;
		sharpness = 0;
		exposure_mode = 0;
		analog_gain = 0;
		auto_exp_target_brightness = 0;

	}
	~MVCamera() {
		CameraUnInit(pCameraHandle);
		delete pCameraInfo;
	}

	void static GrabImageCallback(CameraHandle, BYTE*, tSdkFrameHead*, PVOID);
	bool open();
	void get_Mat(cv::Mat&);
	void load_param(const std::string&);
	void initTrackbar();
	void write_param(const std::string&) const;
	void onceWB();

};

#endif /* CAMDRV_HPP_ */
