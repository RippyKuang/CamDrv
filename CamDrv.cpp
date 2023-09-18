#include <iostream>
#include "CamDrv.hpp"

using namespace std;

bool MVCamera::open() {
	cout << CameraEnumerateDevice(pCameraInfo, &CamNum) << endl;
	if (CameraEnumerateDevice(pCameraInfo, &CamNum) == CAMERA_STATUS_SUCCESS
			&& CamNum > 0) {
		int status = CameraInit(pCameraInfo, -1, -1, &pCameraHandle);

		if (status == CAMERA_STATUS_SUCCESS) {

			tSdkImageResolution imageResolution;
			CameraGetImageResolution(pCameraHandle, &imageResolution);
			imageResolution.iIndex = 0XFF;
			imageResolution.iWidth = frame_width;
			imageResolution.iHeight = frame_height;
			CameraGetCapability(pCameraHandle, &tCapability);
			CameraSetImageResolution(pCameraHandle, &imageResolution);
			CameraGetExposureLineTime(pCameraHandle, &m_fExpLineTime);
			CameraSetTriggerMode(pCameraHandle, 0);
			m_pFrameBuffer = CameraAlignMalloc(frame_width * frame_height * 3,
					16);
			CameraSetFrameSpeed(pCameraHandle, tCapability.iFrameSpeedDesc - 1);
			// 设置gamma
			CameraSetGamma(pCameraHandle, gamma);
			// 设置对比度CameraGetImageBuffer
			CameraSetNoiseFilter(pCameraHandle, true);
			CameraSetGain(pCameraHandle, red_channel_gain, green_channel_gain,
					blue_channel_gain);
			// 设置图像锐化
			CameraSetSharpness(pCameraHandle, sharpness);
			// 自动曝光
			if (exposure_mode == 1) {
				CameraSetAeState(pCameraHandle, true);
				CameraSetAeTarget(pCameraHandle, auto_exp_target_brightness);
			} else {
				CameraSetAeState(pCameraHandle, false);
				double m_fExpLineTime = 0;
				CameraGetExposureLineTime(pCameraHandle, &m_fExpLineTime);
				CameraSetExposureTime(pCameraHandle,
						m_fExpLineTime * exposure_time);
				CameraSetAnalogGain(pCameraHandle, analog_gain);
			}

			if (wb_mode == 0)
				CameraSetWbMode(pCameraHandle, FALSE);
			else
				CameraSetWbMode(pCameraHandle, TRUE);
			CameraSetAntiFlick(this->pCameraHandle, false);
			// 8位三通道
			CameraSetIspOutFormat(pCameraHandle, CAMERA_MEDIA_TYPE_BGR8);
			CameraSetCallbackFunction(pCameraHandle,
					MVCamera::GrabImageCallback, m_pFrameBuffer, NULL);
			CameraPlay(pCameraHandle);

			std::cout << "open camera successfully" << std::endl;
			return true;
		}
	}
	std::cout << "fail to open camera" << std::endl;
	return false;

}
void MVCamera::get_Mat(cv::Mat &img) {

	img = cv::Mat(cv::Size(frame_width, frame_height), CV_8UC3);
	img.data = m_pFrameBuffer;

}
void MVCamera::GrabImageCallback(CameraHandle hCamera, BYTE *pFrameBuffer,
		tSdkFrameHead *pFrameHead, PVOID pContext) {
	CameraImageProcess(hCamera, pFrameBuffer, (unsigned char*) pContext,
			pFrameHead);

}
void MVCamera::load_param(std::string path) {
	cv::FileStorage file(path, cv::FileStorage::READ);
	if (!file.isOpened()) {
		abort();
	}

	file["frame_width"] >> frame_width;
	file["frame_height"] >> frame_height;
	file["blue_channel_gain"] >> blue_channel_gain;
	file["green_channel_gain"] >> green_channel_gain;
	file["red_channel_gain"] >> red_channel_gain;
	file["saturation"] >> saturation;
	file["contrast"] >> contrast;
	file["gamma"] >> gamma;
	file["sharpness"] >> sharpness;
	file["exposure_mode"] >> exposure_mode;
	file["wb_mode"] >> wb_mode;
	file["auto_exp_target_brightness"] >> auto_exp_target_brightness;
	file["exposure_time"] >> exposure_time;
	file["analog_gain"] >> analog_gain;
	file.release();
}
void MVCamera::write_param(std::string path) {
	cv::FileStorage file(path, cv::FileStorage::WRITE);
	if (!file.isOpened()) {
		abort();
	}

	file << "frame_width" << frame_width;
	file << "frame_height" << frame_height;
	file << "blue_channel_gain" << blue_channel_gain;
	file << "green_channel_gain" << green_channel_gain;
	file << "red_channel_gain" << red_channel_gain;
	file << "saturation" << saturation;
	file << "contrast" << contrast;
	file << "gamma" << gamma;
	file << "sharpness" << sharpness;
	file << "auto_exp_target_brightness" << auto_exp_target_brightness;
	file << "exposure_time" << exposure_time;
	file << "analog_gain" << analog_gain;
	file << "exposure_mode" << exposure_mode;
	file << "wb_mode" << wb_mode;
	file.release();

}

void MVCamera::initTrackbar() {
	cv::namedWindow("Trackbars", (1280, 200));

	cv::createTrackbar("蓝增益", "Trackbars", &blue_channel_gain,
			tCapability.sRgbGainRange.iBGainMax
					- tCapability.sRgbGainRange.iBGainMin, CB_SET_BGAIN, this);
	cv::createTrackbar("绿增益", "Trackbars", &green_channel_gain,
			tCapability.sRgbGainRange.iGGainMax
					- tCapability.sRgbGainRange.iGGainMin, CB_SET_GGAIN, this);
	cv::createTrackbar("红增益", "Trackbars", &red_channel_gain,
			tCapability.sRgbGainRange.iRGainMax
					- tCapability.sRgbGainRange.iRGainMin, CB_SET_RGAIN, this);
	cv::createTrackbar("饱和度", "Trackbars", &saturation,
			tCapability.sSaturationRange.iMax
					- tCapability.sSaturationRange.iMin, CB_SET_SAT, this);
	cv::createTrackbar("对比度", "Trackbars", &contrast,
			tCapability.sContrastRange.iMax - tCapability.sContrastRange.iMin,
			CB_SET_CONSTAST, this);
	cv::createTrackbar("伽马值", "Trackbars", &gamma,
			tCapability.sGammaRange.iMax - tCapability.sGammaRange.iMin,
			CB_SET_GAMMA, this);
	cv::createTrackbar("锐化", "Trackbars", &sharpness,
			tCapability.sSharpnessRange.iMax - tCapability.sSharpnessRange.iMin,
			CB_SET_SHARP, this);
	cv::createTrackbar("曝光模式", "Trackbars", &exposure_mode, 1, CB_EXP_MODE,
			this); //0手动 1自动
	cv::createTrackbar("白平衡模式", "Trackbars", &wb_mode, 1, CB_WB_MODE, this); //0手动 1自动
	cv::createTrackbar("曝光时间", "Trackbars", &exposure_time,
			(tCapability.sExposeDesc.uiExposeTimeMax
					- tCapability.sExposeDesc.uiExposeTimeMin), CB_SET_EXP_TIME,
			this);
	cv::createTrackbar("模拟增益", "Trackbars", &analog_gain,
			tCapability.sExposeDesc.uiAnalogGainMax
					- tCapability.sExposeDesc.uiAnalogGainMin, CB_SET_ANA_GAIN,
			this);
	cv::createTrackbar("自动曝光亮度目标", "Trackbars", &auto_exp_target_brightness,
			tCapability.sExposeDesc.uiTargetMax
					- tCapability.sExposeDesc.uiTargetMin, CB_SET_TARGET, this);

}
void MVCamera::CB_EXP_MODE(int x, void *p) {
	if (x == 0)
		CameraSetAeState(((MVCamera*) p)->pCameraHandle, FALSE);
	else
		CameraSetAeState(((MVCamera*) p)->pCameraHandle, TRUE);
}
void MVCamera::CB_WB_MODE(int x, void *p) {
	if (x == 0)
		CameraSetWbMode(((MVCamera*) p)->pCameraHandle, FALSE);
	else
		CameraSetWbMode(((MVCamera*) p)->pCameraHandle, TRUE);
}
void MVCamera::CB_SET_TARGET(int x, void *p) {
	CameraSetAeTarget(((MVCamera*) p)->pCameraHandle,
			((MVCamera*) p)->tCapability.sExposeDesc.uiTargetMin + x);
}
void MVCamera::CB_SET_EXP_TIME(int x, void *p) {

	CameraSetExposureTime(((MVCamera*) p)->pCameraHandle,
			((MVCamera*) p)->m_fExpLineTime
					* (((MVCamera*) p)->tCapability.sExposeDesc.uiExposeTimeMin
							+ x));
}
void MVCamera::CB_SET_ANA_GAIN(int x, void *p) {
	CameraSetAnalogGain(((MVCamera*) p)->pCameraHandle,
			((MVCamera*) p)->tCapability.sExposeDesc.uiAnalogGainMin + x);
}
void MVCamera::CB_SET_SHARP(int x, void *p) {
	CameraSetSharpness(((MVCamera*) p)->pCameraHandle,
			((MVCamera*) p)->tCapability.sSharpnessRange.iMin + x);
}
void MVCamera::CB_SET_RGAIN(int x, void *p) {
	int pR = 0, pG = 0, pB = 0;
	CameraGetGain(((MVCamera*) p)->pCameraHandle, &pR, &pG, &pB);
	CameraSetGain(((MVCamera*) p)->pCameraHandle,
			x + ((MVCamera*) p)->tCapability.sRgbGainRange.iRGainMin, pG, pB);
}
void MVCamera::CB_SET_GGAIN(int x, void *p) {
	int pR = 0, pG = 0, pB = 0;
	CameraGetGain(((MVCamera*) p)->pCameraHandle, &pR, &pG, &pB);
	CameraSetGain(((MVCamera*) p)->pCameraHandle, pR,
			x + ((MVCamera*) p)->tCapability.sRgbGainRange.iGGainMin, pB);
}
void MVCamera::CB_SET_BGAIN(int x, void *p) {
	int pR = 0, pG = 0, pB = 0;
	CameraGetGain(((MVCamera*) p)->pCameraHandle, &pR, &pG, &pB);
	CameraSetGain(((MVCamera*) p)->pCameraHandle, pR, pG,
			x + ((MVCamera*) p)->tCapability.sRgbGainRange.iBGainMin);
}
void MVCamera::CB_SET_GAMMA(int x, void *p) {
	CameraSetGamma(((MVCamera*) p)->pCameraHandle,
			((MVCamera*) p)->tCapability.sGammaRange.iMin + x);
}
void MVCamera::CB_SET_SAT(int x, void *p) {
	CameraSetSaturation(((MVCamera*) p)->pCameraHandle,
			((MVCamera*) p)->tCapability.sSaturationRange.iMin + x);
}
void MVCamera::CB_SET_CONSTAST(int x, void *p) {
	CameraSetContrast(((MVCamera*) p)->pCameraHandle,
			((MVCamera*) p)->tCapability.sContrastRange.iMin + x);
}
void MVCamera::onceWB() {
	CameraSetOnceWB(pCameraHandle);
	int pR = 0, pG = 0, pB = 0;
	CameraGetGain(pCameraHandle, &pR, &pG, &pB);
	blue_channel_gain = pB - tCapability.sRgbGainRange.iBGainMin;
	green_channel_gain = pG - tCapability.sRgbGainRange.iGGainMin;
	red_channel_gain = pR - tCapability.sRgbGainRange.iRGainMin;
	cv::setTrackbarPos("红增益", "Trackbars", red_channel_gain);
	cv::setTrackbarPos("绿增益", "Trackbars", green_channel_gain);
	cv::setTrackbarPos("蓝增益", "Trackbars", blue_channel_gain);

}
int main(int argc, char **argv) {
	MVCamera *c = new MVCamera();
	std::string path = "/home/kuang/project/CamDrv/config/camera.yaml";
	c->load_param(path);
	c->open();
	c->initTrackbar();
	cv::Mat a;
	while (1) {
		c->get_Mat(a);
		cv::imshow("src", a);
		int key = cv::waitKey(30);
		if (key == 'w')
			c->onceWB();
		if (key == 'q')
			break;
		if (key == 's')
			c->write_param(path);

	}
	delete c;
	return 0;
}
