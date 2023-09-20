/*
 * main.cpp
 *
 *  Created on: 20 Sept 2023
 *      Author: kuang
 */
#include "CamDrv.hpp"




int main() {
	std::cout<<CamDrv_VERSION_MAJOR<<"."<<CamDrv_VERSION_MINOR<<std::endl;
	auto *c = new MVCamera();
	const std::string path = "/home/kuang/project/CamDrv/config/camera.yaml";
	c->load_param(path);
	c->open();
	c->initTrackbar();
	cv::Mat a;
	int cnt=0;
	while (true) {
		c->get_Mat(a);
		cv::imshow("src", a);
		int key = cv::waitKey(30);
		if (key == 'w')
			c->onceWB();
		if (key == 'q')
			break;
		if (key == 's')
			c->write_param(path);
		if (key == 'c')
			cv::imwrite("./"+std::to_string(cnt++)+".jpg",a);
	}
	delete c;
	return 0;
}
