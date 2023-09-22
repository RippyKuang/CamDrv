/*
 * main.cpp
 *
 *  Created on: 20 Sept 2023
 *      Author: kuang
 */
#include "CamDrv.hpp"
#include "Armor.hpp"


int main() {
	std::cout<<CamDrv_VERSION_MAJOR<<"."<<CamDrv_VERSION_MINOR<<std::endl;
    cv::Mat src=cv::imread("/home/kuang/project/CamDrv/dengtiao/0.jpg");
    std::vector<LightBar*> LBs;
    LightBar:: findLightBar(src,LBs,RED);





    return 0;
}
