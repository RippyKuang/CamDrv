/*
 * main.cpp
 *
 *  Created on: 20 Sept 2023
 *      Author: kuang
 */
#include "CamDrv.hpp"
#include "armor.hpp"


int main() {
	std::cout<<CamDrv_VERSION_MAJOR<<"."<<CamDrv_VERSION_MINOR<<std::endl;
    cv::Mat src=cv::imread("/home/kuang/project/CamDrv/dengtiao/0.jpg");
    std::vector<LightBar*> LBs;
    LightBar:: findLightBar(src,LBs,RED);
    Armor::drawBoundary(src,LBs[0],LBs[1]);
    cv::imshow("qqq",src);
    cv::waitKey(0);




    return 0;
}
