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
    for(int x=0;x<1000;x++) {

            cv::Mat src = cv::imread("/home/kuang/project/CamDrv/dengtiao/" + std::to_string(x) + ".jpg");
            std::vector<LightBar *> LBs;
            LightBar::findLightBar(src, LBs, RED);
            if(LBs.size()!=0) {
                Armor::lightBarCluster(src,LBs);
            }
            cv::imshow("qqq", src);
            cv::waitKey(0);

    }



    return 0;
}
