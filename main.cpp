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
for(int x=0;x<100000;x++) {
    cv::Mat src = cv::imread("/home/kuang/project/CamDrv/dengtiao/"+std::to_string(x)+".jpg");
    std::vector<LightBar *> LBs;
    std::vector<Armor *> AMs;
    LightBar::findLightBar(src, LBs, RED);
    if (!LBs.empty()) {
        Armor::lightBarCluster(src, LBs, AMs);
        if (!AMs.empty()) {
          //  AMs[0]->showArmor("arm");
            std::vector<float> res = AMs[0]->forward();
            int index = 0;
            for (int x = 0; x < res.size(); x++) {
                if (res[x] > res[index])
                    index = x;
            }
            std::cout << index << std::endl;
        }
    }
}





    return 0;
}
