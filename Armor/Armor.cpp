//
// Created by kuang on 20/09/23.
//
#include "Armor.hpp"
#include "CamDrv.hpp"
#include <vector>

void findLightBar(){
    std::vector<cv::Mat> imgs;
    for(int x=0;x<17;x++){
        Mat temp=cv::imread("/home/kuang/project/CamDrv/dengtiao/"+to_string(x)+".jpg");
        imgs.push_back(temp);
    }
    



}