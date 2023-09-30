/*
 * main.cpp
 *
 *  Created on: 20 Sept 2023
 *      Author: kuang
 */
#include "CamDrv.hpp"
#include "armor.hpp"


int main() {

    cv::VideoCapture capture("/home/kuang/project/CamDrv/out.avi");
    cv::Mat src;
    Tracker tracker(RED);
    while (1) {
        capture >> src;
        tracker.push(src);
        int a=0;
        std::vector<cv::Point2d> ps;
        bool b;
        tracker.get(ps,a,b);
        cv::imshow("src", src);
        if(cv::waitKey(0)=='q') break;
    }

    return 0;
}
