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
    int threhdGray = 200, threhdRed = 245,threWrite=240;
    cv::Mat kernel1 = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat grayimg,aftr_Graythd,aftr_Redthd,and_img,white_mask,imgDil,imgDil2,imgEro;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Mat> Trichlimgs;
    cv::namedWindow("Trackbars");
    cv::createTrackbar("threhdGray", "Trackbars", &threhdGray, 255);
    cv::createTrackbar("threhdRed", "Trackbars", &threhdRed, 255);
    cv::createTrackbar("threhdWrite", "Trackbars", &threWrite, 255);
    int cnt=0;

    while(true) {
        int x=cv::waitKey(0);
        if(x =='n'){
            src=cv::imread("/home/kuang/project/CamDrv/dengtiao/"+std::to_string(++cnt)+".jpg");
        }
        cv::split(src, Trichlimgs);
        cv::Mat rchlimg = Trichlimgs[2];
        cv::cvtColor(src, grayimg, cv::COLOR_BGR2GRAY);
        cv::threshold(grayimg, aftr_Graythd, threhdGray, 255, cv::THRESH_BINARY);
        cv::threshold(rchlimg, aftr_Redthd, threhdRed, 255, cv::THRESH_BINARY);
        cv::threshold(grayimg, white_mask, threWrite, 255, cv::THRESH_BINARY_INV);
        cv::bitwise_and(aftr_Redthd, aftr_Graythd&white_mask, and_img);
        dilate(and_img, imgDil, kernel1);



        cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
        for(int x=0;x<contours.size();x++){
            if(cv::contourArea(contours[x])>30) {
                cv::RotatedRect ellipse = cv::fitEllipse(contours[x]);

                if(ellipse.size.height/ellipse.size.width>3 &&( ellipse.angle<30 ||ellipse.angle>150)){
                    cv::putText(src, std::to_string(ellipse.angle), ellipse.center, cv::FONT_HERSHEY_DUPLEX, 1,
                            cv::Scalar(255, 0, 255), 2);
                    cv::drawContours(src, contours, x, cv::Scalar(255, 0, 255), 3);
                    cv::ellipse(src, ellipse, cv::Scalar(0,255,0), 1);

                }
            }
        }

        cv::imshow("and_img", and_img);
        cv::imshow("src", src);

    }
	return 0;
}
