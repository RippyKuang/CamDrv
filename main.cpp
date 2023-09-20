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

    cv::Mat src=cv::imread("/home/kuang/project/CamDrv/dengtiao/17.jpg");
    int threhdGray = 142, threhdRed = 245,threWrite=240;
    cv::Mat kernel1 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat kernel2 = getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    std::vector<cv::Mat> Trichlimgs;
    cv::Mat grayimg,aftr_Graythd,aftr_Redthd,and_img,white_mask;
    cv::namedWindow("Trackbars");
    cv::createTrackbar("threhdGray", "Trackbars", &threhdGray, 255);
    cv::createTrackbar("threhdRed", "Trackbars", &threhdRed, 255);
    cv::createTrackbar("threhdWrite", "Trackbars", &threWrite, 255);
    cv::split(src, Trichlimgs);
    cv::Mat rchlimg = Trichlimgs[2];
    cv::cvtColor(src, grayimg, cv::COLOR_BGR2GRAY);
    cv::imshow("src", src);
    while(true) {
        cv::threshold(grayimg, aftr_Graythd, threhdGray, 255, cv::THRESH_BINARY);
        cv::threshold(rchlimg, aftr_Redthd, threhdRed, 255, cv::THRESH_BINARY);
        cv::threshold(grayimg, white_mask, threWrite, 255, cv::THRESH_BINARY_INV);
        cv::bitwise_and(aftr_Redthd, aftr_Graythd&white_mask, and_img);
        cv::Mat imgDil, imgEro, imgDil2;
        cv::dilate(and_img, imgDil, kernel1);
        cv::dilate(imgDil, imgDil2, kernel1);
        cv::erode(imgDil2, imgEro, kernel2);
        cv::imshow("andimg", and_img);
        cv::waitKey(10);
    }
	return 0;
}
