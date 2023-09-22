//
// Created by kuang on 20/09/23.
//
#include "armor.hpp"
#include "CamDrv.hpp"
#include <vector>

LightBar::LightBar(std::vector<cv::Point> c, cv::RotatedRect rRect) {
    contour = c;
    ellipse = std::move(rRect);
}

void LightBar::findLightBar(cv::Mat &src, std::vector<LightBar *> &LBs, unsigned char targetColor) {
    if (targetColor == RED) {

        int threhdGray = 200, threhdRed = 245, threWrite = 240;
        cv::Mat kernel1 = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::Mat grayimg, aftr_Graythd, aftr_Redthd, and_img, white_mask, imgDil;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<cv::Mat> Trichlimgs;
        cv::split(src, Trichlimgs);
        cv::Mat rchlimg = Trichlimgs[2];
        cv::cvtColor(src, grayimg, cv::COLOR_BGR2GRAY);
        cv::threshold(grayimg, aftr_Graythd, threhdGray, 255, cv::THRESH_BINARY);
        cv::threshold(rchlimg, aftr_Redthd, threhdRed, 255, cv::THRESH_BINARY);
        cv::threshold(grayimg, white_mask, threWrite, 255, cv::THRESH_BINARY_INV);
        cv::bitwise_and(aftr_Redthd, aftr_Graythd & white_mask, and_img);
        dilate(and_img, imgDil, kernel1);

        cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        for (int x = 0; x < contours.size(); x++) {
            if (cv::contourArea(contours[x]) > 30) {
                cv::RotatedRect ellipse = cv::fitEllipse(contours[x]);
                if (ellipse.size.height / ellipse.size.width > 3 && (ellipse.angle < 30 || ellipse.angle > 150)) {
                    auto *l = new LightBar(contours[x], ellipse);
                    LBs.push_back(l);
                }
            }
        }

    } else {
        //TODO: Remove this comment
    }
};


void Armor::drawBoundary(cv::Mat& src, LightBar *lb_1, LightBar *lb_2) {
    std::vector<cv::Point> v;
    std::vector<cv::Point> v1 = lb_1->getContour();
    std::vector<cv::Point> v2 = lb_2->getContour();
    for (int x = 0; x < v1.size(); ++x)
        v.push_back(v1[x]);
    for (int x = 0; x < v2.size(); ++x)
        v.push_back(v2[x]);
    cv::RotatedRect rect = cv::minAreaRect(v);
    cv::Point2f pts[4];
    rect.points(pts);

    for (int i = 0; i < 4; i++) {
        cv::line(src, pts[i % 4], pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 2, 8, 0);
    }
    cv::Point2f cpt = rect.center;
    cv::circle(src, cpt, 2, cv::Scalar(255, 0, 0), 2, 8, 0);
};