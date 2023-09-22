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
}

bool Armor::isParallel(LightBar *lb_1, LightBar *lb_2) {
    float angle_1 = std::abs(lb_1->getEllipse().angle);
    float angle_2 = std::abs(lb_2->getEllipse().angle);
    if (angle_1 > angle_2) {
        float temp = angle_1;
        angle_1 = angle_2;
        angle_2 = temp;
    }
    if (angle_2 - angle_1 < 5 || angle_1 + 180 - angle_2 < 5) {
        return true;
    } else {
        return false;
    }
}

void Armor::drawBoundary(cv::Mat &src, LightBar *lb_1, LightBar *lb_2) {
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
//    cv::putText(src, std::to_string(int(rect.angle)), cpt, cv::FONT_HERSHEY_DUPLEX, 2,
//                cv::Scalar(255, 0, 255), 1);
};

void Armor::lightBarCluster(cv::Mat src, std::vector<LightBar *> &LBs) {
    for (int x = 0; x < LBs.size(); x++) {
        cv::putText(src, std::to_string(int(LBs[x]->getEllipse().angle)), LBs[x]->getEllipse().center,
                    cv::FONT_HERSHEY_DUPLEX, 1,
                    cv::Scalar(255, 0, 0), 1);
        cv::ellipse(src, LBs[x]->getEllipse(), cv::Scalar(0, 255, 0), 1);
    }
    int nofLightBar = LBs.size();
    std::vector<int> matched;
    std::vector<std::vector<int>> parallelSet;
    for (int x = 0; x < nofLightBar; x++) {
        std::vector<int> par;
        par.push_back(x);
        matched.push_back(x);
        for (int i = 0; i < nofLightBar; i++) {
            bool isRepeat = false;
            for (int j = 0; j < matched.size(); j++) {
                if (i == matched[j]) {
                    isRepeat = true;
                    break;
                }
            }
            if (Armor::isParallel(LBs[x], LBs[i]) && isRepeat == false) {
                matched.push_back(i);
                par.push_back(i);
            }
        }
        if (par.size() != 1)
            parallelSet.push_back(par);
    }
    for (int x = 0; x < parallelSet.size(); x++) {
        if (parallelSet[x].size() == 2) {
            Armor::drawBoundary(src, LBs[parallelSet[x][0]], LBs[parallelSet[x][1]]);
        } else {
            float d1 = std::sqrt(std::pow(
                    LBs[parallelSet[x][0]]->getEllipse().center.x - LBs[parallelSet[x][1]]->getEllipse().center.x, 2) +
                                 std::pow(LBs[parallelSet[x][0]]->getEllipse().center.y -
                                          LBs[parallelSet[x][1]]->getEllipse().center.y, 2));
            float d2 = std::sqrt(std::pow(
                    LBs[parallelSet[x][0]]->getEllipse().center.x - LBs[parallelSet[x][2]]->getEllipse().center.x, 2) +
                                 std::pow(LBs[parallelSet[x][0]]->getEllipse().center.y -
                                          LBs[parallelSet[x][2]]->getEllipse().center.y, 2));
            float d3 = std::sqrt(std::pow(
                    LBs[parallelSet[x][1]]->getEllipse().center.x - LBs[parallelSet[x][2]]->getEllipse().center.x, 2) +
                                 std::pow(LBs[parallelSet[x][1]]->getEllipse().center.y -
                                          LBs[parallelSet[x][2]]->getEllipse().center.y, 2));
            if ((d1 > d2 && d2 > d3) || (d1 < d2 && d2 < d3))
                Armor::drawBoundary(src, LBs[parallelSet[x][0]], LBs[parallelSet[x][2]]);
            if ((d3 > d1 && d1 > d2) || (d2 > d1 && d1 > d3))
                Armor::drawBoundary(src, LBs[parallelSet[x][0]], LBs[parallelSet[x][1]]);
            if ((d1 > d3 && d3 > d2) || (d2 > d3 && d3 > d1))
                Armor::drawBoundary(src, LBs[parallelSet[x][1]], LBs[parallelSet[x][2]]);

        }
    }
}