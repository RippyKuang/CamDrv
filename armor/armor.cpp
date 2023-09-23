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
        for (const auto &contour: contours) {
            if (cv::contourArea(contour) > 30) {
                cv::RotatedRect ellipse = cv::fitEllipse(contour);
                if (ellipse.size.height / ellipse.size.width > 3 && (ellipse.angle < 30 || ellipse.angle > 150)) {
                    auto *l = new LightBar(contour, ellipse);
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

Armor::Armor(cv::Mat &img, LightBar *lb_1, LightBar *lb_2) {
    LB_1 = lb_1;
    LB_2 = lb_2;
    std::vector<cv::Point> v1 = LB_1->getContour();
    std::vector<cv::Point> v2 = LB_2->getContour();

    cv::RotatedRect rect_1 = cv::minAreaRect(v1);
    cv::RotatedRect rect_2 = cv::minAreaRect(v2);
    cv::Point2f vertex_1[4], vertex_2[4];
    rect_1.points(vertex_1);
    rect_2.points(vertex_2);
    if ((std::pow(vertex_1[0].x - vertex_1[3].x, 2) + std::pow(vertex_1[0].y - vertex_1[3].y, 2)) >
        (std::pow(vertex_1[0].x - vertex_1[1].x, 2) + std::pow(vertex_1[0].y - vertex_1[1].y, 2))) {
        cv::Point2f pt1 = (vertex_1[0] + vertex_1[1]) * 0.5;
        cv::Point2f pt2 = (vertex_1[2] + vertex_1[3]) * 0.5;
        boundaryPoints[0] = pt1 + 0.65 * (pt1 - pt2);
        boundaryPoints[1] = pt2 + 0.65 * (pt2 - pt1);
    } else {
        cv::Point2f pt1 = (vertex_1[0] + vertex_1[3]) * 0.5;
        cv::Point2f pt2 = (vertex_1[2] + vertex_1[1]) * 0.5;
        boundaryPoints[1] = pt1 + 0.65 * (pt1 - pt2);
        boundaryPoints[0] = pt2 + 0.65 * (pt2 - pt1);
    }
    if ((std::pow(vertex_2[0].x - vertex_2[3].x, 2) + std::pow(vertex_2[0].y - vertex_2[3].y, 2)) >
        (std::pow(vertex_2[0].x - vertex_2[1].x, 2) + std::pow(vertex_2[0].y - vertex_2[1].y, 2))) {
        cv::Point2f pt1 = (vertex_2[0] + vertex_2[1]) * 0.5;
        cv::Point2f pt2 = (vertex_2[2] + vertex_2[3]) * 0.5;
        boundaryPoints[2] = pt1 + 0.65 * (pt1 - pt2);
        boundaryPoints[3] = pt2 + 0.65 * (pt2 - pt1);
    } else {
        cv::Point2f pt1 = (vertex_2[0] + vertex_2[3]) * 0.5;
        cv::Point2f pt2 = (vertex_2[2] + vertex_2[1]) * 0.5;
        boundaryPoints[3] = pt1 + 0.65 * (pt1 - pt2);
        boundaryPoints[2] = pt2 + 0.65 * (pt2 - pt1);

    }
    if (boundaryPoints[0].x > boundaryPoints[2].x) {
        cv::Point2f temp = boundaryPoints[0];
        boundaryPoints[0] = boundaryPoints[2];
        boundaryPoints[2] = temp;
        temp = boundaryPoints[1];
        boundaryPoints[1] = boundaryPoints[3];
        boundaryPoints[3] = temp;
    }
    cv::Point2f src[4] = {boundaryPoints[0], boundaryPoints[1], boundaryPoints[3], boundaryPoints[2]};
    cv::Point2f dst[4] = {{0,  0},
                          {0,  48},
                          {36, 48},
                          {36, 0}};
    cv::Mat permatrix = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(img, imgwarp, permatrix, cv::Size(36, 48));

}

void Armor::lightBarCluster(cv::Mat &src, std::vector<LightBar *> &LBs, std::vector<Armor *> &ARMORs) {
    unsigned long nofLightBar = LBs.size();
    std::vector<int> matched;
    std::vector<std::vector<int>> parallelSet;
    for (int x = 0; x < nofLightBar; x++) {
        std::vector<int> par;
        par.push_back(x);
        matched.push_back(x);
        for (int i = 0; i < nofLightBar; i++) {
            bool isRepeat = false;
            for (int j: matched) {
                if (i == j) {
                    isRepeat = true;
                    break;
                }
            }
            if (Armor::isParallel(LBs[x], LBs[i]) && !isRepeat) {
                matched.push_back(i);
                par.push_back(i);
            }
        }
        if (par.size() != 1)
            parallelSet.push_back(par);
    }
    for (auto &x: parallelSet) {
        if (x.size() == 2) {
            ARMORs.push_back(new Armor(src, LBs[x[0]], LBs[x[1]]));
        } else {
            double d1 = std::sqrt(std::pow(
                    LBs[x[0]]->getEllipse().center.x - LBs[x[1]]->getEllipse().center.x, 2) +
                                  std::pow(LBs[x[0]]->getEllipse().center.y -
                                           LBs[x[1]]->getEllipse().center.y, 2));
            double d2 = std::sqrt(std::pow(
                    LBs[x[0]]->getEllipse().center.x - LBs[x[2]]->getEllipse().center.x, 2) +
                                  std::pow(LBs[x[0]]->getEllipse().center.y -
                                           LBs[x[2]]->getEllipse().center.y, 2));
            double d3 = std::sqrt(std::pow(
                    LBs[x[1]]->getEllipse().center.x - LBs[x[2]]->getEllipse().center.x, 2) +
                                  std::pow(LBs[x[1]]->getEllipse().center.y -
                                           LBs[x[2]]->getEllipse().center.y, 2));
            if ((d1 > d2 && d2 > d3) || (d1 < d2 && d2 < d3)) {
                ARMORs.push_back(new Armor(src, LBs[x[0]], LBs[x[2]]));
                continue;
            }
            if ((d3 > d1 && d1 > d2) || (d2 > d1 && d1 > d3)) {
                ARMORs.push_back(new Armor(src, LBs[x[0]], LBs[x[1]]));
                continue;
            }
            if ((d1 > d3 && d3 > d2) || (d2 > d3 && d3 > d1)) {
                ARMORs.push_back(new Armor(src, LBs[x[1]], LBs[x[2]]));
                continue;
            }

        }
    }
}

void Armor::drawArmorBoundary(cv::Mat src) {
    cv::line(src, boundaryPoints[0], boundaryPoints[1], cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::line(src, boundaryPoints[1], boundaryPoints[3], cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::line(src, boundaryPoints[3], boundaryPoints[2], cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::line(src, boundaryPoints[2], boundaryPoints[0], cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::putText(src, "0", boundaryPoints[0], cv::FONT_HERSHEY_DUPLEX, 2,
                cv::Scalar(255, 0, 0), 1);
    cv::putText(src, "1", boundaryPoints[1], cv::FONT_HERSHEY_DUPLEX, 2,
                cv::Scalar(255, 0, 0), 1);
    cv::putText(src, "2", boundaryPoints[2], cv::FONT_HERSHEY_DUPLEX, 2,
                cv::Scalar(255, 0, 0), 1);
    cv::putText(src, "3", boundaryPoints[3], cv::FONT_HERSHEY_DUPLEX, 2,
                cv::Scalar(255, 0, 0), 1);
}

void Armor::showArmor(std::string s) {
    cv::imshow(s, imgwarp);
    cv::waitKey(0);
}

std::vector<float> Armor::forward() {
    std::vector<float> res;
    cv::dnn::Net net = cv::dnn::readNetFromONNX("/home/kuang/project/CamDrv/model.onnx");
    cv::Scalar mean;
    cv::Mat stddevMat, blob, grayimg;
    cv::cvtColor(imgwarp, grayimg, cv::COLOR_BGR2GRAY);
    cv::meanStdDev(grayimg, mean, stddevMat);
    cv::dnn::blobFromImage(grayimg, blob, 1, cv::Size(48, 36), mean, true, false);
    net.setInput(blob);  // 设置模型输入
    cv::Mat predict = net.forward(); // 推理出结果
    double total = 0;
    double MAX = predict.at<float>(0);
    for (int x = 0; x < predict.cols; x++)
        MAX = std::fmax(predict.at<float>(x), MAX);
    for (int x = 0; x < predict.cols; x++)
        total += std::exp(predict.at<float>(x) - MAX);
    for (int x = 0; x < predict.cols; x++)
        res.push_back(std::exp((predict.at<float>(x) - MAX) / total));
    return res;
}