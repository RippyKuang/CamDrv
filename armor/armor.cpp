//
// Created by kuang on 20/09/23.
//
#include "armor.hpp"
#include "CamDrv.hpp"
#include <vector>

LightBar::LightBar(std::vector<cv::Point> c, cv::RotatedRect rRect) {
    contour = c;
    ellipse = std::move(rRect);
    cv::RotatedRect rect_1 = cv::minAreaRect(contour);
    cv::Point2f vertex_1[4];
    rect_1.points(vertex_1);
    double d01 = std::sqrt(std::pow(vertex_1[0].x - vertex_1[1].x, 2) + std::pow(vertex_1[0].y - vertex_1[1].y, 2));
    double d03 = std::sqrt(std::pow(vertex_1[0].x - vertex_1[3].x, 2) + std::pow(vertex_1[0].y - vertex_1[3].y, 2));
    if (d03 > d01)
        length = d03;
    else
        length = d01;
}

void LightBar::findLightBar(cv::Mat &src, std::vector<LightBar *> &LBs, unsigned char targetColor) {
    int threhdGray, threhdRed, threWrite;
    if (targetColor == RED)
        threhdGray = 170, threhdRed = 235, threWrite = 250;
    else
        threhdGray = 160, threhdRed = 240, threWrite = 240;

    cv::Mat kernel1 = getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    cv::Mat grayimg, aftr_Graythd, aftr_Redthd, and_img, white_mask, imgDil, rchlimg;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Mat> Trichlimgs;
    cv::split(src, Trichlimgs);
    if (targetColor == RED)
        rchlimg = Trichlimgs[2];
    else
        rchlimg = Trichlimgs[0];
    cv::cvtColor(src, grayimg, cv::COLOR_BGR2GRAY);
    cv::threshold(grayimg, aftr_Graythd, threhdGray, 255, cv::THRESH_BINARY);
    cv::threshold(rchlimg, aftr_Redthd, threhdRed, 255, cv::THRESH_BINARY);
    cv::threshold(grayimg, white_mask, threWrite, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(aftr_Redthd, aftr_Graythd & white_mask, and_img);
    dilate(and_img, imgDil, kernel1);

    int cnt = 0;
    cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (const auto &contour: contours) {
        if (cv::contourArea(contour) > 30) {
            cv::RotatedRect ellipse = cv::fitEllipse(contour);
            if (ellipse.size.height / ellipse.size.width > 2 && (ellipse.angle < 30 || ellipse.angle > 150))
                LBs.push_back(new LightBar(contour, ellipse));
        }
    }
}

double LightBar::getLength() const {
    return length;
}

bool Armor::isParallel(LightBar *lb_1, LightBar *lb_2) {
    float angle_1 = std::abs(lb_1->getEllipse().angle);
    float angle_2 = std::abs(lb_2->getEllipse().angle);
    if (angle_1 > angle_2) {
        float temp = angle_1;
        angle_1 = angle_2;
        angle_2 = temp;
    }
    if (angle_2 - angle_1 < 3 || angle_1 + 180 - angle_2 < 3)
        return true;
    else
        return false;

}

Armor::Armor(cv::Mat &img, LightBar *lb_1, LightBar *lb_2) {
    LB_1 = lb_1;
    LB_2 = lb_2;
    std::vector<cv::Point> v1 = LB_1->getContour();
    std::vector<cv::Point> v2 = LB_2->getContour();

    cv::RotatedRect rect_1 = cv::minAreaRect(v1);
    cv::RotatedRect rect_2 = cv::minAreaRect(v2);
    if (rect_1.center.x > rect_2.center.x) {
        cv::RotatedRect temp;
        temp = rect_1;
        rect_1 = rect_2;
        rect_2 = temp;
    }

    cv::Point2f vertex_1[4], vertex_2[4];
    rect_1.points(vertex_1);
    rect_2.points(vertex_2);
    if ((std::pow(vertex_1[0].x - vertex_1[3].x, 2) + std::pow(vertex_1[0].y - vertex_1[3].y, 2)) >
        (std::pow(vertex_1[0].x - vertex_1[1].x, 2) + std::pow(vertex_1[0].y - vertex_1[1].y, 2))) {
        cv::Point2f pt1 = (vertex_1[0] + vertex_1[1]) * 0.5;
        cv::Point2f pt2 = (vertex_1[2] + vertex_1[3]) * 0.5;
        boundaryPoints[0] = pt1 + 0.5 * (pt1 - pt2);
        boundaryPoints[1] = pt2 + 0.5 * (pt2 - pt1);
        if (vertex_1[3].x < vertex_1[2].x) {                  // 1   0
            lightBarBoundary[0] = vertex_1[3];             // 2   3
            lightBarBoundary[1] = vertex_1[0];
        } else {
            lightBarBoundary[0] = vertex_1[2];
            lightBarBoundary[1] = vertex_1[1];
        }
    } else {
        cv::Point2f pt1 = (vertex_1[0] + vertex_1[3]) * 0.5;
        cv::Point2f pt2 = (vertex_1[2] + vertex_1[1]) * 0.5;
        boundaryPoints[1] = pt1 + 0.5 * (pt1 - pt2);
        boundaryPoints[0] = pt2 + 0.5 * (pt2 - pt1);
        if (vertex_1[0].x < vertex_1[3].x) {                     //2   1
            lightBarBoundary[0] = vertex_1[0];                //3   0
            lightBarBoundary[1] = vertex_1[1];
        } else {
            lightBarBoundary[0] = vertex_1[3];
            lightBarBoundary[1] = vertex_1[2];
        }
    }
    if ((std::pow(vertex_2[0].x - vertex_2[3].x, 2) + std::pow(vertex_2[0].y - vertex_2[3].y, 2)) >
        (std::pow(vertex_2[0].x - vertex_2[1].x, 2) + std::pow(vertex_2[0].y - vertex_2[1].y, 2))) {
        cv::Point2f pt1 = (vertex_2[0] + vertex_2[1]) * 0.5;
        cv::Point2f pt2 = (vertex_2[2] + vertex_2[3]) * 0.5;
        boundaryPoints[2] = pt1 + 0.5 * (pt1 - pt2);
        boundaryPoints[3] = pt2 + 0.5 * (pt2 - pt1);
        if (vertex_2[3].x < vertex_2[2].x) {                  // 1   0
            lightBarBoundary[2] = vertex_2[2];             // 2   3
            lightBarBoundary[3] = vertex_2[1];
        } else {
            lightBarBoundary[2] = vertex_2[3];
            lightBarBoundary[3] = vertex_2[0];
        }

    } else {
        cv::Point2f pt1 = (vertex_2[0] + vertex_2[3]) * 0.5;
        cv::Point2f pt2 = (vertex_2[2] + vertex_2[1]) * 0.5;
        boundaryPoints[3] = pt1 + 0.5 * (pt1 - pt2);
        boundaryPoints[2] = pt2 + 0.5 * (pt2 - pt1);
        if (vertex_2[0].x < vertex_2[3].x) {                     //2   1
            lightBarBoundary[2] = vertex_2[3];                //3   0
            lightBarBoundary[3] = vertex_2[2];
        } else {
            lightBarBoundary[2] = vertex_2[0];
            lightBarBoundary[3] = vertex_2[1];
        }

    }
    if (boundaryPoints[0].x > boundaryPoints[2].x) {
        cv::Point2f temp = boundaryPoints[0];
        boundaryPoints[0] = boundaryPoints[2];
        boundaryPoints[2] = temp;
        temp = boundaryPoints[1];
        boundaryPoints[1] = boundaryPoints[3];
        boundaryPoints[3] = temp;
    }
    if (lightBarBoundary[0].x > lightBarBoundary[2].x) {
        cv::Point2f temp = lightBarBoundary[0];
        lightBarBoundary[0] = lightBarBoundary[2];
        lightBarBoundary[2] = temp;
        temp = lightBarBoundary[1];
        lightBarBoundary[1] = lightBarBoundary[3];
        lightBarBoundary[3] = temp;
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
    auto *isReflect = (unsigned char *) malloc(sizeof(unsigned char) * LBs.size());
    auto *isMatched = (unsigned char *) malloc(sizeof(unsigned char) * LBs.size());
    memset(isReflect, 0, sizeof(unsigned char) * LBs.size() + 1);
    memset(isMatched, 0, sizeof(unsigned char) * LBs.size() + 1);
    for (int x = 0; x < LBs.size(); x++) {
        for (int i = 0; i < LBs.size(); i++) {
            if (x == i)continue;
            if (std::abs(LBs[x]->getEllipse().center.x - LBs[i]->getEllipse().center.x) < 25 &&
                (LBs[x]->getEllipse().center.y - LBs[i]->getEllipse().center.y) > 40) {
                isReflect[x] = 1;
                isMatched[x] = 1;

            }
        }
    }

    std::vector<std::vector<int>> parallelSet = {};
    for (int x = 0; x < LBs.size(); x++) {
        if (isReflect[x] == 0) {
            std::vector<int> par = {};
            par.push_back(x);
            if (isMatched[x] == 0) {
                for (int i = 0; i < LBs.size(); i++) {
                    if (isReflect[i] == 0 && isMatched[i] == 0 && i != x) {
                        if (Armor::isParallel(LBs[x], LBs[i])) {
                            isMatched[i] = 1;
                            isMatched[x] = 1;
                            par.push_back(i);
                        }
                    }
                }
                if (par.size() != 1)
                    parallelSet.push_back(par);
            }
        }
    }


    for (int x = 0; x < LBs.size(); x++) {
        if (isMatched[x] == 0) {
            bool f = false;

            for (auto &w: parallelSet) {
                if (f)break;
                for (int p = 0; p < w.size(); p++) {
                    if (isParallel(LBs[x], LBs[w[p]])) {
                        w.push_back(x);
                        isMatched[x] = 1;
                        f = true;
                        break;
                    }
                }
            }
        }
    }
    for (int x = 0; x < LBs.size(); x++)
        if (isMatched[x] == 0 || isReflect[x] == 1)
            delete LBs[x];


    for (auto &x: parallelSet) {

        if (x.size() == 2) {

            float a = std::abs(LBs[x[0]]->getEllipse().center.y - LBs[x[1]]->getEllipse().center.y);
            float b = std::abs(LBs[x[0]]->getEllipse().center.x - LBs[x[1]]->getEllipse().center.x);
            double l1 = LBs[x[0]]->getLength();
            double l2 = LBs[x[1]]->getLength();
            if (l1 < l2) {
                double t = l1;
                l1 = l2;
                l2 = t;
            }
            if (a < 10 && l1 / l2 < 1.3)
                ARMORs.push_back(new Armor(src, LBs[x[0]], LBs[x[1]]));
        } else {
            float a, b, c;

            a = std::abs(LBs[x[1]]->getEllipse().center.y - LBs[x[0]]->getEllipse().center.y);
            b = std::abs(LBs[x[1]]->getEllipse().center.y - LBs[x[2]]->getEllipse().center.y);
            c = std::abs(LBs[x[0]]->getEllipse().center.y - LBs[x[2]]->getEllipse().center.y);
            double x02 = std::sqrt(std::pow(LBs[x[2]]->getEllipse().center.x - LBs[x[0]]->getEllipse().center.x, 2) +
                                   std::pow(LBs[x[2]]->getEllipse().center.y - LBs[x[0]]->getEllipse().center.y, 2));
            double x21 = std::sqrt(std::pow(LBs[x[2]]->getEllipse().center.x - LBs[x[1]]->getEllipse().center.x, 2) +
                                   std::pow(LBs[x[2]]->getEllipse().center.y - LBs[x[1]]->getEllipse().center.y, 2));
            double x01 = std::sqrt(std::pow(LBs[x[1]]->getEllipse().center.x - LBs[x[0]]->getEllipse().center.x, 2) +
                                   std::pow(LBs[x[1]]->getEllipse().center.y - LBs[x[0]]->getEllipse().center.y, 2));
            double l0 = LBs[x[0]]->getLength();
            double l1 = LBs[x[1]]->getLength();
            double l2 = LBs[x[0]]->getLength();


            if (std::abs(a - b) > 10 || std::abs(c - b) > 10 || std::abs(a - c) > 10) {
                if (a < b && a < c) {
                    if (x01 < x02 || x01 < x21)
                        ARMORs.push_back(new Armor(src, LBs[x[0]], LBs[x[1]]));

                } else {
                    if (b < c) {
                        if (x21 < x02 || x21 < x01)
                            ARMORs.push_back(new Armor(src, LBs[x[2]], LBs[x[1]]));

                    } else {
                        if (x02 < x01 || x02 < x21)
                            ARMORs.push_back(new Armor(src, LBs[x[0]], LBs[x[2]]));
                    }
                }


            } else {
                float angle_1 = std::abs(LBs[x[0]]->getEllipse().angle);
                float angle_2 = std::abs(LBs[x[1]]->getEllipse().angle);
                float angle_3 = std::abs(LBs[x[2]]->getEllipse().angle);
                if (angle_1 < 90)angle_1 += 180;
                if (angle_2 < 90)angle_2 += 180;
                if (angle_3 < 90)angle_3 += 180;
                float d12 = std::abs(angle_1 - angle_2);
                float d23 = std::abs(angle_3 - angle_2);
                float d31 = std::abs(angle_3 - angle_1);
                int _a, _b;
                if (d12 < d23) {
                    if (d12 < d31)
                        _a = 0, _b = 1;
                    else
                        _a = 0, _b = 2;
                } else //d23<d12
                if (d23 < d31)
                    _a = 1, _b = 2;
                else
                    _a = 2, _b = 0;

                int _c, _d;

                if (x02 > x21) {
                    if (x02 > x01) {
                        if (x01 > x21)
                            _c = 1, _d = 0;
                        else
                            _c = 2, _d = 1;
                    } else
                        _c = 2, _d = 0;

                } else {
                    if (x01 > x21)
                        _c = 1, _d = 2;
                    else {
                        if (x01 > x02)
                            _c = 0, _d = 1;
                        else
                            _c = 2, _d = 0;
                    }
                }


                bool depend_on_angle = false;
                if (x01 > x02) {
                    if (x01 > x21) {
                        if (std::abs(x02 - x21) < 5)
                            depend_on_angle = true;
                    } else {
                        if (std::abs(x01 - x02) < 5)
                            depend_on_angle = true;
                    }
                } else { //x02<x01
                    if (x01 > x21) {
                        if (std::abs(x21 - x02) < 5)
                            depend_on_angle = true;
                    } else {
                        if (std::abs(x01 - x02) < 5)
                            depend_on_angle = true;
                    }
                }

                if (depend_on_angle)
                    ARMORs.push_back(new Armor(src, LBs[x[_a]], LBs[x[_b]]));
                else
                    ARMORs.push_back(new Armor(src, LBs[x[_c]], LBs[x[_d]]));


            }
        }
    }
}

void Armor::drawArmorBoundary(cv::Mat &src) {
    cv::line(src, boundaryPoints[0], boundaryPoints[1], cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::line(src, boundaryPoints[1], boundaryPoints[3], cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::line(src, boundaryPoints[3], boundaryPoints[2], cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::line(src, boundaryPoints[2], boundaryPoints[0], cv::Scalar(0, 0, 255), 2, 8, 0);
}

void Armor::showArmor(const std::string &s) {
    cv::imshow(s, imgwarp);
    cv::waitKey(0);
}

double Armor::getScore() {
    double x1 = std::abs(boundaryPoints[1].x - boundaryPoints[3].x);
    double x2 = std::abs(boundaryPoints[0].x - boundaryPoints[2].x);
    double x3 = std::abs(boundaryPoints[1].y - boundaryPoints[0].y);
    double x4 = std::abs(boundaryPoints[3].y - boundaryPoints[2].y);

    double k1 = (boundaryPoints[1].y - boundaryPoints[3].y) / (boundaryPoints[1].x - boundaryPoints[3].x);
    double k2 = (boundaryPoints[0].y - boundaryPoints[2].y) / (boundaryPoints[0].x - boundaryPoints[2].x);
    double d1 = std::abs((boundaryPoints[1].y - boundaryPoints[0].y) - (boundaryPoints[3].y - boundaryPoints[2].y));

    return x1 + x2 + x3 + x4 - 10 * std::abs(k1 - k2) - 25 * d1;
}

std::vector<double> Armor::forward() {
    std::vector<double> res;
    cv::dnn::Net net = cv::dnn::readNetFromONNX("/home/kuang/project/CamDrv/model.onnx");
    cv::Scalar mean;
    cv::Mat stddevMat, blob, grayimg;
    cv::cvtColor(imgwarp, grayimg, cv::COLOR_BGR2GRAY);
    cv::meanStdDev(grayimg, mean, stddevMat);
    cv::dnn::blobFromImage(grayimg, blob, 1, cv::Size(48, 36), mean, true, false);

    net.setInput(blob);
    cv::Mat predict = net.forward();

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

cv::Point2f *Armor::getBoundary() {
    return boundaryPoints;
}

void Armor::drawLBarBoundary(cv::Mat &src) {
    cv::line(src, lightBarBoundary[0], lightBarBoundary[1], cv::Scalar(0, 255, 255), 2, 8, 0);
    cv::line(src, lightBarBoundary[1], lightBarBoundary[3], cv::Scalar(0, 255, 255), 2, 8, 0);
    cv::line(src, lightBarBoundary[3], lightBarBoundary[2], cv::Scalar(0, 255, 255), 2, 8, 0);
    cv::line(src, lightBarBoundary[2], lightBarBoundary[0], cv::Scalar(0, 255, 255), 2, 8, 0);
}

cv::Point2f *Armor::getLightBarBoundary() {
    return lightBarBoundary;
}


Armor *Tracker::getTarget() {
    return target;
}

bool Tracker::push(cv::Mat &src) {
    std::vector<LightBar *> LBs;
    std::vector<Armor *> AMs;
    LightBar::findLightBar(src, LBs, target_color);

    if (!LBs.empty()) {
        Armor::lightBarCluster(src, LBs, AMs);
        if (!AMs.empty()) {
            if (target == nullptr || patience == 0) {
                double bestScore = -999999;
                for (auto am: AMs) {
                    double s = am->getScore();
                    if (s > bestScore) {
                        target = am;
                        bestScore = s;
                    }
                }
                patience = 2;

            } else {
                cv::Point2f *b = target->getBoundary();
                cv::Point2f center = (b[0] + b[1] + b[2] + b[3]) / 4;
                Armor *temp_target;
                double minDist = 999999;
                for (auto am: AMs) {
                    cv::Point2f *t_b = am->getBoundary();
                    cv::Point2f t_center = (t_b[0] + t_b[1] + t_b[2] + t_b[3]) / 4;
                    double dist = std::sqrt(std::pow(center.x - t_center.x, 2) + std::pow(center.y - t_center.y, 2));
                    if (dist < minDist) {
                        minDist = dist;
                        temp_target = am;
                    }
                }

                if (minDist < 100) {
                    delete target;
                    patience = 2;
                    target = temp_target;
                } else
                    patience--;
            }
            target->drawLBarBoundary(src);
            for (auto a: AMs)
                if (a != target)
                    delete a;

            return true;
        } else {
            if (target != nullptr && patience > 0)
                patience--;
            return false;
        }
    } else {
        if (target != nullptr && patience > 0)
            patience--;
        return false;
    }
}

void Tracker::get(std::vector<cv::Point2d> &corners, int &num, bool &isHero) {
    if (target != nullptr) {
        cv::Point2f *t = target->getLightBarBoundary();
        corners.push_back(t[1]);
        corners.push_back(t[3]);
        corners.push_back(t[2]);
        corners.push_back(t[0]);
        std::vector<double> res = target->forward();
        int maxIndex = 0;
        for (int x = 0; x < res.size(); x++)
            if (res[x] > res[maxIndex])
                maxIndex = x;
        num = maxIndex;
        if (std::abs(corners[1].x - corners[0].x) > 3 * std::abs(corners[1].y - corners[2].y))
            isHero = true;
        else
            isHero = false;
    }
}
