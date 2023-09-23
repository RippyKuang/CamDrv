//
// Created by kuang on 20/09/23.
//

#ifndef CAMDRV_ARMOR_HPP
#define CAMDRV_ARMOR_HPP

#include <opencv2/core/mat.hpp>

#define RED 0
#define BLUE 1

class LightBar {
private:
    std::vector<cv::Point> contour;
    cv::RotatedRect ellipse;
public:
    LightBar(std::vector<cv::Point>, cv::RotatedRect);

    cv::RotatedRect &getEllipse() {
        return ellipse;
    };

    std::vector<cv::Point> &getContour() {
        return contour;
    };

    void static findLightBar(cv::Mat &, std::vector<LightBar *> &, unsigned char);
};

class Armor {
private:
    LightBar *LB_1;
    LightBar *LB_2;
    cv::Point2f boundaryPoints[4];
    cv::Mat imgwarp;
    bool isHero = false;

    bool static isParallel(LightBar *, LightBar *);

public:
    Armor(cv::Mat& img,LightBar *, LightBar *);

    void drawArmorBoundary(cv::Mat);
    void showArmor(std::string);
    void static lightBarCluster(cv::Mat &,std::vector<LightBar *> &, std::vector<Armor *> &ARMORs);


};

#endif //CAMDRV_ARMOR_HPP