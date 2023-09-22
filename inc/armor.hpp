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
    LightBar(std::vector<cv::Point>,cv::RotatedRect);
    cv::RotatedRect& getEllipse(){
        return ellipse;
    };
     std::vector<cv::Point>& getContour(){
        return contour;
    };
    void static findLightBar(cv::Mat&,std::vector<LightBar*>&,unsigned char);
};

class Armor{
private:
    bool static isParallel(LightBar*,LightBar*);
public:
    void static drawBoundary(cv::Mat&,LightBar*,LightBar*);
    void static lightBarCluster(cv::Mat,std::vector<LightBar*>&);

};

#endif //CAMDRV_ARMOR_HPP