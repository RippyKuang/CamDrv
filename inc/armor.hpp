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
    double length;
public:
    LightBar(std::vector<cv::Point>, cv::RotatedRect);

    cv::RotatedRect &getEllipse() {
        return ellipse;
    };

    std::vector<cv::Point> &getContour() {
        return contour;
    };

    double getLength() const;

    void static findLightBar(cv::Mat &, std::vector<LightBar *> &, unsigned char);
};

class Armor {
private:
    LightBar *LB_1;
    LightBar *LB_2;
    cv::Point2f boundaryPoints[4];
    cv::Point2f lightBarBoundary[4];
    cv::Mat imgwarp;

    bool static isParallel(LightBar *, LightBar *);

public:
    Armor(cv::Mat &img, LightBar *, LightBar *);

    ~Armor() {
        delete LB_1;
        delete LB_2;

    }

    std::vector<double> forward();

    double getScore();

    void drawArmorBoundary(cv::Mat &);

    void drawLBarBoundary(cv::Mat &);

    cv::Point2f *getBoundary();

    cv::Point2f *getLightBarBoundary();

    void showArmor(const std::string &);

    void static lightBarCluster(cv::Mat &, std::vector<LightBar *> &, std::vector<Armor *> &ARMORs);
};


class Tracker {
private:
    unsigned char target_color;
    Armor *target = nullptr;
    int patience = 5;
public:
    explicit Tracker(unsigned char t) : target_color(t) {};

    bool push(cv::Mat &);

    Armor *getTarget();

    void get(std::vector<cv::Point2d> &, int &, bool &);

};

#endif //CAMDRV_ARMOR_HPP