/*
 * main.cpp
 *
 *  Created on: 20 Sept 2023
 *      Author: kuang
 */
#include "CamDrv.hpp"
#include "armor.hpp"


int main() {


    cv::VideoCapture capture("/home/kuang/project/CamDrv/2.mp4");
    cv::Mat src;
    Tracker tracker;
    while(1) {
        std::vector<LightBar *> LBs;
        std::vector<Armor *> AMs;
        capture >> src;
        LightBar::findLightBar(src, LBs, BLUE);
        if (!LBs.empty()) {
            Armor::lightBarCluster(src, LBs, AMs);
            if (!AMs.empty()) {
//               for(int x=0;x<AMs.size();x++) {
//                   AMs[x]->drawArmorBoundary(src);
//                   std::vector<float> res=AMs[x]->forward();
//                   int maxIndex=0;
//                   for(int x=0;x<res.size();x++){
//                       if(res[x]>res[maxIndex])
//                           maxIndex=x;
//                   }
//                   cv::putText(src, std::to_string(maxIndex), AMs[x]->getBoundary()[0], cv::FONT_HERSHEY_DUPLEX, 2,
//                               cv::Scalar(255, 0, 0), 2);
//              }
            tracker.push(AMs);
            Armor* a=tracker.getTarget();
            a->drawArmorBoundary(src);

            }
        }
        cv::imshow("src",src);
        cv::waitKey( 0);
    }



//    MVCamera *c = new MVCamera();
//    cv::VideoWriter out;  //创建写入的对象
//
//    int cnt=0;
//    out.open("out.avi",cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),60,
//             cv::Size(640,
//                  480)
//    );
//    std::string path = "/home/kuang/project/CamDrv/config/camera.yaml";
//    c->load_param(path); //加载参数
//    c->open(); //打开摄像头
//    c->initTrackbar(); //初始化滑动条
//    cv::Mat a;
//    while (1) {
//        c->get_Mat(a);
//        cv::imshow("src", a);
//        int key = cv::waitKey(30);
//        if (key == 'w')
//            c->onceWB(); //做一次白平衡
//        if (key == 'q')
//            break;
//        if (key == 's')
//            c->write_param(path); //把参数写进区
//        if (key == 'c')
//            cv::imwrite("./"+std::to_string(cnt++)+".jpg",a);
//        if(key=='v')
//        {
//            while(1) {
//                c->get_Mat(a);
//                cv::imshow("src", a);
//                int key = cv::waitKey(30);
//                out.write(a);
//                if(key=='q')
//                    break;
//            }
//        }
//    }
//    delete c;
    return 0;
}
