//
// Created by tangyizhi on 18-12-18.
//

#include "camodocal/camera_models/CataCamera.h"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    camodocal::CataCamera::Parameters parameters;
    parameters.readFromYamlFile("camera_camera_calib.yaml");
    camodocal::CataCamera cata_camera(parameters);
    cv::Mat img = cv::imread("../data/images/img0.bmp");

    cv::Mat map1, map2;
    cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F);
    rmat.at<float>(0, 0) = 0.1;
    rmat.at<float>(1, 1) = 0.1;

    cata_camera.initUndistortRectifyMap(map1, map2,
                                        parameters.gamma1(), parameters.gamma2(),
                                        img.size(),
                                        parameters.u0(), parameters.v0(),
                                        rmat);
    cv::Mat un_img;
    cv::remap(img, un_img, map1, map2, cv::INTER_CUBIC);

    cv::resize(img, img, img.size() / 2);
    cv::imshow("img", img);
    cv::imshow("un_img", un_img);

    cv::waitKey(0);

    return 0;
}