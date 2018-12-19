//
// Created by tangyizhi on 18-12-18.
//

#include "camodocal/camera_models/CataCamera.h"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    camodocal::CataCamera::Parameters parameters;
    parameters.readFromYamlFile("camera_camera_calib.yaml");
    camodocal::CataCamera cata_camera(parameters);
    cv::Mat img = cv::imread("../data/images/img1.bmp");

    cv::Point mark_point(160, 650);
    cv::circle(img, mark_point, 6, cv::Scalar(0, 0, 255), 2);

    int ipm_width = 800;
    int ipm_height = 800;
    double ipm_center_x = 400;
    double ipm_center_y = 600;
    double tz = 1.1;
    double pixel_per_meter = 80;

    int img_width = img.size().width;
    int img_height = img.size().height;

    cv::Mat ipm_mat(cv::Size(ipm_width, ipm_height), CV_8UC3, cv::Scalar(0, 0, 0));

    for (int y = 0; y < ipm_height; y++) {
        for (int x = 0; x < ipm_width; x++) {
            Eigen::Vector3d P;
            P(0) = (x - ipm_center_x) / pixel_per_meter;
            P(1) = tz;
            P(2) = (ipm_center_y - y) / pixel_per_meter;
            Eigen::Vector2d p;
            cata_camera.spaceToPlane(P, p);

            if (p.y() >= 0 && p.y() < img_height && p.x() >= 0 && p.x() < img_width) {
                ipm_mat.at<cv::Vec3b>(y, x) = img.at<cv::Vec3b>(p.y(), p.x());
            }
        }
    }

    Eigen::Vector2d p;
    p << mark_point.x, mark_point.y;
    Eigen::Vector3d P;
    cata_camera.liftProjective(p, P);

    cv::Point mark_point_ground;
    mark_point_ground.x = P(0) * tz / P(1) * pixel_per_meter + ipm_center_x;
    mark_point_ground.y = ipm_center_y - P(2) * tz / P(1) * pixel_per_meter;
    cv::circle(ipm_mat, mark_point_ground, 2, cv::Scalar(0, 255, 0), 1);

    cv::resize(img, img, img.size() / 2);
    cv::imshow("img", img);
    cv::imshow("ipm", ipm_mat);
    cv::waitKey(0);

    return 0;
}