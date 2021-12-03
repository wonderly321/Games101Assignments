//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = std::max(0.0f, std::min(1.0f, u));
        v = std::max(0.0f, std::min(1.0f, v));
        auto u_img = u * width;
        auto v_img = (1.0f - v) * height;
        float u_min = std::floor(u_img);
        float u_max = std::min((float)width, std::ceil(u_img));

        float v_min = std::floor(v_img);
        float v_max = std::min((float)height, std::ceil(v_img));

        auto color_11 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto color_12 = image_data.at<cv::Vec3b>(v_max, u_max);
        auto color_21 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto color_22 = image_data.at<cv::Vec3b>(v_min, u_max);

        float ru = (u_img - u_min) / (u_max - u_min);
        float rv = (v_img - v_max) / (v_min - v_max);
        auto color_top = (1.0f - ru) * color_21 + ru * color_22;
        auto color_bottom = (1.0f - ru) * color_11 + ru * color_12;
        auto color = (1.0f - rv) * color_bottom + rv * color_top;

        // auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
