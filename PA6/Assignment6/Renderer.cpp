//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <opencv2/opencv.hpp>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(-1, 5, 10);
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            // TODO: Find the x and y positions of the current pixel to get the
            // direction
            //  vector that passes through it.
            // Also, don't forget to multiply both of them with the variable
            // *scale*, and x (horizontal) variable with the *imageAspectRatio*

            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            Ray ray(eye_pos, normalize(dir));
            framebuffer[m++] = scene.castRay(ray, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);   

    
    // show as PNG
    // auto h = scene.height;
    // auto w = scene.width;
    // cv::Mat window = cv::Mat(cv::Size(w, h), CV_8UC3, cv::Scalar(0));
    // cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    // cv::namedWindow("Ray Tracing", cv::WINDOW_AUTOSIZE);

    // for (auto i = 0; i < h * w; ++i)
    // {
    //     window.at<cv::Vec3b>(i / w, i % w)[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
    //     window.at<cv::Vec3b>(i / w, i % w)[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
    //     window.at<cv::Vec3b>(i / w, i % w)[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
    // }
    // // cv::imwrite("raytrace.jpg", window);
    // int key = 0;
    // while (key != 27)
    // {
    //     cv::imshow("Ray Tracing", window);
    //     key = cv::waitKey(10);
    // } 
}
