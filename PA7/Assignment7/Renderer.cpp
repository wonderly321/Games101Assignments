//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

int MAX_THREAD_SIZE = 8;
int MAX_QUEUE_SIZE = 1000;
std::mutex _lock, _buf_lock;
std::condition_variable _no_empty, _no_full;

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;


// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene)
{   
    // 多线程 start
    
    ExitThread = false;
    std::vector<std::thread*> threads;
    for (int i = 0; i < MAX_THREAD_SIZE; i++)
    {
        std::thread* t = new std::thread([=] {Run(i);});
        threads.push_back(t);
    }

    renderer_scene = (Scene*)(&scene);

    // std::vector<Vector3f> framebuffer(scene.width * scene.height);
    framebuffer.resize(scene.width * scene.height);
    Thread_ID = 0;
    // 多线程 end

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j)
    {
        for (uint32_t i = 0; i < scene.width; ++i)
        {

            for (int k = 0; k < spp; k++)
            {   
                // generate primary ray direction
                float x = (2 * (i + get_random_float()) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + get_random_float()) / (float)scene.height) * scale;
                Vector3f dir = normalize(Vector3f(-x, y, 1));
                // 多线程 start
                task_data data = task_data{m, eye_pos, dir, 0, spp};
                auto func = std::bind(&Renderer::CastRayTask, this, std::placeholders::_1);
                task task = {data, func};
                Add_Thread(task);
                
                // framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                // 多线程 end
            }
            m++;
        }
    }
    ExitThread = true;
    for (auto& t : threads)
    {
        //上一个线程结束后，再开启下一个线程
        t->join();
    }

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
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
void Renderer::Add_Thread(const task& t)
{
    while (true)
    {
        std::unique_lock<std::mutex> Lock(_lock);
        if(!task_queue.size() < MAX_QUEUE_SIZE)
        {
            task_queue.push(t);
            Lock.unlock();
            _no_empty.notify_all();
            break;
        }
        else
        {
            _no_full.wait(Lock);
        }
    }
    
}
void Renderer::Run(int id)
{
    while (true)
    {
        std::unique_lock<std::mutex> Lock(_lock);
        if (!task_queue.empty())
        {
            task t = task_queue.front();
            task_queue.pop();

            Lock.unlock();
            t.data.id = id;
            t.func(t.data);

            _no_full.notify_all();
        }
        else if(ExitThread)
        {
            Lock.unlock();
            break;
        }
        else
        {
            _no_empty.wait_for(Lock, std::chrono::milliseconds(50));
        }
    }
    
    
}
void Renderer::CastRayTask(task_data data)
{
    std::unique_lock<std::mutex> Lock(_buf_lock);
    framebuffer[data.m] += renderer_scene->castRay(Ray(data.eye_pos, data.dir), data.depth) /data.SPP;
    float line = (Thread_ID++)/(renderer_scene->width * data.SPP);
    UpdateProgress(line);
    Lock.unlock();
}

