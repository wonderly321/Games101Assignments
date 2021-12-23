//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <iostream>
// #include "Scene.hpp"
// #include "Renderer.hpp"
// #include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <functional>
#include <queue>

int MAX_THREAD_SIZE = 4;
int MAX_QUEUE_SIZE = 1000;
std::mutex _lock, _buf_lock;
std::condition_variable _no_empty, _no_full;




struct task_data
{
    int m;
    unsigned int id;
};
struct task
{
    task_data data;
    std::function<void(task_data)> func;
};

bool ExitThread;
int Thread_ID;
std::queue<task> task_queue;
int width = 500;
int height = 300;

void Add_Thread(const task& t)
{
    while (true)
    {
        std::unique_lock<std::mutex> Lock(_lock);
        if (!task_queue.size() < MAX_QUEUE_SIZE)
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
void Run(int id)
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
        else if (ExitThread)
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
void UpdateProgress(float index)
{
    int barWidth = 70;
    float progress = index;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << int(progress * 100.0) << " %\r";
    std::cout.flush();
}
void CastRayTask(task_data data)
{
    std::unique_lock<std::mutex> Lock(_buf_lock);
    Thread_ID++;
    UpdateProgress(Thread_ID / 15000.0);
    Lock.unlock();
}



int main(int argc, char** argv)
{
    // 多线程 start

    ExitThread = false;
    std::vector<std::thread*> threads;
    for (int i = 0; i < MAX_THREAD_SIZE; i++)
    {
        std::thread* t = new std::thread([=] {Run(i); });
        threads.push_back(t);
    }

    Thread_ID = 0;
    // 多线程 end

    int m = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < height; ++j)
    {
        for (uint32_t i = 0; i < width; ++i)
        {

            for (int k = 0; k < spp; k++)
            {
                // 多线程 start
                task_data data = task_data{m};
                auto func = std::bind(CastRayTask, std::placeholders::_1);
                task task = { data, func };
                Add_Thread(task);
                // 多线程 end
            }
            m++;
        }
    }
    ExitThread = true;
    for (auto& t : threads)
    {
        //上一个线程结束后，再开启下一个线程
        try
        {
            t->join();
        }

        catch (const std::system_error& e) {
            std::cout << "Caught system_error with code " << e.code()
                << " meaning " << e.what() << '\n';
        }
    }

    return 0;

}


