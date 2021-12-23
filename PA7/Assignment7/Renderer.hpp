//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"
#include <functional>
#include <queue>

#pragma once


struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object *hit_obj;
};
struct task_data
{
    int m;
    Vector3f eye_pos;
    Vector3f dir;
    int depth;
    int SPP;
    unsigned int id;
};
struct task
{
    task_data data;
    std::function<void(task_data)> func;
};

class Renderer
{
public:
    void Render(const Scene &scene);
    void CastRayTask(task_data);
    void Run(int id);
    void Add_Thread(const task&);
   
private:
    Scene *renderer_scene;
    std::vector<Vector3f> framebuffer;
    bool ExitThread;
    unsigned int Thread_ID;
    std::queue<task> task_queue;
};