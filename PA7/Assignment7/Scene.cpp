//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir(0.0, 0.0, 0.0), L_indir(0.0, 0.0, 0.0);

    Intersection intersection = intersect(ray);
    if (!intersection.happened)
        return {};
    if (intersection.m->hasEmission())
        return depth == 0 ? intersection.m->getEmission() : Vector3f(0, 0, 0);
    Intersection light_obj;
    float light_pdf = 0.0;
    sampleLight(light_obj, light_pdf);
    Vector3f light_obj_dir = light_obj.coords - intersection.coords;
    Vector3f light_dir = light_obj_dir.normalized();
    float dis = std::pow(light_obj_dir.norm(), 2);
    Ray ray_obj2light = Ray(intersection.coords, light_dir);
    Intersection inter_light = intersect(ray_obj2light);
    if (inter_light.happened && (inter_light.coords - light_obj.coords).norm() < 1e-2)
    {
        L_dir = light_obj.emit * intersection.m->eval(ray.direction, light_dir, intersection.normal) *
                dotProduct(light_dir, intersection.normal) * dotProduct(-light_dir, light_obj.normal) / dis / light_pdf;
    }
    // double rand_p = (double) rand() / (RAND_MAX)
    if (get_random_float() > RussianRoulette)
        return L_dir;

    Vector3f wi = intersection.m->sample(ray.direction, intersection.normal).normalized();
    Ray ray_obj2obj = Ray(intersection.coords, wi);
    Intersection intersection1 = intersect(ray_obj2obj);
    if (intersection1.happened && !intersection1.m->hasEmission())
    {
        float pdf = intersection.m->pdf(ray.direction, wi, intersection.normal);
        L_indir = castRay(ray_obj2obj, depth + 1) * intersection.m->eval(ray.direction, wi, intersection.normal) *
                  dotProduct(wi, intersection.normal) / pdf / RussianRoulette;
    }
    return L_dir + L_indir;
}