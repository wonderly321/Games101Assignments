#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include <functional>
#include <map>
#include <chrono>

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    auto start = std::chrono::system_clock::now();
    if (primitives.empty())
        return;
    printf("pirmitives size: %d\n", primitives.size());
    root = recursiveBuild(primitives, maxPrimsInNode, splitMethod);

    auto stop = std::chrono::system_clock::now();
    // std::cout << " \rBVH Generation complete: \nTime Taken: " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";
    std::cout << " \rBVH Generation complete: \nTime Taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " milliseconds\n";

}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects, int maxPrimsInNode, SplitMethod splitMethod)
{
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) // 这里是因为maxPrimsInNode = 1?
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{objects[0]}, maxPrimsInNode, splitMethod);
        node->right = recursiveBuild(std::vector{objects[1]}, maxPrimsInNode, splitMethod);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        //不用整体的bbox而是用中点，因为更希望划分开每个object，而不是划分开bbox。
        //即希望得到object之间的差异最大的维度，而不是包围盒跨度最大的维度。
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
        {
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }

        int dim = centroidBounds.maxExtent();
        std::vector<Object *> leftshapes;
        std::vector<Object *> rightshapes;
        switch (splitMethod)
        {
        case SplitMethod::NAIVE:
            switch (dim)
            {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().x <
                                   f2->getBounds().Centroid().x; });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().y <
                                   f2->getBounds().Centroid().y; });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().z <
                                   f2->getBounds().Centroid().z; });
                break;
            }

            leftshapes = std::vector<Object *>(objects.begin(), objects.begin() + objects.size() / 2);
            rightshapes = std::vector<Object *>(objects.begin() + objects.size() / 2, objects.end());
            break;
        case SplitMethod::SAH:
            int min_pos = 0;
            int bucketCount = 32;
            std::vector<Bounds3> boundsBuckets(bucketCount, Bounds3());
            std::map<int, std::vector<int>> objBuckets;

            for (int j = 0; j < objects.size(); ++j)
            {
                auto centroid = objects[j]->getBounds().Centroid();
                auto tmp = bucketCount * centroidBounds.Offset(centroid);
                int bid = std::min(bucketCount - 1, int(dim == 0 ? tmp.x : (dim == 1 ? tmp.y : tmp.z)));
                boundsBuckets[bid] = Union(boundsBuckets[bid], centroid);
                objBuckets[bid].push_back(j);
            }
            std::vector<Bounds3> pre_bounds_l, pre_bounds_r;
            std::vector<int> pre_count_l, pre_count_r;

            Bounds3 bound_l, bound_r;
            int cnt_l = 0, cnt_r = 0;

            for (int k = 0; k < bucketCount - 1; ++k)
            {
                bound_l = Union(bound_l, boundsBuckets[k]);
                pre_bounds_l.push_back(bound_l);
                bound_r = Union(bound_r, boundsBuckets[bucketCount - 1 - k]);
                pre_bounds_r.push_back(bound_r);
                cnt_l += objBuckets[k].size();
                pre_count_l.push_back(cnt_l);
                cnt_r += objBuckets[bucketCount - 1 - k].size();
                pre_count_r.push_back(cnt_r);
            }
            float min_cost = std::numeric_limits<float>::max();
            for (int p = 0; p < bucketCount - 1; ++p)
            {
                if (objBuckets[p].size() == 0)
                    continue;
                float cur_cost = pre_count_l[p] * pre_bounds_l[p].SurfaceArea() + pre_count_r[bucketCount - 2 - p] * pre_bounds_r[bucketCount - 2 - p].SurfaceArea();
                min_cost = std::min(min_cost, cur_cost);
                if (min_cost == cur_cost)
                {
                    min_pos = p;
                }
            }
            for (int q = 0; q < bucketCount; ++q)
            {
                Object *obj;
                for (int m = 0; m < objBuckets[q].size(); ++m)
                {
                    auto j = objBuckets[q][m];
                    obj = objects[objBuckets[q][m]];

                    if (q <= min_pos)
                        leftshapes.push_back(obj);
                    else
                        rightshapes.push_back(obj);
                }
            }
            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            break;
        }

        node->left = recursiveBuild(leftshapes, maxPrimsInNode, splitMethod);
        node->right = recursiveBuild(rightshapes, maxPrimsInNode, splitMethod);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // TODO Traverse the BVH to find intersection
    //递归BVH结构求最小的光线交点
    Intersection intersect;
    std::array<int, 3> dirIsNeg = {int(ray.direction.x > 0),
                                   int(ray.direction.y > 0),
                                   int(ray.direction.z > 0)};
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return intersect;
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);
    Intersection intersect1 = getIntersection(node->left, ray);
    Intersection intersect2 = getIntersection(node->right, ray);
    return intersect1.distance < intersect2.distance ? intersect1 : intersect2;
}