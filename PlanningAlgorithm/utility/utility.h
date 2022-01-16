#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <queue>
#include <assert.h>

#include <typedefine.h>

namespace environment
{
    /**
     * @brief 对路径进行插值
     * 
     * @param global_plan_in 待插值的路径
     * @param global_plan_out 插值完成的路径
     * @param resolution 目标插值分辨率
     */
    inline void adjustPlanResolution(const std::vector<PathNode> &global_plan_in,
                              std::vector<PathNode> &global_plan_out, float resolution)
    {
        if (global_plan_in.empty())
        {
            return;
        }
        float last_x = global_plan_in[0].x;
        float last_y = global_plan_in[0].y;
        global_plan_out.push_back(global_plan_in[0]);

        // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
        float min_sq_resolution = resolution * resolution * 4;

        for (unsigned int i = 1; i < global_plan_in.size(); ++i)
        {
            float loop_x = global_plan_in[i].x;
            float loop_y = global_plan_in[i].y;
            float sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
            if (sqdist > min_sq_resolution)
            {
                int steps = ((sqrt(sqdist) - sqrt(min_sq_resolution)) / resolution) - 1;
                // add a points in-between
                float deltax = (loop_x - last_x) / steps;
                float deltay = (loop_y - last_y) / steps;
                // TODO: Interpolate orientation
                for (int j = 1; j < steps; ++j)
                {
                    PathNode pose;
                    pose.x = last_x + j * deltax;
                    pose.y = last_y + j * deltay;
                    pose.r = global_plan_in[i].r;
                    pose.g = global_plan_in[i].g;
                    pose.b = global_plan_in[i].b;
                    pose.a = global_plan_in[i].a;
                    global_plan_out.push_back(pose);
                }
            }
            global_plan_out.push_back(global_plan_in[i]);
            last_x = loop_x;
            last_y = loop_y;
        }
    }
}
