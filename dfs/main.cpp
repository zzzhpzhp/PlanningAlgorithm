#include "main.h"

planner::Environment env;

void
eventCallback(int event, int x, int y, int flags, void *param)
{
    env.eventCallback(event, x, y, flags, param);
}

int
main(int argc, char* argv[])
{
    env.initialize(100, 200, 5);

    cv::namedWindow("disp_img");
    cv::namedWindow("planning_grid");
    cv::setMouseCallback("disp_img", eventCallback);

    while (true)
    {
        imshow("disp_img", env.getDisplayObj());
        imshow("planning_grid", env.getPlanningObj());
        cv::waitKey(30);
    }
    return 0;
}