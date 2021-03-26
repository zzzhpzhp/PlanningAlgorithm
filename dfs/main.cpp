#include "main.h"

std::shared_ptr<EnvironmentInterface> env_ptr;

void
eventCallback(int event, int x, int y, int flags, void *param)
{
    if ((event == CV_EVENT_LBUTTONDOWN) || (flags & CV_EVENT_FLAG_LBUTTON))
    {
        if (!env_ptr->insideGrid(x, y))
        {
            return;
        }
        env_ptr->setGridValue(x, y, 0);
        env_ptr->getGridValue(x, y);
    }
}

int
main(int argc, char* argv[])
{
    env_ptr = std::make_shared<planner::Environment>();
    env_ptr->initialize(100, 200, 10);

    cv::namedWindow("disp_img");
    cv::namedWindow("planning_grid");
    cv::setMouseCallback("disp_img", eventCallback);

    while (true)
    {
        env_ptr->display();
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}