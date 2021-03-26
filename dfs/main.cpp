#include "main.h"

std::shared_ptr<EnvironmentInterface> env_ptr;
std::shared_ptr<planner::Dfs> dfs;

void
eventCallback(int event, int x, int y, int flags, void *param)
{
    if ((event == CV_EVENT_LBUTTONDOWN) || (flags & CV_EVENT_FLAG_LBUTTON))
    {
        if (!env_ptr->insideGridFromDisp(x, y))
        {
            return;
        }
        env_ptr->setGridValueFromDisp(x, y, 0);
        env_ptr->getGridValueFromDisp(x, y);
    }
    else if (event == CV_EVENT_RBUTTONDBLCLK || (flags & CV_EVENT_FLAG_RBUTTON))
    {
        static int cnt=0;
        dfs->setGoal(x, y);
        dfs->planning();
        std::cout << "Right Button " << cnt++ << std::endl;

    }
    else if (event == CV_EVENT_MBUTTONDBLCLK || (flags & CV_EVENT_FLAG_MBUTTON))
    {
        static int cnt=0;
        dfs->setStart(x, y);
        dfs->planning();
        std::cout << "Middle Button " << cnt++ << std::endl;
    }
}

int
main(int argc, char* argv[])
{
    env_ptr = std::make_shared<planner::Environment>();
    dfs = std::make_shared<planner::Dfs>();
    env_ptr->initialize(100, 200, 5);
    dfs->initialize(env_ptr);

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