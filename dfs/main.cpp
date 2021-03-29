#include "main.h"

bool planning = false;
std::shared_ptr<EnvironmentInterface> env_ptr;
std::shared_ptr<AlgorithmInterface> alg_ptr;

void
eventCallback(int event, int x, int y, int flags, void *param)
{
    if (!env_ptr->insideGridFromDisp(x, y))
    {
        return;
    }
    if ((event == CV_EVENT_LBUTTONDOWN) || (flags & CV_EVENT_FLAG_LBUTTON))
    {
        // 鼠标左键设定起点
        if (planning)
        {
            return;
        }
        alg_ptr->setStart(x, y);
        env_ptr->markStart(x, y, 0, 255, 0);
    }
    else if (event == CV_EVENT_RBUTTONDBLCLK || (flags & CV_EVENT_FLAG_RBUTTON))
    {
        // 鼠标右键设定终点
        if (planning)
        {
            return;
        }
        alg_ptr->setGoal(x, y);
        env_ptr->markGoal(x, y, 255, 0, 0);
    }
    else if (event == CV_EVENT_MBUTTONDBLCLK || (flags & CV_EVENT_FLAG_MBUTTON))
    {
        // 鼠标中键(滚轮按下)设定障碍物
        if (!env_ptr->insideGridFromDisp(x, y))
        {
            return;
        }
        env_ptr->setGridValueFromDisp(x, y, 0);
        env_ptr->markObstacle(x, y);
    }
}

void
invoke()
{
    planning = true;
    if (!alg_ptr->planning())
    {
        std::cerr << "Planning failed." << std::endl;
    }
    else
    {
        std::cout << "Planning Succeed." << std::endl;
    }
    env_ptr->showStartGoalPose();
    planning = false;
}

int
main(int argc, char* argv[])
{
    env_ptr = std::make_shared<planner::Environment>();
    env_ptr->initialize(100, 200, 5);

    alg_ptr = std::make_shared<planner::Dfs>();
    alg_ptr->initialize(env_ptr);

    cv::namedWindow("InteractiveWindow");
    cv::namedWindow("PlanningGrid");
    cv::setMouseCallback("InteractiveWindow", eventCallback);

    std::future<void> future_planning;

    while (true)
    {
        // 更新交互显示
        env_ptr->display();
        // 读取键盘值
        auto key = cv::waitKey(5);
        // 如果正在执行,则不做任何操作
        if (planning)
        {
            continue;
        }
        if (key == 's' || key == 'S')
        {
            // 如果按下s键,执行算法
            std::cout << "Start planning..." << std::endl;
            env_ptr->clear();
            future_planning = std::async(std::launch::async, invoke);
        }
        else if (key == 'c' || key == 'C')
        {
            // 按下c键,清楚算法的执行痕迹,但是不清除已设定的起点\终点和障碍物信息
            env_ptr->clear();
        }
        else if (key == 'r' || key == 'R')
        {
            // 按下r键,清除算法执行痕迹和设定的障碍物信息
            env_ptr->reset();
        }
    }
}