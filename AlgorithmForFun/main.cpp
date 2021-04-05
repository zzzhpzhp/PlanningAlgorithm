#include "main.h"

// 算法执行成功后，是否显示算法给出的路径
bool show_path = true;
bool planning = false;
int algorithm_num = 4;
std::shared_ptr<environment::EnvironmentInterface> env_ptr;
std::shared_ptr<algorithm::AlgorithmInterface> alg_ptr;

void
eventCallback(int event, int x, int y, int flags, void *param)
{
    if (!env_ptr->insideGridFromDisp(x, y))
    {
        return;
    }
    if ((event == cv::EVENT_LBUTTONDOWN) || (flags & cv::EVENT_FLAG_LBUTTON))
    {
        // 鼠标左键设定起点
        if (planning)
        {
            return;
        }
        alg_ptr->setStart(x, y);
        env_ptr->markStart(x, y, 0, 0, 255);
    }
    else if (event == cv::EVENT_RBUTTONDBLCLK || (flags & cv::EVENT_FLAG_RBUTTON))
    {
        // 鼠标右键设定终点
        if (planning)
        {
            return;
        }
        alg_ptr->setGoal(x, y);
        env_ptr->markGoal(x, y, 255, 0, 0);
    }
    else if (event == cv::EVENT_MBUTTONDBLCLK || (flags & cv::EVENT_FLAG_MBUTTON))
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
    auto result = alg_ptr->planning();
    if (!result)
    {
        std::cerr << "Planning failed." << std::endl;
    }
    else
    {
        std::cout << "Planning Succeed." << std::endl;
    }
    if (result)
    {
        if (show_path)
        {
            auto path = alg_ptr->getPath();
            if (!path.empty())
            {
                env_ptr->drawPath(path);
            }
        }
    }
    env_ptr->showStartGoalPose();
    planning = false;
}

void switch_algorithm(int index)
{
    if (index == 0)
    {
        std::cout << "Selected algorithm is DFS." << std::endl;
        alg_ptr = std::make_shared<algorithm::Dfs>();
        alg_ptr->initialize(env_ptr);
    }
    else if (index == 1)
    {
        std::cout << "Selected algorithm is BFS." << std::endl;
        alg_ptr = std::make_shared<algorithm::Bfs>();
        alg_ptr->initialize(env_ptr);
    }
    else if (index == 2)
    {
        std::cout << "Selected algorithm is BCD." << std::endl;
        alg_ptr = std::make_shared<algorithm::Bcd>();
        alg_ptr->initialize(env_ptr);
    }
    else if (index == 3)
    {
        std::cout << "Selected algorithm is Dijkstra." << std::endl;
        alg_ptr = std::make_shared<algorithm::Dijkstra>();
        alg_ptr->initialize(env_ptr);
    }
    else
    {
        std::cerr << "The first argument is " << index << ", have no corresponding algorithm." << std::endl;
    }
}

int
main(int argc, char* argv[])
{
    env_ptr = std::make_shared<environment::Environment>();
    env_ptr->initialize(60, 100, 11);

    int selected_algorithm = 0;
    std::cout << argc << std::endl;
    if (argc > 1)
    {
        selected_algorithm = std::stoi(argv[1]);
    }

    switch_algorithm(selected_algorithm);

    if (!alg_ptr)
    {
        std::cerr << "Please set correct algorithm." << std::endl;
        return 0;
    }

    cv::setMouseCallback("InteractiveWindow", eventCallback);

    std::future<void> future_planning;

    while (true)
    {
        // 更新交互显示
        env_ptr->display();
        // 读取键盘值
        auto key = cv::waitKey(2);
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
        else if (key == 'a' || key == 'A')
        {
            // 按下a鍵，切換算法
            selected_algorithm = ++selected_algorithm % algorithm_num;
            switch_algorithm(selected_algorithm);
        }
    }
}