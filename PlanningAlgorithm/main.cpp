#include "main.h"

// 交互窗口宽度（网格）
int window_width = 162;
// 交互窗口高度（网格）
int window_height = 100;
// 一个网格的宽度，单位：像素
int cell_with = 5;
// 算法步骤执行延时（秒）
float running_delay_time = 0.0001;
// 算法执行结果演示时的步骤延时（秒）
float display_delay_time = 0.01;
// 算法执行成功后，是否显示算法给出的路径
bool show_path = true;
// 障碍物标记的宽度，以给定坐标为中心
int obstacle_radius = 3;
// 机器人半径
int robot_radius = 5;
// 当前运行的算法的索引
int selected_algorithm = 4;
// 当前运行算法的指针
std::shared_ptr<algorithm::AlgorithmInterface> alg_ptr;
// 当前环境的指针
std::shared_ptr<environment::EnvironmentInterface> env_ptr;
// 包含所有算法实例指针的容器
std::vector<std::shared_ptr<algorithm::AlgorithmInterface>> algorithms_ptr;

// 忙标志
std::atomic_bool busy{false};

// 鼠标事件回调
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
        if (busy)
        {
            return;
        }
        alg_ptr->setStart(x, y);
        env_ptr->markStart(x, y, 0, 0, 255);
    }
    else if (event == cv::EVENT_RBUTTONDBLCLK || (flags & cv::EVENT_FLAG_RBUTTON))
    {
        // 鼠标右键设定终点
        if (busy)
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
        env_ptr->markObstacle(x, y, obstacle_radius);
    }
}

// 算法执行线程
void
invoke()
{
    busy = true;
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
    busy = false;
}

// 算法执行结果演示线程
void play()
{
    busy = true;
    env_ptr->play(alg_ptr->getPath());
    std::cout << "Play over." << std::endl;
    busy = false;
}

// 算法切换函数
void switch_algorithm(int index)
{
    if (index >= algorithms_ptr.size())
    {
        std::cerr << "Algorithm index \"" << index << "\" dose not exist." << std::endl;
        return;
    }

    alg_ptr = algorithms_ptr[index];
    std::cout << "Selected algorithm is " << alg_ptr->getName() << std::endl;
}

int
main(int argc, char* argv[])
{
    env_ptr = std::make_shared<environment::Environment>();
    env_ptr->initialize(window_height, window_width, cell_with);
    env_ptr->setRobotRadius(robot_radius);
    env_ptr->setAlgorithmRunningDelayTime(running_delay_time);
    env_ptr->setDisplayDelayTime(display_delay_time);

    algorithms_ptr.emplace_back(std::make_shared<algorithm::Dfs>(env_ptr, "Dfs"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Bfs>(env_ptr, "Bfs"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Bcd>(env_ptr, "Bcd"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Dijkstra>(env_ptr, "Dijkstra"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Astar>(env_ptr, "Astar"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::BcdWidthDijkstra>(env_ptr, "BcdWidthDijkstra"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::BcdWithFootprint>(env_ptr, "BcdWithFootprint"));

    std::cout << "Obstacle radius: " << obstacle_radius << " " << "robot radius: " << robot_radius << std::endl;
    std::cout << "Algorithms number: " << algorithms_ptr.size() << std::endl;
    for (const auto &a : algorithms_ptr)
    {
        std::cout << a->getName() << std::endl;
    }
    std::cout << "Arguments number: " << argc << std::endl;
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

    std::future<void> future_working;

    while (true)
    {
        // 更新交互显示
        env_ptr->display();
        // 读取键盘值
        auto key = cv::waitKey(2);
        // 如果正在执行,则不做任何操作
        if (busy)
        {
            continue;
        }
        if (key == 's' || key == 'S')
        {
            // 如果按下s键,执行算法
            std::cout << "Working..." << std::endl;
            env_ptr->clear();
            future_working = std::async(std::launch::async, invoke);
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
            env_ptr->showStartGoalPose();
        }
        else if (key == 'a' || key == 'A')
        {
            // 按下a鍵，切換算法
            selected_algorithm = ++selected_algorithm % algorithms_ptr.size();
            switch_algorithm(selected_algorithm);
            auto start = env_ptr->getStart();
            auto goal = env_ptr->getGoal();
            alg_ptr->setStart(std::get<0>(start), std::get<1>(start));
            alg_ptr->setGoal(std::get<0>(goal), std::get<1>(goal));
        }
        else if (key == 'p' || key == 'P')
        {
            // 启动算法结果演示
            std::cout << "Playing..." << std::endl;
            future_working = std::async(std::launch::async, play);
        }
    }
}