#include "main.h"

// 交互窗口宽度（网格）
int window_width = 162;
// 交互窗口高度（网格）
int window_length = 100;
// 一个网格的宽度，单位：像素
int width = 5;
// 算法步骤执行延时（秒）
float running_delay_time = 0.0000;
// 算法执行结果演示时的步骤延时（秒）
float display_delay_time = 0.001;
// 算法执行成功后，是否显示算法给出的路径
bool show_path = true;
// 障碍物标记的宽度，以给定坐标为中心
int obstacle_radius = 3;
int free_radius = 2;
// 机器人半径
int robot_radius = 5;
// 当前运行的算法的索引
int selected_algorithm = 3;
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

        switch (env_ptr->getMarkMode())
        {
            case environment::MarkMode::MARK_OBSTACLE:
                env_ptr->markObstacle(x, y, obstacle_radius);
                break;
            case environment::MarkMode::MARK_FREE_SPACE:
                env_ptr->markFreeSpace(x, y, free_radius);
                break;
        }
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
bool switch_algorithm(int index)
{
    if (index >= algorithms_ptr.size())
    {
        std::cerr << "Algorithm index \"" << index << "\" dose not exist." << std::endl;
        return false;
    }

    alg_ptr = algorithms_ptr[index];
    std::cout << "Selected algorithm is " << alg_ptr->getName() << std::endl;
    return true;
}

void print_info()
{
    std::cout << "================= 搜索算法演示程序 =================" << std::endl;
    std::cout << "鼠标左键标记起点" << std::endl;
    std::cout << "鼠标右键标记目标点" << std::endl;
    std::cout << "鼠标中键执行标记，默认标记障碍物图，按数字键3切换标记类型（障碍物标记、清除障碍物标记等）" << std::endl;
    std::cout << "按键A： 切换算法" << std::endl;
    std::cout << "按键S： 启动算法运行" << std::endl;
    std::cout << "按键T： 结束算法运行" << std::endl;
    std::cout << "按键P： 演示算法运行结果" << std::endl;
    std::cout << "数字按键1： 保存当前环境" <<std::endl;
    std::cout << "数字按键2： 加载环境配置文件" <<std::endl;
    std::cout << "数字按键3： 切换标记类型" <<std::endl;
}

int
main(int argc, char* argv[])
{
    print_info();
    env_ptr = std::make_shared<environment::Environment>();
    env_ptr->initialize(window_length, window_width, width);
    env_ptr->setRobotRadius(robot_radius);
    env_ptr->setAlgorithmRunningDelayTime(running_delay_time);
    env_ptr->setDisplayDelayTime(display_delay_time);

    algorithms_ptr.emplace_back(std::make_shared<algorithm::Dfs>(env_ptr, "Dfs"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Bfs>(env_ptr, "Bfs"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Bcd>(env_ptr, "Bcd"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Astar>(env_ptr, "AStar"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::Dijkstra>(env_ptr, "Dijkstra"));
    algorithms_ptr.emplace_back(std::make_shared<algorithm::BcdWidthDijkstra>(env_ptr, "BcdWithDijkstra"));
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

    std::shared_ptr<std::thread> future_working;

    bool running = true;
    while (running)
    {
        // 更新交互显示
        env_ptr->display();
        // 读取键盘值
        auto key = cv::waitKey(2);
        switch (key)
        {
            case 't': case 'T':
                std::cout << "Terminate algorithm" << std::endl;
                alg_ptr->stop();
                env_ptr->stop();
                break;
            case '+': case'=':
                display_delay_time *= 2.0f;
                env_ptr->setDisplayDelayTime(display_delay_time);
                std::cout << "Increase display delay time: " << display_delay_time << std::endl;
                break;
            case '-': case '_':
                display_delay_time /= 2.0f;
                env_ptr->setDisplayDelayTime(display_delay_time);
                std::cout << "Decrease display delay time: " << display_delay_time << std::endl;
                break;
        }

        // 如果正在执行,则不做以下操作
        if (busy)
        {
            continue;
        }
        switch (key)
        {
            case 's': case 'S':
                {
                    // 如果按下s键,执行算法
                    std::cout << "Working..." << std::endl;
                    env_ptr->clear();
                    env_ptr->start();
                    auto start = env_ptr->getStart();
                    auto goal = env_ptr->getGoal();
                    alg_ptr->setStart(std::get<0>(start), std::get<1>(start));
                    alg_ptr->setGoal(std::get<0>(goal), std::get<1>(goal));
                    alg_ptr->start();
                    future_working = std::make_shared<std::thread>(invoke);
                    future_working->detach();
                }
                break;
            case 'c': case 'C':
                // 按下c键,清楚算法的执行痕迹,但是不清除已设定的起点\终点和障碍物信息
                env_ptr->clear();
                break;
            case 'r': case 'R':
                // 按下r键,清除算法执行痕迹和设定的障碍物信息
                env_ptr->reset();
                env_ptr->showStartGoalPose();
                break;
            case 'a': case 'A':
                // 按下a鍵，切換算法
                selected_algorithm = ++selected_algorithm % algorithms_ptr.size();
                if (switch_algorithm(selected_algorithm))
                {
                    env_ptr->setCurrentAlgorithmIndex(selected_algorithm);
                    auto start = env_ptr->getStart();
                    auto goal = env_ptr->getGoal();
                    alg_ptr->setStart(std::get<0>(start), std::get<1>(start));
                    alg_ptr->setGoal(std::get<0>(goal), std::get<1>(goal));
                }
                break;
            case 'p': case 'P':
                // 启动算法结果演示
                std::cout << "Playing..." << std::endl;
                future_working = std::make_shared<std::thread>(play);
                future_working->detach();
                break;
            case 'q': case 'Q':
                std::cout << "Exit" << std::endl;
                running = false;
                break;
            case '1':
                env_ptr->setCurrentAlgorithmIndex(selected_algorithm);
                env_ptr->saveEnvironmntToDisk("../environment.json");
                break;
            case '2':
                env_ptr->loadEnvironmentFromDisk("../environment.json");
                selected_algorithm = env_ptr->getCurrentAlgorithmIndex();
                alg_ptr = algorithms_ptr[env_ptr->getCurrentAlgorithmIndex()];
                std::cout << "Current algorithm: " << alg_ptr->getName() <<std::endl;
                break;
            case '3':
                env_ptr->switchMarkMode();
            default:
                break;
        }
    }
}