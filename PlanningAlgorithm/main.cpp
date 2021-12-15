#include "main.h"

// 算法执行成功后，是否显示算法给出的路径
bool busy = false;
bool show_path = true;
int cell_with = 5;
int obstacle_radius = 8;
int selected_algorithm = 5;
std::shared_ptr<algorithm::AlgorithmInterface> alg_ptr;
std::shared_ptr<environment::EnvironmentInterface> env_ptr;
std::vector<std::shared_ptr<algorithm::AlgorithmInterface>> algorithm_ptrs;

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

void play()
{
    busy = true;
    env_ptr->play(alg_ptr->getPath());
    std::cout << "Play over." << std::endl;
    busy = false;
}

void switch_algorithm(int index)
{
    if (index >= algorithm_ptrs.size())
    {
        std::cerr << "Algorithm index \"" << index << "\" dose not exist." << std::endl;
        return;
    }

    alg_ptr = algorithm_ptrs[index];
    std::cout << "Selected algorithm is " << alg_ptr->getName() << std::endl;
}

int
main(int argc, char* argv[])
{
//    environment::Vector3 vec(1.1, 2.2, 3.3);
//    std::cout << vec[0] << " " << vec[1] << " " << vec[2] << " " << vec[3] << std::endl;
    env_ptr = std::make_shared<environment::Environment>();
    env_ptr->initialize(100, 100, cell_with);

    algorithm_ptrs.emplace_back(std::make_shared<algorithm::Dfs>(env_ptr, "Dfs"));
    algorithm_ptrs.emplace_back(std::make_shared<algorithm::Bfs>(env_ptr, "Bfs"));
    algorithm_ptrs.emplace_back(std::make_shared<algorithm::Bcd>(env_ptr, "Bcd"));
    algorithm_ptrs.emplace_back(std::make_shared<algorithm::Dijkstra>(env_ptr, "Dijkstra"));
    algorithm_ptrs.emplace_back(std::make_shared<algorithm::BcdWidthDijkstra>(env_ptr, "BcdWidthDijkstra"));
    algorithm_ptrs.emplace_back(std::make_shared<algorithm::BcdWithFootprint>(env_ptr, "BcdWithFootprint"));

    std::cout << "Arguments num: " << argc << std::endl;
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
            selected_algorithm = ++selected_algorithm % algorithm_ptrs.size();
            switch_algorithm(selected_algorithm);
            auto start = env_ptr->getStart();
            auto goal = env_ptr->getGoal();
            alg_ptr->setStart(std::get<0>(start), std::get<1>(start));
            alg_ptr->setGoal(std::get<0>(goal), std::get<1>(goal));
        }
        else if (key == 'p' || key == 'P')
        {
            std::cout << "Playing..." << std::endl;
            future_working = std::async(std::launch::async, play);
        }
    }
}