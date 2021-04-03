### 依赖安装：
sudo apt-get install libopencv-dev
(编译时若是出现opencv相关的错误，可能需要调整头文件的包含路径。)

### 编译与执行：
mkdir build\
cd build\
cmake ..\
make -j\
./AFF

### 测试环境
Ubuntu 20.04

### 测试说明
运行后会弹出一个InteractWindow窗口，点击鼠标左键标记搜索起点，点击鼠标右键标记搜索目标，鼠标中键按下后标记障碍物。在键盘按下s键启动搜索，c键清除上次的搜索痕迹。