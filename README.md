### 依赖安装：
```shell
sudo apt-get install libopencv-dev libbost-dev
```
(编译时若是出现opencv相关的错误，可能需要调整头文件的包含路径。)

### 编译与执行：
```shell
mkdir build\
cd build\
cmake ..\
make -j\
./PlanningAlgorithm
```
### 测试环境
```shell
Ubuntu 20.04
```
### 测试说明
运行后会弹出一个InteractiveWindow窗口和原始数据窗口，在InteractiveWindow中：  
1、点击鼠标左键标记搜索起点；  
2、点击鼠标右键标记搜索目标；  
3、鼠标中键按下后标记、清除障碍物；  
4、在键盘按下s键启动搜索；  
5、“c”键清除上次的搜索痕迹；  
6、“p”键启动结果演示；  
7、“t”键终止当前搜索、演示；  
8、“1”键保存当前环境到磁盘；  
9、“2”键切换鼠标中键的标记、清除障碍物功能。   
 
### 效果  
#### Dijkstra  
![Dijkstra](https://github.com/zzzhpzhp/PlanningAlgorithm/blob/ubuntu16.04/Dijkstra.gif)  
#### AStar  
![AStar](https://github.com/zzzhpzhp/PlanningAlgorithm/blob/ubuntu16.04/AStar.gif)  
#### JPS  
![JPS](https://github.com/zzzhpzhp/PlanningAlgorithm/blob/ubuntu16.04/JPS.gif)  
#### Complete Coverage  
![JPS](https://github.com/zzzhpzhp/PlanningAlgorithm/blob/ubuntu16.04/Coverage.gif)  
