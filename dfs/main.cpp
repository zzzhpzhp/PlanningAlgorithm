#include "main.h"


using namespace std;
using namespace cv;

Mat grid;
Point p1, p2;
int point_num = 0;
clock_t previous=clock(),current;
double duration;


//*********************************************
//响应函数，在点击的地方画圆
//传递过来的参数：鼠标事件，鼠标事件发生处的坐标，FLAG
//********************************************
static void draw_circle(int event, int x, int y, int flags, void *)
{
    if ((event == CV_EVENT_MOUSEMOVE)&&(flags&CV_EVENT_FLAG_LBUTTON))//鼠标左键按下并且光标移动
    {
        p2 = Point(x, y);
        current = clock();//clock返回ms, time则返回s
        duration = (double)(current - previous);//返回的是double类型的s
        cout << duration << endl;

        //如果停顿时间很长，则重新起点画线
        if (duration > 200)//经验阈值200
        {
            point_num = 0;
        }
        cout << "Mouse Event" << endl;

        //判断当前点是否是起点，起点只画圆，后续点画圆连线
        if (point_num != 0)
        {
            //circle(grid, p2, 1, Scalar(0, 0, 255)); //描以后的点
            line(grid, p1, p2, Scalar(0, 0, 255));//与前一点进行连线  参数：画图板背景、第一个点位置、第二个点位置、BGR颜色
        }
        else
        {
            circle(grid, p2, 1, Scalar(0, 0, 255));//描第一个点 参数：画图板背景、原点的位置、半径、BGR颜色
        }
        p1 = p2;
        previous = current;
        point_num++;

    }
}


int
main(int argc, char* argv[])
{
    //创建Mat图像（像素值自定义）
    grid = Mat(10, 10, CV_8UC1, Scalar(255,255,255));// 参数(int rows, int cols, int type, const Scalar& s)

//    grid = imread("picture.png");
    namedWindow("image");
    setMouseCallback("image", draw_circle);//回调draw_circle()，获取鼠标信息
//    resize(grid, grid, Size(),100,100);
    while (1)
    {
        imshow("image", grid);
        waitKey(30);
    }
    return 0;
}