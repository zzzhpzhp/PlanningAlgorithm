#include "main.h"


using namespace std;
using namespace cv;

Mat grid;
Point p1, p2;
int point_num = 0;
clock_t previous=clock(),current;
double duration;
int rect_size = 10;
int length = 1000, width = 1000;

//*********************************************
//响应函数，在点击的地方画圆
//传递过来的参数：鼠标事件，鼠标事件发生处的坐标，FLAG
//********************************************
static void draw_circle(int event, int x, int y, int flags, void *)
{
    if ((event == CV_EVENT_LBUTTONDOWN) || (flags&CV_EVENT_FLAG_LBUTTON))//鼠标左键按下并且光标移动
    {
        static int cnt = 0;
        cout << "Mouse Event " << cnt++ << endl;
        p2 = Point(x, y);
        current = clock();//clock返回ms, time则返回s
        duration = (double)(current - previous);//返回的是double类型的s
        cout << duration << endl;

        rectangle(grid, Rect(x,y,10,10), Scalar(0, 0, 255), -1);

        imshow("image", grid);
        //判断当前点是否是起点，起点只画圆，后续点画圆连线
    }
}


int
main(int argc, char* argv[])
{
    //创建Mat图像（像素值自定义）
    grid = Mat(width, length, CV_8UC1, Scalar(255,255,255));// 参数(int rows, int cols, int type, const Scalar& s)
//    grid = imread("picture.png");
    namedWindow("image");
    setMouseCallback("image", draw_circle);

    while (1)
    {
        imshow("image", grid);
        waitKey(0);
    }
    return 0;
}