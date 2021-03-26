#ifndef DFS_ENVIRONMENT_INTERFACE_H
#define DFS_ENVIRONMENT_INTERFACE_H

#include <iostream>
#include <chrono>
#include <thread>

#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

class EnvironmentInterface
{
public:
    virtual void
    initialize(int length, int width, int display_scale) = 0;

    virtual uint8_t
    getGridValue(int x, int y) = 0;

    virtual bool
    setGridValue(int x, int y, uint8_t value) = 0;

    virtual int
    getGridXSizeInCells() = 0;

    virtual int
    getGridYSizeInCells() = 0;

    virtual bool
    insideGrid(int x, int y) = 0;

    virtual int
    getScale() = 0;

    virtual void
    display() = 0;

};

#endif //DFS_ENVIRONMENT_INTERFACE_H
