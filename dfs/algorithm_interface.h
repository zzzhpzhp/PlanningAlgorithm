#ifndef DFS_ALGORITHM_INTERFACE_H
#define DFS_ALGORITHM_INTERFACE_H

#include "environment_interface.h"

class AlgorithmInterface
{
public:
    virtual void
    initialize(EnvironmentInterfacePtr &env) = 0;

    virtual void
    setGoal(int x, int y) = 0;

    virtual void
    setStart(int x, int y) = 0;

    virtual bool
    planning() = 0;

protected:

    inline bool
    _get_middle_higher(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x;
        sy = y - 1;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }

    inline bool
    _get_middle_lower(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x;
        sy = y + 1;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }

    inline bool
    _get_left(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x - 1;
        sy = y;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }

    inline bool
    _get_right(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x + 1;
        sy = y;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }

    inline bool
    _get_higher_left(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x - 1;
        sy = y - 1;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }

    inline bool
    _get_higher_right(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x + 1;
        sy = y - 1;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }

    inline bool
    _get_lower_left(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x - 1;
        sy = y + 1;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }

    inline bool
    _get_lower_right(EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
    {
        sx = x + 1;
        sy = y + 1;
        if (ep->insideGrid(sx, sy))
        {
            return true;
        }
        return false;
    }
};

#endif //DFS_ALGORITHMINTERFACE_H
