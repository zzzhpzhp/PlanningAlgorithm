#ifndef DFS_ALGORITHM_INTERFACE_H
#define DFS_ALGORITHM_INTERFACE_H

#include "environment_interface.h"

namespace algorithm
{
    class AlgorithmInterface
    {
    public:
        virtual void
        initialize(environment::EnvironmentInterfacePtr &env) = 0;

        virtual void
        setGoal(int x, int y) = 0;

        virtual void
        setStart(int x, int y) = 0;

        virtual bool
        planning() = 0;

        virtual environment::Path&
        getPath() = 0;
    protected:

        inline bool
        _get_middle_higher(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
        _get_middle_lower(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
        _get_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
        _get_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
        _get_higher_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
        _get_higher_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
        _get_lower_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
        _get_lower_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy)
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
}

#endif //AFF_ALGORITHMINTERFACE_H
