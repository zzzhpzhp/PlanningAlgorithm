#pragma once
#include "environment_interface.h"

namespace algorithm
{
    class AlgorithmInterface
    {
    public:
        virtual void
        initialize(environment::EnvironmentInterfacePtr &env, std::string name) = 0;

        virtual std::string
        getName()
        {
            return name_;
        }

        virtual void
        setGoal(int x, int y) {};

        virtual void
        setStart(int x, int y) {};

        virtual bool
        planning() {};

        virtual environment::Path&
        getPath() {};

        virtual void
        start()
        {
            is_running_.store(true);
        }

        virtual void
        stop()
        {
            is_running_.store(false);
        }

    protected:

        inline bool
        _get_middle_higher(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x;
            sy = y - step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        inline bool
        _get_middle_lower(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x;
            sy = y + step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        inline bool
        _get_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x - step_size;
            sy = y;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        inline bool
        _get_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x + step_size;
            sy = y;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        inline bool
        _get_higher_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x - step_size;
            sy = y - step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        inline bool
        _get_higher_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x + step_size;
            sy = y - step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        inline bool
        _get_lower_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x - step_size;
            sy = y + step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        inline bool
        _get_lower_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x + step_size;
            sy = y + step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

    protected:
        std::string name_{};
        std::atomic_bool is_running_{true};
    };
}