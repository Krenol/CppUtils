#include "stoppable_thread.hpp"
#include <exception>

namespace utils
{
    void StoppableThread::setThreadArgs() 
    {
        if(!has_policy_) return;

        if (pthread_setschedparam(thrd_->native_handle(), policy_, &params_))
        {
            throw std::runtime_error("couldn't update threads priority and policy!");
        }
    }

    void StoppableThread::loop()
    {
        while (continue_)
        {
            f_();
        }
    }

    StoppableThread::StoppableThread()
    {
        has_policy_ = false;
    }

    StoppableThread::StoppableThread(int policy, int priority)
    {
        
        params_.sched_priority = priority;
        policy_ = policy;
        has_policy_ = true;
    }

    StoppableThread::~StoppableThread()
    {
        stop();
    }

    void StoppableThread::run(const std::function<void(void)> &f)
    {
        std::lock_guard<std::mutex> guard(mtx_);
        if (!continue_)
        {
            f_ = f;
            continue_ = true;
            thrd_ = std::make_unique<std::thread>(&StoppableThread::loop, this);
            setThreadArgs();
        }
    }

    void StoppableThread::stop()
    {
        std::lock_guard<std::mutex> guard(mtx_);
        if (continue_)
        {
            continue_ = false;
            thrd_->join();
        }
    }
} // namespace utils