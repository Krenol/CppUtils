#include <atomic>
#include <thread>
#include <pthread.h>
#include <functional>
#include <memory>
#include <mutex>

#ifndef UTILS_STOPPABLE_THREAD_H
#define UTILS_STOPPABLE_THREAD_H

namespace utils
{

    class StoppableThread
    {
    private:
        std::atomic_bool continue_{false};
        std::unique_ptr<std::thread> thrd_;
        std::function<void(void)> f_;
        std::mutex mtx_;
        bool has_policy_{false};
        int policy_;
        sched_param params_;

        /**
         * Function to set thread args if they we're given
         */
        void setThreadArgs();

        /**
         * Loop that is executed by thread
         */
        void loop();

    public:
        /**
         * Constructor
         */
        StoppableThread();

        /**
         * Constructor
         * @param policy: Policy for the thread scheduling
         * @param priority: Priority of this thread
         */
        StoppableThread(int policy, int priority);

        /**
         * Destuctor
         */
        ~StoppableThread();

        /**
         * Method to start the endless loop of this thread
         * Stop me with the stop() method
         * @param f: The function to be executed at each iteration
         */
        void run(const std::function<void(void)>& f);

        /**
         * Method to stop the currently running thread
         */
        void stop();
    };
} // namespace utils

#endif