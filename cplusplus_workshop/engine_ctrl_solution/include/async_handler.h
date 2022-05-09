/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef ASYNC_HANDLER
#define ASYNC_HANDLER


#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include "protected_queue.h"
#include "sl_log.h"


namespace hek_tutorials
{
    template<typename T>
    class AsyncHandler
    {
    public:
        AsyncHandler()
        {
            start_handle_thread();

            SLLog::log_info("AsyncHandler::AsyncHandler - Parent constructed");
        }

        ~AsyncHandler()
        {
            stop_handle_thread();
            SLLog::log_info("AsyncHandler::~AsyncHandler - AsyncHandler terminated gracefully");
        }

        //! This is not a pure virtual function, as this function is called in the running thread:
        //! In case the child is already distructed, but the AsyncHandler parent is still alive,
        //! the application will crash with a "pure virtual method called" exception
        virtual void handle_trigger_action(T& t) { (void) t; }


    protected:
        void
        trigger_handler_thread(const T& trigger_action_struct)
        {
            if (!m_handle_thread_running) {  return; }

            {   // Scope for mutex
                std::unique_lock<std::mutex> lk(m_handle_thread_mutex);
                m_trigger_action_queue.push(trigger_action_struct);
            }

            m_handle_thread_cond_var.notify_all();
        }

        void
        stop()
        {
            SLLog::log_info("AsyncHandler::Stop - stop_handle_thread");
            stop_handle_thread();
        }

    private:    // functions
        void
        start_handle_thread()
        {
            // Terminate thread first if already running
            stop_handle_thread();

            {   // Scope for mutex
                std::unique_lock<std::mutex> lk(m_handle_thread_mutex);
                m_handle_thread_running = true;
            }

            m_handle_thread = std::thread(&AsyncHandler::run_handle_thread, this);
        }


        void
        stop_handle_thread()
        {
            {   // Scope for mutex
                std::unique_lock<std::mutex> lk(m_handle_thread_mutex);

                // Stop triggering the handler thread to do something
                m_trigger_action_queue.clear();

                // Stop the state machine thread if it is running
                if (!m_handle_thread_running) { return; }

                m_handle_thread_running = false;
            }

            m_handle_thread_cond_var.notify_all();

            if (m_handle_thread.joinable())
                m_handle_thread.join();
        }


        void
        run_handle_thread()
        {
            SLLog::log_info("AsyncHandler::run_handle_thread - handle thread started");

            while (m_handle_thread_running) {
                {   // Scope for mutex
                    // Wait until triggers are available for processing a state OR
                    // until this thread is signaled to stop
                    // How it works:
                    // 1)   mutex lock is acquired
                    // 2)   the predicates are checked
                    // 3a)  IF any of the predicates evaluate to FALSE, then
                    //      the mutex is unlocked and thread is put in waiting
                    //      (blocking) state
                    // 3b)  IF any of the predicates evaluate to TRUE, then
                    //      the thread continues
                    //
                    // NOTE: In between 2) and 3), it should not be allowed to change
                    //       a predicate variable, as this could lead to a DEADLOCK state:
                    //       Hence, writing to this variable must be protected by the
                    //       same mutex
                    std::unique_lock<std::mutex> lk(m_handle_thread_mutex);
                    m_handle_thread_cond_var.wait(lk,
                                                  [this]{return ((!m_trigger_action_queue.empty()) ||
                                                                 (!m_handle_thread_running));});
                }

                if (!m_handle_thread_running) {
                    SLLog::log_info("AsyncHandler::run_handle_thread - handle thread terminated gracefully");
                    return;
                }

                // Handle actions while there are triggers available
                while (!m_trigger_action_queue.empty()) {
                    T trigger_action_struct = {};
                    if (m_trigger_action_queue.pop(trigger_action_struct)) {
                        handle_trigger_action(trigger_action_struct);
                    } else {
                        SLLog::log_error("AsyncHandler::run_handle_thread - ERROR! Failed to pop an action request from the queue");
                    }
                }
            }

            SLLog::log_info("AsyncHandler::run_handle_thread - handle thread terminated gracefully");
        }

    private:    // variables
        ProtectedQueue<T> m_trigger_action_queue;
        std::thread m_handle_thread;
        std::atomic<bool> m_handle_thread_running;
        std::condition_variable m_handle_thread_cond_var;
        std::mutex m_handle_thread_mutex;

    };

} // namespace hek_tutorials

#endif // ASYNC_HANDLER
