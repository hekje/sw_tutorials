/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef PROTECTED_QUEUE
#define PROTECTED_QUEUE

#include <queue>
#include <thread>
#include <mutex>


namespace hek_tutorials
{
    template <typename T>
    class ProtectedQueue
    {
    public:
        ProtectedQueue(){}
        virtual ~ProtectedQueue(){}

        ProtectedQueue(const ProtectedQueue& other)
        {
            std::lock_guard<std::mutex> guard( other.m_mutex );
            m_queue = other.m_queue;
        }

        ProtectedQueue& operator= (ProtectedQueue& other)
        {
            if (&other == this)
            {
                return *this;
            }

            std::unique_lock<std::mutex> lock1(m_mutex, std::defer_lock);
            std::unique_lock<std::mutex> lock2(other.m_mutex, std::defer_lock);
            std::lock(lock1, lock2);
            m_queue = other.m_queue;

            return *this;
        }

        bool pop(T& item)
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            if (!m_queue.empty())
            {
                item = m_queue.front();
                m_queue.pop();
                return true;
            }

            return false;
        }

        bool pop()
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            if (!m_queue.empty())
            {
                m_queue.pop();
                return true;
            }

            return false;
        }

        void push(const T& item)
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_queue.push(std::move(item));
        }

        void push(T&& item)
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_queue.push(std::move(item));
        }

        bool empty()
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            return m_queue.empty();
        }

       int size()
       {
           std::lock_guard<std::mutex> guard(m_mutex);
           return static_cast<int>(m_queue.size());
       }

       void clear()
       {
           std::lock_guard<std::mutex> guard(m_mutex);
           std::queue<T> empty;
           std::swap(m_queue, empty);
       }

    private:
        std::queue<T> m_queue;
        std::mutex m_mutex;
    };

} // namespace hek_tutorials

#endif // PROTECTED_QUEUE
