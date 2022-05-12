#include <dirent.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>

#include "engine_timer.h"
#include "sl_log.h"
#include <chrono>

using namespace hek_tutorials;

class TimerTester
{
   public:
    TimerTester() : m_tp_begin{std::chrono::steady_clock::now()}, m_counter{0}, m_elapsed_ms{0}
    {
        m_timer.set_timer_callback(std::bind(&TimerTester::timer_callback, this));
    }

    ~TimerTester()
    {
        m_timer.req_timer_stop();
        m_timer.unset_timer_callback();

        SLLog::log_info("TimerTester::~TimerTester - TimerTester terminated gracefully");
    }

    void start_timer(uint32_t timeout_ms)
    {
        m_elapsed_ms = 0;

        // Start a single shot timer, no repeats
        m_timer.req_timer_start(timeout_ms, false);

        m_tp_begin = std::chrono::steady_clock::now();
    }

    void stop_timer() { m_timer.req_timer_stop(); }

    void timer_callback()
    {
        m_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_tp_begin).count();
        m_counter.store(m_counter.load() + 1);
    }

    uint32_t get_counter() const { return m_counter.load(); }

    uint32_t get_elapsed_time_ms() const { return m_elapsed_ms; }

   private:
    EngineTimer m_timer;
    std::chrono::steady_clock::time_point m_tp_begin;
    std::atomic_uint32_t m_counter;
    uint32_t m_elapsed_ms;
};

class TimerDeadlockTester
{
   public:
    TimerDeadlockTester() : m_counter{0} { m_timer.set_timer_callback(std::bind(&TimerDeadlockTester::timer_callback, this)); }

    ~TimerDeadlockTester()
    {
        m_timer.req_timer_stop();
        m_timer.unset_timer_callback();

        SLLog::log_info("TimerDeadlockTester::~TimerDeadlockTester - TimerDeadlockTester terminated gracefully");
    }

    void start_timer(uint32_t timeout_ms)
    {
        // Start a single shot timer, no repeats
        m_timer.req_timer_start(timeout_ms, false);
    }

    void stop_timer() { m_timer.req_timer_stop(); }

    void timer_callback()
    {
        m_counter.store(m_counter.load() + 1);

        SLLog::log_info("TimerDeadlockTester::timer_callback - Illegal call to EngineTimer!");

        // Do something illegal here: inside this callback,
        // call a function from the caller
        m_timer.req_timer_start(345, false);
    }

    uint32_t get_counter() const { return m_counter.load(); }

   private:
    EngineTimer m_timer;
    std::atomic_uint32_t m_counter;
};

class TriggerTimerTester
{
   public:
    TriggerTimerTester(uint32_t nr_trigger_callbacks)
        : m_nr_trigger_callbacks{nr_trigger_callbacks}, m_tp_begin{std::chrono::steady_clock::now()}, m_counter{0}, m_elapsed_ms{0}
    {
        m_trigger_timer.set_timer_callback(std::bind(&TriggerTimerTester::timer_callback, this));
    }

    ~TriggerTimerTester()
    {
        m_trigger_timer.req_timer_stop();
        m_trigger_timer.unset_timer_callback();

        SLLog::log_info("TriggerTimerTester::~TriggerTimerTester - TriggerTimerTester terminated gracefully");
    }

    void start_repeat_timer(uint32_t timeout_ms)
    {
        m_elapsed_ms = 0;
        m_trigger_timer.req_timer_start(timeout_ms, true);
        m_tp_begin = std::chrono::steady_clock::now();
    }

    void stop_timer() { m_trigger_timer.req_timer_stop(); }

    void timer_callback()
    {
        m_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_tp_begin).count();
        m_counter.store(m_counter.load() + 1);

        // Did we complete the requested amount of timer callbacks?
        if (m_nr_trigger_callbacks == m_counter.load()) {
            m_trigger_timer.req_timer_stop();
        }
    }

    uint32_t get_counter() const { return m_counter.load(); }

    uint32_t get_elapsed_time_ms() const { return m_elapsed_ms; }

   private:
    EngineTimer m_trigger_timer;
    uint32_t m_nr_trigger_callbacks;
    std::chrono::steady_clock::time_point m_tp_begin;
    std::atomic_uint32_t m_counter;
    uint32_t m_elapsed_ms;
};

class TriggerTimerDeadlockTester
{
   public:
    TriggerTimerDeadlockTester() : m_counter{0}
    {
        m_trigger_timer.set_timer_callback(std::bind(&TriggerTimerDeadlockTester::timer_callback, this));
    }

    ~TriggerTimerDeadlockTester()
    {
        m_trigger_timer.req_timer_stop();
        m_trigger_timer.unset_timer_callback();

        SLLog::log_info("TriggerTimerDeadlockTester::~TriggerTimerDeadlockTester - TriggerTimerDeadlockTester terminated gracefully");
    }

    void start_repeat_timer(uint32_t timeout_ms) { m_trigger_timer.req_timer_start(timeout_ms, true); }

    void stop_timer() { m_trigger_timer.req_timer_stop(); }

    void timer_callback()
    {
        m_counter.store(m_counter.load() + 1);

        SLLog::log_info("TriggerTimerDeadlockTester::timer_callback - Illegal call to EngineTimer!");

        // Do something illegal here: inside this callback,
        // call a function from the caller
        m_trigger_timer.req_timer_start(678, true);
    }

    uint32_t get_counter() const { return m_counter.load(); }

   private:
    EngineTimer m_trigger_timer;
    std::atomic_uint32_t m_counter;
};

// ----------------------------------------------------------------------------
//
// UNIT TESTS
//
// ----------------------------------------------------------------------------

TEST(TimerTester, on_time_callback)
{
    TimerTester ttt;

    // Set timer to end after 69ms
    ttt.start_timer(69);

    // Wait for 69 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(69));

    // Allow 1000ms overhead time
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Obtain the timer duration
    uint32_t elapsed_ms = ttt.get_elapsed_time_ms();
    SLLog::log_info("TimerTester::on_time_callback - elapsed_ms = " + std::to_string(elapsed_ms));

    EXPECT_EQ(ttt.get_counter(), 1);
    EXPECT_LT(abs(static_cast<int>(elapsed_ms) - 69), 20);
}

TEST(TimerTester, no_premature_callback)
{
    TimerTester ttt;

    // Set timer to end after 69ms
    ttt.start_timer(357);

    // Wait for 356 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(356));

    // We should NOT have received a callback
    EXPECT_EQ(ttt.get_counter(), 0);
}

TEST(TimerTester, no_repeated_callbacks)
{
    TimerTester ttt;

    // Set timer to end after 69ms
    ttt.start_timer(137);

    // Wait for 500ms
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Expect just one callback
    EXPECT_EQ(ttt.get_counter(), 1);
}

TEST(TimerTester, graceful_exit_after_forced_stop)
{
    TimerTester ttt;

    // Set timer to end after 69ms
    ttt.start_timer(234);

    // Wait for 173ms, then force stop
    std::this_thread::sleep_for(std::chrono::milliseconds(173));
    ttt.stop_timer();

    // Expect no callbacks and no crashes
    EXPECT_EQ(ttt.get_counter(), 0);
}

TEST(TriggerTimerTester, no_missed_callbacks)
{
    TriggerTimerTester ttt(500);

    // Set timer to trigger every 20ms
    ttt.start_repeat_timer(20);

    // Wait for 10 seconds, this should trigger exactly 500 callbacks
    // NOTE: the timer is stooped after 500 callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // Allow 1000ms overhead time
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Obtain the timer duration
    uint32_t elapsed_ms = ttt.get_elapsed_time_ms();
    SLLog::log_info("TimerTester::on_time_callback - elapsed_ms = " + std::to_string(elapsed_ms));

    EXPECT_EQ(ttt.get_counter(), 500);
    EXPECT_LT(abs(static_cast<int>(elapsed_ms) - 10000), 20);
}

TEST(TriggerTimerTester, no_premature_callback)
{
    TriggerTimerTester ttt(500);
    ;

    // Set timer to trigger every 753ms
    ttt.start_repeat_timer(753);

    // Wait for 752 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(752));

    // We should NOT have received a callback
    EXPECT_EQ(ttt.get_counter(), 0);
}

TEST(TriggerTimerTester, graceful_exit_after_forced_stop)
{
    TriggerTimerTester ttt(500);
    ;

    // Set timer to trigger every 49ms
    ttt.start_repeat_timer(49);

    // Wait for 261, then force stop
    std::this_thread::sleep_for(std::chrono::milliseconds(261));
    ttt.stop_timer();

    // Expect 5 callbacks and no crashes
    EXPECT_EQ(ttt.get_counter(), 5);
}

TEST(TimerDeadlockTester, calling_back_in_callback)
{
    TimerDeadlockTester ttt;

    // Set timer to end after 123ms,
    // after 123ms, from inside the callback function in TimerDeadlockTester,
    // a (highly illegal!) call is made to the caller
    ttt.start_timer(123);

    // Wait for 257 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(257));

    // Allow 50 ms for handling overhead
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_EQ(ttt.get_counter(), 1);
}

TEST(TriggerTimerDeadlockTester, calling_back_in_callback)
{
    TriggerTimerDeadlockTester ttt;

    // Set timer to end after 123ms,
    // after 123ms, from inside the callback function in TriggerTimerDeadlockTester,
    // a (highly illegal!) call is made to the caller to start a timer of 678ms
    ttt.start_repeat_timer(123);

    // Wait for 876 seconds, this should give us our second timeouts
    std::this_thread::sleep_for(std::chrono::milliseconds(876));

    // Allow 50 ms for handling overhead
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_EQ(ttt.get_counter(), 2);
}
