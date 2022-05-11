#include <dirent.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>

#include "async_handler.h"
#include "sl_log.h"

using namespace hek_tutorials;

enum class CallbackTypeTest {
  undefined = 0,
  one_callback,
  two_callback,
  three_callback,
  four_callback,
  five_callback
};

struct CallbackActionTest {
  CallbackTypeTest type = CallbackTypeTest::undefined;
  bool req_one = false;
  bool req_two = false;
  int32_t val_three = 0;
  int32_t val_four = 0;

  bool operator==(const CallbackActionTest &other) const {
    return (type == other.type && req_one == other.req_one &&
            req_two == other.req_two && val_three == other.val_three &&
            val_four == other.val_four);
  }
};

class CustomHandler : public AsyncHandler<struct CallbackActionTest> {
public:
  CustomHandler() : m_counter{0} {}
  virtual ~CustomHandler() {
    // Stop the AsyncHandler parent first, to prevent calls to an
    // already destructed child
    stop();

    SLLog::log_info(
        "CustomHandler::~CustomHandler - CustomHandler terminated gracefully");
  }

  void handle_trigger_action(struct CallbackActionTest &action) override {
    (void)action;
    m_counter.store(m_counter.load() + 1);
  }

  void req_async_handle() {
    // Trigger this action to be handled on a different thread, so this callback
    // can return immediately
    struct CallbackActionTest action;
    action.type = CallbackTypeTest::one_callback;
    action.req_one = true;
    trigger_handler_thread(action);
  }

  uint32_t get_counter() const { return m_counter.load(); }

  void test_parent_stop() {
    SLLog::log_info("CustomHandler::test_parent_stop - Simulate stopping the "
                    "AsyncHandler parent");

    // Simulate stopping the AsyncHandler parent;
    // After this, no actions should be handled
    stop();
  }

private:
  std::atomic_uint32_t m_counter;
};

//! This is for testing parent-child relationship between AsyncHandler and
//! BadHandler: To test if the application will not crash if the AsyncHandler
//! parent will call the virtual "handle_trigger_action" function if the child
//! has already been destructed
class BadHandler : public AsyncHandler<struct CallbackActionTest> {
public:
  BadHandler() : m_counter{0} {}
  virtual ~BadHandler() {
    //! Create a giant queue, so the AsyncHandler parent will try to call
    //! this child when it is already destroyed
    for (int i = 0; i < 1000000; i++) {
      req_async_handle();
    }

    SLLog::log_info(
        "CustomHandler::~CustomHandler - CustomHandler terminated gracefully");
  }

  void handle_trigger_action(struct CallbackActionTest &action) override {
    (void)action;
    m_counter.store(m_counter.load() + 1);
  }

  void req_async_handle() {
    // Trigger this action to be handled on a different thread, so this callback
    // can return immediately
    struct CallbackActionTest action;
    action.type = CallbackTypeTest::one_callback;
    action.req_one = true;
    trigger_handler_thread(action);
  }

private:
  std::atomic_uint32_t m_counter;
};

class IllegalCallParentInCallbacklHandler
    : public AsyncHandler<struct CallbackActionTest> {
public:
  IllegalCallParentInCallbacklHandler() : m_counter{0} {}
  virtual ~IllegalCallParentInCallbacklHandler() {
    // Stop the AsyncHandler parent first, to prevent calls to an
    // already destructed child
    stop();

    SLLog::log_info("IllegalCallParentInCallbacklHandler::~"
                    "IllegalCallParentInCallbacklHandler - CustomHandler "
                    "terminated gracefully");
  }

  void handle_trigger_action(struct CallbackActionTest &action) override {
    (void)action;
    m_counter.store(m_counter.load() + 1);

    // Calling a parent function in this callback is illegal and dangerous
    // Prone to deadlocks!
    req_async_handle();
  }

  void req_async_handle() {
    // Trigger this action to be handled on a different thread, so this callback
    // can return immediately
    struct CallbackActionTest action;
    action.type = CallbackTypeTest::one_callback;
    action.req_one = true;
    trigger_handler_thread(action);
  }

  uint32_t get_counter() const { return m_counter.load(); }

private:
  std::atomic_uint32_t m_counter;
};

TEST(AsyncHandler, no_missed_callbacks) {
  CustomHandler handler;
  uint32_t counter = 0;

  for (int i = 0; i < 1000000; i++) {
    handler.req_async_handle();
    counter++;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  EXPECT_EQ(handler.get_counter(), counter);
}

TEST(AsyncHandler, test_handling_after_stop) {
  CustomHandler handler;
  uint32_t counter = 0;

  handler.test_parent_stop();

  for (int i = 0; i < 1000000; i++) {
    handler.req_async_handle();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(handler.get_counter(), counter);
}

TEST(AsyncHandler, premature_child_death) {
  CustomHandler *handler = new CustomHandler();
  uint32_t counter = 0;

  for (int i = 0; i < 1000000; i++) {
    handler->req_async_handle();
    counter++;
  }

  //! Kill child before parent had time to handle all requests;
  //! This should not cause an exception:
  delete handler;
}

TEST(AsyncHandler, test_handling_after_child_destruction) {
  BadHandler *handler = new BadHandler();

  //! This should not cause an exception:
  delete handler;
}

TEST(AsyncHandler, no_deadlock_when_calling_parent_in_callback) {
  IllegalCallParentInCallbacklHandler *handler =
      new IllegalCallParentInCallbacklHandler();
  uint32_t counter = 0;

  for (int i = 0; i < 1000000; i++) {
    handler->req_async_handle();
    counter++;
  }

  delete handler;
}
