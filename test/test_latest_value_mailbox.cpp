#include <gtest/gtest.h>

#include <future>
#include <thread>

#include "latest_value_mailbox.h"

namespace {

using hesai_lidar::internal::LatestValueMailbox;
using hesai_lidar::internal::LatestValuePushResult;

TEST(LatestValueMailbox, DeliversStoredValue) {
  LatestValueMailbox<int> mailbox;
  int value = 0;

  EXPECT_EQ(mailbox.push(10), LatestValuePushResult::kStored);
  EXPECT_TRUE(mailbox.wait_pop(value));
  EXPECT_EQ(value, 10);
}

TEST(LatestValueMailbox, ReplacesPendingValue) {
  LatestValueMailbox<int> mailbox;
  int value = 0;

  EXPECT_EQ(mailbox.push(10), LatestValuePushResult::kStored);
  EXPECT_EQ(mailbox.push(20), LatestValuePushResult::kReplaced);
  EXPECT_TRUE(mailbox.wait_pop(value));
  EXPECT_EQ(value, 20);
}

TEST(LatestValueMailbox, WaiterReceivesNextValue) {
  LatestValueMailbox<int> mailbox;
  std::promise<int> received;
  auto result = received.get_future();

  std::thread consumer([&mailbox, &received]() {
    int value = 0;
    if (mailbox.wait_pop(value)) {
      received.set_value(value);
    } else {
      received.set_value(-1);
    }
  });

  EXPECT_EQ(mailbox.push(42), LatestValuePushResult::kStored);
  EXPECT_EQ(result.get(), 42);
  consumer.join();
}

TEST(LatestValueMailbox, StopDiscardsPendingValueAndRejectsPush) {
  LatestValueMailbox<int> mailbox;
  int value = 0;

  EXPECT_EQ(mailbox.push(10), LatestValuePushResult::kStored);
  mailbox.stop();
  EXPECT_FALSE(mailbox.wait_pop(value));
  EXPECT_EQ(mailbox.push(20), LatestValuePushResult::kStopped);
}

TEST(LatestValueMailbox, StopWakesWaiter) {
  LatestValueMailbox<int> mailbox;
  std::promise<bool> popped;
  auto result = popped.get_future();

  std::thread consumer([&mailbox, &popped]() {
    int value = 0;
    popped.set_value(mailbox.wait_pop(value));
  });

  mailbox.stop();
  EXPECT_FALSE(result.get());
  consumer.join();
}

}  // namespace
