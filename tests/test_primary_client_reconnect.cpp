// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 HDS Global
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

// PrimaryClient::sendScript() against a robot that has gone away. It reconnects the
// primary stream when a write fails; that reconnect used to call connect() with no
// arguments -- unlimited attempts, 10 s apart -- so sendScript() did not return until the
// robot came back, however start() had been configured. Needs no robot: a local server
// on the primary port stands in for one and is then taken away.

#include <gtest/gtest.h>

#include <csignal>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <thread>

#include <ur_client_library/primary/primary_client.h>
#include "test_utils.h"

using namespace urcl;

namespace
{
constexpr int PRIMARY_PORT = 30001;  // PrimaryClient always connects to the primary port.

// The outcome of one sendScript() call, or nothing if it is still blocked after `limit`.
// The call runs on a detached thread so a hang fails the test instead of wedging it; the
// client is then deliberately leaked, since that thread is still inside it.
std::optional<bool> sendScriptWithin(primary_interface::PrimaryClient* client, std::chrono::seconds limit)
{
  auto result = std::make_shared<std::promise<bool>>();
  auto future = result->get_future();
  std::thread([client, result] { result->set_value(client->sendScript("textmsg(\"test\")")); }).detach();
  if (future.wait_for(limit) != std::future_status::ready)
    return std::nullopt;
  return future.get();
}
}  // namespace

TEST(PrimaryClientReconnect, send_script_gives_up_on_an_unreachable_robot)
{
  // The robot's side is closed under the client; a write to it must fail, not kill the test.
  std::signal(SIGPIPE, SIG_IGN);

  auto robot = std::make_unique<TestableTcpServer>(PRIMARY_PORT);
  robot->start();

  comm::INotifier notifier;
  auto* client = new primary_interface::PrimaryClient("127.0.0.1", notifier);
  client->start(2, std::chrono::milliseconds(100));
  ASSERT_TRUE(robot->waitForConnectionCallback());

  robot.reset();  // the control box shuts down: connection closed, nothing listening

  // A write may still land in the socket buffer before the close is noticed; after that,
  // sendScript() must fail -- within 2 attempts 100 ms apart, not after the robot returns.
  bool gave_up = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!gave_up && std::chrono::steady_clock::now() < deadline)
  {
    const auto sent = sendScriptWithin(client, std::chrono::seconds(5));
    if (!sent)
      FAIL() << "sendScript() still blocked after 5 s against an unreachable robot";
    gave_up = !*sent;
    if (!gave_up)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(gave_up) << "sendScript() kept reporting success with nothing listening";

  client->stop();
  delete client;
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
