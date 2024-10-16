// Copyright 2021 Real-Time Innovations, Inc. (RTI)
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

#include <gtest/gtest.h>
#include <rcutils/env.h>

#include <cstdlib>
#include <memory>
#include <string>

#include "ros2dds/resolve.hpp"

#include "rclcpp/init_options.hpp"
#include "rclcpp/node_options.hpp"

#include "std_msgs/msg/String.hpp"
#include "example_interfaces/srv/AddTwoInts.hpp"

class TestResolve : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    other_node = std::make_shared<rclcpp::Node>("other_node", "/other_ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr other_node;
};

TEST_F(TestResolve, resolve_topic_name_ros) {
  using ros2dds::resolve_topic_name;
  using ros2dds::TopicKind;

  //
  // Test resolve_topic_name(*node, ...)
  //

  // Standard ROS Topic
  std::string resolved = resolve_topic_name(*node, "foo");
  ASSERT_STREQ("rt/ns/foo", resolved.c_str());

  // ROS Request Topic
  resolved = resolve_topic_name(*node, "foo", TopicKind::Request);
  ASSERT_STREQ("rq/ns/fooRequest", resolved.c_str());

  // ROS Reply Topic
  resolved = resolve_topic_name(*node, "foo", TopicKind::Reply);
  ASSERT_STREQ("rr/ns/fooReply", resolved.c_str());

  //
  // Test resolve_topic_name(*other_node, ...)
  //

  resolved = resolve_topic_name(*other_node, "foo");
  ASSERT_STREQ("rt/other_ns/foo", resolved.c_str());

  // ROS Request Topic
  resolved = resolve_topic_name(*other_node, "foo", TopicKind::Request);
  ASSERT_STREQ("rq/other_ns/fooRequest", resolved.c_str());

  // ROS Reply Topic
  resolved = resolve_topic_name(*other_node, "foo", TopicKind::Reply);
  ASSERT_STREQ("rr/other_ns/fooReply", resolved.c_str());

  //
  // Test resolve_topic_name(node_name, node_namespace, ...)
  //

  // Standard ROS Topic
  resolved = resolve_topic_name("a_node", "/a_ns", "foo");
  ASSERT_STREQ("rt/a_ns/foo", resolved.c_str());

  resolved = resolve_topic_name("a_node", "/", "foo");
  ASSERT_STREQ("rt/foo", resolved.c_str());

  // ROS Request Topic to service
  resolved = resolve_topic_name("a_node", "/a_ns", "foo", TopicKind::Request);
  ASSERT_STREQ("rq/a_ns/fooRequest", resolved.c_str());

  resolved = resolve_topic_name("a_node", "/", "foo", TopicKind::Request);
  ASSERT_STREQ("rq/fooRequest", resolved.c_str());

  // ROS Reply Topic to service
  resolved = resolve_topic_name("a_node", "/a_ns", "foo", TopicKind::Reply);
  ASSERT_STREQ("rr/a_ns/fooReply", resolved.c_str());

  resolved = resolve_topic_name("a_node", "/", "foo", TopicKind::Reply);
  ASSERT_STREQ("rr/fooReply", resolved.c_str());
}

TEST_F(TestResolve, resolve_topic_name_dds) {
  using ros2dds::resolve_topic_name;
  using ros2dds::TopicKind;

  //
  // Test resolve_topic_name(*node, ...)
  //

  // Standard Topic
  std::string resolved = resolve_topic_name(*node, "foo", TopicKind::Topic, false);
  ASSERT_STREQ("foo", resolved.c_str());

  // Request Topic
  resolved = resolve_topic_name(*node, "foo", TopicKind::Request, false);
  ASSERT_STREQ("fooRequest", resolved.c_str());

  // ROS Reply Topic
  resolved = resolve_topic_name(*node, "foo", TopicKind::Reply, false);
  ASSERT_STREQ("fooReply", resolved.c_str());

  //
  // Test resolve_topic_name(*other_node, ...)
  //

  resolved = resolve_topic_name(*other_node, "foo", TopicKind::Topic, false);
  ASSERT_STREQ("foo", resolved.c_str());

  resolved = resolve_topic_name(*other_node, "foo", TopicKind::Request, false);
  ASSERT_STREQ("fooRequest", resolved.c_str());

  resolved = resolve_topic_name(*other_node, "foo", TopicKind::Reply, false);
  ASSERT_STREQ("fooReply", resolved.c_str());

  //
  // Test resolve_topic_name(node_name, node_namespace, ...)
  //

  // Standard Topic
  resolved = resolve_topic_name("a_node", "/a_ns", "foo", TopicKind::Topic, false);
  ASSERT_STREQ("foo", resolved.c_str());

  resolved = resolve_topic_name("a_node", "/", "foo", TopicKind::Topic, false);
  ASSERT_STREQ("foo", resolved.c_str());

  // ROS Request Topic to service
  resolved = resolve_topic_name("a_node", "/a_ns", "foo", TopicKind::Request, false);
  ASSERT_STREQ("fooRequest", resolved.c_str());

  resolved = resolve_topic_name("a_node", "/", "foo", TopicKind::Request, false);
  ASSERT_STREQ("fooRequest", resolved.c_str());

  // ROS Reply Topic to service
  resolved = resolve_topic_name("a_node", "/a_ns", "foo", TopicKind::Reply, false);
  ASSERT_STREQ("fooReply", resolved.c_str());

  resolved = resolve_topic_name("a_node", "/", "foo", TopicKind::Reply, false);
  ASSERT_STREQ("fooReply", resolved.c_str());
}

TEST_F(TestResolve, resolve_topic_name_invalid) {
  using ros2dds::resolve_topic_name;
  using ros2dds::TopicKind;

  // Invalid topic name (empty)
  ASSERT_THROW(
  {
    resolve_topic_name(*node, "");
  }, rclcpp::exceptions::InvalidTopicNameError);

  // Invalid node name (empty)
  ASSERT_THROW(
  {
    resolve_topic_name("", "/", "foo");
  }, rclcpp::exceptions::InvalidNodeNameError);

  // Invalid node namespace (empty)
  ASSERT_THROW(
  {
    resolve_topic_name("a_node", "", "foo");
  }, rclcpp::exceptions::InvalidNamespaceError);

  // TopicKind shouldn't make a difference in these cases
  ASSERT_THROW(
  {
    resolve_topic_name(*node, "", TopicKind::Topic);
  }, rclcpp::exceptions::InvalidTopicNameError);
  ASSERT_THROW(
  {
    resolve_topic_name("", "/", "foo", TopicKind::Topic);
  }, rclcpp::exceptions::InvalidNodeNameError);
  ASSERT_THROW(
  {
    resolve_topic_name("a_node", "", "foo", TopicKind::Topic);
  }, rclcpp::exceptions::InvalidNamespaceError);

  ASSERT_THROW(
  {
    resolve_topic_name(*node, "", TopicKind::Request);
  }, rclcpp::exceptions::InvalidTopicNameError);
  ASSERT_THROW(
  {
    resolve_topic_name("", "/", "foo", TopicKind::Request);
  }, rclcpp::exceptions::InvalidNodeNameError);
  ASSERT_THROW(
  {
    resolve_topic_name("a_node", "", "foo", TopicKind::Request);
  }, rclcpp::exceptions::InvalidNamespaceError);

  ASSERT_THROW(
  {
    resolve_topic_name(*node, "", TopicKind::Reply);
  }, rclcpp::exceptions::InvalidTopicNameError);
  ASSERT_THROW(
  {
    resolve_topic_name("", "/", "foo", TopicKind::Reply);
  }, rclcpp::exceptions::InvalidNodeNameError);
  ASSERT_THROW(
  {
    resolve_topic_name("a_node", "", "foo", TopicKind::Reply);
  }, rclcpp::exceptions::InvalidNamespaceError);

  // If we disable ROS naming conventions, only invalid topic name should throw
  ASSERT_THROW(
  {
    resolve_topic_name(*node, "", TopicKind::Topic, false);
  }, rclcpp::exceptions::InvalidTopicNameError);
  ASSERT_THROW(
  {
    resolve_topic_name(*node, "", TopicKind::Request, false);
  }, rclcpp::exceptions::InvalidTopicNameError);
  ASSERT_THROW(
  {
    resolve_topic_name(*node, "", TopicKind::Reply, false);
  }, rclcpp::exceptions::InvalidTopicNameError);

  ASSERT_NO_THROW(
  {
    resolve_topic_name("", "/", "foo", TopicKind::Topic, false);
    resolve_topic_name("a_node", "", "foo", TopicKind::Topic, false);

    resolve_topic_name("", "/", "foo", TopicKind::Request, false);
    resolve_topic_name("a_node", "", "foo", TopicKind::Request, false);

    resolve_topic_name("", "/", "foo", TopicKind::Reply, false);
    resolve_topic_name("a_node", "", "foo", TopicKind::Reply, false);
  });
}
