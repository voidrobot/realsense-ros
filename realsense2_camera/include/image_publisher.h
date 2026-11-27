// Copyright 2023 RealSense, Inc. All Rights Reserved.
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

#pragma once

#include "ros_node_base.h"
#include <sensor_msgs/msg/image.hpp>

#include <image_transport/image_transport.hpp>

#ifdef BUILD_WITH_QIRP
#include "dmabuf_pool.h"
#include <qrb_ros_transport_image_type/image.hpp>
#include <qrb_ros_transport_image_type/image_utils.hpp>
#endif

namespace realsense2_camera {
class image_publisher
{
public:
    virtual void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) = 0;
    virtual bool supports_raw_publish() const { return false; }
    virtual bool publish_raw( const std_msgs::msg::Header & header,
                              int width,
                              int height,
                              const std::string & encoding,
                              int step,
                              const void * data )
    {
        (void)header; (void)width; (void)height; (void)encoding; (void)step; (void)data;
        return false;
    }
    virtual size_t get_subscription_count() const = 0;
    virtual ~image_publisher() = default;
};

// Native RCL implementation of an image publisher (needed for intra-process communication)
class image_rcl_publisher : public image_publisher
{
public:
    image_rcl_publisher( RosNodeBase & node,
                         const std::string & topic_name,
                         const rmw_qos_profile_t & qos );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    size_t get_subscription_count() const override;

private:
    rclcpp::Publisher< sensor_msgs::msg::Image >::SharedPtr image_publisher_impl;
};

// image_transport implementation of an image publisher (adds a compressed image topic)
class image_transport_publisher : public image_publisher
{
public:
    image_transport_publisher( rclcpp::Node & node,
                               const std::string & topic_name,
                               const rmw_qos_profile_t & qos );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    size_t get_subscription_count() const override;

private:
    std::shared_ptr< image_transport::Publisher > image_publisher_impl;
};

#ifdef BUILD_WITH_QIRP
// Qualcomm QIRP DMA-BUF TypeAdapter image publisher (zero-copy for QIRP subscribers,
// automatic TypeAdapter conversion to sensor_msgs::msg::Image for legacy subscribers).
class image_qirp_publisher : public image_publisher
{
public:
    image_qirp_publisher( RosNodeBase & node,
                          const std::string & topic_name,
                          const rmw_qos_profile_t & qos,
                          std::size_t pool_size = 8 );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    bool supports_raw_publish() const override { return true; }
    bool publish_raw( const std_msgs::msg::Header & header,
                      int width,
                      int height,
                      const std::string & encoding,
                      int step,
                      const void * data ) override;
    size_t get_subscription_count() const override;

private:
    rclcpp::Publisher< qrb_ros::transport::type::Image >::SharedPtr image_publisher_impl;
    DmaBufferPool _dmabuf_pool;
};
#endif

}  // namespace realsense2_camera
