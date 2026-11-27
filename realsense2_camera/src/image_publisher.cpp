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

#include <image_publisher.h>

using namespace realsense2_camera;

// --- image_rcl_publisher implementation ---
image_rcl_publisher::image_rcl_publisher( RosNodeBase & node,
                                          const std::string & topic_name,
                                          const rmw_qos_profile_t & qos )
{
    image_publisher_impl = node.create_publisher< sensor_msgs::msg::Image >(
        topic_name,
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ) );
}

void image_rcl_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    image_publisher_impl->publish( std::move( image_ptr ) );
}

size_t image_rcl_publisher::get_subscription_count() const
{
    return image_publisher_impl->get_subscription_count();
}

// --- image_transport_publisher implementation ---
image_transport_publisher::image_transport_publisher( rclcpp::Node & node,
                                                      const std::string & topic_name,
                                                      const rmw_qos_profile_t & qos )
{
#if defined( LYRICAL ) || defined( ROLLING )
    // On Rolling/Lyrical, image_transport::create_publisher deduces NodeT
    // from a Node reference (it then calls node.get_node_base_interface()),
    // and the QoS argument is rclcpp::QoS rather than rmw_qos_profile_t.
    rclcpp::QoS rclcpp_qos( rclcpp::QoSInitialization::from_rmw( qos ), qos );
    image_publisher_impl = std::make_shared< image_transport::Publisher >(
        image_transport::create_publisher( node, topic_name, rclcpp_qos ) );
#else
    image_publisher_impl = std::make_shared< image_transport::Publisher >(
        image_transport::create_publisher( &node, topic_name, qos ) );
#endif
}
void image_transport_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    image_publisher_impl->publish( *image_ptr );
}

size_t image_transport_publisher::get_subscription_count() const
{
    return image_publisher_impl->getNumSubscribers();
}

#ifdef BUILD_WITH_QIRP
// --- image_qirp_publisher implementation ---
image_qirp_publisher::image_qirp_publisher( RosNodeBase & node,
                                            const std::string & topic_name,
                                            const rmw_qos_profile_t & qos )
{
    image_publisher_impl = node.create_publisher< qrb_ros::transport::type::Image >(
        topic_name,
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ) );
}

bool image_qirp_publisher::publish_raw( const std_msgs::msg::Header & header,
                                        int width,
                                        int height,
                                        const std::string & encoding,
                                        int step,
                                        const void * data )
{
    if (!data || width <= 0 || height <= 0 || step <= 0)
        return false;
    if (!qrb_ros::transport::image_utils::is_support_encoding(encoding))
        return false;

    const int align_size = qrb_ros::transport::image_utils::get_image_align_size(width, height, encoding);
    if (align_size <= 0)
        return false;

    auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(static_cast<std::size_t>(align_size), "/dev/dma_heap/system");
    if (!dmabuf)
        return false;

    if (!qrb_ros::transport::image_utils::save_image_to_dmabuf(dmabuf, data, width, height, step, encoding, true))
        return false;

    auto qirp_msg = std::make_unique< qrb_ros::transport::type::Image >();
    qirp_msg->header = header;
    qirp_msg->width = width;
    qirp_msg->height = height;
    qirp_msg->encoding = encoding;
    qirp_msg->dmabuf = std::move(dmabuf);

    image_publisher_impl->publish( std::move( qirp_msg ) );
    return true;
}

void image_qirp_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    if (!image_ptr)
        return;
    publish_raw(image_ptr->header,
                static_cast<int>(image_ptr->width),
                static_cast<int>(image_ptr->height),
                image_ptr->encoding,
                static_cast<int>(image_ptr->step),
                image_ptr->data.data());
}

size_t image_qirp_publisher::get_subscription_count() const
{
    return image_publisher_impl->get_subscription_count();
}
#endif
