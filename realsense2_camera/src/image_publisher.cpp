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
#include <cstring>
#include <sensor_msgs/image_encodings.hpp>

// --- image_qirp_publisher implementation ---
image_qirp_publisher::image_qirp_publisher( RosNodeBase & node,
                                            const std::string & topic_name,
                                            const rmw_qos_profile_t & qos,
                                            std::size_t pool_size )
    : _dmabuf_pool( pool_size )
{
    image_publisher_impl = node.create_publisher< qrb_ros::transport::type::Image >(
        topic_name,
        rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ) );
}

std::shared_ptr<lib_mem_dmabuf::DmaBuffer> image_qirp_publisher::copy_rgb8_to_dmabuf(
    const RawImageView & view,
    int align_size )
{
    constexpr int kRgb8BytesPerPixel = 3;
    const int row_bytes = view.width * kRgb8BytesPerPixel;
    const int dst_stride = qrb_ros::transport::image_utils::get_image_stride(view.width, view.encoding);
    if (row_bytes <= 0 || dst_stride < row_bytes || view.step < row_bytes)
        return nullptr;

    // Enforce stride * height <= aligned DMA-BUF size invariant before writing.
    const std::size_t required_write_bytes =
        static_cast<std::size_t>(view.height) * static_cast<std::size_t>(dst_stride);
    if (required_write_bytes > static_cast<std::size_t>(align_size))
        return nullptr;

    auto dmabuf = _dmabuf_pool.acquire(static_cast<std::size_t>(align_size));
    if (!dmabuf || !dmabuf->addr())
        return nullptr;

    dmabuf->sync_start();
    auto* dst = static_cast<uint8_t*>(dmabuf->addr());
    const auto* src = static_cast<const uint8_t*>(view.data);
    if (dst_stride == row_bytes && view.step == row_bytes)
    {
        std::memcpy(dst, src, static_cast<std::size_t>(view.height) * static_cast<std::size_t>(row_bytes));
    }
    else
    {
        for (int y = 0; y < view.height; ++y)
        {
            std::memcpy(dst + static_cast<std::size_t>(y) * static_cast<std::size_t>(dst_stride),
                        src + static_cast<std::size_t>(y) * static_cast<std::size_t>(view.step),
                        static_cast<std::size_t>(row_bytes));
        }
    }
    dmabuf->sync_end();
    return dmabuf;
}

bool image_qirp_publisher::publish_raw( const RawImageView & view )
{
    if (!view.data || view.width <= 0 || view.height <= 0 || view.step <= 0)
        return false;
    if (!qrb_ros::transport::image_utils::is_support_encoding(view.encoding))
        return false;

    const int align_size = qrb_ros::transport::image_utils::get_image_align_size(
        view.width, view.height, view.encoding);
    if (align_size <= 0)
        return false;

    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf;
    if (view.encoding == sensor_msgs::image_encodings::RGB8 && _dmabuf_pool.max_slots() > 0)
    {
        dmabuf = copy_rgb8_to_dmabuf(view, align_size);
        if (!dmabuf)
            return false;
    }
    else
    {
        dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(
            static_cast<std::size_t>(align_size), qirp_defaults::kDefaultDmaHeapPath);
        if (!dmabuf)
            return false;
        if (!qrb_ros::transport::image_utils::save_image_to_dmabuf(
                dmabuf, view.data, view.width, view.height, view.step, view.encoding, true))
            return false;
    }

    auto qirp_msg = std::make_unique< qrb_ros::transport::type::Image >();
    qirp_msg->header = view.header;
    qirp_msg->width = view.width;
    qirp_msg->height = view.height;
    qirp_msg->encoding = view.encoding;
    qirp_msg->dmabuf = std::move(dmabuf);

    image_publisher_impl->publish( std::move( qirp_msg ) );
    return true;
}

void image_qirp_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    if (!image_ptr)
        return;
    RawImageView view;
    view.header = image_ptr->header;
    view.width = static_cast<int>(image_ptr->width);
    view.height = static_cast<int>(image_ptr->height);
    view.encoding = image_ptr->encoding;
    view.step = static_cast<int>(image_ptr->step);
    view.data = image_ptr->data.data();
    publish_raw(view);
}

size_t image_qirp_publisher::get_subscription_count() const
{
    return image_publisher_impl->get_subscription_count();
}
#endif

std::shared_ptr<image_publisher> realsense2_camera::create_image_publisher( RosNodeBase & node,
                                                         const std::string & topic_name,
                                                         const rmw_qos_profile_t & qos,
                                                         bool use_intra_process,
                                                         bool use_qirp_transport,
                                                         const std::string & ros_encoding,
                                                         std::size_t qirp_pool_size,
                                                         bool & created_qirp_out )
{
    created_qirp_out = false;
#ifdef BUILD_WITH_QIRP
    if (use_qirp_transport && !ros_encoding.empty() &&
        qrb_ros::transport::image_utils::is_support_encoding(ros_encoding))
    {
        created_qirp_out = true;
        return std::make_shared<image_qirp_publisher>(node, topic_name, qos, qirp_pool_size);
    }
#else
    (void)use_qirp_transport;
    (void)ros_encoding;
    (void)qirp_pool_size;
#endif

#ifdef USE_LIFECYCLE_NODE
    (void)use_intra_process;
    return std::make_shared<image_rcl_publisher>(node, topic_name, qos);
#else
    if (use_intra_process)
    {
        return std::make_shared<image_rcl_publisher>(node, topic_name, qos);
    }
    return std::make_shared<image_transport_publisher>(node, topic_name, qos);
#endif
}
