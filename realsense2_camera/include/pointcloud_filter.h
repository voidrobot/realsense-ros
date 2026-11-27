// Copyright 2025 RealSense, Inc. All Rights Reserved.
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

#include <string>
#include <memory>
#include <librealsense2/rs.hpp>
#include <sensor_params.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros_sensor.h>
#include "dmabuf_pool.h"
#include "named_filter.h"

#ifdef BUILD_WITH_QIRP
#include <qrb_ros_transport_point_cloud2_type/point_cloud2.hpp>
#endif

namespace realsense2_camera
{
    class PointcloudFilter : public NamedFilter
    {
        public:
            PointcloudFilter(std::shared_ptr<rs2::filter> filter,
                             RosNodeBase& node,
                             std::shared_ptr<Parameters> parameters,
                             rclcpp::Logger logger,
                             bool is_enabled=false,
                             bool use_qirp_transport=false,
                             std::size_t qirp_pool_size=qirp_defaults::kDefaultPoolSize);

            void setPublisher();
            void Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id);

        private:
            struct TextureInfo
            {
                int width{0};
                int height{0};
                int bytes_per_pixel{0};
                const uint8_t* data{nullptr};
                std::string field_name;
            };

            struct PointCloudLayout
            {
                bool initialized{false};
                bool has_texture{false};
                std::string field_name;
                bool is_bigendian{false};
                uint32_t point_step{0};
                uint32_t color_offset{0};
                std::vector<sensor_msgs::msg::PointField> fields;
            };

            void setParameters();
            TextureInfo extractTextureInfo(rs2::frameset::iterator texture_frame_itr) const;
#ifdef BUILD_WITH_QIRP
            const PointCloudLayout& getOrBuildLayout(bool has_color_field, const std::string& field_name);
            size_t countValidPoints(const rs2::vertex* vertices,
                                    const rs2::texture_coordinate* tex_coords,
                                    size_t pc_size,
                                    bool use_texture) const;
            size_t packPointsToBuffer(uint8_t* dst_base,
                                      const PointCloudLayout& layout,
                                      const rs2::vertex* vertices,
                                      const rs2::texture_coordinate* tex_coords,
                                      size_t pc_size,
                                      const TextureInfo* texture) const;
#endif

        private:
            bool _is_enabled_pc;
            RosNodeBase& _node;
            bool _allow_no_texture_points;
            bool _ordered_pc;
            bool _use_qirp_transport;
            std::mutex _mutex_publisher;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
#ifdef BUILD_WITH_QIRP
            rclcpp::Publisher<qrb_ros::transport::type::PointCloud2>::SharedPtr _qirp_pointcloud_publisher;
            DmaBufferPool _dmabuf_pool;
            PointCloudLayout _cached_layout;
#endif
            std::string _pointcloud_qos;
    };
}
