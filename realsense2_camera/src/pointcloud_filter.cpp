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

#include <pointcloud_filter.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <sensor_msgs/point_cloud2_iterator.hpp>


using namespace realsense2_camera;


PointcloudFilter::PointcloudFilter(std::shared_ptr<rs2::filter> filter,
                                   RosNodeBase& node,
                                   std::shared_ptr<Parameters> parameters,
                                   rclcpp::Logger logger,
                                   bool is_enabled,
                                   bool use_qirp_transport,
                                   std::size_t qirp_pool_size):
    NamedFilter(filter, parameters, logger, is_enabled, false),
    _node(node),
    _allow_no_texture_points(ALLOW_NO_TEXTURE_POINTS),
    _ordered_pc(ORDERED_PC),
    _use_qirp_transport(use_qirp_transport)
#ifdef BUILD_WITH_QIRP
    , _dmabuf_pool(qirp_pool_size)
#endif
    {
#ifndef BUILD_WITH_QIRP
        (void)qirp_pool_size;
#endif
        setParameters();
    }

void PointcloudFilter::setParameters()
{
    std::string module_name = create_graph_resource_name(rs2_to_ros(_filter->get_info(RS2_CAMERA_INFO_NAME)));
    std::string param_name(module_name + "." + "allow_no_texture_points");
    _params.getParameters()->setParamT(param_name, _allow_no_texture_points);
    _parameters_names.push_back(param_name);

    param_name = module_name + "." + std::string("ordered_pc");
    _params.getParameters()->setParamT(param_name, _ordered_pc);
    _parameters_names.push_back(param_name);

    param_name = module_name + "." + std::string("pointcloud_qos");
    rcl_interfaces::msg::ParameterDescriptor crnt_descriptor;
    crnt_descriptor.description = "Available options are:\n" + list_available_qos_strings();
    _pointcloud_qos = _params.getParameters()->setParam<std::string>(param_name, DEFAULT_QOS, [this](const rclcpp::Parameter& parameter)
            {
                try
                {
                    qos_string_to_qos(parameter.get_value<std::string>());
                    _pointcloud_qos = parameter.get_value<std::string>();
                    ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR_STREAM("Given value, " << parameter.get_value<std::string>() << " is unknown. Set ROS param back to: " << _pointcloud_qos);
                    _params.getParameters()->queueSetRosValue(parameter.get_name(), _pointcloud_qos);
                }
            }, crnt_descriptor);
    _parameters_names.push_back(param_name);
    NamedFilter::setParameters([this](const rclcpp::Parameter& )
        {
            setPublisher();
        });
}

void PointcloudFilter::setPublisher()
{
    std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
#ifdef BUILD_WITH_QIRP
    if (_use_qirp_transport)
    {
        if ((_is_enabled) && (!_qirp_pointcloud_publisher))
        {
            _qirp_pointcloud_publisher = _node.create_publisher<qrb_ros::transport::type::PointCloud2>(
                "~/depth/color/points",
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_string_to_qos(_pointcloud_qos)),
                            qos_string_to_qos(_pointcloud_qos)));
        }
        else if ((!_is_enabled) && (_qirp_pointcloud_publisher))
        {
            _qirp_pointcloud_publisher.reset();
        }
        return;
    }
#endif
    if ((_is_enabled) && (!_pointcloud_publisher))
    {
        _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("~/depth/color/points",
                                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_string_to_qos(_pointcloud_qos)),
                                            qos_string_to_qos(_pointcloud_qos)));
    }
    else if ((!_is_enabled) && (_pointcloud_publisher))
    {
        _pointcloud_publisher.reset();
    }
}

void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

PointcloudFilter::TextureInfo PointcloudFilter::extractTextureInfo(
    rs2::frameset::iterator texture_frame_itr) const
{
    rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
    TextureInfo info;
    info.width = texture_frame.get_width();
    info.height = texture_frame.get_height();
    info.bytes_per_pixel = texture_frame.get_bytes_per_pixel();
    info.data = static_cast<const uint8_t*>(texture_frame.get_data());
    switch (texture_frame.get_profile().format())
    {
        case RS2_FORMAT_RGB8:
            info.field_name = "rgb";
            break;
        case RS2_FORMAT_Y8:
            info.field_name = "intensity";
            break;
        case RS2_FORMAT_Z16:
            // Depth can't be used as color texture — skip coloring
            info.field_name = "";
            break;
        default:
            throw std::runtime_error("Unhandled texture format passed in pointcloud " +
                                     std::to_string(texture_frame.get_profile().format()));
    }
    return info;
}

#ifdef BUILD_WITH_QIRP
const PointcloudFilter::PointCloudLayout& PointcloudFilter::getOrBuildLayout(
    bool has_color_field,
    const std::string& field_name)
{
    if (_cached_layout.initialized &&
        _cached_layout.has_texture == has_color_field &&
        _cached_layout.field_name == field_name)
    {
        return _cached_layout;
    }

    sensor_msgs::msg::PointCloud2 layout_msg;
    sensor_msgs::PointCloud2Modifier modifier(layout_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    if (has_color_field && !field_name.empty())
    {
        layout_msg.point_step = addPointField(
            layout_msg, field_name.c_str(), 1,
            sensor_msgs::msg::PointField::FLOAT32, layout_msg.point_step);
    }

    _cached_layout.initialized = true;
    _cached_layout.has_texture = has_color_field;
    _cached_layout.field_name = field_name;
    _cached_layout.is_bigendian = layout_msg.is_bigendian;
    _cached_layout.point_step = layout_msg.point_step;
    _cached_layout.color_offset =
        (has_color_field && !layout_msg.fields.empty()) ? layout_msg.fields.back().offset : 0;
    _cached_layout.fields = std::move(layout_msg.fields);
    return _cached_layout;
}

size_t PointcloudFilter::countValidPoints(const rs2::vertex* vertices,
                                          const rs2::texture_coordinate* tex_coords,
                                          size_t pc_size,
                                          bool use_texture) const
{
    size_t valid_points = 0;
    if (use_texture)
    {
        for (size_t idx = 0; idx < pc_size; ++idx)
        {
            const auto& uv = tex_coords[idx];
            const bool valid_color_pixel = (uv.u >= 0.f && uv.u <= 1.f && uv.v >= 0.f && uv.v <= 1.f);
            if (vertices[idx].z > 0 && (valid_color_pixel || _allow_no_texture_points))
                ++valid_points;
        }
    }
    else
    {
        for (size_t idx = 0; idx < pc_size; ++idx)
        {
            if (vertices[idx].z > 0)
                ++valid_points;
        }
    }
    return valid_points;
}

size_t PointcloudFilter::packPointsToBuffer(uint8_t* dst_base,
                                            const PointCloudLayout& layout,
                                            const rs2::vertex* vertices,
                                            const rs2::texture_coordinate* tex_coords,
                                            size_t pc_size,
                                            const TextureInfo* texture) const
{
    static_assert(sizeof(rs2::vertex) == 3 * sizeof(float), "rs2::vertex must be packed float[3]");
    constexpr size_t kXyzBytes = sizeof(rs2::vertex);
    const uint32_t point_step = layout.point_step;
    const size_t tail_bytes = (point_step > kXyzBytes) ? (point_step - kXyzBytes) : 0;
    const bool write_color = (texture != nullptr && !texture->field_name.empty() &&
                              texture->data != nullptr && texture->width > 0 && texture->height > 0 &&
                              texture->bytes_per_pixel > 0);
    const bool use_texture = (texture != nullptr);
    size_t written = 0;

    for (size_t idx = 0; idx < pc_size; ++idx)
    {
        const rs2::vertex& v = vertices[idx];
        bool valid_color_pixel = false;
        bool valid_pixel = (v.z > 0);
        if (use_texture)
        {
            const rs2::texture_coordinate& uv = tex_coords[idx];
            valid_color_pixel = (uv.u >= 0.f && uv.u <= 1.f && uv.v >= 0.f && uv.v <= 1.f);
            valid_pixel = (v.z > 0 && (valid_color_pixel || _allow_no_texture_points));
        }

        if (!valid_pixel && !_ordered_pc)
            continue;

        uint8_t* p = dst_base + written * point_step;
        std::memcpy(p, &v, kXyzBytes);
        if (tail_bytes > 0)
        {
            std::memset(p + kXyzBytes, 0, tail_bytes);
        }
        if (write_color && valid_color_pixel)
        {
            const rs2::texture_coordinate& uv = tex_coords[idx];
            const int pixx = std::clamp(static_cast<int>(uv.u * texture->width), 0, texture->width - 1);
            const int pixy = std::clamp(static_cast<int>(uv.v * texture->height), 0, texture->height - 1);
            const size_t offset = (static_cast<size_t>(pixy) * static_cast<size_t>(texture->width) +
                                   static_cast<size_t>(pixx)) * static_cast<size_t>(texture->bytes_per_pixel);
            reverse_memcpy(p + layout.color_offset,
                           texture->data + offset,
                           static_cast<size_t>(texture->bytes_per_pixel));  // PointCloud2 order of rgb is bgr.
        }
        ++written;
    }
    return written;
}
#endif

void PointcloudFilter::Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id)
{
#ifdef BUILD_WITH_QIRP
    bool publish_qirp = false;
#endif
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
#ifdef BUILD_WITH_QIRP
        if (_use_qirp_transport)
        {
            if ((!_qirp_pointcloud_publisher) || (!(_qirp_pointcloud_publisher->get_subscription_count())))
                return;
            publish_qirp = true;
        }
        else
#endif
        {
            if ((!_pointcloud_publisher) || (!(_pointcloud_publisher->get_subscription_count())))
                return;
        }
    }

    rs2_stream texture_source_id = static_cast<rs2_stream>(_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(15);
    rs2::frameset::iterator texture_frame_itr = frameset.end();

    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8, rs2_format::RS2_FORMAT_Z16 };

        texture_frame_itr = std::find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f)
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = _filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
    }
    else {
        warn_count++;
        std::string texture_source_name = _filter->get_option_value_description(
            rs2_option::RS2_OPTION_STREAM_FILTER,
            static_cast<float>(texture_source_id)
        );
        ROS_WARN_STREAM_COND(
            warn_count == DISPLAY_WARN_NUMBER,
            "No matching stream for texture '" << texture_source_name
            << "'. Set 'pointcloud.stream_profile' to 'depth(1)', 'color(2)', or 'infrared(3)' to enable textured pointclouds."
        );
    }

    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();

    rs2_intrinsics depth_intrin = pc.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

#ifdef BUILD_WITH_QIRP
    if (publish_qirp)
    {
        TextureInfo texture_info;
        const TextureInfo* texture_ptr = nullptr;
        if (use_texture)
        {
            texture_info = extractTextureInfo(texture_frame_itr);
            texture_ptr = &texture_info;
        }

        const bool has_color_field = (texture_ptr != nullptr && !texture_ptr->field_name.empty());
        const PointCloudLayout& layout = getOrBuildLayout(
            has_color_field, has_color_field ? texture_ptr->field_name : std::string());

        const size_t pc_size = pc.size();
        const bool use_pool = (_dmabuf_pool.max_slots() > 0);
        const size_t alloc_points =
            (!use_pool && !_ordered_pc)
                ? countValidPoints(vertex, color_point, pc_size, use_texture)
                : pc_size;
        const size_t alloc_bytes = alloc_points * static_cast<size_t>(layout.point_step);
        if (alloc_bytes == 0)
        {
            return;
        }

        auto dmabuf = _dmabuf_pool.acquire(alloc_bytes);
        if (!dmabuf || !dmabuf->addr())
        {
            ROS_ERROR("QIRP PointCloud2 DmaBuffer allocation/map failed. Frame dropped.");
            return;
        }

        dmabuf->sync_start();
        const size_t written = packPointsToBuffer(
            static_cast<uint8_t*>(dmabuf->addr()), layout, vertex, color_point, pc_size, texture_ptr);
        dmabuf->sync_end();

        if (written == 0 && !_ordered_pc)
        {
            return;
        }

        auto qirp_msg = std::make_unique<qrb_ros::transport::type::PointCloud2>();
        qirp_msg->header.stamp = t;
        qirp_msg->header.frame_id = frame_id;
        qirp_msg->fields = layout.fields;
        qirp_msg->is_bigendian = layout.is_bigendian;
        qirp_msg->point_step = layout.point_step;
        if (_ordered_pc)
        {
            qirp_msg->width = static_cast<uint32_t>(depth_intrin.width);
            qirp_msg->height = static_cast<uint32_t>(depth_intrin.height);
            qirp_msg->is_dense = false;
        }
        else
        {
            qirp_msg->width = static_cast<uint32_t>(written);
            qirp_msg->height = 1;
            qirp_msg->is_dense = true;
        }
        qirp_msg->row_step = qirp_msg->width * qirp_msg->point_step;
        qirp_msg->data_fd = std::move(dmabuf);

        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (_qirp_pointcloud_publisher)
            _qirp_pointcloud_publisher->publish(std::move(qirp_msg));
        return;
    }
#endif

    sensor_msgs::msg::PointCloud2::UniquePtr msg_pointcloud = std::make_unique<sensor_msgs::msg::PointCloud2>();

    sensor_msgs::PointCloud2Modifier modifier(*msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pc.size());
    if (_ordered_pc)
    {
        msg_pointcloud->width = depth_intrin.width;
        msg_pointcloud->height = depth_intrin.height;
        msg_pointcloud->is_dense = false;
    }

    vertex = pc.get_vertices();
    size_t valid_count(0);
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            case RS2_FORMAT_Z16:
                // Depth can't be used as color texture — skip coloring
                format_str = "";  // Don't add any color field
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }        
        msg_pointcloud->point_step = addPointField(*msg_pointcloud, format_str.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32, msg_pointcloud->point_step);
        msg_pointcloud->row_step = msg_pointcloud->width * msg_pointcloud->point_step;
        msg_pointcloud->data.resize(msg_pointcloud->height * msg_pointcloud->row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(*msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(*msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(*msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(*msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
        {
            float i(color_point->u);
            float j(color_point->v);
            bool valid_color_pixel(i >= 0.f && i <=1.f && j >= 0.f && j <=1.f);
            bool valid_pixel(vertex->z > 0 && (valid_color_pixel || _allow_no_texture_points));
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                if (valid_color_pixel)
                {
                    color_pixel[0] = i * texture_width;
                    color_pixel[1] = j * texture_height;
                    int pixx = static_cast<int>(color_pixel[0]);
                    int pixy = static_cast<int>(color_pixel[1]);
                    int offset = (pixy * texture_width + pixx) * num_colors;
                    reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
                }
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_color;
                ++valid_count;
            }
        }
    }
    else
    {
        msg_pointcloud->row_step = msg_pointcloud->width * msg_pointcloud->point_step;
        msg_pointcloud->data.resize(msg_pointcloud->height * msg_pointcloud->row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(*msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(*msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(*msg_pointcloud, "z");

        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++)
        {
            bool valid_pixel(vertex->z > 0);
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                ++iter_x; ++iter_y; ++iter_z;
                ++valid_count;
            }
        }
    }
    msg_pointcloud->header.stamp = t;
    msg_pointcloud->header.frame_id = frame_id;
    if (!_ordered_pc)
    {
        msg_pointcloud->width = valid_count;
        msg_pointcloud->height = 1;
        msg_pointcloud->is_dense = true;
        modifier.resize(valid_count);
    }
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (_pointcloud_publisher)
            _pointcloud_publisher->publish(std::move(msg_pointcloud));
    }
}
