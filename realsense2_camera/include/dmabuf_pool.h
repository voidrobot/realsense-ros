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

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace realsense2_camera
{

// Single source of truth for QIRP DMA-BUF transport defaults and bounds.
namespace qirp_defaults
{
inline constexpr std::size_t kDefaultPoolSize = 8;
inline constexpr std::size_t kMaxPoolSize = 32;
inline constexpr const char* kDefaultDmaHeapPath = "/dev/dma_heap/system";
}  // namespace qirp_defaults

// RAII custom-deleter DMA-BUF ring pool.
//
// Returned std::shared_ptr<BufferT> leases keep the exact external type required
// by qrb_ros::transport::type::{Image,PointCloud2} while automatically returning
// pooled slots to the free list (or unmapping transient/stale-generation buffers)
// when the last downstream subscriber releases its reference.
template <typename BufferT>
class BasicDmaBufferPool
{
public:
    explicit BasicDmaBufferPool(std::size_t max_slots = qirp_defaults::kDefaultPoolSize,
                                std::string heap_path = qirp_defaults::kDefaultDmaHeapPath)
        : _state(std::make_shared<PoolState>(max_slots, std::move(heap_path)))
    {
    }

    ~BasicDmaBufferPool()
    {
        std::vector<std::shared_ptr<BufferT>> to_unmap;
        {
            std::lock_guard<std::mutex> lock(_state->mtx);
            _state->shutdown = true;
            ++_state->generation;
            to_unmap.swap(_state->free_list);
            _state->slot_bytes = 0;
            _state->allocated_slots = 0;
        }
        for (auto& buf : to_unmap)
        {
            if (buf)
                buf->unmap();
        }
    }

    BasicDmaBufferPool(const BasicDmaBufferPool&) = delete;
    BasicDmaBufferPool& operator=(const BasicDmaBufferPool&) = delete;

    std::shared_ptr<BufferT> acquire(std::size_t required_bytes)
    {
        if (required_bytes == 0)
            return nullptr;

        if (_state->max_slots == 0)
        {
            return make_transient(required_bytes);
        }

        std::vector<std::shared_ptr<BufferT>> stale_buffers;
        std::shared_ptr<BufferT> reused;
        std::uint64_t alloc_generation = 0;
        std::size_t alloc_size = 0;
        bool should_alloc_pooled = false;

        {
            std::lock_guard<std::mutex> lock(_state->mtx);

            const bool need_resize =
                (_state->slot_bytes == 0) ||
                (required_bytes > _state->slot_bytes) ||
                (required_bytes * 2 < _state->slot_bytes);
            if (need_resize)
            {
                ++_state->generation;
                stale_buffers.swap(_state->free_list);
                _state->slot_bytes = required_bytes;
                _state->allocated_slots = 0;
            }

            while (!_state->free_list.empty())
            {
                auto candidate = std::move(_state->free_list.back());
                _state->free_list.pop_back();
                if (!candidate)
                    continue;
                // External legacy subscribers invoke TypeAdapter::convert_to_ros_message(),
                // which may unmap the shared buffer; transparently restore mapping if needed.
                if (!candidate->addr() && !candidate->map())
                {
                    if (_state->allocated_slots > 0)
                        --_state->allocated_slots;
                    stale_buffers.push_back(std::move(candidate));
                    continue;
                }
                reused = std::move(candidate);
                alloc_generation = _state->generation;
                break;
            }

            if (!reused && _state->allocated_slots < _state->max_slots)
            {
                ++_state->allocated_slots;
                alloc_generation = _state->generation;
                alloc_size = _state->slot_bytes;
                should_alloc_pooled = true;
            }
        }

        for (auto& stale : stale_buffers)
        {
            if (stale)
                stale->unmap();
        }

        if (reused)
        {
            return wrap_pooled(std::move(reused), alloc_generation);
        }

        if (should_alloc_pooled)
        {
            auto fresh = alloc_and_map(alloc_size, true);
            if (!fresh)
            {
                std::lock_guard<std::mutex> lock(_state->mtx);
                if (alloc_generation == _state->generation && _state->allocated_slots > 0)
                {
                    --_state->allocated_slots;
                }
                return nullptr;
            }
            return wrap_pooled(std::move(fresh), alloc_generation);
        }

        // All pooled slots are currently held by downstream consumers;
        // allocate a transient buffer with automatic unmap on release.
        return make_transient(required_bytes);
    }

    std::size_t max_slots() const { return _state->max_slots; }

    std::size_t free_count() const
    {
        std::lock_guard<std::mutex> lock(_state->mtx);
        return _state->free_list.size();
    }

    std::uint64_t generation() const
    {
        std::lock_guard<std::mutex> lock(_state->mtx);
        return _state->generation;
    }

private:
    struct PoolState
    {
        PoolState(std::size_t max_slots_in, std::string heap_path_in)
            : max_slots(max_slots_in),
              heap_path(std::move(heap_path_in))
        {
            free_list.reserve(max_slots);
        }

        const std::size_t max_slots;
        const std::string heap_path;
        std::mutex mtx;
        std::vector<std::shared_ptr<BufferT>> free_list;
        std::size_t slot_bytes{0};
        std::size_t allocated_slots{0};
        std::uint64_t generation{0};
        bool shutdown{false};
    };

    std::shared_ptr<BufferT> alloc_and_map(std::size_t bytes, bool zero_init) const
    {
        auto buf = BufferT::alloc(bytes, _state->heap_path);
        if (!buf || !buf->map())
            return nullptr;
        if (zero_init && buf->addr())
        {
            buf->sync_start();
            std::memset(buf->addr(), 0, bytes);
            buf->sync_end();
        }
        return buf;
    }

    std::shared_ptr<BufferT> make_transient(std::size_t bytes) const
    {
        auto raw = alloc_and_map(bytes, false);
        if (!raw)
            return nullptr;
        return std::shared_ptr<BufferT>(
            raw.get(),
            [raw](BufferT*) mutable {
                if (raw)
                    raw->unmap();
            });
    }

    std::shared_ptr<BufferT> wrap_pooled(std::shared_ptr<BufferT> raw, std::uint64_t gen) const
    {
        auto state = _state;
        return std::shared_ptr<BufferT>(
            raw.get(),
            [state, gen, raw](BufferT*) mutable {
                release_to_pool(state, gen, std::move(raw));
            });
    }

    static void release_to_pool(const std::shared_ptr<PoolState>& state,
                                std::uint64_t gen,
                                std::shared_ptr<BufferT> raw)
    {
        if (!raw)
            return;
        bool returned = false;
        {
            std::lock_guard<std::mutex> lock(state->mtx);
            if (!state->shutdown &&
                gen == state->generation &&
                state->free_list.size() < state->max_slots)
            {
                state->free_list.push_back(std::move(raw));
                returned = true;
            }
        }
        if (!returned && raw)
        {
            raw->unmap();
        }
    }

    std::shared_ptr<PoolState> _state;
};

}  // namespace realsense2_camera

#ifdef BUILD_WITH_QIRP
#include <lib_mem_dmabuf/dmabuf.hpp>

namespace realsense2_camera
{
using DmaBufferPool = BasicDmaBufferPool<lib_mem_dmabuf::DmaBuffer>;
}  // namespace realsense2_camera
#endif  // BUILD_WITH_QIRP
