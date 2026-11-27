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

#ifdef BUILD_WITH_QIRP

#include <cstddef>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <lib_mem_dmabuf/dmabuf.hpp>

namespace realsense2_camera
{

// Pre-mapped DMA-BUF ring pool that reuses slots whose shared_ptr use_count() == 1
// (no subscriber holds a reference). Eliminates per-frame kernel alloc/mmap/munmap
// TLB shootdown overhead while guaranteeing zero data races with active subscribers.
class DmaBufferPool
{
public:
    explicit DmaBufferPool(std::size_t max_slots = 8,
                           std::string heap_path = "/dev/dma_heap/system")
        : _max_slots(max_slots),
          _heap_path(std::move(heap_path))
    {
    }

    ~DmaBufferPool()
    {
        std::lock_guard<std::mutex> lock(_mtx);
        release_locked();
    }

    DmaBufferPool(const DmaBufferPool&) = delete;
    DmaBufferPool& operator=(const DmaBufferPool&) = delete;

    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> acquire(std::size_t required_bytes)
    {
        if (required_bytes == 0)
            return nullptr;

        if (_max_slots == 0)
        {
            return alloc_and_map(required_bytes, false);
        }

        std::lock_guard<std::mutex> lock(_mtx);

        if (required_bytes > _slot_bytes)
        {
            release_locked();
            _slot_bytes = required_bytes;
            _next_idx = 0;
        }

        const std::size_t n = _slots.size();
        for (std::size_t k = 0; k < n; ++k)
        {
            const std::size_t idx = (_next_idx + k) % n;
            auto& slot = _slots[idx];
            if (slot && slot.use_count() == 1)
            {
                // External legacy subscribers invoke TypeAdapter::convert_to_ros_message(),
                // which may unmap the shared DmaBuffer; re-map transparently if needed.
                if (!slot->addr() && !slot->map())
                {
                    continue;
                }
                _next_idx = (idx + 1) % n;
                return slot;
            }
        }

        if (_slots.size() < _max_slots)
        {
            auto fresh = alloc_and_map(_slot_bytes, true);
            if (!fresh)
                return nullptr;
            _slots.push_back(fresh);
            _next_idx = 0;
            return fresh;
        }

        // All pooled slots are currently held by downstream consumers;
        // allocate a transient buffer so we never overwrite in-flight data.
        return alloc_and_map(required_bytes, false);
    }

    std::size_t max_slots() const { return _max_slots; }

private:
    std::shared_ptr<lib_mem_dmabuf::DmaBuffer> alloc_and_map(std::size_t bytes, bool zero_init) const
    {
        auto buf = lib_mem_dmabuf::DmaBuffer::alloc(bytes, _heap_path);
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

    void release_locked()
    {
        for (auto& slot : _slots)
        {
            if (slot && slot.use_count() == 1)
            {
                slot->unmap();
            }
        }
        _slots.clear();
        _slot_bytes = 0;
    }

    std::size_t _max_slots{8};
    std::string _heap_path;
    std::mutex _mtx;
    std::vector<std::shared_ptr<lib_mem_dmabuf::DmaBuffer>> _slots;
    std::size_t _slot_bytes{0};
    std::size_t _next_idx{0};
};

}  // namespace realsense2_camera

#endif  // BUILD_WITH_QIRP
