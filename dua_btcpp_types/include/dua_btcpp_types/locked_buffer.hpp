/**
 * Locked, data-agnostic buffer for thread-safe blackboard access.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * February 20, 2026
 */

/**
 * Copyright 2026 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <mutex>
#include <utility>

namespace dua_btcpp_types
{

/**
 * A thread-safe, type-agnostic locked buffer that can be used for blackboard access in behavior trees.
 */
template<typename DataT>
class LockedBuffer
{
public:
  using SharedPtr = std::shared_ptr<LockedBuffer<DataT>>;
  using WeakPtr = std::weak_ptr<LockedBuffer<DataT>>;
  using UniquePtr = std::unique_ptr<LockedBuffer<DataT>>;
  using ConstSharedPtr = std::shared_ptr<const LockedBuffer<DataT>>;
  using ConstWeakPtr = std::weak_ptr<const LockedBuffer<DataT>>;

  /**
   * @brief Constructor.
   */
  explicit LockedBuffer() = default;

  /**
   * @brief Destructor.
   */
  ~LockedBuffer() = default;

  /**
   * @brief Copies provided data into the buffer, if not already filled or allowed to overwrite.
   *
   * @param data The data to copy into the buffer.
   * @param overwrite If true, allows overwriting existing valid data.
   * @return Whether the data was actually written to the buffer.
   */
  bool write(const DataT & data, bool overwrite = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (valid_ && !overwrite) {
      return false;
    }

    buffer_ = data;
    valid_ = true;
    return true;
  }

  /**
   * @brief Copies provided data into the buffer, if not already filled or allowed to overwrite.
   *
   * @param data The data to copy into the buffer.
   * @param overwrite If true, allows overwriting existing valid data.
   * @return Whether the data was actually written to the buffer.
   */
  bool write(DataT && data, bool overwrite = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (valid_ && !overwrite) {
      return false;
    }

    buffer_ = std::move(data);
    valid_ = true;
    return true;
  }

  /**
   * @brief Reads data from the buffer into the provided reference, if valid.
   *
   * @param data Reference to store the read data.
   * @param consume If true, marks the buffer as invalid after reading (consuming the data).
   * @return Whether valid data was read from the buffer.
   */
  bool read(DataT & data, bool consume = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!valid_) {
      return false;
    }

    if (consume) {
      data = std::move(buffer_);
      valid_ = false;
    } else {
      data = buffer_;
    }
    return true;
  }

  /**
   * @brief Checks whether the buffer contains valid data.
   *
   * @return Whether the buffer contains valid, unconsumed data.
   */
  bool valid() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return valid_;
  }

  /**
   * @brief Invalidates the buffer, discarding any stored data.
   */
  void invalidate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = false;
  }

private:
  /* The actual data buffer. */
  DataT buffer_;

  /* Mutex for synchronizing access to the buffer. */
  mutable std::mutex mutex_;

  /* Valid data flag, to indicate whether the data has been set or consumed. */
  bool valid_ = false;
};

} // namespace dua_btcpp_types
