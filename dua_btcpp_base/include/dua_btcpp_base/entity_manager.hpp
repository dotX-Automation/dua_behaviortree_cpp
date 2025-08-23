/**
 * Map from entity names to ROS 2 object references to avoid endpoint duplication.
 *
 * dotX Automation s.r.l. <info@dotxautomation.com>
 *
 * August 21, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
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

#include <any>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include <dua_node_cpp/dua_node.hpp>

namespace dua_btcpp_base
{

/**
 * A thread-safe cache for ROS 2 entities, which can be used to retrieve existing entity
 * instances, avoiding duplication of entities that do not require it.
 */
class EntityManager
{
public:
  using SharedPtr = std::shared_ptr<EntityManager>;
  using WeakPtr = std::weak_ptr<EntityManager>;
  using UniquePtr = std::unique_ptr<EntityManager>;
  using ConstSharedPtr = std::shared_ptr<const EntityManager>;
  using ConstWeakPtr = std::weak_ptr<const EntityManager>;

  /**
   * @brief Constructor.
   *
   * @param node ROS 2 node pointer.
   * @throws std::invalid_argument if node pointer is invalid.
   */
  explicit EntityManager(const dua_node::NodeBase::SharedPtr & node)
  : node_(node)
  {
    if (!node_) {
      throw std::invalid_argument(
          "dua_btcpp_base:EntityManager::EntityManager: Invalid ROS 2 node pointer.");
    }
  }

  /**
   * @brief Destructor.
   */
  ~EntityManager()
  {
    clear();
  }

  /**
   * @brief Clears all clients from the cache.
   */
  void clear()
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    cache_.clear();
  }

  /**
   * @brief Removes an entity for a given service/action from the cache.
   *
   * @param entity_name Name of the entity (key in the cache).
   * @return True if the entity was found and removed, false otherwise.
   */
  bool remove_entity(const std::string & entity_name)
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    auto it = cache_.find(entity_name);
    if (it != cache_.end()) {
      cache_.erase(it);
      return true;
    }
    return false;
  }

  /**
   * @brief Retrieves a topic publisher from the cache, creating it if it doesn't exist.
   *
   * @param topic_name Name of the topic (key in the cache).
   * @return Pointer to the topic publisher.
   * @throws std::runtime_error if the found publisher's type does not match the expected type.
   */
  template<typename MessageT>
  std::shared_ptr<rclcpp::Publisher<MessageT>> get_publisher(const std::string & topic_name)
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    auto it = cache_.find(topic_name);
    if (it != cache_.end()) {
      // Existing publisher found, cast and return it
      return extract_typed_entity_ptr<rclcpp::Publisher<MessageT>>(it->second, topic_name);
    }

    // Create a new publisher for the given topic, store it, and return it
    auto new_publisher = node_->dua_create_publisher<MessageT>(topic_name);
    cache_[topic_name] = new_publisher;
    return new_publisher;
  }

  /**
   * @brief Retrieves a service client from the cache, creating it if it doesn't exist.
   *
   * @param service_name Service name (key in the cache).
   * @param wait_server Wait for server to come online upon client creation.
   * @return Pointer to the service client.
   * @throws std::runtime_error if the found client's type does not match the expected type.
   */
  template<typename ServiceT>
  std::shared_ptr<simple_serviceclient::Client<ServiceT>> get_service_client(
    const std::string & service_name,
    bool wait_server = false)
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    auto it = cache_.find(service_name);
    if (it != cache_.end()) {
      // Existing client found, cast and return it
      return extract_typed_entity_ptr<simple_serviceclient::Client<ServiceT>>(it->second, service_name);
    }

    // Create a new client for the given service, store it, and return it
    auto new_client = node_->dua_create_service_client<ServiceT>(service_name, wait_server);
    cache_[service_name] = new_client;
    return new_client;
  }

  /**
   * @brief Retrieves an action client from the cache, creating it if it doesn't exist.
   *
   * @param action_name Action name (key in the cache).
   * @param wait_server Wait for server to come online upon client creation.
   * @return Pointer to the action client.
   * @throws std::runtime_error if the found client's type does not match the expected type.
   */
  template<typename ActionT>
  std::shared_ptr<simple_actionclient::Client<ActionT>> get_action_client(
    const std::string & action_name,
    bool wait_server = false)
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    auto it = cache_.find(action_name);
    if (it != cache_.end()) {
      // Existing client found, cast and return it
      return extract_typed_entity_ptr<simple_actionclient::Client<ActionT>>(it->second, action_name);
    }

    // Create a new client for the given action, store it, and return it
    auto new_client = node_->dua_create_action_client<ActionT>(action_name, nullptr, wait_server);
    cache_[action_name] = new_client;
    return new_client;
  }

private:
  /**
   * @brief Extracts and type-checks an entity reference from the cache.
   *
   * @param entity_any Entity reference as std::any.
   * @param entity_name Expected entity name (for error reporting).
   * @return The typed entity.
   * @throws std::runtime_error if types don't match.
   */
  template<typename EntityT>
  std::shared_ptr<EntityT> extract_typed_entity_ptr(
    const std::any & entity_any,
    const std::string & entity_name) const
  {
    try {
      return std::any_cast<std::shared_ptr<EntityT>>(entity_any);
    } catch (const std::bad_any_cast & e) {
      const auto & stored_type = entity_any.type();
      const auto & expected_type = typeid(std::shared_ptr<EntityT>);
      std::string err_msg =
        "dua_btcpp_base::EntityManager::extract_typed_entity_ptr: Type mismatch for entity '" +
        entity_name + "'" +
        " (expected: " + expected_type.name() +
        ", got: " + stored_type.name() + ")";
      RCLCPP_FATAL(
        node_->get_logger(),
        "%s",
        err_msg.c_str());
      throw std::runtime_error(err_msg);
    }
  }

  /* Pointer to the node. */
  dua_node::NodeBase::SharedPtr node_;

  /* Entities cache. */
  std::map<std::string, std::any> cache_;

  /* Entities cache lock. */
  std::mutex cache_lock_;
};

} // namespace dua_btcpp_base
