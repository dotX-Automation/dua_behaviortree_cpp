/**
 * Map from entity names to client object references to avoid endpoint duplication.
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

#include "visibility_control.h"

#include <any>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include <dua_node_cpp/dua_node.hpp>

namespace dua_btcpp
{

/**
 * A thread-safe cache for service and action clients, which can be used to retrieve existing client
 * instances avoiding duplication.
 */
class DUA_BTCPP_PUBLIC ClientManager{
public:
  /**
   * @brief Constructor.
   *
   * @param node ROS 2 node pointer.
   * @throws std::invalid_argument if node pointer is invalid.
   */
  explicit ClientManager(const dua_node::NodeBase::SharedPtr & node)
  : node_(node)
  {
    if (!node_) {
      throw std::invalid_argument(
          "dua_btcpp::ClientManager::ClientManager: Invalid ROS 2 node pointer.");
    }
  }

  /**
   * @brief Destructor.
   */
  ~ClientManager()
  {
    clear();
  }

  /**
   * @brief Clears all clients from the cache.
   */
  void clear()
  {
    std::lock_guard<std::mutex> lock(clients_cache_lock_);

    clients_cache_.clear();
  }

  /**
   * @brief Removes a client for a given service/action from the cache.
   *
   * @param endpoint_name Service/action name (key in the cache).
   * @return True if the client was found and removed, false otherwise.
   */
  bool remove_client(const std::string & endpoint_name)
  {
    std::lock_guard<std::mutex> lock(clients_cache_lock_);

    auto it = clients_cache_.find(endpoint_name);
    if (it != clients_cache_.end()) {
      clients_cache_.erase(it);
      return true;
    }
    return false;
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
    std::lock_guard<std::mutex> lock(clients_cache_lock_);

    auto it = clients_cache_.find(service_name);
    if (it != clients_cache_.end()) {
      // Existing client found, cast and return it
      return extract_typed_client<simple_serviceclient::Client<ServiceT>>(it->second, service_name);
    }

    // Create a new client for the given service, store it, and return it
    auto new_client = node_->dua_create_service_client<ServiceT>(service_name, wait_server);
    clients_cache_[service_name] = new_client;
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
    std::lock_guard<std::mutex> lock(clients_cache_lock_);

    auto it = clients_cache_.find(action_name);
    if (it != clients_cache_.end()) {
      // Existing client found, cast and return it
      return extract_typed_client<simple_actionclient::Client<ActionT>>(it->second, action_name);
    }

    // Create a new client for the given action, store it, and return it
    auto new_client = node_->dua_create_action_client<ActionT>(action_name, wait_server);
    clients_cache_[action_name] = new_client;
    return new_client;
  }

private:
  /**
   * @brief Extracts and type-checks a client reference from the cache.
   *
   * @param client_any Client reference as std::any.
   * @param client_name Expected client name (for error reporting).
   * @return The typed client.
   * @throws std::runtime_error if types don't match.
   */
  template<typename ClientT>
  std::shared_ptr<ClientT> extract_typed_client(
    const std::any & client_any,
    const std::string & client_name) const
  {
    try {
      return std::any_cast<std::shared_ptr<ClientT>>(client_any);
    } catch (const std::bad_any_cast & e) {
      const auto & stored_type = client_any.type();
      const auto & expected_type = typeid(std::shared_ptr<ClientT>);
      std::string err_msg =
        "dua_btcpp::ClientManager::extract_typed_client: Type mismatch for client '" +
        client_name + "'" +
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

  /* Clients cache. */
  std::map<std::string, std::any> clients_cache_;

  /* Clients cache lock. */
  std::mutex clients_cache_lock_;
};

} // namespace dua_btcpp
