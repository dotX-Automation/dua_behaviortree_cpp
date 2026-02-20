/**
 * BehaviorTree.CPP tree executor auxiliary routines.
 *
 * August 23, 2025
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

#define UNUSED(arg) (void)(arg)

#include <btcpp_executor/btcpp_executor.hpp>

namespace btcpp_executor
{

bool BTExecutor::start_bt_executor()
{
  bool ok = true;

  // Create factory
  bt_factory_ = std::make_unique<BT::BehaviorTreeFactory>();

  // Load plugins
  std::vector<std::string> plugins = get_parameter("plugins").as_string_array();
  for (auto & p : plugins) {
    if (p.empty()) {
      continue;
    }

    std::shared_ptr<dua_btcpp_base::BaseRegister> plugin = nullptr;
    try {
      plugin = plugins_loader_->createSharedInstance(p);
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to load plugin '%s': %s",
        p.c_str(), e.what());
      ok = false;
    }
    if (!ok) {
      break;
    }

    plugins_list_[p] = plugin;
  }
  if (!ok) {
    plugins_list_.clear();
    bt_factory_.reset();
    return false;
  }

  // Create global blackboard
  create_global_blackboard();

  // Load global blackboard contents from file
  std::string global_bb_init_file = get_parameter("btcpp.global_blackboard.init_file").as_string();
  if (!global_bb_init_file.empty()) {
    if (!global_bb_persistent_ || !global_bb_init_loaded_) {
      try {
        std::ifstream in_file(global_bb_init_file);
        nlohmann::json bb_init_j;
        in_file >> bb_init_j;
        BT::ImportBlackboardFromJSON(bb_init_j, *global_blackboard_);
        global_bb_init_loaded_ = true;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to load global blackboard init file: %s", e.what());
        global_bb_init_loaded_ = false;
        ok = false;
      }
    }
  }
  if (!ok) {
    delete_global_blackboard();
    bt_factory_.reset();
    entity_manager_->clear();
    plugins_list_.clear();
    return false;
  }

  // Register nodes from the plugins
  for (auto & pp : plugins_list_) {
    try {
      pp.second->register_nodes(
        *bt_factory_,
        this,
        entity_manager_,
        global_blackboard_,
        wait_servers_,
        false);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to register nodes for plugin '%s': %s",
        pp.first.c_str(), e.what());
      ok = false;
    }
    if (!ok) {
      break;
    }

    RCLCPP_INFO(get_logger(), "[BT.CPP PLUGIN] %s", pp.first.c_str());
  }
  if (!ok) {
    delete_global_blackboard();
    bt_factory_.reset();
    entity_manager_->clear();
    plugins_list_.clear();
    return false;
  }

  // Load tree XML files
  std::vector<std::string> tree_files = get_parameter("btcpp.tree_files").as_string_array();
  for (auto & tf : tree_files) {
    if (tf.empty()) {
      continue;
    }

    try {
      bt_factory_->registerBehaviorTreeFromFile(tf);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to load behavior tree from file '%s': %s",
        tf.c_str(), e.what());
      ok = false;
    }
    if (!ok) {
      break;
    }
  }
  if (!ok) {
    delete_global_blackboard();
    bt_factory_.reset();
    entity_manager_->clear();
    plugins_list_.clear();
    return false;
  }

  // Load main tree
  std::string main_tree_name = get_parameter("btcpp.main_tree_name").as_string();
  try {
    auto main_tree_bb = BT::Blackboard::create(global_blackboard_);
    bt_ = std::make_unique<BT::Tree>(bt_factory_->createTree(main_tree_name, main_tree_bb));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create behavior tree '%s': %s",
      main_tree_name.c_str(), e.what());
    ok = false;
  }
  if (!ok) {
    bt_.reset();
    delete_global_blackboard();
    bt_factory_.reset();
    entity_manager_->clear();
    plugins_list_.clear();
    return false;
  }
  RCLCPP_INFO(get_logger(), "[BT.CPP TREE] %s", main_tree_name.c_str());

  // Perform debugging operations
  if (debug_) {
    // Print tree recursively
    std::cout << "------ BT ------" << std::endl;
    BT::printTreeRecursively(bt_->rootNode());

    // Print blackboard
    std::cout << "------ BB ------" << std::endl;
    global_blackboard_->debugMessage();

    // Write TreeNodesModel to file
    std::string model_xml_path = get_parameter("btcpp.debug.treenodes_model.path").as_string();
    bool include_builtins = get_parameter("btcpp.debug.treenodes_model.builtins").as_bool();
    if (!model_xml_path.empty()) {
      std::string model_xml = BT::writeTreeNodesModelXML(*bt_factory_, include_builtins);
      std::ofstream out_file(model_xml_path);
      out_file << model_xml;
    }

    // Attach observer
    bt_observer_ = std::make_unique<BT::TreeObserver>(*bt_);
  }

  // Connect to Groot2
  if (get_parameter("btcpp.groot2_publisher").as_bool()) {
    // Reset the publisher first to delete old instances
    groot2_publisher_.reset();
    groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(*bt_);
  }

  // Start executor
  if (bt_executor_.joinable()) {
    bt_executor_.join();
  }
  running_.store(true, std::memory_order_release);
  bt_executor_ = std::thread(&BTExecutor::bt_executor_routine, this);

  return true;
}

void BTExecutor::create_global_blackboard()
{
  if (global_blackboard_ != nullptr && global_bb_persistent_) {
    return;
  }
  global_blackboard_ = BT::Blackboard::create();
}

void BTExecutor::delete_global_blackboard()
{
  if (global_bb_persistent_) {
    return;
  }
  global_blackboard_.reset();
}

std::string BTExecutor::node_status_to_string(const BT::NodeStatus & status)
{
  switch (status) {
    case BT::NodeStatus::IDLE:
      return "IDLE";
    case BT::NodeStatus::RUNNING:
      return "RUNNING";
    case BT::NodeStatus::SUCCESS:
      return "SUCCESS";
    case BT::NodeStatus::FAILURE:
      return "FAILURE";
    case BT::NodeStatus::SKIPPED:
      return "SKIPPED";
    default:
      return "UNKNOWN";
  }
}

bool BTExecutor::validate_global_blackboard_init_file(const rclcpp::Parameter & p)
{
  UNUSED(p);

  // Just signal that this has to be reloaded in any case
  global_bb_init_loaded_ = false;
  return true;
}

} // namespace btcpp_executor
