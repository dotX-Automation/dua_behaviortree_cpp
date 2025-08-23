# dua_btcpp_nodes

Library of `BT.CPP` nodes for the DUA framework.

## Contents

The tree nodes provided in this package allow the tree to interact with various parts of a ROS 2 software architecture.

### Action clients

- [x] **`ArmComponent`**: Arms a component calling the `dua_hardware_interfaces/action/Arm` action.
- [x] **`DisarmComponent`**: Disarms a component calling the `dua_hardware_interfaces/action/Disarm` action.

### Service clients

- [x] **`SetComponentState`**: Sets the state of a component calling the `std_srvs/srv/SetBool` service.
- [x] **`TriggerComponent`**: Triggers a component calling the `std_srvs/srv/Trigger` service.

### Topic subscribers

- [x] **`SubscriberTrigger`**: `StatefulActionNode` that waits for an `std_msgs/msg/Empty` message.

### Topic publishers

- [x] **`PublishString`**: Publishes a string message to a ROS 2 topic.

## Usage

This library can be included in a project and linked against.

Alternatively, it can be loaded as a plugin using `pluginlib`. It inherits from the `dua_btcpp_base::BaseRegister` class.

See [`dua_btcpp_nodes.hpp`](include/dua_btcpp_nodes/dua_btcpp_nodes.hpp) and the `register_nodes()` method for more information on how to initialize this library with `BehaviorTree.CPP` at runtime.

---

## Copyright and License

Copyright 2025 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
