# dua_behaviortree_cpp

ROS 2 components for the integration of the [`BehaviorTree.CPP`](https://github.com/BehaviorTree/BehaviorTree.CPP) library in the DUA framework.

## Contents

The integration of the `BehaviorTree.CPP` library in DUA is primarily aimed at robotics applications.

For this reason, the contents of this repository are organized to provide an integration framework for ROS 2-based software architectures.

The user is provided with a set of software components, which include a composable node acting as a Tree executor, than can be used to both build BTs quickly using existing components or to create and integrate custom nodes.

The following software packages are provided.

- [x] [**`dua_btcpp`**](dua_btcpp/README.md): Library of `BT.CPP` nodes and tools for the DUA framework.
- [x] [**`btcpp_executor`**](btcpp_executor/README.md): ROS 2 composable node acting as BT executor.
- [x] [**`dua_btcpp_extender`**](dua_btcpp_extender/README.md): Base class for `dua_btcpp` custom extensions, based on `pluginlib` and integrated in the `btcpp_executor`.
- [x] [**`dua_behaviortree_cpp`**](dua_behaviortree_cpp/): Metapackage.

---

## Copyright and License

Copyright 2025 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
