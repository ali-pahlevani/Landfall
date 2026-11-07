// Copyright 2026 Ali Pahlevani
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

// The behavior tree plugin library `landfall_nav2_bt_nodes`: Nav2's navigator loads it from its
// plugin_lib_names parameter (docs/nav2_integration.md).

#include "landfall/nav2/compat/bt.hpp"
#include "landfall/nav2/is_localized_condition.hpp"
#include "landfall/nav2/relocalize_action.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<landfall::nav2::ReLocalize>("ReLocalize");
  factory.registerNodeType<landfall::nav2::IsLocalized>("IsLocalized");
}
