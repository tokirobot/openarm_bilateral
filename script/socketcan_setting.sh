#!/bin/bash
#
# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

echo "spesify the can device like  "can0" "

if [ -z "$1" ]; then
  echo "Usage: $0 <can_interface> (e.g. can0)"
  exit 1
fi

CAN_IF=$1

sudo ip link set $CAN_IF down
sudo ip link set $CAN_IF type can bitrate 1000000 
sudo ip link set $CAN_IF up

echo "$CAN_IF is now set to CAN mode (1Mbps / 1Mbps)."

