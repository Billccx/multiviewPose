# Copyright 2020 The MediaPipe Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "mypose2_cpu",
    #srcs = ["//mediapipe/examples/desktop/mypose2.cc",],
    deps = [
        "//mediapipe/examples/desktop:mypose2",
        #"//mediapipe/graphs/pose_tracking:pose_tracking_cpu_deps",
        "//mediapipe/calculators/mycalculator:examples",  #add new defined calculator
        "@usr_local//:librealsense2",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
    ],
)
