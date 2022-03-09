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
    name = "mypose3_cpu",
    #srcs = ["//mediapipe/examples/desktop/mypose2.cc",],
    deps = [
        "//mediapipe/examples/desktop:mypose3",
        "//mediapipe/graphs/pose_tracking:pose_tracking_cpu_deps",
        "//mediapipe/calculators/mycalculator:examples",  #add new defined calculator
        "//mediapipe/calculators/mycalculator:triangulation",
        "//mediapipe/calculators/core:pass_through_calculator",
        "@usr_local//:librealsense2",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/port:file_helpers",
        "//mediapipe/framework/port:opencv_highgui",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/framework/port:opencv_video",
        "//mediapipe/framework/port:parse_text_proto",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "//mediapipe/framework/formats:detection_cc_proto",
		"//mediapipe/framework/formats:landmark_cc_proto",
		"//mediapipe/framework/formats:rect_cc_proto",
    ],
)
