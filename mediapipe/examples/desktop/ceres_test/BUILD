# BUILD
licenses(["notice"])
package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
  name = "ceres_ba",
  srcs = [
    "ceres_ba.cpp",
    "projectionError.hpp",
    "projectionError2.hpp"
  ],
  deps = [
    "@usr_local//:librealsense2",
    "@usr_local//:opencv",
    "@usr_local//:opencv2",
    "@usr_local//:ceres",
    "@eigen//:eigen",
    #"//mediapipe/framework:calculator_framework",
    #"//mediapipe/framework/port:file_helpers",
    "//mediapipe/framework/port:status",
    "@com_google_absl//absl/flags:flag",
    "@com_google_absl//absl/flags:parse",
  ],
)
