# third_party/usr_local.BUILD
cc_library(
  name = "librealsense2",
  hdrs = glob(["include/librealsense2/**"]),
  srcs = glob(["lib/*.so*"]),
  includes = ["include/librealsense2",],
  visibility = ["//visibility:public"],
  linkstatic = 1,
)

cc_library(
  name = "opencv2",
  hdrs = glob(["include/opencv2/**"]),
  srcs = glob(["lib/*.so*"]),
  includes = ["include/opencv2",],
  visibility = ["//visibility:public"],
  linkstatic = 1,
)

cc_library(
  name = "opencv",
  hdrs = glob(["include/opencv/**"]),
  srcs = glob(["lib/*.so*"]),
  includes = ["include/opencv",],
  visibility = ["//visibility:public"],
  linkstatic = 1,
)

cc_library(
  name = "ceres",
  hdrs = glob(["include/ceres/**"]),
  srcs = glob(["lib/*.so*"]),
  includes = ["include/ceres",],
  visibility = ["//visibility:public"],
  linkstatic = 1,
)

