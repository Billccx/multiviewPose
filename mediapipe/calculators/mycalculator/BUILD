load("//mediapipe/framework/port:build_config.bzl", "mediapipe_cc_proto_library")

cc_library(
    name = "examples",
    srcs = ["examples.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/port:status",
    ],
    alwayslink = 1,
)

cc_library(
    name = "triangulation",
    srcs = ["triangulation.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
    ],
    alwayslink = 1,
)

cc_library(
    name = "triangulation2",
    srcs = ["triangulation2.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
    ],
    alwayslink = 1,
)

cc_library(
    name = "triangulation3",
    srcs = ["triangulation3.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
    ],
    alwayslink = 1,
)

cc_library(
    name = "unpacker",
    srcs = ["unpacker.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        #"//mediapipe/examples/desktop/mypose5:cv_helper",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework:packet",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
        
    ],
    alwayslink = 1,
)


cc_library(
    name = "triangulation4",
    srcs = ["triangulation4.cc","reProjectionError.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
        "@usr_local//:ceres",
        "@eigen//:eigen",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
    alwayslink = 1,
)


cc_library(
    name = "triangulation5",
    srcs = ["triangulation5.cc","reProjectionError.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
        "@usr_local//:ceres",
        "@eigen//:eigen",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
    alwayslink = 1,
)


cc_library(
    name = "unpacker2",
    srcs = ["unpacker2.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        #"//mediapipe/examples/desktop/mypose5:cv_helper",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "//mediapipe/framework:packet",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
        
    ],
    alwayslink = 1,
)

cc_library(
    name = "triangulation6",
    srcs = ["triangulation6.cc","reProjectionError2.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
        "@usr_local//:ceres",
        "@eigen//:eigen",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
    alwayslink = 1,
)

cc_library(
    name = "triangulation7",
    srcs = ["triangulation7.cc","reProjectionError2.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
        "@usr_local//:ceres",
        "@eigen//:eigen",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
    alwayslink = 1,
)


proto_library(
    name = "triangulation_calculator_proto",
    srcs = ["triangulation_calculator.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//mediapipe/framework:calculator_proto",
    ]
)

mediapipe_cc_proto_library(
    name = "triangulation_calculator_cc_proto",
    srcs = ["triangulation_calculator.proto"],
    cc_deps = ["//mediapipe/framework:calculator_cc_proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":triangulation_calculator_proto",
        "//mediapipe/framework:calculator_proto",
    ],
)


cc_library(
    name = "triangulation_calculator",
    srcs = ["triangulation_calculator.cc","reProjectionError2.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":triangulation_calculator_cc_proto",
        "//mediapipe/framework:calculator_framework",
        "//mediapipe/framework/port:ret_check",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework:timestamp",
        "//mediapipe/framework/formats:landmark_cc_proto",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
        "@usr_local//:ceres",
        "@eigen//:eigen",
        "//mediapipe/framework/port:status",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
    alwayslink = 1,
)
