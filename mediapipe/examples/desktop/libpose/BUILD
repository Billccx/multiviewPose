licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "libpose.so",
    deps = [
        ":bg_remover"
    ],
    linkopts = ["-Wl,-soname,libpose.so"],
    linkshared = 1 ## Very important
)

cc_library(
    name = "bg_remover",
    srcs = ["pose.cpp", "pose_impl.cpp"],
    hdrs = ["pose.h", "pose_impl.h"],
    visibility = ["//visibility:public"],
    deps = [
        # Deps for example_impl.cpp
        "//mediapipe/framework/port:parse_text_proto",
        "@com_google_absl//absl/flags:parse",

        "//mediapipe/framework:packet",

        # Remove any of these if you need to build only one example
        # Pose tracking
        "//mediapipe/graphs/pose_tracking:pose_tracking_cpu_deps",
        "//mediapipe/calculators/mycalculator:triangulation_calculator",
        "//mediapipe/calculators/mycalculator:unpacker2",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:librealsense2",
	],
    alwayslink = 1,
)
