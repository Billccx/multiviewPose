licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_binary(
    name = "libposetracking.so",
    srcs = ["my_pose_tracking.cpp","my_pose_tracking.hpp"],
    deps = [
        "//mediapipe/framework/port:status",
        "//mediapipe/framework/port:parse_text_proto",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "//mediapipe/graphs/pose_tracking:pose_tracking_cpu_deps",
        "//mediapipe/calculators/video:opencv_video_decoder_calculator",
        "//mediapipe/calculators/video:opencv_video_encoder_calculator",
    ],
    linkshared=True,
)
