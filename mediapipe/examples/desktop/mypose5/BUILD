licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

cc_binary(
    name = "mypose5_cpu",
    #srcs = ["//mediapipe/examples/desktop/mypose2.cc",],
    deps = [
        "//mediapipe/examples/desktop:mypose5",
        "//mediapipe/graphs/pose_tracking:pose_tracking_cpu_deps",
        "//mediapipe/calculators/mycalculator:examples",  #add new defined calculator
        "//mediapipe/calculators/mycalculator:triangulation3",
        "//mediapipe/calculators/mycalculator:unpacker",
        "//mediapipe/calculators/core:pass_through_calculator",
        "@usr_local//:librealsense2",
        "@usr_local//:opencv",
        "@usr_local//:opencv2",
        "@usr_local//:ceres",
        "@eigen//:eigen",
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

# cc_library(
#     name = "cv_helper",
#     srcs = ["cv_helper.cpp"],
#     hdrs = ["cv_helper.hpp"],
#     visibility = ["//visibility:public"],
#     deps = [
#         "@usr_local//:librealsense2",
#         "@usr_local//:opencv",
#         "@usr_local//:opencv2",
#     ],
# )
