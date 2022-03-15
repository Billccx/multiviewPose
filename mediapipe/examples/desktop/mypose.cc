// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.
#include <cstdlib>
#include <iostream>
#include <librealsense2/rs.hpp>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

#include "mediapipe/framework/formats/detection.pb.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/rect.pb.h"

constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
constexpr char kWindowName[] = "MediaPipe";

ABSL_FLAG(std::string, calculator_graph_config_file, "",
          "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "",
          "Full path of video to load. "
          "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "",
          "Full path of where to save result (.mp4 only). "
          "If not provided, show result in a window.");

static cv::Mat frame_to_mat(const rs2::frame& f){
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8){
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8){
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16){
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8){
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32){
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

absl::Status RunMPPGraph(){
  std::string calculator_graph_config_contents;

  //读取解析计算图
  MP_RETURN_IF_ERROR(
    mediapipe::file::GetContents(
      absl::GetFlag(FLAGS_calculator_graph_config_file),
      &calculator_graph_config_contents
    )
  );

  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;

  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
        calculator_graph_config_contents
      );

  //依照解析结果初始化计算图
  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  //解析视频输入参数，默认使用摄像头
  LOG(INFO) << "Initialize the camera or load the video.";
  cv::VideoCapture capture;
  const bool load_video = !absl::GetFlag(FLAGS_input_video_path).empty();
  if (load_video){
    capture.open(absl::GetFlag(FLAGS_input_video_path));
  }
  else{
    capture.open(0);
  }
  RET_CHECK(capture.isOpened());

  //解析输出参数，默认为产生一个窗口
  cv::VideoWriter writer;
  const bool save_video = !absl::GetFlag(FLAGS_output_video_path).empty();
  if (!save_video){
    cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(cv::CAP_PROP_FPS, 30);
#endif
  }

  LOG(INFO) << "Start running the calculator graph.";
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));

  MP_RETURN_IF_ERROR(graph.StartRun({}));

  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;


  //new add
  rs2::pipeline p;
  p.start();

  while (grab_frames){
    // Capture opencv camera or video frame.
    //new add
    rs2::frameset frames = p.wait_for_frames();
    rs2::video_frame color = frames.get_color_frame();
    cv::Mat camera_frame =frame_to_mat(color);
    cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGR2RGB);

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::SRGB, 
      camera_frame.cols, 
      camera_frame.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary
    );
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

    // Send image packet into the graph.
    size_t frame_timestamp_us = (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

    MP_RETURN_IF_ERROR(
        graph.AddPacketToInputStream(
          kInputStream, 
          mediapipe::Adopt(input_frame.release()).At(mediapipe::Timestamp(frame_timestamp_us))
        )
    );

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    if (!poller.Next(&packet))
      break;
    auto &output_frame = packet.Get<mediapipe::ImageFrame>();

    // Convert back to opencv for display or saving.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
    cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
    if (save_video)
    {
      if (!writer.isOpened())
      {
        LOG(INFO) << "Prepare video writer.";
        writer.open(absl::GetFlag(FLAGS_output_video_path),
                    mediapipe::fourcc('a', 'v', 'c', '1'), // .mp4
                    capture.get(cv::CAP_PROP_FPS), output_frame_mat.size());
        RET_CHECK(writer.isOpened());
      }
      writer.write(output_frame_mat);
    }
    else
    {
      cv::Mat merge;
      hconcat(output_frame_mat,output_frame_mat,merge);
      cv::imshow(kWindowName, merge);
      // Press any key to exit.
      const int pressed_key = cv::waitKey(5);
      if (pressed_key >= 0 && pressed_key != 255)
        grab_frames = false;
    }
  }

  LOG(INFO) << "Shutting down.";
  if (writer.isOpened())
    writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok())
  {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  }
  else
  {
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}
