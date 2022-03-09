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
#include <vector>
#include <map>
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

constexpr char kInputStream0[] = "input_video_0";
constexpr char kInputStream1[] = "input_video_1";
constexpr char kWindowName[] = "MediaPipe";

ABSL_FLAG(std::string, calculator_graph_config_file, "","Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "","Full path of video to load. ""If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "","Full path of where to save result (.mp4 only). ""If not provided, show result in a window.");

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

  LOG(INFO) << "Get calculator graph config contents: "<< calculator_graph_config_contents;

  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
        calculator_graph_config_contents
      );

  //依照解析结果初始化计算图
  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  MP_RETURN_IF_ERROR(graph.StartRun({}));

  LOG(INFO) << "Start grabbing and processing frames.";
  bool grab_frames = true;


  //new add
  rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::context ctx;
  std::vector<rs2::pipeline> pipelines;
  std::map<std::string, rs2::colorizer> colorizers;
  std::vector<std::string> serials; //相机id

  auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
  if (list.size() == 0) {
      throw std::runtime_error("No device detected. Is it plugged in?");
  }
  else{
      std::cout<<list.size()<<std::endl;
  }

  for(auto&& item : list){
      std::string serial=item.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout<<serial<<std::endl;
      serials.push_back(serial);
  } 

  for (auto&& serial : serials){
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 480, rs2_format::RS2_FORMAT_Z16); 
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 640, 480, rs2_format::RS2_FORMAT_RGB8); 
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
    // Map from each device's serial number to a different colorizer
    colorizers[serial] = rs2::colorizer();
  }

/*
  int validcnt=0;
  int cnt=0;
  while(cnt<10000){
    cnt++;
    rs2::frameset fs0,fs1;
    fs0=pipelines[0].wait_for_frames();
    fs1=pipelines[1].wait_for_frames();
    std::cout<<cnt<<std::endl;
    
    if (pipelines[0].poll_for_frames(&fs0) && pipelines[1].poll_for_frames(&fs1)){
      std::cout<<validcnt++<<std::endl;
    }
    else{
      std::cout<<"error"<<std::endl;
    }
    

  }
*/



  
  int cnt=0;
  int validcnt=0;

  while (cnt<10000){
    cnt++;
    //std::cout<<cnt<<std::endl;

    /*
    std::vector<rs2::frame> new_frames;
    std::vector<cv::Mat> rgbframe;
    std::vector<cv::Mat> depthframe;
    int flag=0;
    for(auto&& pipe : pipelines){
      rs2::frameset fs;
      if (pipe.poll_for_frames(&fs)){
        rs2::frameset aligned_frames = align_to_color.process(fs);
        rs2::video_frame color=aligned_frames.get_color_frame();
        rs2::depth_frame depth=aligned_frames.get_depth_frame();
        auto serial = rs2::sensor_from_frame(depth)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        auto colorized_depth = colorizers[serial].colorize(depth);

        cv::Mat color2=frame_to_mat(color);
        cv::Mat depth2=frame_to_mat(colorized_depth);

        rgbframe.push_back(color2);
        depthframe.push_back(depth2);
      }
      else{
          std::cout<<"error"<<std::endl;
          flag=1;
          break;
      }
    }
    */


    rs2::frameset fs0,fs1;
    fs0=pipelines[0].wait_for_frames();
    fs1=pipelines[1].wait_for_frames();
    rs2::frameset aligned_frames0 = align_to_color.process(fs0);
    rs2::frameset aligned_frames1 = align_to_color.process(fs1);

    rs2::video_frame color0=aligned_frames0.get_color_frame();
    rs2::depth_frame depth0=aligned_frames0.get_depth_frame();

    rs2::video_frame color1=aligned_frames1.get_color_frame();
    rs2::depth_frame depth1=aligned_frames1.get_depth_frame();

    cv::Mat color0_mat=frame_to_mat(color0);
    cv::Mat color1_mat=frame_to_mat(color1);
    

    // Wrap Mat into an ImageFrame.
    
    /*
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::SRGB, 
      camera_frame.cols, 
      camera_frame.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary
    );
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);
    */


   
    auto input_frame0 = absl::make_unique<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::SRGB, 
      color0_mat.cols, 
      color0_mat.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary
    );
    cv::Mat input_frame_mat0 = mediapipe::formats::MatView(input_frame0.get());
    color0_mat.copyTo(input_frame_mat0);

    auto input_frame1 = absl::make_unique<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::SRGB, 
      color1_mat.cols, 
      color1_mat.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary
    );
    cv::Mat input_frame_mat1 = mediapipe::formats::MatView(input_frame1.get());
    color1_mat.copyTo(input_frame_mat1);

    // Send image packet into the graph.
    size_t frame_timestamp_us = (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

    MP_RETURN_IF_ERROR(
        graph.AddPacketToInputStream(
          kInputStream0, 
          mediapipe::Adopt(input_frame0.release()).At(mediapipe::Timestamp(frame_timestamp_us))
        )
    );

    MP_RETURN_IF_ERROR(
        graph.AddPacketToInputStream(
          kInputStream1, 
          mediapipe::Adopt(input_frame1.release()).At(mediapipe::Timestamp(frame_timestamp_us))
        )
    );

    //std::cout<<validcnt++<<std::endl;

  }

  LOG(INFO) << "Shutting down.";
  MP_RETURN_IF_ERROR(graph.CloseAllInputStreams());
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
