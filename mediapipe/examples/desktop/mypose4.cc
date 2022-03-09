#include <cstdlib>
#include <iostream>
#include <vector>
#include <map>
#include <librealsense2/rs.hpp>
#include <time.h>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/packet.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/formats/landmark.pb.h"

constexpr char kInputStream0[] = "input_video0";
constexpr char kInputStream1[] = "input_video1";
constexpr char kInputStream2[] = "input_depth0";
constexpr char kInputStream3[] = "input_depth1";
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


void printIntrinsics(rs2_intrinsics intrin){
  std::cout<<"\ncolor intrinsics: ";
  std::cout<<intrin.width<<"  "<<intrin.height<<"  ";
  std::cout<<intrin.ppx<<"  "<<intrin.ppy<<"  ";
  std::cout<<intrin.fx<<"  "<<intrin.fy<<std::endl;
  std::cout<<"coeffs: ";
  for(auto value : intrin.coeffs)
    std::cout<<value<<"  ";
  std::cout<<std::endl;
  std::cout<<"distortion model: "<<(int)intrin.model<<std::endl;///畸变模型 4
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

  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller("output_landmarks"));

  // ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller0,
  //                  graph.AddOutputStreamPoller("pose_landmarks0"));
  // ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller1,
  //                  graph.AddOutputStreamPoller("pose_landmarks1"));
  // ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller2,
  //                  graph.AddOutputStreamPoller("output_landmarks"));

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
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 480, rs2_format::RS2_FORMAT_Z16,30); 
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 640, 480, rs2_format::RS2_FORMAT_RGB8,30); 
    pipe.start(cfg);
    pipelines.emplace_back(pipe);
    // Map from each device's serial number to a different colorizer
    colorizers[serial] = rs2::colorizer();
  }

  int cnt=0;
  int validcnt=0;

  std::cout<<"ready to fetch"<<std::endl;

  rs2::frameset prefs0,prefs1;
  prefs0=pipelines[0].wait_for_frames();
  prefs1=pipelines[1].wait_for_frames();
  rs2::video_frame precolor0=prefs0.get_color_frame();
  rs2::depth_frame predepth0=prefs0.get_depth_frame();
  rs2::video_frame precolor1=prefs1.get_color_frame();
  rs2::depth_frame predepth1=prefs1.get_depth_frame();

  rs2::stream_profile dprofile0 =  predepth0.get_profile();
  rs2::stream_profile cprofile0 =  precolor0.get_profile();
  rs2::stream_profile dprofile1 =  predepth1.get_profile();
  rs2::stream_profile cprofile1 =  precolor1.get_profile();

  std::cout<<"fps:"<<std::endl;
  std::cout<<dprofile0.fps()<<std::endl;
  std::cout<<cprofile0.fps()<<std::endl;
  std::cout<<dprofile1.fps()<<std::endl;
  std::cout<<cprofile1.fps()<<std::endl;

  rs2::video_stream_profile cvsprofile0(cprofile0);
  rs2_intrinsics color_intrin0 =  cvsprofile0.get_intrinsics();
  std::cout<<"for this first camera:"<<std::endl;
  printIntrinsics(color_intrin0);

  std::cout<<"for this second camera:"<<std::endl;
  rs2::video_stream_profile cvsprofile1(cprofile1);
  rs2_intrinsics color_intrin1 =  cvsprofile1.get_intrinsics();
  printIntrinsics(color_intrin1);


  while (cnt<1000){
    cnt++;
    std::cout<<"cnt="<<cnt<<std::endl;
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

    cv::Mat depth0_mat=frame_to_mat(depth0);
    cv::Mat depth1_mat=frame_to_mat(depth1);
    
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


    // auto depth_map0 = absl::make_unique<rs2::depth_frame>(
    //   mediapipe::ImageFormat::GRAY16, 
    //   depth0_mat.cols, 
    //   depth0_mat.rows,
    //   mediapipe::ImageFrame::kDefaultAlignmentBoundary
    // );
    // cv::Mat input_depth_mat0 = mediapipe::formats::MatView(depth_map0.get());
    // depth0_mat.copyTo(input_depth_mat0);

    auto depth_map0 = absl::make_unique<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::GRAY16, 
      depth0_mat.cols, 
      depth0_mat.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary
    );
    cv::Mat input_depth_mat0 = mediapipe::formats::MatView(depth_map0.get());
    depth0_mat.copyTo(input_depth_mat0);


    auto depth_map1 = absl::make_unique<mediapipe::ImageFrame>(
      mediapipe::ImageFormat::GRAY16, 
      depth1_mat.cols, 
      depth1_mat.rows,
      mediapipe::ImageFrame::kDefaultAlignmentBoundary
    );
    cv::Mat input_depth_mat1 = mediapipe::formats::MatView(depth_map1.get());
    depth1_mat.copyTo(input_depth_mat1);


    // Send image packet into the graph.
    size_t frame_timestamp_us = (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

    std::cout<<"already packet the data at time:"<<mediapipe::Timestamp(frame_timestamp_us)<<std::endl;

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


    // mediapipe::Packet depth_map0 = 
    // mediapipe::MakePacket<rs2::depth_frame>(depth0).At(mediapipe::Timestamp(frame_timestamp_us));
    // MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(kInputStream2, depth_map0));

    MP_RETURN_IF_ERROR(
        graph.AddPacketToInputStream(
          kInputStream2, 
          mediapipe::Adopt(depth_map0.release()).At(mediapipe::Timestamp(frame_timestamp_us))
        )
    );

    MP_RETURN_IF_ERROR(
        graph.AddPacketToInputStream(
          kInputStream3, 
          mediapipe::Adopt(depth_map1.release()).At(mediapipe::Timestamp(frame_timestamp_us))
        )
    );
    
    //Get the graph result packet, or stop if that fails.
    // mediapipe::Packet packet0;
    // if (!poller0.Next(&packet0)) break;
    // auto output0_time = packet0.Timestamp();
    // std::cout<<"out0_time:"<<output0_time<<std::endl;

    // mediapipe::Packet packet1;
    // if (!poller1.Next(&packet1)) break;
    // auto output1_time = packet1.Timestamp();
    // std::cout<<"out1_time:"<<output1_time<<std::endl;

    mediapipe::Packet packet;
    if (!poller.Next(&packet)) break;
    auto result_landmarks = packet.Get<mediapipe::NormalizedLandmarkList>();

    //auto result_landmarks = packet.Timestamp();
    //std::cout<<"pose_landmark1_time:"<<result_landmarks<<std::endl;
    //frame_timestamp_us = (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
    //std::cout<<"now time:"<<mediapipe::Timestamp(frame_timestamp_us)<<std::endl;

    const mediapipe::NormalizedLandmark landmark = result_landmarks.landmark(0);
    std::cout << "x:" << landmark.x() << " y:" << landmark.y() << " z:" << landmark.z()<< " visibility:" << landmark.visibility() << std::endl;
    // std::cout<<"show result:"<<std::endl;
    // for (int i = 0; i < result_landmarks.landmark_size(); ++i){
    //   std::cout << "ID:" << i << std::endl;
    //   const mediapipe::NormalizedLandmark landmark = result_landmarks.landmark(i);
    //   std::cout << "x:" << landmark.x() << " y:" << landmark.y() << " z:" << landmark.z()<< " visibility:" << landmark.visibility() << std::endl;
    // }


  }

  LOG(INFO) << "Shutting down.";
  MP_RETURN_IF_ERROR(graph.CloseAllInputStreams());
  return graph.WaitUntilDone();
}

int main(int argc, char **argv){
  google::InitGoogleLogging(argv[0]);
  absl::ParseCommandLine(argc, argv);
  absl::Status run_status = RunMPPGraph();
  if (!run_status.ok()){
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
    return EXIT_FAILURE;
  }
  else{
    LOG(INFO) << "Success!";
  }
  return EXIT_SUCCESS;
}
