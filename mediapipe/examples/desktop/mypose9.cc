#include <cstdlib>
#include <iostream>
#include <vector>
#include <map>
#include <librealsense2/rs.hpp>
#include <time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
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

constexpr char kInputStream0[] = "input_frameset0";
constexpr char kInputStream1[] = "input_frameset1";
constexpr char kInputStream2[] = "input_depth0";
constexpr char kInputStream3[] = "input_depth1";
constexpr char kWindowName[] = "MediaPipe";

ABSL_FLAG(std::string, calculator_graph_config_file, "", "Name of file containing text format CalculatorGraphConfig proto.");
ABSL_FLAG(std::string, input_video_path, "", "Full path of video to load. "
                                             "If not provided, attempt to use a webcam.");
ABSL_FLAG(std::string, output_video_path, "", "Full path of where to save result (.mp4 only). "
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

    //LOG(INFO) << "Get calculator graph config contents: " << calculator_graph_config_contents;

    //std::cout<<"calculator_graph_config_contents:"<<calculator_graph_config_contents<<std::endl;

    mediapipe::CalculatorGraphConfig config = 
        mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(calculator_graph_config_contents);

    //依照解析结果初始化计算图
    LOG(INFO) << "Initialize the calculator graph.";
    mediapipe::CalculatorGraph graph;
    MP_RETURN_IF_ERROR(graph.Initialize(config));

    //ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,graph.AddOutputStreamPoller("output_landmarks"));
    ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller0,graph.AddOutputStreamPoller("output_video0"));
    ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller1,graph.AddOutputStreamPoller("output_video1"));

    //mediapipe::OutputStreamObserver

    cv::namedWindow("pose0",cv::WINDOW_AUTOSIZE);
    cv::namedWindow("pose1",cv::WINDOW_AUTOSIZE);

    MP_RETURN_IF_ERROR(
        graph.ObserveOutputStream(
            "output_video0",
            [&graph](const mediapipe::Packet& packet) -> ::mediapipe::Status {
                std::cout<<"in callback func"<<std::endl;
                auto &output_frame0 = packet.Get<mediapipe::ImageFrame>();
                cv::Mat output_frame_mat0 = mediapipe::formats::MatView(&output_frame0);
                if(!output_frame_mat0.data) std::cout<<"[error] in calllback mat is empty"<<std::endl;
                cv::imshow("pose0", output_frame_mat0);
                cv::waitKey(1);
                std::cout<<"[callback] already show the picture?"<<std::endl;
                return mediapipe::OkStatus();
            }
        )
    );

    MP_RETURN_IF_ERROR(
        graph.ObserveOutputStream(
            "output_video1",
            [&graph](const mediapipe::Packet& packet) -> ::mediapipe::Status {
                std::cout<<"in callback func"<<std::endl;
                auto &output_frame1 = packet.Get<mediapipe::ImageFrame>();
                cv::Mat output_frame_mat1 = mediapipe::formats::MatView(&output_frame1);
                if(!output_frame_mat1.data) std::cout<<"[error] in calllback mat is empty"<<std::endl;
                cv::imshow("pose1", output_frame_mat1);
                cv::waitKey(1);
                std::cout<<"[callback] already show the picture?"<<std::endl;
                return mediapipe::OkStatus();
            }
        )
    );

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
    if (list.size() == 0){
        throw std::runtime_error("No device detected. Is it plugged in?");
    }
    else{
        std::cout << list.size() << std::endl;
    }

    for (auto &&item : list){
        std::string serial = item.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cout << serial << std::endl;
        serials.push_back(serial);
    }

    for (auto &&serial : serials){
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

    int cnt = 0;
    int validcnt = 0;
    std::cout << "ready to fetch" << std::endl;

    while (cnt < 10000){
        cnt++;
        rs2::frameset fs0, fs1;
        fs0 = pipelines[0].wait_for_frames();
        fs1 = pipelines[1].wait_for_frames();

        // Send image packet into the graph.
        size_t frame_timestamp_us = (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
        std::cout << "already packet the data at time:" << mediapipe::Timestamp(frame_timestamp_us) << std::endl;

        mediapipe::Packet frmset0 = mediapipe::MakePacket<rs2::frameset>(fs0).At(mediapipe::Timestamp(frame_timestamp_us));
        mediapipe::Packet frmset1 = mediapipe::MakePacket<rs2::frameset>(fs1).At(mediapipe::Timestamp(frame_timestamp_us));


        MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(kInputStream0, frmset0));
        MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(kInputStream1, frmset1));
        
        std::cout<<"success send in the data"<<std::endl;
        std::cout<<"waiting for output frame"<<std::endl;


        



        /*
        mediapipe::Packet packet0,packet1;
        if (!poller0.Next(&packet0)) break;
        auto &output_frame0 = packet0.Get<mediapipe::ImageFrame>();
        std::cout<<"already got the first output frame"<<std::endl;
        
        if (!poller1.Next(&packet1)) break;
        auto &output_frame1 = packet1.Get<mediapipe::ImageFrame>();
        std::cout<<"already got the second output frame"<<std::endl;


        // Convert back to opencv for display or saving.
        cv::Mat output_frame_mat0 = mediapipe::formats::MatView(&output_frame0);
        //cv::cvtColor(output_frame_mat0, output_frame_mat0, cv::COLOR_RGB2BGR);

        cv::Mat output_frame_mat1 = mediapipe::formats::MatView(&output_frame1);
        //cv::cvtColor(output_frame_mat1, output_frame_mat1, cv::COLOR_RGB2BGR);


        cv::Mat merge;
        hconcat(output_frame_mat0,output_frame_mat1,merge);
        cv::imshow("pose", merge);
        const int pressed_key = cv::waitKey(5);
        if (pressed_key >= 0 && pressed_key != 255) cnt=100000;
        */
        
        
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
