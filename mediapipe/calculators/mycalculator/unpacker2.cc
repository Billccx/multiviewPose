#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>  
#include <time.h>
#include <chrono>
#include <string>
#include <librealsense2/rs.hpp>

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/timestamp.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/packet.h"

namespace mediapipe{

    constexpr char framesetTag[] = "FRAMESET";
    constexpr char rgbTag[] = "IMAGE";
    constexpr char depthTag[] ="DEPTH";



    class UnpackerCalculator2 : public CalculatorBase {
        public:
        int unpackercnt;
        rs2::align align_to_color;
        std::chrono::_V2::steady_clock::time_point start,now;

        UnpackerCalculator2():align_to_color(RS2_STREAM_COLOR){
            unpackercnt=0;
        }

        static absl::Status GetContract(CalculatorContract* cc){
            cc->Inputs().Tag(framesetTag).Set<rs2::frameset>();
            cc->Outputs().Tag(rgbTag).Set<ImageFrame>();
            cc->Outputs().Tag(depthTag).Set<rs2::depth_frame>();
            return absl::OkStatus();
        }

        absl::Status Open(CalculatorContext* cc) override{
            return absl::OkStatus();
        }

        cv::Mat frame_to_mat(const rs2::frame& f){
            auto vf = f.as<rs2::video_frame>();
            const int w = vf.get_width();
            const int h = vf.get_height();

            if (f.get_profile().format() == RS2_FORMAT_BGR8){
                return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
            }
            else if (f.get_profile().format() == RS2_FORMAT_RGB8){
                auto r_rgb = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat r_bgr;
                cvtColor(r_rgb, r_bgr, cv::COLOR_RGB2BGR);
                return r_bgr;
            }
            else if (f.get_profile().format() == RS2_FORMAT_Z16){
                return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
            }
            else if (f.get_profile().format() == RS2_FORMAT_Y8){
                return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
            }
            else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32){
                return cv::Mat(cv::Size(w, h), CV_32FC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
            }

            throw std::runtime_error("Frame format is not supported yet!");
        }


        absl::Status Process(CalculatorContext* cc) override{
            rs2::frameset frmset=cc->Inputs().Tag(framesetTag).Get<rs2::frameset>();
            rs2::frameset aligned_frames = align_to_color.process(frmset);
            rs2::video_frame color=aligned_frames.get_color_frame();
            rs2::depth_frame depth=aligned_frames.get_depth_frame();


            rs2::stream_profile profile=color.get_profile();
            rs2::video_stream_profile cvsprofile0(profile);

            cv::Mat color_mat=frame_to_mat(color);
            
            /*
            cv::imwrite(
                "/home/cuichenxi/mediapipe/mediapipe/examples/desktop/mypose7/pic/"
                +cc->InputTimestamp().DebugString()+"_"+std::to_string(cvsprofile0.get_intrinsics().coeffs[3])
                +".jpg",
                color_mat
            );
            */
            

            auto output_rgb = absl::make_unique<mediapipe::ImageFrame>(
                mediapipe::ImageFormat::SRGB, 
                color_mat.cols, 
                color_mat.rows,
                mediapipe::ImageFrame::kDefaultAlignmentBoundary
            );
            cv::Mat output_frame_mat = mediapipe::formats::MatView(output_rgb.get());
            color_mat.copyTo(output_frame_mat);

            auto output_depth = std::make_unique<rs2::depth_frame>(depth);
            
            cc->Outputs().Tag(rgbTag).Add(output_rgb.release(), cc->InputTimestamp());
            cc->Outputs().Tag(depthTag).Add(output_depth.release(),cc->InputTimestamp());
            return absl::OkStatus();
        }
    };
    REGISTER_CALCULATOR(UnpackerCalculator2);

}