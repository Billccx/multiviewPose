#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>  
#include <time.h>
#include <chrono>
#include <librealsense2/rs.hpp>

//#include "absl/algorithm/container.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/timestamp.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
// #include "mediapipe/framework/port/opencv_highgui_inc.h"
// #include "mediapipe/framework/port/opencv_imgproc_inc.h"
// #include "mediapipe/framework/port/opencv_video_inc.h"

namespace mediapipe{

    constexpr char kNormalizedFilteredLandmarksTag[] = "NORM_FILTERED_LANDMARKS";
    constexpr char kfusedNormalizedFilteredLandmarksTag[] = "FUSED_NORM_FILTERED_LANDMARKS";
    constexpr char depthMapTag[] ="DEPTH_MAT";

    class TriangulationCalculator2 : public CalculatorBase {
        public:
        int gcnt;
        std::chrono::_V2::steady_clock::time_point start,now;
        static absl::Status GetContract(CalculatorContract* cc){
            const int signal_index = cc->Inputs().NumEntries();
            std::cout << " GetContract() " << std::endl;

            cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).Set<NormalizedLandmarkList>();
            cc->Inputs().Get(kNormalizedFilteredLandmarksTag,1).Set<NormalizedLandmarkList>();

            //cc->Inputs().Get(depthMapTag,0).Set<rs2::depth_frame>();
            cc->Inputs().Get(depthMapTag,0).Set<ImageFrame>();
            cc->Inputs().Get(depthMapTag,1).Set<ImageFrame>();

            //cc->Outputs().Get(kfusedNormalizedFilteredLandmarksTag,0).Set<NormalizedLandmarkList>();
            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Set<NormalizedLandmarkList>();
            return absl::OkStatus();
        }
        absl::Status Open(CalculatorContext* cc) override{
            gcnt=0;
            //cc->SetOffset(TimestampDiff(0));
            return absl::OkStatus();
        }
        absl::Status Process(CalculatorContext* cc) override{
            if(gcnt==0){
                start=std::chrono::steady_clock::now();
            }
            else{
                start=now;
            }
            std::cout<<"in triangulation, cnt="<<gcnt++<<std::endl;
            auto& landmarks0 = cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).Get<NormalizedLandmarkList>();
            auto& landmarks1 = cc->Inputs().Get(kNormalizedFilteredLandmarksTag,1).Get<NormalizedLandmarkList>();


            //auto& ptrdepth0 = cc->Inputs().Get(depthMapTag,0).Get<rs2::depth_frame>();
            auto& ptrdepth0 = cc->Inputs().Get(depthMapTag,0).Get<ImageFrame>();
            auto& ptrdepth1 = cc->Inputs().Get(depthMapTag,1).Get<ImageFrame>();


            // Get the depth frame's dimensions
            // auto width = ptrdepth0.get_width();
            // auto height = ptrdepth0.get_height();

            // // Query the distance from the camera to the object in the center of the image
            // float dist_to_center = ptrdepth0.get_distance(width / 2, height / 2);

            // // Print the distance
            // std::cout << "The camera is facing an object " << dist_to_center << " meters away"<<std::endl;



            //cv::Mat depth0=formats::MatView(&ptrdepth0);
            cv::Mat depth1=formats::MatView(&ptrdepth1);

            std::cout<<"depth map: ("<<depth1.cols<<","<<depth1.rows<<","<<depth1.channels()<<")"<<std::endl;
            std::cout<<"(320,240):"<<(uint16)depth1.at<uint16>(320,240)<<std::endl;


            int landmarks0_size,landmarks1_size,landmark_sz;
            landmarks0_size=landmarks0.landmark_size();
            landmarks1_size=landmarks1.landmark_size();
            landmark_sz=std::min(landmarks0_size,landmarks1_size);
            std::cout<<"received "<<landmarks0_size<<" points from position 0, "
            <<landmarks1_size<<"  points from position 1"<<std::endl;

            auto fused_landmarks = absl::make_unique<NormalizedLandmarkList>();
            for(int i=0;i<landmark_sz;i++){
                auto* fused_landmark = fused_landmarks->add_landmark();
                fused_landmark->set_x((landmarks0.landmark(i).x()+landmarks1.landmark(i).x())/2.0f);
                fused_landmark->set_y((landmarks0.landmark(i).y()+landmarks1.landmark(i).y())/2.0f);
                fused_landmark->set_z((landmarks0.landmark(i).z()+landmarks1.landmark(i).z())/2.0f);
                fused_landmark->set_visibility(landmarks0.landmark(i).visibility());
                fused_landmark->set_presence(landmarks0.landmark(i).presence());
            }

            now=std::chrono::steady_clock::now();
            std::cout << "fps:"<<
            1000000/(double)(std::chrono::duration_cast<std::chrono::microseconds>(now - start).count()) 
            << std::endl;

            //std::cout<<"fps:"<<1/((double)(now-start)/CLOCKS_PER_SEC)<<std::endl;
            for (int i = 0; i < 0; i++){
                std::cout << "ID:" << i << std::endl;
                mediapipe::NormalizedLandmark landmark = landmarks0.landmark(i);
                std::cout << "position0 - x:" << landmark.x() << 
                            " y:" << landmark.y() << 
                            " z:" << landmark.z() << 
                            " visibility:" << landmark.visibility() << std::endl;

                landmark = landmarks1.landmark(i);
                std::cout << "position1 -x:" << landmark.x() << 
                            " y:" << landmark.y() << 
                            " z:" << landmark.z() << 
                            " visibility:" << landmark.visibility() << std::endl;

                landmark = fused_landmarks->landmark(i);
                std::cout << "fused -x:" << landmark.x() << 
                            " y:" << landmark.y() << 
                            " z:" << landmark.z() << 
                            " visibility:" << landmark.visibility() << std::endl;
            }

            //std::cout<<"_________________________________________________"<<std::endl;
        /*
            size_t frame_timestamp_us = (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
            Packet pOut = MakePacket<double>(1).At(cc->InputTimestamp());
            cc->Outputs().Tag("SIGNAL").AddPacket(pOut);

            std::cout<<"in triangulation timestamp:"<<cc->InputTimestamp()<<std::endl;
            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).AddPacket(cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).Value());
            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp().NextAllowedInStream());
            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp());
            std::cout<<"turn finished"<<std::endl;
        */
            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp());
            return absl::OkStatus();
        }
    };
    REGISTER_CALCULATOR(TriangulationCalculator2);

}
