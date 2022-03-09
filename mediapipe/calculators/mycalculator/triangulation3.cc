#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>  
#include <time.h>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <string>
#include <vector>
#include <fstream>

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

    class TriangulationCalculator3 : public CalculatorBase {
        public:
        int gcnt;
        std::vector<std::string> v={
            "nose",
            "left eye (inner)",
            "left eye",
            "left eye (outer)",
            "right eye (inner)",
            "right eye",
            "right eye (outer)",
            "left ear",
            "right ear",
            "mouth (left)",
            "mouth (right)",
            "left shoulder",
            "right shoulder",
            "left elbow",
            "right elbow",
            "left wrist",
            "right wrist",
            "left pinky",
            "right pinky",
            "left index",
            "right index",
            "left thumb",
            "right thumb",
            "left hip",
            "right hip",
            "left knee",
            "right knee",
            "left ankle",
            "right ankle",
            "left heel",
            "right heel",
            "left foot index",
            "right foot index",
        };

        std::chrono::_V2::steady_clock::time_point start,now;
        static absl::Status GetContract(CalculatorContract* cc){
            const int signal_index = cc->Inputs().NumEntries();
            std::cout << " GetContract() " << std::endl;

            cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).Set<NormalizedLandmarkList>();
            cc->Inputs().Get(kNormalizedFilteredLandmarksTag,1).Set<NormalizedLandmarkList>();

            //cc->Inputs().Get(depthMapTag,0).Set<rs2::depth_frame>();
            cc->Inputs().Get(depthMapTag,0).Set<rs2::depth_frame>();
            cc->Inputs().Get(depthMapTag,1).Set<rs2::depth_frame>();

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
            auto& ptrdepth0 = cc->Inputs().Get(depthMapTag,0).Get<rs2::depth_frame>();
            auto& ptrdepth1 = cc->Inputs().Get(depthMapTag,1).Get<rs2::depth_frame>();


            // Get the depth frame's dimensions
            auto width = ptrdepth0.get_width();
            auto height = ptrdepth0.get_height();

            //Query the distance from the camera to the object in the center of the image
            float dist_to_center = ptrdepth0.get_distance(width / 2, height / 2);

            //Print the distance
            std::cout << "The camera is facing an object " << dist_to_center << " meters away"<<std::endl;

            //cv::Mat depth0=formats::MatView(&ptrdepth0);
            //cv::Mat depth1=formats::MatView(&ptrdepth1);
            // std::cout<<"depth map: ("<<depth1.cols<<","<<depth1.rows<<","<<depth1.channels()<<")"<<std::endl;
            // std::cout<<"(320,240):"<<(uint16)depth1.at<uint16>(320,240)<<std::endl;

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


            std::ofstream outfile("/home/cuichenxi/mediapipe/mediapipe/examples/desktop/mypose5/data/fold1/"+
            cc->InputTimestamp().DebugString()+".txt", std::ios::app);

            outfile<<"camera0:"<<std::endl;
            for(int i=0;i<33;i++){
                outfile<<landmarks0.landmark(i).x()<<" "
                <<landmarks0.landmark(i).y()<<" "
                <<landmarks0.landmark(i).visibility()<<std::endl;
                //<<" "<<v[i]<<std::endl;
            }
            outfile<<"camera1:"<<std::endl;
            for(int i=0;i<33;i++){
                outfile<<landmarks1.landmark(i).x()<<" "
                <<landmarks1.landmark(i).y()<<" "
                <<landmarks1.landmark(i).visibility()<<std::endl;
                //<<" "<<v[i]<<std::endl;
            }
            outfile.close();

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

            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp());
            return absl::OkStatus();
        }
    };
    REGISTER_CALCULATOR(TriangulationCalculator3);

}


// # auxiliary key points.
// # 0 - nose
// # 1 - left eye (inner)
// # 2 - left eye
// # 3 - left eye (outer)
// # 4 - right eye (inner)
// # 5 - right eye
// # 6 - right eye (outer)
// # 7 - left ear
// # 8 - right ear
// # 9 - mouth (left)
// # 10 - mouth (right)
// # 11 - left shoulder
// # 12 - right shoulder
// # 13 - left elbow
// # 14 - right elbow
// # 15 - left wrist
// # 16 - right wrist
// # 17 - left pinky
// # 18 - right pinky
// # 19 - left index
// # 20 - right index
// # 21 - left thumb
// # 22 - right thumb
// # 23 - left hip
// # 24 - right hip
// # 25 - left knee
// # 26 - right knee
// # 27 - left ankle
// # 28 - right ankle
// # 29 - left heel
// # 30 - right heel
// # 31 - left foot index
// # 32 - right foot index
