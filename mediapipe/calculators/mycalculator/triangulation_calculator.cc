#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>  
#include <time.h>
#include <chrono>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <string>
#include <vector>
#include <fstream>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "reProjectionError2.hpp"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/timestamp.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/calculators/mycalculator/triangulation_calculator.pb.h"


namespace mediapipe{

    constexpr char kNormalizedFilteredLandmarksTag[] = "NORM_FILTERED_LANDMARKS";
    constexpr char kfusedNormalizedFilteredLandmarksTag[] = "FUSED_NORM_FILTERED_LANDMARKS";
    constexpr char depthMapTag[] ="DEPTH_MAT";

    class TriangulationCalculator8 : public CalculatorBase {
        public:
        int gcnt;
        int udp_socket; 
        const int port;
        char buffer[2048];
        struct sockaddr_in servaddr;
        Eigen::Matrix<double,3,3> poseR0,poseR1;
        Eigen::Matrix<double,3,1> poset0,poset1;
        const std::vector<double> coeffs0={0.171335,-0.528704,-0.00208366,2.20354e-05,0.495262,600.764,600.986,331.948,248.697};
        const std::vector<double> coeffs1={0.157447,-0.49804 ,-0.00211366,-0.00051181,0.47009 ,601.325,601.453,333.593,244.954};
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

        TriangulationCalculator8():gcnt(0),port(31201){
            if ( (udp_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
                perror("socket creation failed");
                exit(EXIT_FAILURE);
            }

            memset(buffer,0,sizeof(buffer));
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_port = htons(port);
            //servaddr.sin_addr.s_addr = INADDR_ANY;inet_addr("127.101.100.111");
            servaddr.sin_addr.s_addr = inet_addr("192.168.137.1");//"192.168.137.1";

            poseR0<< 1,0,0,
                     0,1,0,
                     0,0,1;
            poset0<< 0,0,0;

            std::ifstream parameter("/home/cuichenxi/下载/第三方库/apriltag-master/test/result/RT.txt",std::ios::in);
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    double x;
                    parameter>>x;
                    poseR1(i,j)=x;
                }
            }

            for(int i=0;i<3;i++){
                double x;
                parameter>>x;
                poset1(i,0)=x;
            }

            std::cout<<"already load the camera extrinsic\n";
            std::cout<<"rotation:\n"<<poseR1<<std::endl;
            std::cout<<"trans:\n"<<poset1<<std::endl;
        }


        ~TriangulationCalculator8(){
            close(udp_socket);
        }


        static absl::Status GetContract(CalculatorContract* cc){
            const int signal_index = cc->Inputs().NumEntries();
            std::cout << " GetContract() " << std::endl;

            cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).Set<NormalizedLandmarkList>();
            cc->Inputs().Get(kNormalizedFilteredLandmarksTag,1).Set<NormalizedLandmarkList>();

            cc->Inputs().Get(depthMapTag,0).Set<rs2::depth_frame>();
            cc->Inputs().Get(depthMapTag,1).Set<rs2::depth_frame>();

            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Set<NormalizedLandmarkList>();
            return absl::OkStatus();
        }

        absl::Status Open(CalculatorContext* cc) override{
            const auto& options = cc->Options<::mediapipe::TriangulationCalculatorOptions>();
            auto var = options.parameterfile();
            std::cout<<var<<std::endl;
            return absl::OkStatus();
        }

        absl::Status Process(CalculatorContext* cc) override{
            if(gcnt==0){
                start=std::chrono::steady_clock::now();
            }
            else{
                start=now;
            }

            std::vector< Eigen::Vector2d > points2d0,points2d1;
	        std::vector<double> vis0,vis1;

            if (cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).IsEmpty() || cc->Inputs().Get(kNormalizedFilteredLandmarksTag,1).IsEmpty()) {
                auto fused_landmarks = absl::make_unique<NormalizedLandmarkList>();
                cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp());
                return absl::OkStatus();
            }


            std::cout<<"in triangulation, cnt="<<gcnt++<<std::endl;
            auto& landmarks0 = cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).Get<NormalizedLandmarkList>();
            auto& landmarks1 = cc->Inputs().Get(kNormalizedFilteredLandmarksTag,1).Get<NormalizedLandmarkList>();

            auto& ptrdepth0 = cc->Inputs().Get(depthMapTag,0).Get<rs2::depth_frame>();
            auto& ptrdepth1 = cc->Inputs().Get(depthMapTag,1).Get<rs2::depth_frame>();


            int landmarks0_size,landmarks1_size,landmark_sz;
            landmarks0_size=landmarks0.landmark_size();
            landmarks1_size=landmarks1.landmark_size();
            landmark_sz=std::min(landmarks0_size,landmarks1_size);

            for(int i=0;i<landmark_sz;i++){
                double x0,y0,v0,x1,y1,v1;
                x0=landmarks0.landmark(i).x();
                y0=landmarks0.landmark(i).y();
                v0=landmarks0.landmark(i).visibility();

                x1=landmarks1.landmark(i).x();
                y1=landmarks1.landmark(i).y();
                v1=landmarks1.landmark(i).visibility();

                points2d0.push_back(Eigen::Vector2d(x0*640,y0*480));
                points2d1.push_back(Eigen::Vector2d(x1*640,y1*480));
                vis0.push_back(v0);
                vis1.push_back(v1); 
            }

            std::vector<std::vector<double>> points3d;
            for(int i=0;i<33;i++){
                std::vector<double> x={0.0,0.0,0.8};
                points3d.push_back(x);
            }

            ceres::Problem problem;

            for(int i=0;i<33;i++){
                //ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
                ceres::CostFunction *cost_function0;
                cost_function0 = ReprojectionError::Create(poseR0,poset0,coeffs0,points2d0[i],vis0[i]);
                ceres::CostFunction *cost_function1;
                cost_function1 = ReprojectionError::Create(poseR1,poset1,coeffs1,points2d1[i],vis1[i]);

                problem.AddResidualBlock(
                    cost_function0,  //损失函数
                    nullptr,   //核函数
                    &points3d[i][0]         //待优化变量
                );

                problem.AddResidualBlock(
                    cost_function1,  //损失函数
                    nullptr,   //核函数
                    &points3d[i][0]         //待优化变量
                );
            }


            ceres::Solver::Options options;
            options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
            options.minimizer_progress_to_stdout = true;
            //options.max_num_iterations=300;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;


            std::cout<<"points3d: "<<points3d[0][0]<<','<<points3d[0][1]<<','<<points3d[0][2]<<std::endl;


            auto fused_landmarks = absl::make_unique<NormalizedLandmarkList>();
            for(int i=0;i<33;i++){
                auto* fused_landmark = fused_landmarks->add_landmark();
                fused_landmark->set_x(points3d[i][0]);
                fused_landmark->set_y(points3d[i][1]);
                fused_landmark->set_z(points3d[i][2]);
                fused_landmark->set_visibility(std::max(vis0[i],vis1[i]));
            }

            memset(buffer,0,sizeof(buffer));
            int p=0;
            for(int i=0;i<33;i++){
                p+=sprintf(buffer+p,"%.5f ",fused_landmarks->landmark(i).x());
                p+=sprintf(buffer+p,"%.5f ",fused_landmarks->landmark(i).z());
                p+=sprintf(buffer+p,"%.5f ",-fused_landmarks->landmark(i).y());
            }

            sendto(udp_socket, (const char *)buffer, strlen(buffer),MSG_CONFIRM, 
            (const struct sockaddr *) &servaddr, sizeof(servaddr));

            now=std::chrono::steady_clock::now();
            
            cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp());
            std::cout << "fps:"<<
            1000000/(double)(std::chrono::duration_cast<std::chrono::microseconds>(now - start).count()) 
            << std::endl;

            return absl::OkStatus();
        }
    };
    REGISTER_CALCULATOR(TriangulationCalculator8);

}
