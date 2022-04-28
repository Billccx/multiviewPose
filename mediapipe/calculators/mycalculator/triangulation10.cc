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
#include <cmath>
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


namespace mediapipe{

    constexpr char kNormalizedFilteredLandmarksTag[] = "NORM_FILTERED_LANDMARKS";
    constexpr char kfusedNormalizedFilteredLandmarksTag[] = "FUSED_NORM_FILTERED_LANDMARKS";
    constexpr char depthMapTag[] ="DEPTH_MAT";

    class TriangulationCalculator10 : public CalculatorBase {
        public:
        int gcnt;
        int udp_socket; 
        const int port;
        char buffer[2048];
        struct sockaddr_in servaddr;
        Eigen::Matrix<double,3,3> poseR0,poseR1,poseR1_;
        Eigen::Matrix<double,3,1> poset0,poset1,poset1_;
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

        float points3d_gt[33][3];
        float points3d1_gt[33][3];
        float points2d0_gt[33][2];
        float points2d1_gt[33][2];
        float depth_gt[33];
        float depth1_gt[33];
        double vis0_gt[33];
        double vis1_gt[33];
        float error[33];

        std::chrono::_V2::steady_clock::time_point start,now;

        rs2_intrinsics intrin0,intrin1;
        
        TriangulationCalculator10():gcnt(0),port(31201){
            intrin0.width=640;
            intrin0.height=480;
            intrin0.ppx=331.948;
            intrin0.ppy=248.697;
            intrin0.fx=600.764;
            intrin0.fy=600.986;
            intrin0.model=RS2_DISTORTION_BROWN_CONRADY;
            intrin0.coeffs[0]=0.171335;
            intrin0.coeffs[1]=-0.528704;
            intrin0.coeffs[2]=-0.00208366;
            intrin0.coeffs[3]=2.20354e-05;
            intrin0.coeffs[4]=0.495262;


            intrin1.width=640;
            intrin1.height=480;
            intrin1.ppx=333.593;
            intrin1.ppy=244.954;
            intrin1.fx=601.325;
            intrin1.fy=601.453;
            intrin1.model=RS2_DISTORTION_BROWN_CONRADY;
            intrin1.coeffs[0]=0.157447;
            intrin1.coeffs[1]=-0.49804;
            intrin1.coeffs[2]=-0.00211366;
            intrin1.coeffs[3]=-0.00051181;
            intrin1.coeffs[4]=0.47009;


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

            std::ifstream parameter("/home/cuichenxi/code/Qt/qmakeProj/extrinsics/RT.txt",std::ios::in);
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

            parameter.close();

            std::ifstream parameter_("/home/cuichenxi/code/Qt/qmakeProj/extrinsics/RT2.txt",std::ios::in);
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    double x;
                    parameter_>>x;
                    poseR1_(i,j)=x;
                }
            }

            for(int i=0;i<3;i++){
                double x;
                parameter_>>x;
                poset1_(i,0)=x;
            }

            parameter_.close();

            std::cout<<"already load the reverse camera extrinsic\n";
            std::cout<<"rotation:\n"<<poseR1_<<std::endl;
            std::cout<<"trans:\n"<<poset1_<<std::endl;
        }

        ~TriangulationCalculator10(){
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
            return absl::OkStatus();
        }

        absl::Status Process(CalculatorContext* cc) override{
            memset(points3d_gt,0,sizeof(points3d_gt));
            memset(points3d1_gt,0,sizeof(points3d1_gt));
            memset(points2d0_gt,0,sizeof(points2d0_gt));
            memset(points2d1_gt,0,sizeof(points2d1_gt));
            memset(vis0_gt,0,sizeof(vis0_gt));
            memset(vis1_gt,0,sizeof(vis1_gt));
            memset(depth_gt,0,sizeof(depth_gt));
            memset(depth1_gt,0,sizeof(depth1_gt));
            memset(error,0,sizeof(error));

            if(gcnt==0){
                start=std::chrono::steady_clock::now();
            }
            else{
                start=now;
            }

            std::vector< Eigen::Vector2d > points2d0,points2d1;
	        std::vector<double> vis0,vis1;

            if (cc->Inputs().Get(kNormalizedFilteredLandmarksTag,0).IsEmpty()) {
                auto fused_landmarks = absl::make_unique<NormalizedLandmarkList>();
                cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp());
                std::cout<<"[triangulation calculator error]: input landmarks0 is empty!"<<std::endl;
                return absl::OkStatus();
            }

            if (cc->Inputs().Get(kNormalizedFilteredLandmarksTag,1).IsEmpty()) {
                auto fused_landmarks = absl::make_unique<NormalizedLandmarkList>();
                cc->Outputs().Tag(kfusedNormalizedFilteredLandmarksTag).Add(fused_landmarks.release(), cc->InputTimestamp());
                std::cout<<"[triangulation calculator error]: input landmarks1 is empty!"<<std::endl;
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
                points2d0_gt[i][0]=(float)(x0*640);
                points2d0_gt[i][1]=(float)(y0*480);
                vis0_gt[i]=v0;
                if(points2d0_gt[i][0]>=0 && points2d0_gt[i][0]<=640 && points2d0_gt[i][1]>=0 && points2d0_gt[i][1]<=480){
                    depth_gt[i] = ptrdepth0.get_distance(points2d0_gt[i][0],points2d0_gt[i][1]);
                    rs2_deproject_pixel_to_point(points3d_gt[i],&intrin0,points2d0_gt[i],depth_gt[i]);
                }

                x1=landmarks1.landmark(i).x();
                y1=landmarks1.landmark(i).y();
                v1=landmarks1.landmark(i).visibility();
                points2d1_gt[i][0]=(float)(x1*640);
                points2d1_gt[i][1]=(float)(y1*480);
                vis1_gt[i]=v1;
                if(points2d1_gt[i][0]>=0 && points2d1_gt[i][0]<=640 && points2d1_gt[i][1]>=0 && points2d1_gt[i][1]<=480){
                    depth1_gt[i] = ptrdepth1.get_distance(points2d1_gt[i][0],points2d1_gt[i][1]);
                    rs2_deproject_pixel_to_point(points3d1_gt[i],&intrin1,points2d1_gt[i],depth1_gt[i]);
                }

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
            
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;


            auto fused_landmarks = absl::make_unique<NormalizedLandmarkList>();
            for(int i=0;i<33;i++){
                auto* fused_landmark = fused_landmarks->add_landmark();
                fused_landmark->set_x(points3d[i][0]);
                fused_landmark->set_y(points3d[i][1]);
                fused_landmark->set_z(points3d[i][2]);
                fused_landmark->set_visibility(std::max(vis0[i],vis1[i]));
            }

        
            std::ofstream outfile("/home/cuichenxi/mediapipe/mediapipe/examples/desktop/mypose10/data3/"+
            cc->InputTimestamp().DebugString()+".txt", std::ios::app);

            outfile<<"3d:"<<std::endl;
            for(int i=0;i<33;i++){
                double x,y,z;
                x=fused_landmarks->landmark(i).x();
                y=fused_landmarks->landmark(i).y();
                z=fused_landmarks->landmark(i).z();

                if(vis0_gt[i]>0.95){
                    error[i]=sqrt(
                                (x-points3d_gt[i][0])*(x-points3d_gt[i][0])+
                                (y-points3d_gt[i][1])*(y-points3d_gt[i][1])+
                                (z-points3d_gt[i][2])*(z-points3d_gt[i][2])
                            );
                }

                outfile<<v[i]<<"\nGT "<<
                points3d_gt[i][0]<<' '<<
                points3d_gt[i][1]<<' '<<
                points3d_gt[i][2]<<" predict "<<
                x<<' '<<
                y<<' '<<
                z<<' '<<" vis "<<vis0_gt[i]<<
                " error "<<error[i]<<std::endl;
            }


            outfile<<"\n3d1:"<<std::endl;
            for(int i=0;i<33;i++){
                double x_,y_,z_;
                x_=points3d1_gt[i][0];
                y_=points3d1_gt[i][1];
                z_=points3d1_gt[i][2];
                Eigen::Matrix<double, 3, 1> pt_(x_,y_,z_);
                Eigen::Matrix<double, 2, 1> rpcoordinate1 = (poseR1_ * pt_ + poset1_).hnormalized();
                double x1_=rpcoordinate1[0];
                double y1_=rpcoordinate1[1];

                double r2_=x1_*x1_+y1_*y1_;
                double f_=1.0+coeffs0[0]*r2_+coeffs0[1]*r2_*r2_+coeffs0[4]*r2_*r2_*r2_;
                double xf_=x1_*f_;
                double yf_=y1_*f_;
                double dx=xf_+2.0*coeffs0[2]*x1_*y1_+coeffs0[3]*(r2_+2.0*x1_*x1_);
                double dy=yf_+2.0*coeffs0[3]*x1_*y1_+coeffs0[2]*(r2_+2.0*x1_*x1_);
                double rpx0_ = dx * coeffs0[5] + coeffs0[7];
                double rpy0_ = dy * coeffs0[6] + coeffs0[8];


                double rpx0_w = x1_ * coeffs0[5] + coeffs0[7];
                double rpy0_w = y1_ * coeffs0[6] + coeffs0[8];


                double x,y,z;
                x=fused_landmarks->landmark(i).x();
                y=fused_landmarks->landmark(i).y();
                z=fused_landmarks->landmark(i).z();
                Eigen::Matrix<double, 3, 1> pt(x,y,z);

                Eigen::Matrix<double, 2, 1> norm_camera_coordinate0 = (poseR0 * pt + poset0).hnormalized();
                double x0=norm_camera_coordinate0[0];
                double y0=norm_camera_coordinate0[1];
                double rpx0 = x0 * coeffs0[5] + coeffs0[7];
                double rpy0 = y0 * coeffs0[6] + coeffs0[8];

                double rx0=landmarks0.landmark(i).x()*640;
                double ry0=landmarks0.landmark(i).y()*480;

                outfile<<v[i]<<"\n2dPointinIMG1 "<<points2d1_gt[i][0]<<' '<<points2d1_gt[i][1]
                <<" vis "<<vis1_gt[i]
                <<" ChangeToIMG0 "<<rpx0_<<' '<<rpy0_
                <<" ReProjrct "<<rpx0<<' '<<rpy0
                <<" 2dPointinIMG0 "<<rx0<<' '<<ry0
                <<" withoutDisTort "<<rpx0_w<<' '<<rpy0_w <<std::endl;
            }


            outfile<<"\n2d:"<<std::endl;
            for(int i=0;i<33;i++){
                double x,y,z;
                x=fused_landmarks->landmark(i).x();
                y=fused_landmarks->landmark(i).y();
                z=fused_landmarks->landmark(i).z();
                Eigen::Matrix<double, 3, 1> pt(x,y,z);

                Eigen::Matrix<double, 2, 1> norm_camera_coordinate0 = (poseR0 * pt + poset0).hnormalized();
                double x0=norm_camera_coordinate0[0];
                double y0=norm_camera_coordinate0[1];
                double rpx0 = x0 * coeffs0[5] + coeffs0[7];
                double rpy0 = y0 * coeffs0[6] + coeffs0[8];

                Eigen::Matrix<double, 2, 1> norm_camera_coordinate1 = (poseR1 * pt + poset1).hnormalized();
                double x1=norm_camera_coordinate1[0];
                double y1=norm_camera_coordinate1[1];
                double rpx1 = x1 * coeffs1[5] + coeffs1[7];
                double rpy1 = y1 * coeffs1[6] + coeffs1[8];

                double rx0=landmarks0.landmark(i).x()*640;
                double ry0=landmarks0.landmark(i).y()*480;

                double rx1=landmarks1.landmark(i).x()*640;
                double ry1=landmarks1.landmark(i).y()*480;

                double e0,e1;
                e0=sqrt((rx0-rpx0)*(rx0-rpx0)+(ry0-rpy0)*(ry0-rpy0));
                e1=sqrt((rx1-rpx1)*(rx1-rpx1)+(ry1-rpy1)*(ry1-rpy1));

                outfile<<v[i]<<std::endl;
                outfile<<"real0:("<<rx0<<","<<ry0<<") cal0:("<<rpx0<<","<<rpy0<<") error:"<<e0<<std::endl;
                outfile<<"real1:("<<rx1<<","<<ry1<<") cal1:("<<rpx1<<","<<rpy1<<") error:"<<e1<<std::endl;
                outfile<<std::endl;
            }

            outfile.close();
        
            
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
    REGISTER_CALCULATOR(TriangulationCalculator10);

}
