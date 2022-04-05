#include <iostream>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <typeinfo>
#include <fstream>
#include "projectionError.hpp"
#include "projectionError2.hpp"


// bazel build mediapipe/examples/desktop/ceres_test:ceres_ba
// ./bazel-bin/mediapipe/examples/desktop/ceres_test/ceres_ba

cv::Mat angleToRotation(double roll,double pitch,double yaw){
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
	R.at<float>(0, 0) = cos(yaw)*cos(pitch);
	R.at<float>(0, 1) = cos(yaw)*sin(roll)*sin(pitch) - cos(roll)*sin(yaw);
	R.at<float>(0, 2) = sin(yaw)*sin(roll) + cos(yaw)*cos(roll)*sin(pitch);
	R.at<float>(1, 0) = cos(pitch)*sin(yaw);
	R.at<float>(1, 1) = cos(yaw)*cos(roll) + sin(yaw)*sin(roll)*sin(pitch);
	R.at<float>(1, 2) = cos(roll)*sin(yaw)*sin(pitch) - cos(yaw)*sin(roll);
	R.at<float>(2, 0) = - sin(pitch);
	R.at<float>(2, 1) = cos(pitch)*sin(roll);
	R.at<float>(2, 2) = cos(roll)*cos(pitch);
	return R;
}

int main(int argc, char **argv){
	int mode = atoi(argv[1]);
	std::cout<<mode<<std::endl;
	std::ifstream srcFile("/home/cuichenxi/mediapipe/mediapipe/examples/desktop/ceres_test/dt/d.txt",std::ios::in);
	if(!srcFile) { //打开失败
        std::cout << "error opening source file." << std::endl;
        return 0;
    }
	std::string temp;
	//std::vector< std::vector<double> > points0(33,std::vector<double>(3));
	//std::vector< std::vector<double> > points1(33,std::vector<double>(3));

	std::vector< Eigen::Vector2d > points2d0;
	std::vector< Eigen::Vector2d > points2d1;

	std::vector<cv::Point2f> keypoints_0;
  	std::vector<cv::Point2f> keypoints_1;

	std::vector<double> vis0;
	std::vector<double> vis1;

	srcFile>>temp;
	for(int i=0;i<13;i++){
		double x,y,vis,depth;
		srcFile>>x;
		srcFile>>y;
		srcFile>>vis;
		if(mode!=0) srcFile>>depth;
		//Eigen::Vector2d point2d;
		//point2d<<x<<y;
		points2d0.push_back(Eigen::Vector2d(x*640,y*480));
		keypoints_0.push_back(cv::Point2f(x*640,y*480));
		vis0.push_back(vis);
		std::cout<<x<<','<<y<<std::endl;
	}

	srcFile>>temp;
	for(int i=0;i<13;i++){
		double x,y,vis,depth;
		srcFile>>x;
		srcFile>>y;
		srcFile>>vis;
		if(mode!=0) srcFile>>depth;
		//Eigen::Vector2d point2d;
		//point2d<<x<<y;
		points2d1.push_back(Eigen::Vector2d(x*640,y*480));
		keypoints_1.push_back(cv::Point2f(x*640,y*480));
		vis1.push_back(vis);
		std::cout<<x<<','<<y<<std::endl;
	}

	Eigen::Matrix<double,3,4> pose0,pose1;
	Eigen::Matrix<double,3,3> poseR0,poseR1;
	Eigen::Matrix<double,3,1> poset0,poset1;

	pose0<< 1,0,0,0,
			0,1,0,0,
			0,0,1,0;

	pose1<< 0.9923897618791255, 0.00800700532924443, 0.1228757436728235,-0.6437709079221844,
			0.01133709378219977, 0.9877038785817202, -0.1559247207443367,-0.08254567402321357,
			-0.122613338679222, 0.1561311503201186, 0.9800964407024703,0.03528781394617282;

	poseR0<< 1,0,0,
			0,1,0,
			0,0,1;
	poset0<< 0,0,0;

	poseR1<< 0.9923897618791255, 0.00800700532924443, 0.1228757436728235,
			0.01133709378219977, 0.9877038785817202, -0.1559247207443367,
			-0.122613338679222, 0.1561311503201186, 0.9800964407024703;
	poset1<< -0.6437709079221844,-0.08254567402321357,0.03528781394617282;


	cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0
	);


	cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
		0.9923897618791255, 0.00800700532924443, 0.1228757436728235,-0.6437709079221844,
		0.01133709378219977, 0.9877038785817202, -0.1559247207443367,-0.08254567402321357,
		-0.122613338679222, 0.1561311503201186, 0.9800964407024703,0.03528781394617282
	);

	const std::vector<double> coeffs0={
									0.171335,
									-0.528704,
									-0.00208366,
									2.20354e-05,
									0.495262,
									600.764, //fx
									600.986, //fy
									331.948, //ppx
									248.697  //ppy	
								};


	const std::vector<double> coeffs1={
									0.157447,
									-0.49804,
									-0.00211366,
									-0.00051181,
									0.47009,
									601.325, //fx
									601.453, //fy
									333.593, //ppx
									244.954  //ppy	
								};

	//std::cout<<angleToRotation(0,0,2*3.1415926*120.0/360)<<std::endl;

	//rs2_project_point_to_pixel();
	//Eigen::Vector3d Triangulate(std::vector< std::pair<Eigen::Matrix<double, 3, 4>,Eigen::Vector2d> > datas) {
    // datas= [data0,data1,data2...]
    // datax= [poseMatrix,project2dpoint] 该映射点对应的相机位姿矩阵和映射点的二维坐标
	//std::cout<<"____________________________"<<std::endl;
	//std::vector<Eigen::Vector4d> points;

	std::vector<std::vector<double>> points;
	for(int i=0;i<13;i++){
		//Eigen::Vector4d x(0,0,0,1);
		std::vector<double> y={0.2,0.2,0.2};
		std::vector<double> x={1.1,1.1,1.1,1};
		if(mode==0){
			points.push_back(x);
		}
		else{
			points.push_back(y);
		}
		
	}

	// c数组转Eigen::MatrixXd
	// double vec[6] = {1, 2, 3, 4, 5, 6};
	// std::cout << Eigen::Map<Eigen::VectorXd>(&vec[0], sizeof (vec) / sizeof(double)) << "\n";
	// double arr[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
	// std::cout << Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(&arr[0][0], 3, 3) << "\n";
	// ///std::vector<double>转Eigen::MatrixXd
	// std::vector<int> vec2{1, 2, 3, 4, 5, 6};
	// std::cout << Eigen::Map<Eigen::VectorXi>(vec2.data(), 6) << "\n";
	// Eigen::Matrix3d mat2 = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(&arr[0][0], 3, 3);
	// std::cout << mat2 << "\n";
	// Eigen::MatrixXd转std::vector<double>
	// Eigen::Isometry3d vRes = Eigen::Isometry3d::Identity();
	// Eigen::Matrix<double, 4, 4, Eigen::RowMajor> res = vRes.matrix();
	// auto res = std::vector<double>(res.data(), res.data() + res.size());

	ceres::Problem problem;

	for(int i=0;i<13;i++){
		//ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
		ceres::CostFunction *cost_function0;
		if(mode==0){
			cost_function0 = ReprojectionError::Create(pose0,coeffs0,points2d0[i],vis0[i]);
		}
		else{
			cost_function0 = ReprojectionError2::Create(poseR0,poset0,coeffs0,points2d0[i],vis0[i]);
		}
		
		ceres::CostFunction *cost_function1;
		if(mode==0){
			cost_function1 = ReprojectionError::Create(pose1,coeffs1,points2d1[i],vis1[i]);
		}
		else{
			cost_function1 = ReprojectionError2::Create(poseR1,poset1,coeffs1,points2d1[i],vis1[i]);
		}

		problem.AddResidualBlock(
            cost_function0,  //损失函数
            nullptr,   //核函数
            &points[i][0]         //待优化变量
        );

		problem.AddResidualBlock(
            cost_function1,  //损失函数
            nullptr,   //核函数
            &points[i][0]         //待优化变量
        );
	}

	// for(int i=0;i<13;i++){
	// 	std::cout<<"point"<<i<<":("<<points[i][0]<<','<<points[i][1]<<','<<points[i][2]<<") "<<points[i][3]<<std::endl;
	// }

    std::cout << "Solving ceres triangulation ... " << std::endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";


	std::vector<cv::Point3d> points_;
	cv::Mat pts_4d;
	cv::triangulatePoints(T1, T2, keypoints_0, keypoints_1, pts_4d);

	// 转换成非齐次坐标
	for (int i = 0; i < pts_4d.cols; i++) {
		cv::Mat x = pts_4d.col(i);
		std::cout<<x<<std::endl;
		x /= x.at<float>(3, 0); // 归一化
		cv::Point3d p(
			x.at<float>(0, 0),
			x.at<float>(1, 0),
			x.at<float>(2, 0)
		);
		points_.push_back(p);
	}



	std::ofstream outfile("/home/cuichenxi/mediapipe/mediapipe/examples/desktop/ceres_test/dt/result.txt", std::ios::app);

	if(mode==0){
		outfile<<"mode0"<<std::endl;
		for(int i=0;i<13;i++){
			std::cout<<"point"<<i<<":("<<points[i][0]/points[i][3]<<','<<points[i][1]/points[i][3]<<','<<points[i][2]/points[i][3]<<") "<<points[i][3]<<std::endl;
			outfile<<points[i][0]/points[i][3]<<" "<<points[i][1]/points[i][3]<<" "<<points[i][2]/points[i][3]<<std::endl;
		}
	}
	else{
		outfile<<"mode1"<<std::endl;
		for(int i=0;i<13;i++){
			std::cout<<"point"<<i<<":("<<points[i][0]<<','<<points[i][1]<<','<<points[i][2]<<") "<<std::endl;
			outfile<<points[i][0]<<" "<<points[i][1]<<" "<<points[i][2]<<std::endl;
		}
	}

	outfile<<"cv::triangulate"<<std::endl;
	for(int i=0;i<13;i++){
		outfile<<points_[i].x<<" "<<points_[i].y<<" "<<points_[i].z<<std::endl;
	}


	outfile<<std::endl;
	outfile.close();

	return 0;

}
