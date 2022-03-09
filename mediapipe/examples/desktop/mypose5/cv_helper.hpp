#include <iostream>
#include <vector>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp> 

static cv::Mat frame_to_mat(const rs2::frame& f);