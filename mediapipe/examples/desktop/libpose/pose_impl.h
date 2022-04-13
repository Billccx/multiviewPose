#ifndef EXAMPLE_IMPL_H
#define EXAMPLE_IMPL_H

#include "pose.h"
#include "absl/status/status.h"
#include "mediapipe/framework/calculator_framework.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>  
#include "mediapipe/framework/packet.h"

namespace mediapipe {

class PoseImpl : public Pose {
public:
    PoseImpl(){}
    ~PoseImpl();

    absl::Status Init(const std::string& graphpath);

    cv::Mat Process(rs2::frameset& fs0,rs2::frameset& fs1) override;

private:
    mediapipe::CalculatorGraph m_graph;
    absl::StatusOr<OutputStreamPoller> m_poller0;
    absl::StatusOr<OutputStreamPoller> m_poller1;
    size_t m_frame_timestamp = 0;
};

}
#endif
