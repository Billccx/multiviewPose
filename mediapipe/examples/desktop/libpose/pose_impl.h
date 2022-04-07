#ifndef EXAMPLE_IMPL_H
#define EXAMPLE_IMPL_H

#include "pose.h"
#include "absl/status/status.h"
#include "mediapipe/framework/calculator_framework.h"

#include <librealsense2/rs.hpp>
#include "mediapipe/framework/packet.h"

namespace mediapipe {

class PoseImpl : public Pose {
public:
    PoseImpl(){}
    ~PoseImpl();

    absl::Status Init(const std::string& graphpath);

    absl::Status Process(rs2::frameset& fs0,rs2::frameset& fs1) override;

private:
    mediapipe::CalculatorGraph m_graph;
    absl::StatusOr<OutputStreamPoller> m_poller;
    size_t m_frame_timestamp = 0;
};

}
#endif
