#ifndef EXAMPLE_H
#define EXAMPLE_H

#include <string>
#include <librealsense2/rs.hpp>
#include "absl/status/status.h"

namespace mediapipe {

class Pose {
public:
    // Create and initialize using provided graph
    // Returns nullptr if initialization failed
    static Pose* Create(const std::string& graphpath);
    virtual ~Pose(){}

    // Processes one frame and blocks until finished
    // Input data is expected to be ImageFormat::SRGB (24bits)
    // Returns nullptr if failed to run graph
    // Returns pointer to image whose size is the same as input image
    // and whose format is ImageFormat::SRGB
    // ImageFormat::SRGB is QImage::Format_RGB888 in Qt
    // Function does not take ownership of input data
    //virtual uint8_t* Process(uint8_t* data, int width, int height) = 0;
    virtual absl::Status Process(rs2::frameset& fs0,rs2::frameset& fs1) = 0;
};

}
#endif
