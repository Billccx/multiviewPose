#include "pose.h"
#include "pose_impl.h"

namespace mediapipe {

Pose* Pose::Create(const std::string& graphpath) 
{
    PoseImpl *example = new PoseImpl();
    absl::Status status = example->Init(graphpath);
    if (status.ok()){
        return example;
    }    
    else{
        LOG(INFO) << "Error initializing graph " << status.ToString();
        delete example;
        return nullptr;
    } 
}

}
