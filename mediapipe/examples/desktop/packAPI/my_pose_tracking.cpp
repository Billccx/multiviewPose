#include "my_pose_tracking.h"

myPoseTracking::PoseTracking::PoseTracking(){

}

myPoseTracking::PoseTracking::~PoseTracking(){
    
}

int myPoseTracking::PoseTracking::InitGraph(const char* graphpath){
    absl::Status run_status = Mediapipe_InitGraph(graphpath);
    if (!run_status.ok()) {
        return 0;
    }
    return  1;
}

absl::Status myPoseTracking::PoseTracking::Mediapipe_InitGraph(const char* graphpath){
    std::string calculator_graph_config_contents;
    MP_RETURN_IF_ERROR(
        mediapipe::file::GetContents(graphpath, &calculator_graph_config_contents)
    );

    mediapipe::CalculatorGraphConfig config =
        mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(calculator_graph_config_contents);

    MP_RETURN_IF_ERROR(myGraph.Initialize(config));
    MP_RETURN_IF_ERROR(myGraph.StartRun({}));

    return absl::OkStatus();
}