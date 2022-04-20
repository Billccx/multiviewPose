#include "pose_impl.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "mediapipe/framework/packet.h"

namespace mediapipe {
constexpr char kInputStream0[] = "input_frameset0";
constexpr char kInputStream1[] = "input_frameset1";



PoseImpl::~PoseImpl() 
{
    LOG(INFO) << "Shutting down.";

    absl::Status status = m_graph.CloseAllInputStreams();
    if (status.ok()){
    	absl::Status status1 = m_graph.WaitUntilDone();
        if (!status1.ok()) {
            LOG(INFO) << "Error in WaitUntilDone(): " << status1.ToString();
        }
    } else {
        LOG(INFO) << "Error in CloseInputStream(): " << status.ToString();
    }
}

absl::Status PoseImpl::Init(const std::string& graphpath) {
    LOG(INFO) << "Parsing graph config " << graphpath;

    std::string graph;
    //读取解析计算图
    MP_RETURN_IF_ERROR(mediapipe::file::GetContents(graphpath,&graph));

    mediapipe::CalculatorGraphConfig config = mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(graph);

    LOG(INFO) << "Initialize the calculator graph.";
    MP_RETURN_IF_ERROR(m_graph.Initialize(config));

    //LOG(INFO) << "Register the poller.";
    //ASSIGN_OR_RETURN(m_poller0, m_graph.AddOutputStreamPoller("output_video0"));
    //ASSIGN_OR_RETURN(m_poller1, m_graph.AddOutputStreamPoller("output_video1"));
    
/*
    cv::namedWindow("pose0",cv::WINDOW_AUTOSIZE);
    cv::namedWindow("pose1",cv::WINDOW_AUTOSIZE);

    MP_RETURN_IF_ERROR(
        m_graph.ObserveOutputStream(
            "output_video0",
            [](const mediapipe::Packet& packet) -> ::mediapipe::Status {
                std::cout<<"in callback func0"<<std::endl;
                auto &output_frame0 = packet.Get<mediapipe::ImageFrame>();
                cv::Mat output_frame_mat0 = mediapipe::formats::MatView(&output_frame0);
                if(!output_frame_mat0.data) std::cout<<"[error] in calllback0 mat is empty"<<std::endl;
                else std::cout<<"[callback0] already changed imageframe to mat"<<std::endl;
                cv::imwrite("/home/cuichenxi/code/Qt/qmakeProj/pic/"+packet.Timestamp().DebugString()+"_cam0.jpg",output_frame_mat0);
                // cv::imshow("pose0", output_frame_mat0);
                // cv::waitKey(30);
                return mediapipe::OkStatus();
            }
        )
    );

    MP_RETURN_IF_ERROR(
        m_graph.ObserveOutputStream(
            "output_video1",
            [](const mediapipe::Packet& packet) -> ::mediapipe::Status {
                std::cout<<"in callback func1"<<std::endl;
                auto &output_frame1 = packet.Get<mediapipe::ImageFrame>();
                cv::Mat output_frame_mat1 = mediapipe::formats::MatView(&output_frame1);
                if(!output_frame_mat1.data) std::cout<<"[error] in calllback1 mat is empty"<<std::endl;
                else std::cout<<"[callback1] already changed imageframe to mat"<<std::endl;
                cv::imwrite("/home/cuichenxi/code/Qt/qmakeProj/pic/"+packet.Timestamp().DebugString()+"_cam1.jpg",output_frame_mat1);
                // cv::imshow("pose1", output_frame_mat1);
                // cv::waitKey(30);
                return mediapipe::OkStatus();
            }
        )
    );
    std::cout<<"already register the callback function. "<<std::endl;
*/

    LOG(INFO) << "Start running the calculator graph.";
    MP_RETURN_IF_ERROR(m_graph.StartRun({}));

    return absl::OkStatus();
}

void PoseImpl::Process(rs2::frameset& fs0,rs2::frameset& fs1) {

    rs2::video_frame color0=fs0.get_color_frame();
    mediapipe::Packet frmset0 = mediapipe::MakePacket<rs2::frameset>(fs0).At(mediapipe::Timestamp(m_frame_timestamp));
    mediapipe::Packet frmset1 = mediapipe::MakePacket<rs2::frameset>(fs1).At(mediapipe::Timestamp(m_frame_timestamp));
    std::cout << "already packet the data at time:" << mediapipe::Timestamp(m_frame_timestamp) << std::endl;
    m_frame_timestamp++;
    
    // MP_RETURN_IF_ERROR(m_graph.AddPacketToInputStream(kInputStream0, frmset0));
    // std::cout<<"success send in the packet0"<<std::endl;
    // MP_RETURN_IF_ERROR(m_graph.AddPacketToInputStream(kInputStream1, frmset1));
    // std::cout<<"success send in the packet1"<<std::endl;

    absl::Status s0=m_graph.AddPacketToInputStream(kInputStream0, frmset0);
    if (!s0.ok()) LOG(INFO) << s0.message();

    absl::Status s1=m_graph.AddPacketToInputStream(kInputStream1, frmset1);
    if (!s1.ok()) LOG(INFO) << s1.message();

    /*
    mediapipe::Packet packet0,packet1;

    if (!m_poller0->Next(&packet0)){
        LOG(INFO) << "Poller0 didnt give me a packet, return empty mat";
        return cv::Mat();
    }
    auto &output_frame0 = packet0.Get<mediapipe::ImageFrame>();
    std::cout<<"already got the first output frame"<<std::endl;

    if (!m_poller1->Next(&packet1)){
        LOG(INFO) << "Poller1 didnt give me a packet, return empty mat";
        return cv::Mat();
    }
    auto &output_frame1 = packet1.Get<mediapipe::ImageFrame>();
    std::cout<<"already got the second output frame"<<std::endl;


    cv::Mat output_frame_mat0 = mediapipe::formats::MatView(&output_frame0);
    cv::Mat output_frame_mat1 = mediapipe::formats::MatView(&output_frame1);
    cv::Mat merge;
    hconcat(output_frame_mat0,output_frame_mat1,merge);
    return merge;
    */

    /*
    mediapipe::Packet packet;
    if (!m_poller->Next(&packet)){
        LOG(INFO) << "Poller didnt give me a packet, sorry. Call m_graph.WaitUntilDone() to see error (or destroy Example object). Error probably is that models are not available under mediapipe/models or mediapipe/modules";
        return nullptr;
    }

    const ImageFrame &output_frame = packet.Get<mediapipe::ImageFrame>();
    size_t output_bytes = output_frame.PixelDataSizeStoredContiguously();

    // This could be optimized to not copy but return output_frame.PixelData()
    uint8_t* out_data = new uint8_t[output_bytes];
    output_frame.CopyToBuffer((uint8*)out_data, output_bytes);
    return out_data; 
    */
}

}
