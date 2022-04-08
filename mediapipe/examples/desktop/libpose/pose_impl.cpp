#include "pose_impl.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/formats/image_frame.h"

#include <librealsense2/rs.hpp>
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

    LOG(INFO) << "Start running the calculator graph.";
    //ASSIGN_OR_RETURN(m_poller, m_graph.AddOutputStreamPoller(kOutputStream));
    MP_RETURN_IF_ERROR(m_graph.StartRun({}));

    return absl::OkStatus();
}

absl::Status PoseImpl::Process(rs2::frameset& fs0,rs2::frameset& fs1) {

    rs2::video_frame color0=fs0.get_color_frame();
    std::cout<<"fs0 color width:"<<color0.get_width()<<std::endl;

    
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

    // if (!m_graph.AddPacketToInputStream(kInputStream1, frmset1).ok()) {
    //     LOG(INFO) << "Failed to add frameset1 to input stream. Call m_graph.WaitUntilDone() to see error (or destroy Example object)";
    // }

    return absl::OkStatus();

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
