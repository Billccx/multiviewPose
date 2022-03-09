#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image.h"
#include "mediapipe/framework/formats/image_frame.h"
#include <time.h>

namespace mediapipe{
    class ExampleCalculator: public CalculatorBase {
        public:
        int validcnt;
        clock_t start,now;
        ExampleCalculator(){
            start=clock();
            validcnt=0;
        }
        static absl::Status GetContract(CalculatorContract* cc){
            std::cout << " GetContract() " << std::endl;
            cc->Inputs().Get("INPUT",0).Set<mediapipe::ImageFrame>();
            cc->Inputs().Get("INPUT",1).Set<mediapipe::ImageFrame>();
            return absl::OkStatus();
        }

        absl::Status Open(CalculatorContext* cc){
            std::cout << " Open() " << std::endl;
            return ::mediapipe::OkStatus();
        }

        absl::Status Process(CalculatorContext* cc){
            if(validcnt==0){
                start=clock();
            }
            else{
                start=now;
            }
            std::cout << ++validcnt << std::endl;
            const auto& input_image0 = cc->Inputs().Get("INPUT",0).Get<ImageFrame>();
            std::cout<<"get first input successfully"<<std::endl;
            const auto& input_image1 = cc->Inputs().Get("INPUT",1).Get<ImageFrame>();
            std::cout<<"get second input successfully"<<std::endl;
            std::cout<<cc->InputTimestamp()<<std::endl;
            std::cout<<"image0, width="<<input_image0.Width()<<" height="<<input_image0.Height()<<std::endl;
            std::cout<<"image1, width="<<input_image1.Width()<<" height="<<input_image1.Height()<<std::endl;       
            now=clock();
            std::cout<<"fps:"<<1/((double)(now-start)/CLOCKS_PER_SEC)<<std::endl;
            return ::mediapipe::OkStatus();
        }
    };
    REGISTER_CALCULATOR(ExampleCalculator);
}