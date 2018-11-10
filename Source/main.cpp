#include <string>
#include <iomanip>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "VideoProcessor.h"
#include "FrameProcessor.h"
#include "HAL/USB.h"
#include "HAL/SerialBus.h"

cv::Mat Tubeidentify(const cv::Mat& input);

bool is_interesting(libusb_device* pDevice) {
    libusb_device_descriptor desc {};
    libusb_get_device_descriptor(pDevice, &desc);
    return desc.idVendor == 0x1a86 && desc.idProduct == 0x7523;
}

char get_result();

void lusb_start() {
    try {
        libusb_device** list;
        libusb_device* found = nullptr;
        ssize_t cnt = libusb_get_device_list(nullptr, &list);
        ssize_t i = 0;
        int err = 0;
        for (i = 0; i<cnt; i++) {
            libusb_device* device = list[i];
            if (is_interesting(device)) {
                found = device;
                break;
            }
        }
        if (found) {
            auto bus = OpenAs<IUARTBus>(found);
            UARTConfig cfg;
            cfg.ParityBit = UARTConfig::Parity::Even;
            bus->Configure(cfg);
            while(true) {
                char this_res = get_result();
                if (this_res >=1 && this_res <= 3)
                    std::cout << "write:" << this_res << std::endl;
                    //bus->Write(&this_res, 1);
            }
            bus->Close();
        }
        libusb_free_device_list(list, 1);
    }
    catch (std::exception& exp) {
        std::cout << exp.what() << std::endl;
    }
}

cv::Mat ProcessFrame(const cv::Mat& input);

int main() {
    VideoProcessor processor;
    // 打开摄像头
    processor.setInput(std::make_shared<LogicalCamera>(0));

    // 分别为输入和输出
    processor.displayInput("Input ");
    processor.displayOutput("Output ");

    processor.setDelay(1000. / processor.getFrameRate());

    processor.setFrameProcessor(Tubeidentify);

    // 开始帧处理过程
    auto task = processor.run();
    std::thread([](){
        while(true) {
            char this_res = get_result();
            if (this_res >=1 && this_res <= 3)
                std::cout << "write:" << int(this_res) << std::endl;
            //bus->Write(&this_res, 1);
        }
    }).detach();

    waitKey();
    processor.stop();
    task.wait();
    //TODO 加入识别function
    return 0;

    /**/
}
