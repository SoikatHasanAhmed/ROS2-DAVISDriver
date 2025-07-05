#include <libcaercpp/libcaer.hpp>
#include <libcaercpp/devices/davis.hpp>
#include <atomic>
#include <csignal>
#include <iostream>
#include <string>

// Boost libraries
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Custom message
#include "davisdriver/msg/event.hpp"
#include "davisdriver/msg/event_array.hpp"

#include "davisdriver/msg/image_with_timestamp.hpp"

using namespace cv;
using namespace std;

static atomic_bool globalShutdown(false);

static void globalShutdownSignalHandler(int signal) {
    if (signal == SIGTERM || signal == SIGINT) {
        globalShutdown.store(true);
    }
}

static void usbShutdownHandler(void *ptr) {
    (void)(ptr); // UNUSED.
    globalShutdown.store(true);
}

class DavisNode : public rclcpp::Node {
public:
    DavisNode() : Node("davis_node") {
        std::signal(SIGINT, globalShutdownSignalHandler);
        std::signal(SIGTERM, globalShutdownSignalHandler);

        try {
            davisHandle1 = std::make_unique<libcaer::devices::davis>(1); // First camera
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize DAVIS device: %s", e.what());
            throw;
        }

        logDeviceInfo(davisHandle1);
        configureDavisDevices(davisHandle1);
        startDataCollection(davisHandle1);

        event_array_publisher_1_ = this->create_publisher<davisdriver::msg::EventArray>("davis1/events", 30);
        image_publisher_1_ = this->create_publisher<sensor_msgs::msg::Image>("davis1/image", 30);
        image_ts_publisher_ = this->create_publisher<davisdriver::msg::ImageWithTimestamp>("davis1/image_with_ts", 30);

        spin();
    }

private:
    void logDeviceInfo(const std::unique_ptr<libcaer::devices::davis>& davis) {
        struct caer_davis_info davisInfo = davis->infoGet();
        RCLCPP_INFO(this->get_logger(), "Device ID: %d, DVS Size X: %d, DVS Size Y: %d.", davisInfo.deviceID, davisInfo.dvsSizeX, davisInfo.dvsSizeY);
        RCLCPP_INFO(this->get_logger(), "Device Name: %s", davisInfo.deviceString);
    }

    void configureDavisDevices(const std::unique_ptr<libcaer::devices::davis>& davis) {
        davis->sendDefaultConfig();
        davis->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
        davis->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, APS_FRAME_DEFAULT);
        davis->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 40000);
        configureTrigger(davis);
    }

    void configureTrigger(const std::unique_ptr<libcaer::devices::davis>& davis) {
        davis->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR, true);
        davis->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES, true);
        davis->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, true);
        davis->configSet(DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, true);
    }

    void startDataCollection(const std::unique_ptr<libcaer::devices::davis>& davis) {
        davis->dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);
        davis->configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    }

    void shutdownDevices() {
        davisHandle1->dataStop();
        RCLCPP_INFO(this->get_logger(), "Shutdown successful.");
    }

    void eventCallback(const std::unique_ptr<libcaer::events::EventPacketContainer>& packetContainer) {
        if (!packetContainer) return;

        auto event_array_msg = std::make_shared<davisdriver::msg::EventArray>();
        event_array_msg->height = davisHandle1->infoGet().dvsSizeY;
        event_array_msg->width = davisHandle1->infoGet().dvsSizeX;
        event_array_msg->header.stamp = this->get_clock()->now();

        for (const auto& packet : *packetContainer) {
            if (!packet) continue;

            if (packet->getEventType() == POLARITY_EVENT) {
                auto polarity = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);
                int num_of_event = polarity->size();

                for (int i = 0; i < num_of_event; ++i) {
                    const auto& event = (*polarity)[i];
                    davisdriver::msg::Event e;
                    e.x = event.getX();
                    e.y = event.getY();
                    e.time = event.getTimestamp();
                    e.polarity = event.getPolarity();
                    event_array_msg->events.push_back(e);
                }
            }

            if (packet->getEventType() == FRAME_EVENT) {
                auto frame = std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

                for (const auto& f : *frame) {
                    if (!f.isValid() || f.getROIIdentifier() != 0) continue;

                    cv::Mat cvFrame = f.getOpenCVMat(false);  // Grayscale 16-bit

                    if (cvFrame.empty()) {
                        RCLCPP_WARN(this->get_logger(), "Empty frame received, skipping.");
                        continue;
                    }

                    // Convert to 8-bit if needed
                    cv::Mat image_8bit;
                    if (cvFrame.depth() != CV_8U) {
                        cvFrame.convertTo(image_8bit, CV_8U, 1.0 / 256.0);  // scale 16-bit → 8-bit
                    } else {
                        image_8bit = cvFrame;
                    }

                    // Convert to BGR only if image is single-channel
                    cv::Mat bgrFrame;
                    if (image_8bit.channels() == 1) {
                        cv::cvtColor(image_8bit, bgrFrame, cv::COLOR_GRAY2BGR);
                    } else {
                        bgrFrame = image_8bit;
                    }

                    // Timestamp conversion
                    int32_t ts = f.getTimestamp();
                    rclcpp::Time img_ros_time(static_cast<uint64_t>(ts) * 1000);  // µs to ns

                    std_msgs::msg::Header header;
                    header.stamp = this->get_clock()->now();
                    header.frame_id = "camera_frame";

                    // Publish standard image
                    auto image_msg = cv_bridge::CvImage(header, "bgr8", bgrFrame).toImageMsg();
                    image_publisher_1_->publish(*image_msg);

                    // Publish image with timestamp
                    davisdriver::msg::ImageWithTimestamp ts_msg;
                    ts_msg.header = header;
                    ts_msg.image = *image_msg;
                    ts_msg.img_ts.sec = img_ros_time.seconds();
                    ts_msg.img_ts.nanosec = img_ros_time.nanoseconds() % 1000000000;
                    image_ts_publisher_->publish(ts_msg);
                }
            }
        }

        event_array_publisher_1_->publish(*event_array_msg);
    }

    void spin() {
        while (rclcpp::ok() && !globalShutdown.load()) {
            auto packetContainer = davisHandle1->dataGet();
            eventCallback(packetContainer);
        }
        shutdownDevices();
    }

    std::unique_ptr<libcaer::devices::davis> davisHandle1;
    rclcpp::Publisher<davisdriver::msg::EventArray>::SharedPtr event_array_publisher_1_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_1_;
    rclcpp::Publisher<davisdriver::msg::ImageWithTimestamp>::SharedPtr image_ts_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DavisNode>());
    rclcpp::shutdown();
    return 0;
}
