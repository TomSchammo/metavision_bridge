#include <cstdint>
#include <ranges>
#include <vector>

#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>

struct Event {
  uint64_t t;
  uint16_t x, y;
  bool p;
};

constexpr uint64_t one_ns = 1'000'000'000;

using event_camera_codecs::EventPacket;

class Decoder : public event_camera_codecs::EventProcessor {
public:
  std::vector<Event> events;
  Decoder(size_t event_buffer_size = 100) { events.reserve(event_buffer_size); }
  inline void eventCD(uint64_t t, uint16_t ex, uint16_t ey,
                      uint8_t polarity) override {
    events.push_back({t, ex, ey, polarity == 1});
    // RCLCPP_INFO(rclcpp::get_logger("processor"), "<%u, %u, %lu, %u>", ex, ey,
    // t, polarity);
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void finished() override {
  }; // called after no more events decoded in this packet
  void rawData(const char *, size_t) override {}; // passthrough of raw data
};

class EventBridgeNode : public rclcpp::Node {
private:
  Decoder decoder_;
  event_camera_codecs::DecoderFactory<EventPacket, Decoder> decoder_factory_;
  rclcpp::Subscription<event_camera_msgs::msg::EventPacket>::SharedPtr sub_;
  rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr pub_;

public:
  EventBridgeNode() : Node("event_bridge") {
    auto qos = rclcpp::QoS(10).best_effort();
    this->sub_ = this->create_subscription<event_camera_msgs::msg::EventPacket>(
        "~/events_in", qos,
        std::bind(&EventBridgeNode::callback, this, std::placeholders::_1));
    this->pub_ =
        this->create_publisher<dvs_msgs::msg::EventArray>("~/events_out", qos);
  }

private:
  void
  callback(const event_camera_msgs::msg::EventPacket::ConstSharedPtr &msg) {
    auto decoder = this->decoder_factory_.getInstance(*msg);
    if (!decoder) {
      RCLCPP_WARN(this->get_logger(), "unknown encoding");
      return;
    }
    decoder->decode(*msg, &this->decoder_);

    dvs_msgs::msg::EventArray event_array;
    event_array.header = msg->header;
    event_array.height = msg->height;
    event_array.width = msg->width;
    event_array.events.reserve(this->decoder_.events.size());

    // move events from decoder to publisher
    std::ranges::transform(
        this->decoder_.events, std::back_inserter(event_array.events),
        [](const auto &item) {
          dvs_msgs::msg::Event event;
          event.x = item.x;
          event.y = item.y;
          event.ts.sec = static_cast<int32_t>(item.t / one_ns);
          event.ts.nanosec = static_cast<uint32_t>(item.t % one_ns);
          event.polarity = item.p;
          return event;
        });

    // publish
    if (!event_array.events.empty()) {
      this->pub_->publish(event_array);
    }

    // clear buffer after processing
    this->decoder_.events.clear();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
