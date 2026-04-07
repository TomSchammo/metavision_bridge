#include <concepts>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <ranges>
#include <utility>
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

template <typename T>
  requires std::integral<T>
constexpr T mod(T a, T b) {
  T r = a % b;
  return r >= 0 ? r : r + std::abs(b);
}

constexpr uint64_t one_ns = 1'000'000'000;

using event_camera_codecs::EventPacket;

class Decoder : public event_camera_codecs::EventProcessor {
public:
  std::vector<Event> events;
  Decoder(size_t event_buffer_size = 150000) {
    events.reserve(event_buffer_size);
  }
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
  std::optional<rclcpp::Publisher<builtin_interfaces::msg::Duration>::SharedPtr>
      time_diff_pub_;

public:
  EventBridgeNode() : Node("event_bridge") {
    auto qos = rclcpp::QoS(10).best_effort();
    this->declare_parameter<bool>("is_master", false);
    this->sub_ = this->create_subscription<event_camera_msgs::msg::EventPacket>(
        "~/events_in", qos,
        std::bind(&EventBridgeNode::callback, this, std::placeholders::_1));
    this->pub_ =
        this->create_publisher<dvs_msgs::msg::EventArray>("~/events_out", qos);
    if (this->get_parameter("is_master").as_bool()) {
      this->time_diff_pub_ =
          this->create_publisher<builtin_interfaces::msg::Duration>(
              "/time_offset", qos);
    }
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

    // RCLCPP_INFO(this->get_logger(), "decoding %lu events",
    //             this->decoder_.events.size());

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

    if (this->get_parameter("is_master").as_bool()) {
      builtin_interfaces::msg::Duration time_diff_msg;

      rclcpp::Time ros_time(msg->header.stamp);
      rclcpp::Time event_time(event_array.events.back().ts);
      rclcpp::Duration offset = event_time - ros_time;

      time_diff_msg.sec = static_cast<int32_t>(std::floor(offset.seconds()));
      time_diff_msg.nanosec = static_cast<uint32_t>(
          mod<int64_t>(static_cast<int64_t>(offset.nanoseconds()), one_ns));

      while (!this->time_diff_pub_.has_value()) {}
      this->time_diff_pub_.value()->publish(time_diff_msg);
    }

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
