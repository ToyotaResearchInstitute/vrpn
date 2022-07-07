#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "vrpn_Tracker_ROStf.h"
#include "rclcpp/rclcpp.hpp"

vrpn_Tracker_ROStf::vrpn_Tracker_ROStf(const char *name, vrpn_Connection *c,
  const char *fromFrameRel, const char *toFrameRel)
  : vrpn_Tracker(name, c)
  , fromFrameRel_(fromFrameRel)
  , toFrameRel_(toFrameRel)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  register_server_handlers();

  // Get the time we started
  vrpn_gettimeofday(&start, NULL);
}
void vrpn_Tracker_ROStf::mainloop()
{
  struct timeval current_time;
  char msgbuf[1000];
  vrpn_int32 len;

  // Call the generic server mainloop routine, since this is a server
  server_mainloop();

  // See if its time to generate a new report
  vrpn_gettimeofday(&current_time, NULL);

  // Update the time
  timestamp.tv_sec = current_time.tv_sec;
  timestamp.tv_usec = current_time.tv_usec;

  // Send message for transform if we have a connection
  if (d_connection) {
    geometry_msgs::msg::TransformStamped transformStamped;
    bool success = true;
    try {
      transformStamped =
        tf_buffer_->lookupTransform(
          fromFrameRel_, toFrameRel_,
          tf2::TimePointZero);
    } catch (...) {
      success = false;
    }

    if (success) {
      pos[0] = transformStamped.transform.translation.x;
      pos[1] = transformStamped.transform.translation.y;
      pos[2] = transformStamped.transform.translation.z;
      d_quat[0] = transformStamped.transform.rotation.x;
      d_quat[1] = transformStamped.transform.rotation.y;
      d_quat[2] = transformStamped.transform.rotation.z;
      d_quat[3] = transformStamped.transform.rotation.w;
    }

    // Pack transformation report
    len = encode_to(msgbuf);
    if (d_connection->pack_message(len, timestamp, position_m_id,
      d_sender_id, msgbuf,
      vrpn_CONNECTION_LOW_LATENCY)) {
      fprintf(stderr,
        "NULL tracker: can't write message: tossing\n");
    }
  }
}
