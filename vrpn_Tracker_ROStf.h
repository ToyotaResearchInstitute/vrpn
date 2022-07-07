#include "vrpn_BaseClass.h" // for vrpn_Callback_List, etc
#include "vrpn_Configure.h" // for VRPN_CALLBACK, VRPN_API, etc
#include "vrpn_Connection.h"
#include "vrpn_Shared.h" // for timeval
#include "vrpn_Types.h"  // for vrpn_float64, vrpn_int32, etc
#include "vrpn_Tracker.h"

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "quat.h"

// This is an example of a tracker server.  It stays at the
// origina and spins around the specified axis at the
// specified rate of rotation, reporting orientation and
// orientation velocity at the specified
// rate.  It was designed to help test the smoothness of
// rendering for VR systems by providing a ground-truth
// smoothly-rotating tracker source.

class VRPN_API vrpn_Tracker_ROStf : public vrpn_Tracker{
public:
  vrpn_Tracker_ROStf(const char *name, vrpn_Connection *c,
    const char *fromFrameRel, const char *toFrameRel);
  virtual void mainloop();

protected:
  vrpn_float64 update_rate;
  struct timeval start;
  std::string toFrameRel_;
  std::string fromFrameRel_;

private:
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

