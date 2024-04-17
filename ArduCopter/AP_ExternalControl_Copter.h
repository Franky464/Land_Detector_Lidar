/*
  external control library for copter
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

class AP_ExternalControl_Copter : public AP_ExternalControl {
public:
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads) override WARN_IF_UNUSED;

    /*
        Sets the target orientation, angular velocity, angular acceleration and desired thrust
        in body frame
    */
    bool set_angular_goals(const Quaternion &orientation, const Vector3f &angular_velocity, const Vector3f &angular_acceleration, const float &thrust) override WARN_IF_UNUSED;

private:
    /*
      Return true if Copter is ready to handle external control data.
      Currently checks mode and arm states.
    */
    bool ready_for_external_control() WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
