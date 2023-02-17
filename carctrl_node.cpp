/**
  * Mini-Auto-Drive MAD
  * Modellbasierte Softwareentwicklung
  *
  * Wintersemester 2022/2023
  *
  * Team 1
  * Jannis Daiber   207144
  * Noah Hoehn      207179
  *
  * @file carctrl_node.cpp
  */

#include <ros/ros.h>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include "std_msgs/Float32.h"
#include "madmsgs/CarInputs.h"
#include "madmsgs/CarOutputs.h"
#include "madmsgs/CarOutputsExt.h"
#include "madmsgs/CtrlReference.h"
#include "madmsgs/DriveManeuver.h"
#include "madmsgs/DriveManeuverState.h"
#include "madmsgs/MissionState.h"
#include "CarParameters.h"
#include "speedcontroller.h"
#include "positioncontroller.h"
#include "pathcontroller.h"


/**
 * @brief The CarCtrlNode class
 */
class CarCtrlNode
{
public:
  // Declare Controller types used
  using SpeedControllerType = SpeedController;
  using PositionControllerType = PositionController;
  using PathControllerType = PathController;
  /**
   * @brief Constructor
   * @param[in] sampling time [s]
   */
  CarCtrlNode(const float samplingTimeArg) :
    samplingTime(samplingTimeArg) //initializes the member samplingTime
  {
    // Create subscriber for topic /mad/caroutputsext
    carOutputsExtSub = node.subscribe("/mad/caroutputsext",
                                      2 * CarParameters::p()->carCnt,
                                      &CarCtrlNode::carOutputsExtCallback, this);
    // Create subscriber for topic /mad/car0/navi/maneuver
    driveManeuverSub = node.subscribe("/mad/car0/navi/maneuver",
                                      2 * CarParameters::p()->carCnt,
                                      &CarCtrlNode::driveManeuverCallback, this);
    // Create publisher for topic /mad/carinputs
    carInputsPub = node.advertise<madmsgs::CarInputs>("/mad/carinputs", 1);
    // Create publisher for topic /mad/car0/ctrl/debug/vs
    vs_Pub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/debug/vs", 1);
    // Create publisher for topic /mad/car0/ctrl/debug/wp
    wp_Pub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/debug/wp", 1);
    // Create publisher for topic /mad/car0/ctrl/debug/x_integrated
    xreal_Pub = node.advertise<std_msgs::Float32>("/mad/car0/ctrl/debug/x_integrated", 1);
  }

  void step()
  {
    // Get ROS Time for timestamp of ROS messages
    ros::Time time { ros::Time::now() };

    // Define necessary Variables
    float delta = 0.7F; // Steering input
    int nearest = -1;   // nearest waypoint of spline (needed for lap time in console)

    // Numerical Integration (Trapezoidal rule) for Position control
    realPos += samplingTime*(realSpeed + oldRealSpeed) / 2.0F;
    // Assign Speed Value to old speed value (for numerical integration)
    oldRealSpeed = realSpeed;

    // If Position Control is active
    if (maneuverType == madmsgs::DriveManeuver::TYPE_PARK)
    {
      // Set Driving Mode to CMD_SLOW (protects onboard electronics (speed limitation))
      carInputsMsg.cmd = madmsgs::CarInputs::CMD_SLOW;
      // Positioncontroller step (reference Signal generation and controller Step)
      vs = positioncontroller.step(realPos, x0, xs, samplingTime, positionRunTime, wp, uVp1);
      // Update time of position control
      positionRunTime += samplingTime;
    }
    // If path control and speed control is active
    else if (maneuverType == madmsgs::DriveManeuver::TYPE_PATHFOLLOW)
    {
      // If given reference speed is equal or greater than 0 m/s
      if (vs >= 0.0F)
      {
        // Set Driving Mode to CMD_FORWARD (only-forward driving enabled)
        carInputsMsg.cmd = madmsgs::CarInputs::CMD_FORWARD;
      }
      // Else: given reference speed is below 0 m/s
      else
      {
        // Set Driving Mode to CMD_REVERSE (only-backward driving enabled)
        carInputsMsg.cmd = madmsgs::CarInputs::CMD_REVERSE;
      }
    }

    // If any maneuver is sended
    if (maneuverType == madmsgs::DriveManeuver::TYPE_PATHFOLLOW || maneuverType == madmsgs::DriveManeuver::TYPE_PARK)
    {
      // Pathcontroller step (reference Signal generation and controller and feedforward control Step)
      delta = pathcontroller.step(spline, realS, vs, realPsi, nearest);
      // Speedcontroller step (write value in pedals signal)
      carInputsMsg.pedals = speedcontroller.step(vs, realSpeed, samplingTime);
      // Measurement of Lap time from Model Car
      if (nearest == 0 && oldNearest != 0)
      {
        ROS_INFO("Lap time of last lap is %0.2f", lapTime);
        lapTime = 0.0F;
      }
      else
      {
        lapTime += samplingTime;
      }
      oldNearest = nearest;
    }
    // Publish /mad/carinputs message
    carInputsMsg.steering = delta;
    carInputsMsg.header.stamp = time;
    carInputsPub.publish(carInputsMsg);
    // Publish /mad/car0/ctrl/debug/vs message
    vsMsg.data = vs;
    vs_Pub.publish(vsMsg);
    // Publish /mad/car0/ctrl/debug/wp message for debugging
    wpMsg.data = wp;
    wp_Pub.publish(wpMsg);
    // Publish /mad/car0/ctrl/debug/x_integrated message for debugging
    xMsg.data = realPos;
    xreal_Pub.publish(xMsg);
  }


private:
  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  ros::Subscriber carOutputsExtSub; /**< The /mad/caroutputsext topic subscriber */
  ros::Subscriber driveManeuverSub; /**< The /mad/car0/navi/maneuver topic subscriber */
  ros::Publisher carInputsPub;/**< The /mad/carinputs topic publisher */
  ros::Publisher vs_Pub; /**< The /mad/car0/ctrl/debug/vs topic publisher */
  ros::Publisher wp_Pub; /**< The /mad/car0/ctrl/debug/wp topic publisher */
  ros::Publisher xreal_Pub; /**< The /mad/car0/ctrl/debug/x_integrated topic publisher */
  madmsgs::CarInputs carInputsMsg; /**< The /mad/carinputs topic message */
  std_msgs::Float32 vsMsg; /**< The /mad/car0/ctrl/debug/vs topic message */
  std_msgs::Float32 wpMsg; /**< The /mad/car0/ctrl/debug/wp topic message */
  std_msgs::Float32 xMsg; /**< The /mad/car0/ctrl/debug/x_integrated topic message */
  CarParameters* p = CarParameters::p(); /**< Pointer to needed parameters in madlib/CarParameters.h */
  const float samplingTime = 0.0F; /**< The sample time [s] */
  SpeedControllerType speedcontroller{}; /**< Object of SpeedController class for use in step function */
  PositionControllerType positioncontroller{}; /**< Object of PositionController class for use in step function */
  PathControllerType pathcontroller{}; /**< Object of PathController class for use in step function */
  modbas::Spline spline; /**< Object of Class Spline needed for PathController */
  std::array<float, 2> realS; /**< array with real cartesian coordinates (s1, s2) of car */
  float realPsi = 0.0F; /**< real angle psi car */
  int oldNearest = 0; /**< old nearest waypoint on spline (lap time measurement) */
  float lapTime = 0.0F; /**< lap time of one whole driven lap */
  float realSpeed = 0.0F; /**< real speed of car */
  float oldRealSpeed = 0.0F; /**< real speed value of car before on step is called (for numerical integration) */
  float vs = 0.0F; /**< reference speed */
  float xs = 0.0F; /**< reference position (end of maneuver) */
  float x0 = 0.0F; /**< starting position of parking maneuver (will be updated every time new Maneuver is sended) */
  float realPos = 0.0F; /**< numerical integrated speed over time (position)*/
  float wp = 0.0F; /**< Lead signal of position controller (only for debugging) */
  float uVp1 = 0.0F; /**< Feedforward control signal of position controller (only for debugging) */
  int maneuverType = 0; /**< Maneuver Type defined in DriveManeuver.msg */
  float positionRunTime = 0.0F; /**< Time from start of Parking Maneuver */

  /**
  * @brief callback for /mad/caroutputsext topic
  * @param[in] msg The ROS message ConstPtr
  */
  void carOutputsExtCallback(const madmsgs::CarOutputsExtConstPtr& msg)
  {
    // Write real coordinates, angle and speed of car from message in variables
    realS.at(0) = msg->s.at(0);
    realS.at(1) = msg->s.at(1);
    realPsi = msg->psi;
    realSpeed = msg->v;
  }

  /**
  * @brief callback for /mad/car0/navi/maneuver topic
  * @param[in] msg The ROS message ConstPtr
  */
  void driveManeuverCallback(const madmsgs::DriveManeuverConstPtr& msg)
  {
    // Get reference position, maneuver type and reference speed for controller steps
    xs = msg->xManeuverEnd;
    maneuverType = msg->type;
    vs = msg->vmax;
    // Initialize Position Control time -> set to zero
    positionRunTime = 0.0F;
    // Offline Calculation of coefficients for PositionControl
    x0 = realPos;
    positioncontroller.init(x0, xs, vs);
    // Get messages needed for PathController
    const std::vector<float> breaks = msg->breaks;
    const std::vector<float> s1 = msg->s1;
    const std::vector<float> s2 = msg->s2;
    const std::vector<uint32_t> segments = msg->segments;
    // Calculate data of Spline for PathFollowing
    pathcontroller.init(breaks, s1, s2, segments, spline);
  }
};

int main(int argc, char **argv)
{
  const float samplingTime = 20e-3F; // the constant sample time [ s ]
  ros::init(argc, argv, "carctrl_node"); // initialize ROS
  CarCtrlNode node(samplingTime);
  // define sampling rate as the inverse of the sample time
  ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
  // loop while ROS is running
  while (ros::ok())
  {
    // call the method step() of the CarCtrlNode instance node
    node.step();
    // pass control to ROS for background tasks
    ros::spinOnce();
    // wait for next sampling point
    // neighbor sampling points have a time distance of 20ms
    loopRate.sleep();
  }
  // return success
  return EXIT_SUCCESS;
}
