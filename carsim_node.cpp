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
  * @file carsim_node.cpp
  */

#include <ros/ros.h>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include "madmsgs/CarInputs.h"
#include "madmsgs/CarOutputs.h"
#include "madmsgs/CarOutputsExt.h"
#include "madmsgs/CtrlReference.h"
#include "madmsgs/DriveManeuver.h"
#include "madmsgs/DriveManeuverState.h"
#include "madmsgs/MissionState.h"
#include "CarParameters.h"
#include "carplant.h"

/**
 * @brief The CarSimNode class
 */
class CarSimNode
{
public:
  // use ModelType CarPlant from carplant.h
  using ModelType = CarPlant;
  /**
   * @brief Constructor
   * @param[in] sampling time [s]
   */
  CarSimNode(const float samplingTimeArg) :
    samplingTime(samplingTimeArg) //initializes the member samplingTime
  {
    //create subscriber for /mad/carinputs
    carInputsSub = node.subscribe("/mad/carinputs", 2 * CarParameters::p()->carCnt,
                                  &CarSimNode::carInputsCallback, this);
    //create publisher for /mad/caroutputs
    carOutputsPub = node.advertise<madmsgs::CarOutputs>("/mad/caroutputs", 1);
    //create publisher for /mad/car0/sim/caroutputsext (for simulation)
    carOutputsExtPub = node.advertise<madmsgs::CarOutputsExt>("/mad/car0/sim/caroutputsext", 1);
  }

  void step()
  {
    // Get ROS Time for timestamp of ROS messages
    ros::Time time { ros::Time::now() };

    // compute one sampling point of state space model
    ModelType::OutputsType y;
    model.step(u, y, samplingTime);

    // copy the caroutputs signal to the message
    carOutputsMsg.s.at(0) = y.at(0);
    carOutputsMsg.s.at(1) = y.at(1);
    carOutputsMsg.psi = y.at(2);
    carOutputsMsg.header.stamp = time;
    // copy the caroutputsExt signal to the message
    carOutputsExtMsg.s.at(0) = y.at(0);
    carOutputsExtMsg.s.at(1) = y.at(1);
    carOutputsExtMsg.psi = y.at(2);
    carOutputsExtMsg.beta = y.at(3);
    carOutputsExtMsg.v = y.at(4);
    carOutputsExtMsg.x = y.at(5);
    carOutputsExtMsg.header.stamp = time;

    // publish the messages on the topics
    // Downsample /mad/caroutputs message
    // Increment Counter
    counter++;
    if (counter == 10)
    {
      counter = 0;
      carOutputsPub.publish(carOutputsMsg); // /mad/caroutputs
    }
    carOutputsExtPub.publish(carOutputsExtMsg); // /mad/car0/sim/caroutputsext
  }
 private:
  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  ros::Subscriber carInputsSub; /**< The /mad/carinputs topic subscriber */
  ros::Publisher carOutputsPub; /**< The /mad/caroutputs topic publisher */
  ros::Publisher carOutputsExtPub; /**< The /mad/car0/sim/caroutputsext topic publisher */
  madmsgs::CarOutputs carOutputsMsg; /**< The /mad/caroutputs topic message */
  madmsgs::CarOutputsExt carOutputsExtMsg; /**< The /mad/car0/sim/caroutputsext topic message */
  const float samplingTime = 0.0F; /**< The sample time [s] */
  int counter = 0; /**< The counter needed for downsampling */
  ModelType::InputsType u { { 0 , 0.0F, 0.0F} }; /**< The input signal of the model */
  ModelType model {}; /** < The state space model */

  /**
  * @brief callback for /mad/carinputs topic, u is the input to the model
  * @param[in] msg The ROS message ConstPtr
  */
  void carInputsCallback(const madmsgs::CarInputsConstPtr& msg)
  // copy the Carinputs msg to the member variable u
  {
    // cmd input
    if (msg->cmd == madmsgs::CarInputs::CMD_HALT || msg->cmd == madmsgs::CarInputs::CMD_FORWARD ||
       msg->cmd == madmsgs::CarInputs::CMD_REVERSE || msg->cmd == madmsgs::CarInputs::CMD_SLOW)
    {
      u.at(0) = msg->cmd;
    }
    // pedals input
    // If madmsgs::CarInputs::CMD_HALT then pedals input is zero
    if (msg->cmd == madmsgs::CarInputs::CMD_HALT)
    {
      u.at(1) = 0.0F;
    }
    // Else if madmsgs::CarInputs::CMD_FORWARD then pedals input must be positive or zero
    else if (msg->cmd == madmsgs::CarInputs::CMD_FORWARD)
    {
      if (msg->pedals > 1.0F)
      {
        u.at(1) = 1.0F;
      }
      else if (msg->pedals < 0.0F)
      {
        u.at(1) = 0.0F;
      }
      else
      {
        u.at(1) = msg->pedals;
      }
    }
    // Else if madmsgs::CarInputs::CMD_REVERSE then pedals input must be negative or zero
    else if (msg->cmd == madmsgs::CarInputs::CMD_REVERSE)
    {
      if (msg->pedals > 0.0F)
      {
        u.at(1) = 0.0F;
      }
      else if (msg->pedals < -1.0F)
      {
        u.at(1) = -1.0F;
      }
      else
      {
        u.at(1) = msg->pedals;
      }
    }
    // Else if madmsgs::CarInputs::CMD_SLOW then pedals input is value of message
    else if (msg->cmd == madmsgs::CarInputs::CMD_SLOW)
    {
      u.at(1) = msg->pedals;
    }
    // Steering input
    // If steering message is below -1 set it on lower limit: -1
    if (msg->steering < -1.0F)
    {
      u.at(2) = -1.0F;
    }
    // If steering message is greater 1 set it on upper limit: 1
    else if (msg->steering > 1.0F)
    {
      u.at(2) = 1.0F;
    }
    // Else: use message unchanged as steering input
    else
    {
      u.at(2) = msg->steering;
    }
  }
 };

  int main(int argc, char **argv)
  {
    const float samplingTime = 2e-3F; // the constant sample time [ s ]
    ros::init(argc, argv, "carsim_node"); // initialize ROS
    CarSimNode node(samplingTime);
    // define sampling rate as the inverse of the sample time
    ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
    // loop while ROS is running
    while (ros::ok())
    {
      // call the method step() of the CarSimNode instance node
      node.step();
      // pass control to ROS for background tasks
      ros::spinOnce();
      // wait for next sampling point
      // neighbor sampling points have a time distance of 2ms
      loopRate.sleep();
    }
    // return success
    return EXIT_SUCCESS;
  }
