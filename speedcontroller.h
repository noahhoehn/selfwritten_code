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
  * @file speedcontroller.h
  */

#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#pragma once
#include <array>
#include <deque>
#include <string>
#include <cmath>
#include "CarParameters.h"
/**
 * @brief The SpeedController class
 */

class SpeedController
{
public:

  /**
   * @brief Only constructor
   */
  SpeedController(){}

  /**
   * @brief step Speed Control (PI-Controller)
   * @param[in] refSpeed Reference Speed
   * @param[in] realSpeed Real Speed of model car (or simulation)
   * @param[in] samplingTime Sampling Time from ROS node carctrl_node
   * @return Pedals signal for model car in range [-1, 1]
   */
  float step(float refSpeed, float realSpeed, float samplingTime)
  {
    // Calculate Error between real and reference speed
    const float e_k = refSpeed - realSpeed;
    // Calculate u_ki (Integrat
    const float u_ik = u_ikMinus1 + p->speedKp * samplingTime * e_k / p->speedTi;
    // Calculate u_kp
    const float u_pk = p->speedKp * e_k;
    // Calculate output: u_k
    float u_k = u_ik + u_pk;

    // Clamping-Anti-Windup
    // If u_k is greater than 1.0: set u_k to 1.0 and do NO integration
    if (u_k > 1.0F)
    {
      u_k = 1.0F;
    }
    // If u_k is smaller than -1.0: set u_k to -1.0 and do NO integration
    else if (u_k < -1.0F)
    {
      u_k = -1.0F;
    }
    // Else: Do integration and do NOT limit u_k
    else
    {
      u_ikMinus1 = u_ik;
    }
    return u_k;
  }
private:
  // IMPORTANT: Controller Parameters are stored in UPDATED CarParameters.h file
  CarParameters* p = CarParameters::p(); /**< Pointer to needed parameters in madlib/CarParameters.h */
  float u_ikMinus1 = 0.0F; /**< Old value of integrational part of feedforward control signal u_ik */
};

#endif // SPEEDCONTROLLER_H
