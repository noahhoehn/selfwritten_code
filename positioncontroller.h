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
  * @file positioncontroller.h
  */

#ifndef POSITIONCONTROLLER_H
#define POSITIONCONTROLLER_H

#pragma once
#include <array>
#include <deque>
#include <string>
#include <cmath>
#include "CarParameters.h"
/**
 * @brief The PositionController class
 */

class PositionController
{
public:
  using CoeffType = std::array<float, 6>; /**< type definition for coefficient arrays */
  /**
   * @brief Only constructor
   */
  PositionController(){}

  /**
   * @brief calcTe Calculate time te of position control
   * @param[in] xs Reference position arc length
   * @param[in] vmax Reference speed
   * @return Time te of position control maneuver
   */
  float calcTe(float xs, float vmax)
  {
    // Book
    float te = 15.0F*xs / (8.0F*vmax);
    return te;
  }

  /**
   * @brief calcC Calculates polynomial coefficients of lead signal wp
   * @param[in] x0 Start position arc length (begin of maneuver)
   * @param[in] xs Reference position arc length
   * @param[in] vmax Reference speed
   * @return polynomial coefficients of lead signal wp
   */
  CoeffType calcC(float x0, float xs, float vmax)
  {
    CoeffType c;
    c.at(0) = x0;
    c.at(1) = 0.0F;
    c.at(2) = 0.0F;
    c.at(3) = 1024.0F*std::pow(vmax, 3.0F) / (675.0F*std::pow(xs, 2.0F));
    c.at(4) = -4096.0F*std::pow(vmax, 4.0F) / (3375.0F*std::pow(xs, 3.0F));
    c.at(5) = 65536.0F*std::pow(vmax, 5.0F) / (253125.0F*std::pow(xs, 4.0F));

    return c;
  }

  /**
   * @brief calcCff Calculates polynomial coefficients of feedforward control signal uVp1
   * @param[in] c Coefficients of polynom wp
   * @return polynomial coefficients of feedforward control signal uVp1
   */
  CoeffType calcCff(CoeffType c)
  {
    CoeffType cff;
    cff.at(0) = c.at(0) + p->speedTi*c.at(1)*(1.0F/(p->k*p->speedKp) + 1.0F)
                + 2.0F*p->speedTi*c.at(2)*(p->T + p->Tt)/(p->k*p->speedKp) + 6.0F*p->T*p->speedTi*p->Tt*c.at(3)/(p->k*p->speedKp);
    cff.at(1) = c.at(1) + 2.0F*p->speedTi*c.at(2)*(1.0F/(p->k*p->speedKp) + 1.0F)
                + 6.0F*p->speedTi*c.at(3)*(p->T + p->Tt)/(p->k*p->speedKp) + 24.0F*p->T*p->speedTi*p->Tt*c.at(4)/(p->k*p->speedKp);
    cff.at(2) = c.at(2) + 3.0F*p->speedTi*c.at(3)*(1.0F/(p->k*p->speedKp) + 1.0F)
                + 12.0F*p->speedTi*c.at(4)*(p->T + p->Tt)/(p->k*p->speedKp) + 60.0F*p->T*p->speedTi*p->Tt*c.at(5)/(p->k*p->speedKp);
    cff.at(3) = c.at(3) + 4.0F*p->speedTi*c.at(4)*(1.0F/(p->k*p->speedKp) + 1.0F) + 20.0F*p->speedTi*c.at(5)*(p->T + p->Tt)/(p->k*p->speedKp);
    cff.at(4) = c.at(4) + 5.0F*p->speedTi*c.at(5) * (1.0F/(p->k*p->speedKp) + 1.0F);
    cff.at(5) = c.at(5);

    return cff;
  }

  /**
   * @brief init Combines calcTe, calcC, calcCff (private variables of class object)
   * @param[in] x0 Start position arc length (begin of maneuver)
   * @param[in] xs Reference position arc length
   * @param[in] vmax Reference speed
   */
  void init(float x0, float xs, float vmax)
  {
    te = calcTe(xs, vmax);
    c = calcC(x0, xs, vmax);
    cff = calcCff(c);
  }

  /**
   * @brief step Combines lead and feedforward control signal generation and Controller part
   * @param[in] yp Real position arc length
   * @param[in] x0 Start position arc length (begin of maneuver)
   * @param[in] xs Reference position arc length
   * @param[in] samplingTime Sampling Time from ROS node carctrl_node
   * @param[in] positionRunTime Time since start of parking / position control maneuver
   * @param[out] wp Lead signal which is needed for debugging
   * @param[out] uVp1k Feedforward control signal which is needed for debugging
   * @return Calculated Speed Signal of actual reference speed given to SpeedController in carctrl_node
   */
  float step(float yp, float x0, float xs, float samplingTime, float positionRunTime, float& wp, float& uVp1k)
  {
    // SIGNAL GENERATION PART
    // Generate wp (position lead signal) and uVp1 (feedforward control signal)
    // If time since start of park maneuver is shorter than calculated time te ( see "float calcTe")
    if (positionRunTime <= te)
    {
      // Calculate actual value of wp
      wp = c.at(5)*std::pow(positionRunTime, 5.0F) + c.at(4)*std::pow(positionRunTime, 4.0F) +
          c.at(3)*std::pow(positionRunTime, 3.0F) + c.at(2)*std::pow(positionRunTime, 2.0F) +
          c.at(1)*positionRunTime + c.at(0);
      // Calculate actual value of wp
      uVp1k = cff.at(5)*std::pow(positionRunTime, 5.0F) + cff.at(4)*std::pow(positionRunTime, 4.0F) +
          cff.at(3)*std::pow(positionRunTime, 3.0F) + cff.at(2)*std::pow(positionRunTime, 2.0F) +
          cff.at(1)*positionRunTime + cff.at(0);
    }
    // If time since start of park maneuver is greater than calculated time te ( see "float calcTe")
    else
    {
      // wp is set equal to reference position (+ starting position)
      wp = x0 + xs;
      // uVp1k is set equal to wp (all derivates of wp are zero -> uVp1k is equal wp)
      uVp1k = wp;
    }

    // CONTROLLER PART
    // Calculate Control deviation
    const float error = wp - yp;
    // Calculate P-Controller Value
    float uRpk = p->posKp*error;
    // Calculate feedforward control value
    float uVpk = (2.0F*(uVp1k - uVp1k_Minus1) - (samplingTime - 2.0F*p->speedTi)*uVpk_Minus1) / (samplingTime + 2.0F*p->speedTi);
    // Calculate upk
    const float upk = uRpk + uVpk;

    // Update uVpk_Minus1 and uVpk1_Minus1
    uVpk_Minus1 = uVpk;
    uVp1k_Minus1 = uVp1k;

    // Return Result
    return upk;
  }
private:
  // IMPORTANT: Controller Parameters are stored in UPDATED CarParameters.h file
  CarParameters* p = CarParameters::p(); /**< Pointer to needed parameters in madlib/CarParameters.h */
  float uVpk_Minus1 = 0.0F; /**< Old value of control signal uVp */
  float uVp1k_Minus1 = 0.0F; /**< Old value of control signal uVp1 */
  CoeffType c; /**< Coefficients of lead control signal wp */
  CoeffType cff; /**< Coefficients of feedforward control signal wp */
  float te; /**< Time of position control */
};

#endif // POSITIONCONTROLLER_H
