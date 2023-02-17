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
  * @file pathcontroller.h
  */

#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#pragma once
#include <array>
#include <string>
#include <cmath>
#include <vector>
#include "CarParameters.h"
#include "Spline.h"
/**
 * @brief The PathController class
 */

class PathController
{
public:

  /**
   * @brief Only constructor
   */
  PathController(){}
  /**
   * @brief init generates spline needed for Pathfollowing Control
   * @param[in] breaks List of arc lengths of waypoints [ m ]
   * @param[in] s1 List of x-coordinates of waypoints [ m ]
   * @param[in] s2 List of y-coordinates of waypoints [ m ]
   * @param[in] segments Id of segment at every waypoint
   * @param[out] createdSpline generated and interpolated spline with waypoints
   */
   void init(const std::vector<float>& breaks, const std::vector<float>& s1,
             const std::vector<float>& s2, const std::vector<uint32_t>& segments,
             modbas::Spline &createdSpline)
  {
    createdSpline = modbas::Spline(breaks, s1, s2, segments);
  }

  /**
   * @brief stepRefSig generates Reference Signals for Pathfollowing Control
   * @param[in] usedSpline Created Spline as Class Object modbas::Spline
   * @param[in] s Real cartesian coordinates of model car
   * @param[in] vs Reference Speed used for calculation of future points on spline
   * @param[out] nearest Index of nearest waypoint on spline (needed in carctrl_node for laptime calculation)
   * @return Array containing generated reference signals (s1Xs, s2Xs, psiXs, kappaXHat)
   */
  std::array<float, 4> stepRefSig(modbas::Spline usedSpline,
                                 const std::array<float,2>& s, float vs, int& nearest)
  {
    // Create Output array (s1Xs, s2Xs, psiXs, kappaXHat)
    std::array<float, 4> outputRefSig;
    // Calculate xs (Book 9.1)
    float xs = 0.0F;
    float dist = 0.0F;
    nearest = usedSpline.getNearest(s, xs, dist);
    // Calculate s, sd, sdd from xs
    std::array<float, 2> sXs;
    std::array<float, 2> sdXs;
    std::array<float, 2> sddXs;
    usedSpline.interpolate(xs, sXs, sdXs, sddXs, nearest);
    // Fill in s1Xs and s2Xs
    outputRefSig.at(0) = sXs.at(0); //s1 Reference Position Coordinate
    outputRefSig.at(1) = sXs.at(1); //s2 Reference Position Coordinate
    // Calculate psiXs
    float psiXs = std::atan2(sdXs.at(1), sdXs.at(0));
    // Fill in psiXs
    outputRefSig.at(2) = psiXs;
    // Calculate xHat (Book 9.1, P.253)
    float xHat = xs + vs*p->Tt;
    // Calculate s, sd, sdd from xHat
    std::array<float, 2> sXHat;
    std::array<float, 2> sdXHat;
    std::array<float, 2> sddXHat;
    usedSpline.interpolate(xHat, sXHat, sdXHat, sddXHat);
    // Calculate kappaXHat (Question to sddXs or sddXHat)
    const float tsNs = sdXHat.at(0)*sddXHat.at(1) - sddXHat.at(0)*sdXHat.at(1);
    float kappaXhat = std::sqrt(std::pow(sddXHat.at(0), 2.0F) +
                                std::pow(sddXHat.at(1), 2.0F));
    if (tsNs < 0)
    {
      outputRefSig.at(3) = -kappaXhat;
    }
    else
    {
      outputRefSig.at(3) = kappaXhat;
    }
    return outputRefSig;
  }

  /**
   * @brief stepStateCtrl generates Part deltaE of steering signal
   * @param[in] sXs Reference position of model car in cartesian coordinates [m]
   * @param[in] sReal Real position of model car in cartesian coordinates [m]
   * @param[in] vs Reference speed used for calculation of future points on spline
   * @param[in] psiXs Reference angle of model car
   * @param[in] psiReal Real angle of model car
   * @return Part of steering control signal deltaE
   */
  float stepStateCtrl(const std::array<float, 2>& sXs, const std::array<float, 2>& sReal,
                      const float vs, const float psiXs, const float psiReal)
  {
    // Create return float variable
    float deltaE = 0.0F;
    // Calculate Control Deviation (9.2; P.256 + P.257)
    std::array<float, 3> xe = calcCtrlDev(sXs, sReal, psiXs, psiReal);
    // Calculate state control vector kT(vs) for deltaE (9.23, 9.24, 9.20; P.262)
    std::array<float, 2> kTVs;
    // Limit vs
    const float ve = 0.1F;
    float vsMod;
    vsMod = vs;
    if (vs <= ve && vs >= 0.0F)
    {
      vsMod = ve;
    }
    else if (vs >= -ve && vs <= 0.0F)
    {
      vsMod = -ve;
    }
    // Calculate State control vector
    kTVs.at(0) = p->l/(std::pow(p->pathTw, 2.0F)*std::pow(vsMod, 2.0F));
    kTVs.at(1) = 2.0F*p->l/(p->pathTw*vsMod);
    // Calculate deltaE (steering output)
    deltaE = -kTVs.at(0)*xe.at(1) - kTVs.at(1)*xe.at(2);
    return deltaE;
  }

  /**
   * @brief stepFeedForCtrl generates Part deltaS of steering signal
   * @param[in] kappaXHat Curvature (Krümmung) of interpolated waypoint in future
   * @return Part of steering control signal deltaS
   */
  float stepFeedForCtrl(const float kappaXHat)
  {
    // Calculate deltaS (steering output)
    float deltaS = std::atan(p->l*kappaXHat);
    return deltaS;
  }

  /**
   * @brief stepCtrlSumm calculates and optionally limits steering signal
   * @param[in] deltaE steering signal part given by State Controller
   * @param[in] deltaS steering signal part given by Non-Linear Feedforward Control
   * @return Steering control signal
   */
  float stepCtrlSum(const float deltaE, const float deltaS)
  {
    // Calculate steering signal
    float delta = (deltaE + deltaS)/p->deltaMax;
    // Limit steering signal between -1 and 1
    if (delta >= 1.0F)
    {
      delta = 1.0F;
    }
    else if (delta <= -1.0F)
    {
      delta = -1.0F;
    }
    return delta;
  }

  /**
   * @brief calcCtrlDev calculates vector of control deviation of Pathfollowing Control
   * @param[in] sXs Reference position of model car in cartesian coordinates [m]
   * @param[in] sReal Real position of model car in cartesian coordinates [m]
   * @param[in] psiXs Reference angle of model car
   * @param[in] psiReal Real angle of model car
   */
  std::array<float, 3> calcCtrlDev(const std::array<float, 2>& sXs,
                                   const std::array<float, 2>& sReal,
                                   const float psiXs, const float psiReal)
  {
    std::array<float, 3> xe; //Vector of control deviation (Regelabweichung)
    xe.at(0) = 0.0F; //sc1e (9.7; P257)
    xe.at(1) = (sReal.at(1)-sXs.at(1))*std::cos(psiXs)
                - (sReal.at(0)-sXs.at(0))*std::sin(psiXs); //sc2e (9.6; P256)
    xe.at(2) = modbas::Utils::normalizeRad(psiReal - psiXs); //psi(xs) (9.8; P257)
    return xe;
  }

  /**
   * @brief step combines all other Pathfollowing control functions in one
   * @param[in] spline Created Spline as Class Object modbas::Spline
   * @param[in] sReal Real cartesian coordinates of model car
   * @param[in] vs Reference Speed used for calculation of future points on spline
   * @param[in] psiReal Real angle of model car
   * @param[out] nearest Index of nearest waypoint on spline (needed in carctrl_node for laptime calculation)
   * @return Ready-to-use steering signal input for /mad/carinputs message
   */
  float step(modbas::Spline spline, const std::array<float, 2>& sReal, float vs, float psiReal, int& nearest)
  {
    // Do Reference Signal Generation (s1(xs), s2(xs), psi(xs), kappa(xHat))
    std::array<float, 4> refPathSigs = stepRefSig(spline, sReal, vs, nearest);
    std::array<float, 2> sXs;
    sXs.at(0) = refPathSigs.at(0);
    sXs.at(1) = refPathSigs.at(1);
    const float psiXs = refPathSigs.at(2);
    const float kappaXHat = refPathSigs.at(3);
    // Do State Controller Step
    const float deltaE = stepStateCtrl(sXs, sReal, vs, psiXs, psiReal);
    // Do Nonlinear Feed Forward Control
    const float deltaS = stepFeedForCtrl(kappaXHat);
    // Sum up and limit steering output
    const float delta = stepCtrlSum(deltaE, deltaS);
    // Return steering input
    return delta;
  }



private:
  // IMPORTANT: Controller Parameters are stored in UPDATED CarParameters.h file
  CarParameters* p = CarParameters::p(); /**< Pointer to needed parameters in madlib/CarParameters.h */
};

#endif // PATHCONTROLLER_H
