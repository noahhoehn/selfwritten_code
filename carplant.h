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
  * @file carplant.h
  */

#ifndef CARPLANT_H
#define CARPLANT_H

#pragma once
#include <array>
#include <deque>
#include <string>
#include <boost/numeric/odeint.hpp>
#include <cmath>
#include "CarParameters.h"
/**
 * @brief The CarPlant class
 */
class CarPlant
{
public:
  using InputsType = std::array<float, 3>; /**< type definition for model input */
  using StatesType = std::array<float, 7>; /**< type definition for model state */
  using OutputsType = std::array<float, 6>; /**< type definition for model output */
  /**
   * @brief Only constructor
   */
  CarPlant()
  {
    // initialize model state vector x
    x = p->x0;
    // initialize model input vector u
    u.fill(0.0F);
    // initialize dead time delay
    const std::size_t Tt = static_cast<std::size_t> (p->uTt/p->Ta);
    // initialize deques for dead time on inputs u1, u2
    u_1_delayed.resize(Tt, 0.0F);
    u_2_delayed.resize(Tt, 0.0F);
  }

  /**
  * @brief execute one single integration step
  * @param[in] u input vector
  * @param[out] y output vector
  * @param[in] dt sample time
  */
  void step(const InputsType& u, OutputsType& y, const float dt)
  {
    // Copy Input signals u1 and u2 to dead time delay deque
    u_1_delayed.push_back(u.at(1));
    u_2_delayed.push_back(u.at(2));

    // one step of Runge Kutta
    // *this is this object, Runge Kutta calls this object as a functor
    // which means that the operator() is called at every
    // (major and minor) integration step
    solver.do_step(*this, x, t, dt);
    // Get output of state space model
    y.at(0) = x.at(1) - p->lr*std::cos(x.at(3)); //s.at(0)
    y.at(1) = x.at(2) - p->lr*std::sin(x.at(3)); //s.at(1)
    y.at(2) = x.at(3); //psi
    // If speed is lower than 0.2 m/s: use simple longitudinal dynamics model
    if (std::abs(x.at(0)) < p->speedMin)
    {
      y.at(3) = std::atan(p->lr/p->l * std::tan(p->deltaMax*u_2_delayed.at(0))); //beta
    }
    // Else: use more complex longitudinal dynamics model
    else
    {
      y.at(3) = std::atan2(x.at(5),x.at(0)); //beta
    }
    y.at(4) = x.at(0); //v_c1 oder v_r
    y.at(5) = x.at(6); //x
    t += dt; // increase simulation time by sample time
    // Delete first elements in dead time delay deque of u1 and u2
    u_1_delayed.pop_front();
    u_2_delayed.pop_front();
  }
  /**
  * @brief operator () makes this class to a functor.
  * This operator is called by Runge Kutta internally.
  * @param[in] x the current state vector
  * @param[out] xd the current differential of the state vector
  * @param[in] t the current simulation time
  */
  void operator()(const StatesType& x, StatesType& xd, float t)
  {
    // If speed of car is lower than 0.2 m/s: use simple longitudinal dynamics model
    if (std::abs(x.at(0)) < p->speedMin)
    {
      // Calculate needed Variables
      const float beta = std::atan(p->lr/p->l * std::tan(p->deltaMax*u_2_delayed.at(0)));
      // State Space Model
      xd.at(0) = -1.0F/p->T*x.at(0) + p->k/p->T*u_1_delayed.at(0);
      xd.at(1) = x.at(0)/std::cos(beta) * std::cos(x.at(3)+beta);
      xd.at(2) = x.at(0)/std::cos(beta) * std::sin(x.at(3)+beta);
      xd.at(3) = x.at(0)/p->l * std::tan(p->deltaMax*u_2_delayed.at(0));
      xd.at(4) = 0.0F;
      xd.at(5) = 0.0F;
      xd.at(6) = x.at(0);
    }
    // Else: use more complex longitudinal dynamics model
    else
    {
      // Calculate needed Variables
      const float delta = p->deltaMax * u_2_delayed.at(0);
      const float psiDot = x.at(4);
      float alphaF = 0.0F;
      float alphaR = 0.0F;
      // If speed of car is positive
      if (x.at(0) > 0.0F)
      {
        alphaF = -std::atan((x.at(5) + p->lf*psiDot) / x.at(0)) + delta;
        alphaR = -std::atan((x.at(5) - p->lr*psiDot) / x.at(0));
      }
      // If speed of car is negative
      else if (x.at(0) < 0.0F)
      {
        alphaF = std::atan((x.at(5) + p->lf*psiDot) / x.at(0)) - delta;
        alphaR = std::atan((x.at(5) - p->lr*psiDot) / x.at(0));
      }
      const float Ff = p->Df * std::sin(p->Cf * std::atan(p->Bf*(1-p->Ef)*alphaF - p->Ef*std::atan(p->Bf*alphaF)));
      const float Fr = p->Dr * std::sin(p->Cr * std::atan(p->Br*(1-p->Er)*alphaR - p->Er*std::atan(p->Br*alphaR)));
      // State Space Model
      xd.at(0) = (-Ff*sin(delta) + p->m*x.at(5)*psiDot) / p->m - x.at(0)/p->T + p->k/p->T * u_1_delayed.at(0);
      xd.at(1) = x.at(0)*std::cos(x.at(3)) - x.at(5)*std::sin(x.at(3));
      xd.at(2) = x.at(0)*std::sin(x.at(3)) + x.at(5)*std::cos(x.at(3));
      xd.at(3) = x.at(4);
      xd.at(4) = (Ff*p->lf*std::cos(delta) - Fr*p->lr) / p->J;
      xd.at(5) = (Ff*std::cos(delta) + Fr - p->m*x.at(0)*psiDot) / p->m;
      xd.at(6) = x.at(0);
    }

  }
private:
  boost::numeric::odeint::runge_kutta4<StatesType> solver; /**< the Runge Kutta solver parameterized with states vector type */
  InputsType u; /**< input signal */
  StatesType x; /**< state signal */
  CarParameters* p = CarParameters::p(); /**< Pointer to needed parameters in madlib/CarParameters.h */
  float t = 0.0F; /**< simulation time */
  std::deque<float> u_1_delayed; /**< deque needed for dead time delay of input signal pedals */
  std::deque<float> u_2_delayed; /**< deque needed for dead time delay of input signal steering */
};
#endif // CARPLANT_H
