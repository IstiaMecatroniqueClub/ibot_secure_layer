#ifndef IBOTSTATE_H
#define IBOTSTATE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <iostream>

#include "define.h"
#include "ibot_secure_layer/ultrasonic.h"
#include "ibot_secure_layer/powersupply.h"

#include <cstdlib>

class IbotState
{
public:
  IbotState(ros::NodeHandle& node);

  void  updateState();

  float us_left() const;
  float us_right() const;
  float us_front() const;
  float us_bottom() const;
  float power_state() const;

private:
  uint8_t   m_st;
  bool      m_safety_st;
  double                m_seuil;

  ros::Subscriber       m_request_twist;
  ros::Publisher        m_secure_twist;
  geometry_msgs::Twist  m_requested_twist, m_stop_twist;


  ros::ServiceClient    m_service_us_left;
  ros::ServiceClient    m_service_us_right;
  ros::ServiceClient    m_service_us_front;
  ros::ServiceClient    m_service_us_bottom;
  float                 m_us_left, m_us_right, m_us_front, m_us_bottom;

  float                 m_power_state;

  ibot_secure_layer::ultrasonic m_us_service_data;

  bool                  isValidState(const geometry_msgs::Twist twist);

  void                  callback_cmdvel(geometry_msgs::Twist twist);

};

#endif // IBOTSTATE_H
