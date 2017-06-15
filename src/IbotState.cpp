#include "IbotState.h"

using namespace ros;
using namespace geometry_msgs;
using namespace ibot_secure_layer;

IbotState::IbotState(ros::NodeHandle &node)
  : m_st(0),
    m_us_left(0.0f), m_us_right(0.0f),m_us_front(0.0f), m_us_bottom(0.0f),
    m_power_state(0.0f)
{
  std::string name;

  if(!node.getParam("topic_secure_vel",name))
    name = TOPIC_OUTPUT_VEL;
  m_secure_twist = node.advertise<Twist>(name,10);

  if(!node.getParam("topic_cmd_vel",name))
    name = TOPIC_INPUT_VEL;
  m_request_twist = node.subscribe(name,10,&IbotState::callback_cmdvel,this);

  if(!node.getParam("can_us_left_service",name))
    name = SERVICE_US_LEFT;
  m_service_us_left = node.serviceClient<ultrasonic>(name);

  if(!node.getParam("can_us_right_service",name))
    name = SERVICE_US_RIGHT;
  m_service_us_right = node.serviceClient<ultrasonic>(name);

  if(!node.getParam("can_us_front_service",name))
    name = SERVICE_US_FRONT;
  m_service_us_front = node.serviceClient<ultrasonic>(name);

  if(!node.getParam("can_us_bottom_service",name))
    name = SERVICE_US_GROUND;
  m_service_us_bottom = node.serviceClient<ultrasonic>(name);

  if(!node.getParam("obstacle_threshold",name))
    name = OBSTACLE_THRESHOLD;
  m_seuil = std::stod(name);

  m_us_service_data.request.unused = 1;
  m_stop_twist.linear.x = 0.0f; m_stop_twist.angular.z = 0.0f;
}

void IbotState::updateState()
{

  switch(m_st)
  {
  case 0 :
    if(m_service_us_front.call(m_us_service_data))
    {
      m_us_front = m_us_service_data.response.range;
      if(m_us_front < 0) {
        ROS_WARN("Front Warn value %f",m_us_front);
        if(m_us_front == -3.0f) m_us_front = 10.0f;
      } else {
        ROS_INFO("Front %2.2f",m_us_front);
      }
      m_st = 10;
    } else {
      ROS_ERROR("Error when call front");
    }
    break;
  case 10 :
    if(m_service_us_right.call(m_us_service_data))
    {
      m_us_right = m_us_service_data.response.range;
      if(m_us_right < 0) {
        ROS_WARN("Right Warn value %f",m_us_right);
        if(m_us_right == -3.0f) m_us_right = 10.0f;
      } else {
        ROS_INFO("Right %2.2f",m_us_right);
      }
      m_st = 20;
    } else {
      ROS_ERROR("Error when call ibot_us_right");
    }
    break;
  case 20 :
    if(m_service_us_left.call(m_us_service_data))
    {
      m_us_left = m_us_service_data.response.range;
      if(m_us_left < 0) {
        ROS_WARN("Left Warn value %f",m_us_left);
        if(m_us_left == -3.0f) m_us_left = 10.0f;
      } else {
        ROS_INFO("Left %2.2f",m_us_left);
      }
      m_st = 0;
    } else {
      ROS_ERROR("Error when call ibot_us_left");
    }
    break;
  }
  bool st = isValidState(m_requested_twist);
  if(st != m_safety_st)
    if(st)
    {
      m_secure_twist.publish(m_requested_twist);
      ROS_INFO("STATE IS GOOD");
    }
      else
    {
      m_secure_twist.publish(m_stop_twist);
      ROS_ERROR("STATE IS BAD -> Motors will be stop");
    }
  m_safety_st = st;
}

float IbotState::us_left() const
{
  return m_us_left;
}

float IbotState::us_right() const
{
  return m_us_right;
}

float IbotState::us_front() const
{
  return m_us_front;
}

float IbotState::us_bottom() const
{
  return m_us_bottom;
}

float IbotState::power_state() const
{
  return m_power_state;
}

bool IbotState::isValidState(const Twist twist)
{
  // If request go front and obstacle OR no data, then bad state
  if(twist.linear.x  > 0.0f && (m_us_front < m_seuil || m_us_front <= 0.0f))  return false;
  if(twist.angular.z > 0.0f && (m_us_left  < m_seuil || m_us_left  <= 0.0f))  return false;
  if(twist.angular.z < 0.0f && (m_us_right < m_seuil || m_us_left  <= 0.0f))  return false;
  return true;
}

void IbotState::callback_cmdvel(Twist twist)
{
  ROS_INFO("New Twist with linear %2.2f and angular %2.2f",twist.linear.x,twist.angular.z);
  m_requested_twist = twist;
  m_safety_st = isValidState(m_requested_twist);
  if(m_safety_st)     m_secure_twist.publish(m_requested_twist);
  else                m_secure_twist.publish(m_stop_twist);
}

