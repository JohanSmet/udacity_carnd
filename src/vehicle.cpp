#include "vehicle.h"

#include <cmath>

Vehicle::Vehicle() :
    m_id(0),
    m_s(0),
    m_d(0) ,
    m_speed(0) {
}

Vehicle::Vehicle(int id, double s, double d, double x_vel, double y_vel) :
    m_id(id),
    m_s(s),
    m_d(d) {
  m_speed = std::sqrt((x_vel * x_vel) + (y_vel * y_vel));
}

Vehicle::Vehicle(double s, double d, double speed) :
  m_id(-1),
  m_s(s),
  m_d(d),
  m_speed(speed) {
}

void Vehicle::predict(double delta_t) {
  // assume linear velocity along the current lane
  m_s += delta_t * m_speed;
}