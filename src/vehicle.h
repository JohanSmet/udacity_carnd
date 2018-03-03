#ifndef CARND_VEHICLE_H
#define CARND_VEHICLE_H

class Vehicle {

  // interface functions
  public:
    // construction
    Vehicle();
    Vehicle(int id, double s, double d, double x_vel, double y_vel);
    Vehicle(double s, double d, double speed);

    // information
    int id() const {return m_id;}
    double s() const {return m_s;}
    double d() const {return m_d;}
    double speed() const {return m_speed;}

    // prediction
    void predict(double delta_t);
    void update(double s, double d, double speed);

  // member variables
  private:
    int m_id;
    double m_s;
    double m_d;
    double m_speed;
};

#endif // CARND_VEHICLE_H