#ifndef UDACITY_MAP_H
#define UDACITY_MAP_H
// map handling code extracted from original main.cpp

#include <vector>
#include <cmath>

inline double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

class Map {
  public:
    // construction 
    Map();
    void load_from_file(const char *p_filename);

    // waypoints
    int ClosestWaypoint(double x, double y);
    int NextWaypoint(double x, double y, double theta);

    // frenet
    std::vector<double> getFrenet(double x, double y, double theta);
    std::vector<double> getXY(double s, double d);

  public:
    // fixed dimensions of a lane
    const static int LANE_HALF_WIDTH = 2;
    const static int LANE_WIDTH = LANE_HALF_WIDTH * 2;

    // The max s value before wrapping around the track back to 0
    constexpr static double max_s = 6945.554;

  private:
    std::vector<double> m_waypoints_x;
    std::vector<double> m_waypoints_y;
    std::vector<double> m_waypoints_s;
    std::vector<double> m_waypoints_dx;
    std::vector<double> m_waypoints_dy;

};

#endif // UDACITY_MAP_H