#ifndef PATH_PLANNING_TRAFFIC_VEHICLES_HPP
#define PATH_PLANNING_TRAFFIC_VEHICLES_HPP


class traffic {

private:
    double vx;  // = sensor_fusion[i][3];
    double vy;  // = sensor_fusion[i][4];
    double check_car_s;
public:
    double getVx() const;

    void setVx(double vx);

    double getVy() const;

    void setVy(double vy);

    double getS() const;

    void setS(double car_s);

    float getD() const;

    void setD(float d);

    double checkSpeed() const;

    double calculateS(double prev_traj_path_list_size, double move_to_next_waypoint_in_secs);

    void setCheck_car_s(double check_car_s);

private:
    float d;  // = sensor_fusion[i][6];


public:
    traffic();

    virtual ~traffic();

};


#endif //PATH_PLANNING_TRAFFIC_VEHICLES_HPP
