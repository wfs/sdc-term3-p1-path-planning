//
// Created by andrew on 14/08/17.
//

#ifndef PATH_PLANNING_EGO_VEHICLE_HPP
#define PATH_PLANNING_EGO_VEHICLE_HPP


class ego_vehicle {

    double car_x;
public:
    double getCar_x() const;

    void setCar_x(double car_x);

    double getCar_y() const;

    void setCar_y(double car_y);

    double getCar_s() const;

    void setCar_s(double car_s);

    double getCar_d() const;

    void setCar_d(double car_d);

    double getCar_yaw() const;

    void setCar_yaw(double car_yaw);

    double getCar_speed() const;

    void setCar_speed(double car_speed);

private:
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

public:
    ego_vehicle();

    virtual ~ego_vehicle();



};


#endif //PATH_PLANNING_EGO_VEHICLE_HPP
