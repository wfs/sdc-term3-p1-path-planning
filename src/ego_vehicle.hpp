//
// Created by andrew on 14/08/17.
//

#ifndef PATH_PLANNING_EGO_VEHICLE_HPP
#define PATH_PLANNING_EGO_VEHICLE_HPP

#include <vector>

using namespace std;

class ego_vehicle {

public:
    double getX() const;

    void setX(double car_x);

    double getY() const;

    void setY(double car_y);

    double getS() const;

    void setS(double car_s);

    double getD() const;

    void setD(double car_d);

    double getYaw() const;

    void setYaw(double car_yaw);

    double getSpeed() const;

    void setSpeed(double car_speed);


private:
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
    double ref_pos_x;
public:
    double getRef_pos_x() const;

    void setRef_pos_x(double ref_pos_x);

    double getRef_pos_y() const;

    void setRef_pos_y(double ref_pos_y);

    double getRef_pos_yaw() const;

    void setRef_pos_yaw(double ref_pos_yaw);

private:
    double ref_pos_y;
    double ref_pos_yaw;


public:
    ego_vehicle();

    virtual ~ego_vehicle();


};


#endif //PATH_PLANNING_EGO_VEHICLE_HPP
