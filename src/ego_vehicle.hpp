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


    //const vector<double> &getPrevious_path_x() const;

    //void setPrevious_path_x(const vector<double> &previous_path_x);

    //const vector<double> &getPrevious_path_y() const;

    //void setPrevious_path_y(const vector<double> &previous_path_y);


    //double getEnd_path_s() const;

    //void setEnd_path_s(double end_path_s);

    //double getEnd_path_d() const;

    //void setEnd_path_d(double end_path_d);



private:
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
    //vector<double> previous_path_x;
    //vector<double> previous_path_y;
    //double end_path_s;
    //double end_path_d;


public:
    ego_vehicle();

    virtual ~ego_vehicle();



};


#endif //PATH_PLANNING_EGO_VEHICLE_HPP
