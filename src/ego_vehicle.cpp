//
// Created by andrew on 14/08/17.
//

#include "ego_vehicle.hpp"

ego_vehicle::ego_vehicle() {}

ego_vehicle::~ego_vehicle() {

}

double ego_vehicle::getCar_x() const {
    return car_x;
}

void ego_vehicle::setCar_x(double car_x) {
    ego_vehicle::car_x = car_x;
}

double ego_vehicle::getCar_y() const {
    return car_y;
}

void ego_vehicle::setCar_y(double car_y) {
    ego_vehicle::car_y = car_y;
}

double ego_vehicle::getCar_s() const {
    return car_s;
}

void ego_vehicle::setCar_s(double car_s) {
    ego_vehicle::car_s = car_s;
}

double ego_vehicle::getCar_d() const {
    return car_d;
}

void ego_vehicle::setCar_d(double car_d) {
    ego_vehicle::car_d = car_d;
}

double ego_vehicle::getCar_yaw() const {
    return car_yaw;
}

void ego_vehicle::setCar_yaw(double car_yaw) {
    ego_vehicle::car_yaw = car_yaw;
}

double ego_vehicle::getCar_speed() const {
    return car_speed;
}

void ego_vehicle::setCar_speed(double car_speed) {
    ego_vehicle::car_speed = car_speed;
}
