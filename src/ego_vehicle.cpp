//
// Created by andrew on 14/08/17.
//
#include <vector>

#include "ego_vehicle.hpp"

ego_vehicle::ego_vehicle() {}

ego_vehicle::~ego_vehicle() {

}

double ego_vehicle::getX() const {
    return car_x;
}

void ego_vehicle::setX(double car_x) {
    ego_vehicle::car_x = car_x;
}

double ego_vehicle::getY() const {
    return car_y;
}

void ego_vehicle::setY(double car_y) {
    ego_vehicle::car_y = car_y;
}

double ego_vehicle::getS() const {
    return car_s;
}

void ego_vehicle::setS(double car_s) {
    ego_vehicle::car_s = car_s;
}

double ego_vehicle::getD() const {
    return car_d;
}

void ego_vehicle::setD(double car_d) {
    ego_vehicle::car_d = car_d;
}

double ego_vehicle::getYaw() const {
    return car_yaw;
}

void ego_vehicle::setYaw(double car_yaw) {
    ego_vehicle::car_yaw = car_yaw;
}

double ego_vehicle::getSpeed() const {
    return car_speed;
}

void ego_vehicle::setSpeed(double car_speed) {
    ego_vehicle::car_speed = car_speed;
}

double ego_vehicle::getRef_pos_x() const {
    return ref_pos_x;
}

void ego_vehicle::setRef_pos_x(double ref_pos_x) {
    ego_vehicle::ref_pos_x = ref_pos_x;
}

double ego_vehicle::getRef_pos_y() const {
    return ref_pos_y;
}

void ego_vehicle::setRef_pos_y(double ref_pos_y) {
    ego_vehicle::ref_pos_y = ref_pos_y;
}

double ego_vehicle::getRef_pos_yaw() const {
    return ref_pos_yaw;
}

void ego_vehicle::setRef_pos_yaw(double ref_pos_yaw) {
    ego_vehicle::ref_pos_yaw = ref_pos_yaw;
}


//const vector<double> &ego_vehicle::getPrevious_path_x() const {
//    return previous_path_x;
//}

//void ego_vehicle::setPrevious_path_x(const vector<double> &previous_path_x) {
//    ego_vehicle::previous_path_x = previous_path_x;
//}

//const vector<double> &ego_vehicle::getPrevious_path_y() const {
//    return previous_path_y;
//}

//void ego_vehicle::setPrevious_path_y(const vector<double> &previous_path_y) {
//    ego_vehicle::previous_path_y = previous_path_y;
//}


//double ego_vehicle::getEnd_path_s() const {
//    return end_path_s;
//}

//void ego_vehicle::setEnd_path_s(double end_path_s) {
//    ego_vehicle::end_path_s = end_path_s;
//}

//double ego_vehicle::getEnd_path_d() const {
//    return end_path_d;
//}

//void ego_vehicle::setEnd_path_d(double end_path_d) {
//    ego_vehicle::end_path_d = end_path_d;
//}
