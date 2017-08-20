#include <math.h>
#include "traffic.hpp"

traffic::traffic() {}

traffic::~traffic() {

}

double traffic::getVx() const {
    return vx;
}

void traffic::setVx(double vx) {
    traffic::vx = vx;
}

double traffic::getVy() const {
    return vy;
}

void traffic::setVy(double vy) {
    traffic::vy = vy;
}

double traffic::getS() const {
    return check_car_s;
}

void traffic::setS(double car_s) {
    traffic::check_car_s = car_s;
}

float traffic::getD() const {
    return d;
}

void traffic::setD(float d) {
    traffic::d = d;
}

/**
 * speed = sqrt(distance^2 / time) * m/s
 * See speed formula at top of Acceleration defintion from link below :
 * http://mathworld.wolfram.com/Acceleration.html
 */
double traffic::checkSpeed() const {
    return sqrt(traffic::getVx() * traffic::getVx() + traffic::getVy() * traffic::getVy());
}

void traffic::setCheck_car_s(double check_car_s) {
    traffic::check_car_s = check_car_s;
}

double traffic::calculateS(double prev_traj_path_list_size, double move_to_next_waypoint_in_secs) {
    return (prev_traj_path_list_size * move_to_next_waypoint_in_secs * traffic::checkSpeed());
}