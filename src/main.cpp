#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <string>
#include "spline.hpp"  // smoothly fits to all polynomial trajectory points
#include "ego_vehicle.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

/**
 * Calculates Euclidean (aka straight line) distance between 2 points.
 */
double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * Closest waypoint to you, out of all of the available waypoints in the map.
 */
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

/**
 * Nearest waypoint may be behind you but you are interested in the closest waypoint ahead of you.
 * @param theta : car yaw ; helpful to have this when doing the transformation math.
 */
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx,
                 vector<double> maps_dy) {

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    // Heading vector
    double hx = map_x - x;
    double hy = map_y - y;

    // Normal vector (aka Unit Circle quadrant I) - https://www.mathsisfun.com/geometry/unit-circle.html
    double nx = maps_dx[closestWaypoint];
    double ny = maps_dy[closestWaypoint];

    // Vector into the direction of the road (perpendicular to the normal vector)
    // (aka Unit Circle quadrant IV) - https://www.mathsisfun.com/geometry/unit-circle.html
    double vx = -ny;
    double vy = nx;

    // If the inner product of v (vector into direction of road) and h (vehicle heading vector) is positive then we are
    // behind the waypoint so we do not need to increment closestWaypoint, otherwise we are beyond the waypoint and
    // we need to increment closestWaypoint.

    double inner = hx * vx + hy * vy;
    if (inner < 0.0) {
        closestWaypoint++;
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double>
getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx,
          vector<double> maps_dy) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y, maps_dx, maps_dy);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};

}

// Used by Catch BDD-Style test
/*
std::string showMessage() {
    //std::cout << "Hello, World!" << std::endl;
    return "Hello, World!";
}
*/

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);  // Adds a new element at the end of the vector, after its current last element.
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // Start in lane 1
    // 0 == far left lane, 1 == middle lane, 2 == far right lane.
    int lane = 1;
    int lane_change_wp = 0;

    /**
     * LOCALISATION / TELEMETRY (co-ordinates, yaw angle, speed) &
     * SENSOR FUSION (all other cars travelling in our direction) :
     * Simulator tells us the x,y and s,d coordinates along with the car's angle and speed on each message.
     */
    h.onMessage(
            [&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane, &lane_change_wp](
                    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                //auto sdata = string(data).substr(0, length);
                //cout << sdata << endl;
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                    auto s = hasData(data);

                    if (s != "") {
                        auto j = json::parse(s);

                        string event = j[0].get<string>();

                        if (event == "telemetry") {
                            ego_vehicle ego_car;

                            ego_car.setX(j[1]["x"]);
                            ego_car.setY(j[1]["y"]);
                            ego_car.setS(j[1]["s"]);
                            ego_car.setD(j[1]["d"]);
                            ego_car.setYaw(j[1]["yaw"]);
                            ego_car.setSpeed(j[1]["speed"]);

                            // Previous path data given to the Trajectory Planner
                            auto prev_traj_path_x = j[1]["previous_path_x"];
                            auto prev_traj_path_y = j[1]["previous_path_y"];

                            // Previous path's end s and d values
                            double prev_traj_path_end_s_val = j[1]["end_path_s"];
                            double prev_traj_path_end_d_val = j[1]["end_path_d"];

                            // Sensor Fusion Data, a list of all other cars on the same side of the road.
                            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

                            double target_velocity = 49.5; //mph

                            // A previous list of points that the car was following and will help us when doing a
                            // transition.
                            int prev_traj_path_list_size = prev_traj_path_x.size();

                            // Setup reference position.
                            // Either we will reference the starting point as where the car is or at the previous paths
                            // end point.
                            int next_waypoint = -1;
                            double ref_pos_x = ego_car.getX();
                            double ref_pos_y = ego_car.getY();
                            double ref_pos_yaw = deg2rad(ego_car.getYaw());

                            // If previous size is about empty, use the car as starting reference.
                            if (prev_traj_path_list_size < 2) {
                                next_waypoint = NextWaypoint(ref_pos_x, ref_pos_y,
                                                             ref_pos_yaw, map_waypoints_x, map_waypoints_y,
                                                             map_waypoints_dx, map_waypoints_dy);
                            } else {  // Use the previous path's end point as the starting reference
                                ref_pos_x = prev_traj_path_x[prev_traj_path_list_size -
                                                             1];
                                double prev_ref_pos_x = prev_traj_path_x[
                                        prev_traj_path_list_size - 2];
                                ref_pos_y = prev_traj_path_y[prev_traj_path_list_size -
                                                             1];
                                double prev_ref_pos_y = prev_traj_path_y[
                                        prev_traj_path_list_size - 2];
                                ref_pos_yaw = atan2(ref_pos_y - prev_ref_pos_y,
                                                    ref_pos_x - prev_ref_pos_x);
                                next_waypoint = NextWaypoint(ref_pos_x, ref_pos_y,
                                                             ref_pos_yaw, map_waypoints_x, map_waypoints_y,
                                                             map_waypoints_dx, map_waypoints_dy);

                                ego_car.setS(prev_traj_path_end_s_val);

                                // speed = sqrt(distance^2 / time) in metres per second
                                // see http://mathworld.wolfram.com/Acceleration.html
                                ego_car.setSpeed((sqrt((ref_pos_x - prev_ref_pos_x) *
                                                       (ref_pos_x - prev_ref_pos_x) +
                                                       (ref_pos_y - prev_ref_pos_y) *
                                                       (ref_pos_y - prev_ref_pos_y)) / .02) *
                                                 2.2352);
                            }

                            //find best target velocity to use
                            double closest_dist_s = 100000;  // far away

                            bool change_lanes = false;

                            for (int i = 0; i < sensor_fusion.size(); i++) {  // all traffic heading in our direction
                                float d = sensor_fusion[i][6];

                                /*
                                 * Waypoints are in the middle of the double-yellow driving line
                                 * in the centre of the 6 lane highway. 3 lanes contain traffic heading in the ego cars
                                 * direction. Each lane is 4 metres wide. The ego car starts in the middle lane, 1.
                                 *
                                 * For each car in the traffic heading in the ego cars direction, we will identify those
                                 * that are currently in the ego cars lane.
                                 *
                                 * If the ego car is in the lane 0 (closest to the centre waypoints), then it's d value is
                                 * 2 metres aka the centre of lane 0.
                                 */
                                if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                                    // this sensed car is travelling somewhere inside the ego car's lane
                                    double vx = sensor_fusion[i][3];
                                    double vy = sensor_fusion[i][4];
                                    // speed = sqrt(distance^2 / time) * m/s
                                    // See speed formula at top of Acceleration defintion from link below :
                                    // http://mathworld.wolfram.com/Acceleration.html
                                    double check_speed = sqrt(vx * vx + vy * vy);
                                    double check_car_s = sensor_fusion[i][5];
                                    check_car_s += ((double) prev_traj_path_list_size * .02 * check_speed);

                                    //c heck s values greater than mine and s gap
                                    // traffic car ahead of us is within our 30 point trajectory line
                                    if ((check_car_s > ego_car.getS()) && ((check_car_s - ego_car.getS()) < 30) &&
                                        ((check_car_s - ego_car.getS()) < closest_dist_s)) {

                                        closest_dist_s = (check_car_s - ego_car.getS());

                                        // traffic car is in the last 1/3rd of our trajectory
                                        if ((check_car_s - ego_car.getS()) > 20) {

                                            //match that cars speed
                                            target_velocity = check_speed * 2.2352;
                                            change_lanes = true;
                                        } else {  // traffic car is too close ... slow down.
                                            //go slightly slower than the cars speed
                                            target_velocity = check_speed * 2.2352 - 5;
                                            change_lanes = true;

                                        }
                                    }


                                }
                            }

                            //try to change lanes if too close to car in front
                            if (change_lanes && ((next_waypoint - lane_change_wp) % map_waypoints_x.size() > 2)) {
                                bool changed_lanes = false;
                                //first try to change to left lane
                                if (lane != 0 && !changed_lanes) {
                                    bool lane_safe = true;
                                    for (int i = 0; i < sensor_fusion.size(); i++) {
                                        float d = sensor_fusion[i][6];
                                        if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)) {
                                            //car is in left lane
                                            double vx = sensor_fusion[i][3];
                                            double vy = sensor_fusion[i][4];
                                            double check_speed = sqrt(vx * vx + vy * vy);

                                            double check_car_s = sensor_fusion[i][5];
                                            check_car_s += ((double) prev_traj_path_list_size * .02 *
                                                            check_speed);
                                            double dist_s = check_car_s - ego_car.getS();

                                            // Next line made ego car too aggressive : tail-gating before lane change.
                                            //if (dist_s < 10 && dist_s > -10) {
                                            if (dist_s < 20 && dist_s > -20) {
                                                lane_safe = false;
                                            }
                                        }
                                    }
                                    if (lane_safe) {
                                        changed_lanes = true;
                                        lane -= 1;
                                        lane_change_wp = next_waypoint;
                                    }
                                }
                                //next try to change to right lane
                                if (lane != 2 && !changed_lanes) {
                                    bool lane_safe = true;
                                    for (int i = 0; i < sensor_fusion.size(); i++) {
                                        float d = sensor_fusion[i][6];
                                        if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)) {
                                            //car is in right lane
                                            double vx = sensor_fusion[i][3];
                                            double vy = sensor_fusion[i][4];
                                            double check_speed = sqrt(vx * vx + vy * vy);

                                            double check_car_s = sensor_fusion[i][5];
                                            check_car_s += ((double) prev_traj_path_list_size * .02 *
                                                            check_speed);
                                            //double dist_s = check_car_s - car_s;
                                            double dist_s = check_car_s - ego_car.getS();

                                            // Next line made ego car too aggressive : tail-gating before lane change.
                                            //if (dist_s < 10 && dist_s > -10) {
                                            if (dist_s < 20 && dist_s > -20) {
                                                lane_safe = false;
                                            }
                                        }
                                    }
                                    if (lane_safe) {
                                        changed_lanes = true;
                                        lane += 1;
                                        lane_change_wp = next_waypoint;
                                    }

                                }

                            }


                            vector<double> ptsx;
                            vector<double> ptsy;

                            if (prev_traj_path_list_size < 2) {
                                double prev_car_x = ego_car.getX() - cos(ego_car.getYaw());
                                double prev_car_y = ego_car.getY() - sin(ego_car.getYaw());

                                ptsx.push_back(prev_car_x);
                                ptsx.push_back(ego_car.getX());

                                ptsy.push_back(prev_car_y);
                                ptsy.push_back(ego_car.getY());
                            } else {
                                ptsx.push_back(prev_traj_path_x[prev_traj_path_list_size - 2]);
                                ptsx.push_back(prev_traj_path_x[prev_traj_path_list_size - 1]);

                                ptsy.push_back(prev_traj_path_y[prev_traj_path_list_size - 2]);
                                ptsy.push_back(prev_traj_path_y[prev_traj_path_list_size - 1]);


                            }

                            // In Frenet add evenly 30m spaced points ahead of the starting reference.
                            vector<double> next_wp0 = getXY(ego_car.getS() + 30, (2 + 4 * lane), map_waypoints_s,
                                                            map_waypoints_x, map_waypoints_y);
                            vector<double> next_wp1 = getXY(ego_car.getS() + 60, (2 + 4 * lane), map_waypoints_s,
                                                            map_waypoints_x, map_waypoints_y);
                            vector<double> next_wp2 = getXY(ego_car.getS() + 90, (2 + 4 * lane), map_waypoints_s,
                                                            map_waypoints_x, map_waypoints_y);

                            ptsx.push_back(next_wp0[0]);
                            ptsx.push_back(next_wp1[0]);
                            ptsx.push_back(next_wp2[0]);

                            ptsy.push_back(next_wp0[1]);
                            ptsy.push_back(next_wp1[1]);
                            ptsy.push_back(next_wp2[1]);

                            // Transformation of car's local heading reference from current angle to 0 heading angle.
                            for (int i = 0; i < ptsx.size(); i++) {

                                //shift car reference angle to 0 degrees
                                double shift_x = ptsx[i] - ref_pos_x;
                                double shift_y = ptsy[i] - ref_pos_y;

                                ptsx[i] = (shift_x * cos(0 - ref_pos_yaw) -
                                           shift_y * sin(0 - ref_pos_yaw));
                                ptsy[i] = (shift_x * sin(0 - ref_pos_yaw) +
                                           shift_y * cos(0 - ref_pos_yaw));

                            }

                            // Create a spline.
                            tk::spline s;

                            // Set (x,y) 'anchor' points (or 'knots') around which the smooth spline is formed.
                            s.set_points(ptsx, ptsy);

                            // Define the actual (x,y) points we will use for the planner aka define the future path.
                            vector<double> next_x_vals;
                            vector<double> next_y_vals;

                            // Start with all of the previous path points from last time.
                            for (int i = 0; i < prev_traj_path_x.size(); i++) {
                                next_x_vals.push_back(prev_traj_path_x[i]);
                                next_y_vals.push_back(prev_traj_path_y[i]);
                            }

                            // Calculate how to break up spline points so that we travel at our desired
                            // reference velocity and avoid jerk violation.
                            double target_x = 30.0;  // Our horizon along x-axis. Smaller than 30 values
                            // will cause the car to drive up too close to the car in front before changing lanes.
                            double target_y = s(
                                    target_x);  // ask spline what is the corresponding y-value for the given x-value
                            double target_dist = sqrt(
                                    (target_x) * (target_x) + (target_y) * (target_y));  // Distance from either the car
                            // or the last point in previous path to this target.

                            double x_add_on = 0;  // This relates to the Car Transformation done earlier.
                            // We start at the origin.

                            // Fill up the rest of our path planner after filling it with previous points, here we
                            // will always output 50 points.
                            for (int i = 1; i <= 50 - prev_traj_path_x.size(); i++) {

                                if (target_velocity > ego_car.getSpeed()) {  // speed up
                                    double car_speed;
                                    car_speed = ego_car.getSpeed();
                                    car_speed += .22352;  // 0.22352 m/s = 0.5 mph
                                    ego_car.setSpeed(car_speed);
                                } else if (target_velocity < ego_car.getSpeed()) {  // too close, slow down
                                    double car_speed;
                                    car_speed = ego_car.getSpeed();
                                    car_speed -= .22352;
                                    ego_car.setSpeed(car_speed);
                                }


                                // N is the number of anchor points along the spline curve that the car will visit
                                // every 0.02 seconds (aka simulator moves the car to next point 50 times per second).
                                double N = (target_dist /
                                            (.02 * ego_car.getSpeed() / 2.2352));  // 2.2352 m/s = 5.0 mph
                                // coverts target_velocity from MPH into metres per second.
                                double x_point = x_add_on + (target_x) / N;
                                double y_point = s(
                                        x_point);  // Corresponding y-axis value from x_point -> smooth spline line
                                // This will ensure car will go at the desired speed e.g. 49.5

                                x_add_on = x_point;

                                double x_ref = x_point;
                                double y_ref = y_point;

                                // Rotate back to normal after rotating it earlier.
                                x_point = (x_ref * cos(ref_pos_yaw) - y_ref * sin(ref_pos_yaw));
                                y_point = (x_ref * sin(ref_pos_yaw) + y_ref * cos(ref_pos_yaw));

                                x_point += ref_pos_x;
                                y_point += ref_pos_y;


                                next_x_vals.push_back(x_point);
                                next_y_vals.push_back(y_point);
                            }

                            json msgJson;
                            msgJson["next_x"] = next_x_vals;
                            msgJson["next_y"] = next_y_vals;

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            //this_thread::sleep_for(chrono::milliseconds(1000));
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                        }
                    } else {
                        // Manual driving
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }
            });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}