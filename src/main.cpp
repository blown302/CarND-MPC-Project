#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

const int polynomial_order = 3;
const int shifted_x_value = 0;
const int state_size = 6;
const int num_points = 25;
using namespace Eigen;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main(int argc, char *argv[]) {
    uWS::Hub h;
    // delcare and initialize mpc parameters
    double target_velocity = 105.,
        steering_actuator_weight = 7000.,
        accelerator_actuator_weight = 1.,
        steering_smoothing_weight = 1000.,
        accelerator_smoothing_weight = 1.;

    if (argc >= 2) {
        steering_actuator_weight = atof(argv[1]);
        cout << "steering actuator weight is set to: " << steering_actuator_weight << endl;
    }

    if (argc >= 3) {
        accelerator_actuator_weight = atof(argv[2]);
        cout << "accelerator actuator weight is set to: " << accelerator_actuator_weight << endl;
    }

    if (argc >= 4) {
        steering_smoothing_weight = atof(argv[3]);
        cout << "steering smoothing weight is set to: " << steering_smoothing_weight << endl;
    }

    if (argc >= 5) {
        accelerator_smoothing_weight = atof(argv[4]);
        cout << "accelerator smoothing weight is set to: " << accelerator_smoothing_weight << endl;
    }

    if (argc >= 6) {
        target_velocity = atof(argv[5]);
        cout << "setting target velocity to: " << target_velocity << endl;
    }


    // MPC is initialized here!
    MPC mpc{target_velocity, steering_actuator_weight, accelerator_actuator_weight, steering_smoothing_weight, accelerator_smoothing_weight};

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        // cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {

                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    double steering_angle = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];

                    // shift waypoints into vehicle coordinate system.
                    // TODO: extract method.
                    for (auto i = 0; i < ptsx.size(); i++) {
                        auto shift_x = ptsx[i] - px;
                        auto shift_y = ptsy[i] - py;


                        ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
                        ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
                    }

                    // Contruct Eigen vectors from std::vector.
                    double *ptsx_ptr = &ptsx[0];
                    Map<VectorXd> ptsx_transform(ptsx_ptr, ptsx.size());

                    double *ptsy_ptr = &ptsy[0];
                    Map<VectorXd> ptsy_transform(ptsy_ptr, ptsy.size());

                    // fit polynomial to waypoints.
                    auto coeffs = polyfit(ptsx_transform, ptsy_transform, polynomial_order);

                    // calculate cte
                    // TODO: add a more accurate CTE function here.
                    auto cte = polyeval(coeffs, shifted_x_value);

                    // calculate epsi
                    // TODO: below is simplified, if not using 0 for x and psi use the proper formula.
                    auto epsi = -atan(coeffs[1]);

                    // set state
                    VectorXd state(state_size);
                    state << 0, 0, 0, v, cte, epsi;

                    // predict
                    auto vars = mpc.Solve(state, coeffs, throttle, steering_angle);

                    double steer_value = vars[0] / deg2rad(25);
                    double throttle_value = vars[1];

                    // send results back to sim
                    // Display the waypoints/reference line
                    vector<double> next_x_vals(num_points - 1);
                    vector<double> next_y_vals(num_points - 1);

                    // distance of 2.5 units for each point
                    static const double poly_inc = 2.5;

                    for (auto i = 1; i < num_points; i++) {
                        auto x_inc = poly_inc * i;
                        next_x_vals[i - 1] = x_inc;
                        next_y_vals[i - 1] = polyeval(coeffs, x_inc);
                    }

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = -steer_value;
                    msgJson["throttle"] = throttle_value;

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    for (auto i = 2; i < vars.size(); i++) {
                        if (i % 2 == 0) {
                            mpc_x_vals.push_back(vars[i]);
                        } else {
                            mpc_y_vals.push_back(vars[i]);
                        }
                    }

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
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

    h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
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
