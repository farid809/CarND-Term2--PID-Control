

#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#include <chrono>
#include <cstdint>

// for convenience
using nlohmann::json;
using std::string;

using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


int signal_period_detector = 0;
double prev_signal_value = -999999;

uint64_t osc_star;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  
  pid.InitErrorParams();
  
    //Attempting to PID Online Tuning using ZEiglar Nichols method
    //
    //
    //1. The proportional gain is increased until it reaches the ultimate gain
      //pid.Init(0.3, 0 , 0);
    // period of Tu= 0.8 seconds
    // Ultimate Gain Ku = 0.3
    //calculating gains as following
    // Kp=0.6*Ku = 0.6*0.3= 0.18
    // Ki=1.2*Ku/Tu=(1.2*0.3)/0.8=0.45
    // Kd=3*Ku*Tu/40=(3*0.3*0.8)/40=0.018


    //Attemping to PID online tuning Manually:  (ref: //https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops)
    //Steps:
    // 1. Set all PID Gains to zero
    // 2. Increase Kp gain until system become oscillatory
    // 3. Increase Kd gain until oscillation goes away
    // 4. repeat 2 and 3 until increasing Kd gain doesn't stop oscillation

    //PD Controller Only (works fine)
    //pid.Init(0.2, 0, 2);


    //5. Decrease Kp by a factor of 2
    //6. set Ki to about 1% of Kp
    //7. increase Ki until oscillation start
    //8. decrease Ki by factor of 2

    //Final PID Controller
     pid.Init(0.2, 0.0001, 2);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle_speed = 0.4;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          //Input the cte value to the controller 
          //              +   ________________      _______
          // setpoint ---->- [ PID Controller ] ---[Process]-------- steering 
          //              /|\ ----------------      -------         |
          //               |_______ cte _____________________________ 
          pid.UpdateError(cte);


          //get controller output and send it to the process
          steer_value=pid.TotalError();
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_speed;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;



        //Online tuning of PID Controller gain parameters
        // Detect oscillation in controller output and calculate the period of the oscillation
        // 
        // The oscillation period is used to calculate the controller gain according to 
        // Ziegler–Nichols method

        std::chrono::high_resolution_clock m_clock;
        uint64_t now=std::chrono::duration_cast<std::chrono::milliseconds>
              (m_clock.now().time_since_epoch()).count();
        std::cout << now << std::endl;

        if (prev_signal_value != -999999)
        {
          //Detect oscillation transition
          if(signal_period_detector==0) osc_star=now;

          if((int(prev_signal_value*100000) ^ int(steer_value*100000)) < 0)
          {
            if(signal_period_detector<2)
            {
                ++signal_period_detector;
            }
            else
            {
              int period=now-osc_star;
              std::cout << "Detected Oscillation" ;
              std::cout << period/1000 << std::endl;
              
              signal_period_detector=0;

              
            }
            
          }

          prev_signal_value=steer_value;

        }
        else
        {
          prev_signal_value=steer_value;
        }

    
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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