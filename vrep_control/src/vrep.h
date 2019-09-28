/**
 * @file vrep.h
 * @brief V-REPを使うためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2017/01/24
 * @detail
 */

#ifndef SNAKE_ROBOT_MASTER_VREP_SRC_VREP_H_
#define SNAKE_ROBOT_MASTER_VREP_SRC_VREP_H_

#include <string>
#include <cmath>
#include <vector>
#include <ros/ros.h>
extern "C" {
  #include "extApi.h"
}

class Vrep {
 public:
  class Disconnector;  // デストラクタでシミュレーションを終了して切断するためのクラス．
  class Object;
  class JointObject;
  class ForceSensor;
  class ProximitySensor;

private:
  class ReturnCode;
 public:
  static bool Initialize() {
    is_connecting_to_vrep_ = false;
    is_sim_running_ = false;
    is_sim_pausing_ = false;
    return ConnectToVrep();
  }

 public:
  static bool ConnectToVrep();
  static void StartSimulation();
  static void PauseSimulation();
  static void FinishSimulation();
  static void DisconnectFromVrep();
  static void WaitForReachingLastCommand();

  bool is_connecting_to_vrep(){ return is_connecting_to_vrep_; }
  bool is_sim_running(){ return is_sim_running_; }
  bool is_sim_pausing(){ return is_sim_pausing_; }

 private:

 private:
  static Disconnector disconnector_;
  static simxInt client_id_;  // V-REP接続ID
  static bool is_connecting_to_vrep_;
  static bool is_sim_running_;
  static bool is_sim_pausing_;


 public:

};

class Vrep::Disconnector {
 public:
  ~Disconnector(){
    DisconnectFromVrep();
  }
};

class Vrep::ReturnCode {
  public:
   ReturnCode(simxInt return_code) {
     if (return_code == 0) {
       is_some_error_ = false;
     } else {
       is_some_error_ = true;
       novalue_          = (bool)( (return_code>>0)&0b1 );
       timeout_          = (bool)( (return_code>>1)&0b1 );
       illegal_opmode_   = (bool)( (return_code>>2)&0b1 );
       remote_error_     = (bool)( (return_code>>3)&0b1 );
       split_progress_   = (bool)( (return_code>>4)&0b1 );
       local_error_      = (bool)( (return_code>>5)&0b1 );
       initialize_error_ = (bool)( (return_code>>6)&0b1 );
     }
   }
   void DisplayWarn(char* comment = (char*)" ", bool ignore_novalue = false) {
     if (is_some_error_) {
       if (novalue_ and !ignore_novalue) ROS_WARN("V-REP raise error : novalue %s", comment);
       if (timeout_) ROS_WARN("V-REP raise error : timeout %s", comment);
       if (illegal_opmode_) ROS_WARN("V-REP raise error : illegal_opmode %s", comment);
       if (remote_error_) ROS_WARN("V-REP raise error : remote_error %s", comment);
       if (split_progress_) ROS_WARN("V-REP raise error : split_progress %s", comment);
       if (local_error_) ROS_WARN("V-REP raise error : local_error %s", comment);
       if (initialize_error_) ROS_WARN("V-REP raise error : initialize_error %s", comment);
     }
   }
   void DisplayWarn(std::string comment = " ", bool ignore_novalue = false) {
     DisplayWarn((char*)comment.c_str(), ignore_novalue);
   }
   bool is_some_error(){ return is_some_error_; }
   bool novalue(){ return novalue_; }
   bool timeout(){ return timeout_; }
   bool illegal_opmode(){ return illegal_opmode_; }
   bool remote_error(){ return remote_error_; }
   bool split_progress(){ return split_progress_; }
   bool local_error(){ return local_error_; }
   bool initialize_error(){ return initialize_error_; }
  private:
   bool is_some_error_;
   bool novalue_;
   bool timeout_;
   bool illegal_opmode_;
   bool remote_error_;
   bool split_progress_;
   bool local_error_;
   bool initialize_error_;
 };

class Vrep::Object {
 public:
  typedef std::vector<Object>::iterator ItrType;
  class Orientation {
   public:
    Orientation() {
      Set(0.0, 0.0, 0.0);
    }
    Orientation(double roll_rad, double pitch_rad, double yaw_rad) {
      Set(roll_rad, pitch_rad, yaw_rad);
    }
    void Set(double roll_rad, double pitch_rad, double yaw_rad) {
      roll_rad_ = roll_rad;
      pitch_rad_ = pitch_rad;
      yaw_rad_ = yaw_rad;
    }
    double roll_rad(){ return roll_rad_; }
    double pitch_rad(){ return pitch_rad_; }
    double yaw_rad(){ return yaw_rad_; }
    double roll_deg(){ return roll_rad_*180.0/M_PI; }
    double pitch_deg(){ return pitch_rad_*180.0/M_PI; }
    double yaw_deg(){ return yaw_rad_*180.0/M_PI; }
   private:
    double roll_rad_;
    double pitch_rad_;
    double yaw_rad_;
  };
  class Coordinate {
   public:
    Coordinate() {
      Set(0.0, 0.0, 0.0);
    }
    Coordinate(double x, double y, double z) {
      Set(x, y, z);
    }
    void Set(double x, double y, double z) {
      x_ = x;
      y_ = y;
      z_ = z;
    }
    double x(){ return x_; }
    double y(){ return y_; }
    double z(){ return z_; }
   private:
    double x_;  // [m]
    double y_;  // [m]
    double z_;  // [m]
  };

 public:
  Object(){
    have_never_get_orientation_ = true;
    have_never_get_coordinate_ = true;
    handle_ = -1;
  }
  ~Object(){
  }
  Object(const Object& obj) {  // コピーコンストラクタ
    this->have_never_get_orientation_ = obj.have_never_get_orientation_;
    this->have_never_get_coordinate_ = obj.have_never_get_coordinate_;
  }
  bool GetHandle(char* object_name);
  bool GetHandle(std::string object_name);
  int handle(){ return handle_; }
  Orientation GetAbsoluteOrientation();
  Coordinate GetAbsoluteCoordinate();

 protected:
  simxInt handle_;
  std::string name_;
  bool have_never_get_orientation_;
  bool have_never_get_coordinate_;
};

class Vrep::JointObject : public Vrep::Object {
 public:
  typedef std::vector<JointObject>::iterator ItrType;
  JointObject() {
    Object();
    have_never_get_angle_ = true;
    have_never_get_truque_ = true;
  }
  ~JointObject() {
  }
  JointObject(const JointObject& obj) {  // コピーコンストラクタ
    this->have_never_get_angle_ = obj.have_never_get_angle_;
    this->have_never_get_truque_ = obj.have_never_get_truque_;
  }
  void SetAngle(double angle, bool unit_is_deg);
  void SetTargetAngle(double angle, bool unit_is_deg);
  void SetTargetVelocity(double velocity, bool unit_is_deg);
  void SetMaxTorque(double truque);
  void SetPIDGain(float p_gain, float i_gain, float d_gain);
  double GetAngle(bool unit_is_deg);
  double GetTorque();

 private:
  bool have_never_get_angle_;
  bool have_never_get_truque_;


};

class Vrep::ForceSensor : public Vrep::Object {
 public:
  typedef std::vector<ForceSensor>::iterator ItrType;
  ForceSensor() {
    Object();
    have_never_get_data_ = true;
  }
  ~ForceSensor() {
  }
  ForceSensor(const ForceSensor& obj) {  // コピーコンストラクタ
    this->have_never_get_data_ = obj.have_never_get_data_;
    for (int i=0; i<3; i++) {
      this->force_data_[i] = obj.force_data_[i] ;
      this->torque_data_[i]= obj.torque_data_[i];
    }
    this->state_ = obj.state_;
  }
  void UpdateData();
  double force_data(int i){ return force_data_[i]; }
  double torque_data(int i){ return torque_data_[i]; }

 private:
  bool have_never_get_data_;

  simxFloat force_data_[3];
  simxFloat torque_data_[3];
  uint8_t state_;

};

class Vrep::ProximitySensor : public Vrep::Object {
 public:
  typedef std::vector<ProximitySensor>::iterator ItrType;
  ProximitySensor() {
    Object();
    have_never_get_data_ = true;
  }
  ~ProximitySensor() {
  }
  ProximitySensor(const ProximitySensor& obj) {  // コピーコンストラクタ
    this->have_never_get_data_ = obj.have_never_get_data_;
    this->detection_state_        = obj.detection_state_;
    this->detected_object_handle_ = obj.detected_object_handle_;
    for (int i=0; i<3; i++) {
      this->detected_point_[i] = obj.detected_point_[i];
      this->detected_surface_normal_vector_[i] = obj.detected_surface_normal_vector_[i];
    }
  }
  void UpdateData();
  bool detection_state(){ return detection_state_; }  // 0:no ditection
  double detected_point(int i){ return detected_point_[i]; }
  int detected_object_handle(){ return detected_object_handle_; }
  double detected_surface_normal_vector(int i){ return detected_surface_normal_vector_[i]; }

 private:
  bool have_never_get_data_;
  simxUChar detection_state_;
  simxFloat detected_point_[3];
  simxInt detected_object_handle_;
  simxFloat detected_surface_normal_vector_[3];

};


#endif /* SNAKE_ROBOT_MASTER_VREP_SRC_VREP_H_ */

