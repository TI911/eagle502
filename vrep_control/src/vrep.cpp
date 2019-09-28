/**
 * @file vrep.h
 * @brief V-REPを使うためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2017/01/24
 * 		2018/05  TI modify
 * @detail
 */

#include "vrep.h"

//=== staticメンバの実体 ============================================================//
Vrep::Disconnector disconnector_;
simxInt Vrep::client_id_ = 0;  // v-rep接続ID
bool Vrep::is_connecting_to_vrep_ = false;
bool Vrep::is_sim_running_ = false;
bool Vrep::is_sim_pausing_ = false;

/** @fn
 * @brief V-REPに接続する
 * @param なし
 * @return bool ture:接続成功
 */
bool Vrep::ConnectToVrep() {
  simxFinish(-1);  // 念のため，先に全ての操作を終了
  ROS_INFO("Connecting to v-rep");
  client_id_ = simxStart(/* connectionAddress = */ (simxChar*)"127.0.0.1",
                           /* connectionPort = */ 19997,  // 19997にすればvrep側を手動で開始する必要がない
                           /* waitUntilConnected = */ true,
                           /* doNotReconnectOnceDisconnected = */ true,
                           /* timeOutInMs = */ 5000,
                           /* commThreadCycleInMs = */ 5);
  if (client_id_ == -1) {
    ROS_ERROR("Could not connect to V-REP remote API server");
    return false;
  } else {
    ROS_INFO("Connected to V-REP remote API server");
    is_connecting_to_vrep_ = true;
    return true;
  }
}

/** @fn
 * @brief シミュレーションを開始する
 * @param なし
 * @return なし
 */
void Vrep::StartSimulation() {
  simxStartSimulation(client_id_, simx_opmode_oneshot_wait);
  ROS_INFO("Simulation is started.");
  is_sim_running_ = true;
  is_sim_pausing_ = false;
}

/** @fn
 * @brief シミュレーションを一時停止する
 * @param なし
 * @return なし
 * @detail
 */
void Vrep::PauseSimulation() {
  if (is_sim_running_) {
    WaitForReachingLastCommand();
    simxPauseSimulation(client_id_, simx_opmode_oneshot);
    ROS_INFO("Simulation is paused.");
    is_sim_running_ = false;
    is_sim_pausing_ = true;
  }
}

/** @fn
 * @brief シミュレーションを終了する
 * @param なし
 * @return なし
 * @detail
 */
void Vrep::FinishSimulation() {
  if ( is_sim_running_ or is_sim_pausing_ ) {
    WaitForReachingLastCommand();
    simxStopSimulation(client_id_, simx_opmode_oneshot_wait);
    ROS_INFO("Simulation is finished.");
    is_sim_running_ = false;
  }
}

/** @fn
 * @brief V-REPとの接続を切る
 * @param なし
 * @return なし
 * @detail
 */
void Vrep::DisconnectFromVrep() {
  FinishSimulation();
  simxFinish(client_id_);
  ROS_INFO("Disconnect from V-REP");
  is_connecting_to_vrep_ = false;
}

/** @fn
 * @brief V-REPに最後に送ったコマンドが効くのを待つ
 * @param なし
 * @return なし
 * @detail
 *  時間を確認するコマンドの返事が来た=その前に送ったコマンドが既に効いた
 */
void Vrep::WaitForReachingLastCommand() {
  simxInt pin_time_temp;
  simxGetPingTime(client_id_, &pin_time_temp);  // 最後に送信したコマンドが届くのを待つ
  std::to_string(1);
}


//===== Vrep::Object class ==================================================//
/** @fn
 * @brief V-REP上のオブジェクトのhandleを取得する char*版
 * @param char* object_name オブジェクトの名前
 * @return false:handle取得失敗
 * @detail
 */
bool Vrep::Object::GetHandle(char* object_name) {
  name_ = object_name;
  ReturnCode ret_code(
      simxGetObjectHandle(client_id_, (simxChar*) object_name,
                          &handle_, simx_opmode_oneshot_wait)
    );
  if (ret_code.is_some_error()) {
    ret_code.DisplayWarn(/* comment = */ "@ GetHandle of "+name_);
    ROS_ERROR("Could not get handle of %s", object_name);
    return false;
  } else {
    ROS_INFO("Got handle of %s : %d", object_name, handle_);
    return true;
  }
}

/** @fn
 * @brief V-REP上のオブジェクトのhandleを取得する std::string版
 * @param std::string object_name オブジェクトの名前
 * @return false:handle取得失敗
 * @detail
 */
bool Vrep::Object::GetHandle(std::string object_name) {
  return GetHandle( (char*)object_name.c_str() );
}

/** @fn
 * @brief objectの絶対座標系に対する姿勢を取得
 * @param なし
 * @return (Vrep::Object::Coordinate) 姿勢
 * @detail
 */
Vrep::Object::Orientation Vrep::Object::GetAbsoluteOrientation() {
  simxInt opmode;
  simxFloat euler_angles[3];
  if (have_never_get_orientation_) {
    opmode = simx_opmode_streaming;
     have_never_get_orientation_ = false;
  } else {
    opmode = simx_opmode_buffer;
  }
  ReturnCode ret_code(
      simxGetObjectOrientation(client_id_, handle_, -1, euler_angles, opmode)
      );
  ret_code.DisplayWarn(/* comment = */ "@ GetAbsoluteOrientation of "+name_,
                       /* ignore_novalue = */ true);  // 返信のないopmodeなので無視してよい
  Orientation orientation(euler_angles[0], euler_angles[1], euler_angles[2]);
  return orientation;
}

/** @fn
 * @brief objectの絶対座標を取得
 * @param なし
 * @return (Vrep::Object::Coordinate) 絶対座標
 * @detail
 */
Vrep::Object::Coordinate Vrep::Object::GetAbsoluteCoordinate() {
  simxInt opmode;
  simxFloat pos[3];
  if (have_never_get_coordinate_) {
    opmode = simx_opmode_streaming;
     have_never_get_coordinate_ = false;
  } else {
    opmode = simx_opmode_buffer;
  }
  ReturnCode ret_code(
      simxGetObjectPosition(client_id_, handle_, -1, pos, opmode)
      );
  ret_code.DisplayWarn(/* comment = */ "@ GetAbsoluteCoordinate of "+name_,
                       /* ignore_novalue = */ true);  // 返信のないopmodeなので無視してよい
  Coordinate coordinate(pos[0], pos[1], pos[2]);
  return coordinate;
}

//===== Vrep::JointObject class ==================================================//
/** @fn
 * @brief 関節に関節角を送る．即その角度になる．
 * @param double angle [rad](or[deg]) 関節角度
 * @param bool unit_is_deg angleの単位が[deg]の時trueにする
 * @return なし
 * @detail
 *  SetTargetPosition()との違いに注意
 */
void Vrep::JointObject::SetAngle(double angle, bool unit_is_deg = false) {
  if (unit_is_deg) { angle *= M_PI/180.0; }
  simxSetJointPosition(client_id_, handle_, angle, simx_opmode_oneshot);
}

/** @fn
 * @brief 関節に目標関節角を送る
 * @param double angle [rad](or[deg]) 目標関節角度
 * @param bool unit_is_deg angleの単位が[deg]の時trueにする
 * @return なし
 * @detail
 *  SetPosition()との違いに注意
 */
void Vrep::JointObject::SetTargetAngle(double angle, bool unit_is_deg = false) {
  if (unit_is_deg) { angle *= M_PI/180.0; }
  simxSetJointTargetPosition(client_id_, handle_, angle, simx_opmode_oneshot);
}

/** @fn
 * @brief 関節に目標角速度を送る
 * @param double velocity [rad/sec](or[deg/sec]) 目標関節角度
 * @param bool unit_is_deg velocityeの単位が[deg/sec]の時trueにする
 * @return なし
 * @detail
 *  SetPosition()との違いに注意
 */
void Vrep::JointObject::SetTargetVelocity(double velocity, bool unit_is_deg = false) {
  if (unit_is_deg) { velocity *= 180.0/M_PI; }
  simxSetJointTargetVelocity(client_id_, handle_, velocity, simx_opmode_oneshot);
}

/** @fn
 * @brief 関節の最大トルクを設定する
 * @param double truque [Nm] 最大トルク
 * @return なし
 * @detail
 */
void Vrep::JointObject::SetMaxTorque(double truque) {
  simxSetJointForce(client_id_, handle_, truque, simx_opmode_oneshot);
}

/** @fn
 * @brief 関節のPIDゲインを設定する
 * @param float p_gain  Pゲイン
 * @param float i_gain  Iゲイン
 * @param float d_gain  Dゲイン
 * @return なし
 * @detail
 */
void Vrep::JointObject::SetPIDGain(simxFloat p_gain, simxFloat i_gain, simxFloat d_gain) {
  simxSetObjectFloatParameter( client_id_, handle_,
                              /* parameterID = */2002,
                              /* parameterValue = */ (simxFloat)p_gain,
                              /* operationMode = */ simx_opmode_oneshot);
  simxSetObjectFloatParameter( client_id_, handle_,
                              /* parameterID = */2003,
                              /* parameterValue = */ (simxFloat)i_gain,
                              /* operationMode = */ simx_opmode_oneshot);
  simxSetObjectFloatParameter( client_id_, handle_,
                              /* parameterID = */2004,
                              /* parameterValue = */ (simxFloat)d_gain,
                              /* operationMode = */ simx_opmode_oneshot);
}

/** @fn
 * @brief 関節角を取得
 * @param bool unit_is_deg angleの単位が[deg]の時trueにする
 * @return (double) position [rad](or [deg])
 * @detail
 */
double Vrep::JointObject::GetAngle(bool unit_is_deg = false) {
  simxInt opmode;
  simxFloat angle;
  if (have_never_get_angle_) {
    opmode = simx_opmode_streaming;
     have_never_get_angle_ = false;
  } else {
    opmode = simx_opmode_buffer;
  }
  ReturnCode ret_code(
      simxGetJointPosition(client_id_, handle_, &angle, opmode)
    );
  ret_code.DisplayWarn(/* comment = */ "@ GetAngle of "+name_,
                       /* ignore_novalue = */ true);  // 返信のないopmodeなので無視してよい
  if (unit_is_deg) { angle *= 180.0/M_PI; }
  return (double)angle;
}

/** @fn
 * @brief 関節角を取得
 * @param なし
 * @return (double) truque [Nm]
 * @detail
 */
double Vrep::JointObject::GetTorque() {
  simxInt opmode;
  simxFloat truque;
  if (have_never_get_truque_) {
    opmode = simx_opmode_streaming;
     have_never_get_truque_ = false;
  } else {
    opmode = simx_opmode_buffer;
  }
  ReturnCode ret_code(
      simxGetJointForce(client_id_, handle_, &truque, opmode)
      );
  ret_code.DisplayWarn(/* comment = */ "@ GetTruque of "+name_,
                       /* ignore_novalue = */ true);  // 返信のないopmodeなので無視してよい
  return truque;
}


/** @fn
 * @brief 力センサの出力を取得
 * @param なし
 * @return なし
 * @detail
 */
void Vrep::ForceSensor::UpdateData() {
  simxInt opmode;
  if (have_never_get_data_) {
    opmode = simx_opmode_streaming;
     have_never_get_data_ = false;
  } else {
    opmode = simx_opmode_buffer;
  }
  ReturnCode ret_code(
      simxReadForceSensor(client_id_, handle_, &state_, force_data_, torque_data_, opmode)
      );
  ret_code.DisplayWarn(/* comment = */ "@ GetData of "+name_,
                       /* ignore_novalue = */ true);  // 返信のないopmodeなので無視してよい
}

/** @fn
 * @brief 近接覚センサの出力を取得
 * @param なし
 * @return なし
 * @detail
 */
void Vrep::ProximitySensor::UpdateData() {
  simxInt opmode;
  if (have_never_get_data_) {
    opmode = simx_opmode_streaming;
     have_never_get_data_ = false;
  } else {
    opmode = simx_opmode_buffer;
  }
  ReturnCode ret_code(
      simxReadProximitySensor(client_id_, handle_, &detection_state_, detected_point_,
          &detected_object_handle_, detected_surface_normal_vector_, opmode)
      );
  ret_code.DisplayWarn(/* comment = */ "@ GetData of "+name_,
                       /* ignore_novalue = */ true);  // 返信のないopmodeなので無視してよい
}
