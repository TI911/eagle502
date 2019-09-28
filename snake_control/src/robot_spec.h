/*
 * robot_spec.h
 *
 *  Created on: Feb 21, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_ROBOT_SPEC_H_
#define SNAKE_CONTROL_SRC_ROBOT_SPEC_H_

#include <stdint.h>

class RobotSpec {
 public:
  RobotSpec() {  // 引数なしのコンストラクタを置いておく
	  ;
  }

  RobotSpec(uint8_t num_joint,     			// 関節数
            double link_length_head,  			// [m] 先頭リンク長さ
            double link_length_body,  			// [m] 通常のリンク長さ
            double link_length_tail,  			// [m] 末尾のリンク長さ
            double link_diameter,     			// [m] リンクの直径
            double max_joint_angle,   			// [rad] 最大関節角
            double max_joint_angle_velocity,  	// 奇数関節をYaw関節とするときtrue
            bool odd_joint_is_yaw=true        	// 奇数関節をYaw関節とするときtrue
           ) {
    num_joint_        = num_joint;
    link_length_head_ = link_length_head;
    link_length_body_ = link_length_body;
    link_length_tail_ = link_length_tail;
    link_length_.clear();
    link_diameter_    = link_diameter;
    full_length_      = link_length_head_ + link_length_body_*(num_joint_-1) + link_length_tail_;
    max_joint_angle_  = max_joint_angle;
    max_joint_angle_velocity_ = max_joint_angle_velocity;

    //--- 引数を元によく使う変数をまとめておく ---//
    // 各リンク長さを一つの配列にまとめる
    link_length_.push_back(link_length_head_);
    for (uint8_t i_link_b=0; i_link_b<num_joint_-1; i_link_b++) {
      link_length_.push_back(link_length_body_);
    }
    link_length_.push_back(link_length_tail_);

    // 先頭から各関節までの長さを一つの配列にまとめる
    length_from_head_to_joint_.clear();
    double sum_of_link_length = 0;
    for (uint8_t i_joint=0; i_joint<num_joint_; i_joint++) {
      sum_of_link_length += link_length_[i_joint];
      length_from_head_to_joint_.push_back(sum_of_link_length);
    }

    max_curvature_body_ = max_joint_angle_/(2*link_length_body_);
    min_radius_body_ = 1.0/max_curvature_body_;
    odd_joint_is_yaw_ = odd_joint_is_yaw;

  };

  //--- getter --------------------//
  uint8_t num_joint(){ return num_joint_; }
  double link_length_head(){ return link_length_head_; }
  double link_length_body(){ return link_length_body_; }
  double link_length_tail(){ return link_length_tail_; }
  std::vector<double> link_length(){ return link_length_; }
  std::vector<double> length_from_head_to_joint(){ return length_from_head_to_joint_; };
  double link_diameter(){ return link_diameter_; }
  double full_length(){ return full_length_; }
  double max_joint_angle(){ return max_joint_angle_; }
  double max_joint_angle_velocity(){ return max_joint_angle_velocity_; }
  double max_curvature_body(){ return max_curvature_body_; }
  double min_radius_body(){ return min_radius_body_; }
  bool odd_joint_is_yaw(){ return odd_joint_is_yaw_; }

 private:
  uint8_t num_joint_;        // 関節数

  double link_length_head_;  // [m] 先頭リンク長さ

  double link_length_body_;  // [m] 通常のリンク長さ

  double link_length_tail_;  // [m] 末尾のリンク長さ

  std::vector<double> link_length_;                // [m] 先頭から末尾までのリンク長さをまとめた配列

  std::vector<double> length_from_head_to_joint_;  // [m] 先頭から各関節までの長さをまとめた配列

  double link_diameter_;     // [m] リンクの直径

  double full_length_;       // [m] 全長

  double max_joint_angle_;   // [rad] 最大関節角

  double max_joint_angle_velocity_;  // [rad/sec] 最大関節角速度

  double max_curvature_body_;        // [1/m] ロボットが実現しうる最大曲率

  double min_radius_body_;   // [m] ロボットが実現しうる最大半径

  bool   odd_joint_is_yaw_;  // 奇数関節をYaw関節とするときtrue

};

#endif /* SNAKE_CONTROL_SRC_ROBOT_SPEC_H_ */
