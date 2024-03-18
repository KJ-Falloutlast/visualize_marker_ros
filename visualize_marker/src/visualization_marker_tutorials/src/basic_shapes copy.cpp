/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <std_msgs/ColorRGBA.h>
#include <boost/circular_buffer.hpp>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <condition_variable>  
#include <std_msgs/ColorRGBA.h>
// %EndTag(INCLUDES)%



struct Vec3 {
  Vec3() : x(0), y(0), z(0) {}
  
  Vec3(double x, double y, double z)
    : x(x), y(y), z(z) {}
  union {
    struct {
      double x;
      double y;
      double z;
    };
    float v[3];
  };
  
  Vec3& operator+=(Vec3 const &v2) {
    x += v2.x;
    y += v2.y;
    z += v2.z;
    return *this;
  }

  Vec3 operator-(Vec3 const &v2) {
    return Vec3(x - v2.x,
                y - v2.y,
                z - v2.z);
  }
  
  Vec3 operator+(Vec3 const &v2) {
    return Vec3(x + v2.x,
                y + v2.y,
                z + v2.z);
  }
  
  Vec3 operator/(double const &d) {
    return Vec3(x/d, y/d, z/d);
  }
  Vec3 operator*(double const &d) {
    return Vec3(x*d, y*d, z*d);
  }

  static Vec3 min(Vec3 v1, Vec3 v2) {
    return Vec3(std::min(v1.x, v2.x),
                std::min(v1.y, v2.y),
                std::min(v1.z, v2.z));
  }

  static Vec3 max(Vec3 v1, Vec3 v2) {
    return Vec3(std::max(v1.x, v2.x),
                std::max(v1.y, v2.y),
                std::max(v1.z, v2.z));
  }

  static Vec3 abs(Vec3 v1) {
    return Vec3(fabs(v1.x),
                fabs(v1.y),
                fabs(v1.z));
  }
};

struct ImuMeasurement {
  ImuMeasurement(double timestamp, Vec3 accel, Vec3 gyro)
    : timestamp(timestamp), accel(accel), gyro(gyro) {}
  
  double timestamp;
  Vec3 accel;
  Vec3 gyro;
};





class MeasurementHandler
{
public:

  MeasurementHandler() 
  {
    // Default parameters if a yaml file is not provided
    imu_rate_hz_ = 10.0; 
    observation_period_s_ = 1.0; 
    imu_max_rate_x_ = 0.025;
    imu_max_rate_y_ = 0.025; 
    imu_max_rate_z_ = 0.025;
    imu_max_accel_x_ = 0.75;
    imu_max_accel_y_ = 0.5;
    imu_max_accel_z_ = 0.5;
    imu_circular_buffer_.set_capacity(imu_rate_hz_ * observation_period_s_);
    // parseConfig(yaml_cfg_filename);
  }
  void process()
  {
    
  }
  
  int parseConfig(const std::string &filename) 
  {
    YAML::Node config = YAML::LoadFile(filename);
    // Check version
    int version = config["version"].as<int>();
    if (1 == version) {
      imu_max_rate_x_ = config["imu_max_rate_x"].as<double>();
      imu_max_rate_y_ = config["imu_max_rate_y"].as<double>();
      imu_max_rate_z_ = config["imu_max_rate_z"].as<double>();
      imu_max_accel_x_ = config["imu_max_accel_x"].as<double>();
      imu_max_accel_y_ = config["imu_max_accel_y"].as<double>();
      imu_max_accel_z_ = config["imu_max_accel_z"].as<double>();
      imu_rate_hz_ = config["imu_rate_hz"].as<double>();    
      observation_period_s_ = config["observation_period_s"].as<double>();
    } else {
      throw std::runtime_error("Unrecognized yaml version number in " +
                               filename + ": " + std::to_string(version));
    }
  }


  void addImuMeasurement(const sensor_msgs::Imu::ConstPtr &msg) 
  {
    //给他加锁，让他一直执行，直到满
    ROS_INFO("222");
    std::lock_guard<std::mutex> lock(buffer_mutex_);  
    imu_circular_buffer_.push_back(ImuMeasurement(msg->header.stamp.toSec(),
                                                Vec3(msg->linear_acceleration.x,
                                                      msg->linear_acceleration.y,
                                                      msg->linear_acceleration.z),
                                                Vec3(msg->angular_velocity.x,
                                                      msg->angular_velocity.y,
                                                      msg->angular_velocity.z)));
    if (imu_circular_buffer_.size() == imu_circular_buffer_.capacity()) 
    {  
        buffer_is_full_ = true;  
        buffer_full_condition_.notify_one();  
    } 
    else if (imu_circular_buffer_.size() > imu_circular_buffer_.capacity()) 
    {  
        // Circular buffer handling, remove oldest data if needed  
        imu_circular_buffer_.erase(imu_circular_buffer_.begin());  
    }  

  
  }


  Vec3 getStatus(Vec3 *accel_avg_in) 
  {
    //给该线程加锁，当buffer_is_full_为T时才能拿到锁
    ROS_INFO("333");
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    //此为止有问题，因为当addImuMeasurement执行时，只装入1个imu，虽然拿到mtx了但是buffer_is_full不满足为true
    buffer_full_condition_.wait(lock, [this]{ return buffer_is_full_; });  
    // Process the IMU data here  
    // ...  
    Vec3 accel_avg;
    Vec3 accel_max = imu_circular_buffer_[0].accel;
    Vec3 accel_min = imu_circular_buffer_[0].accel;
    Vec3 accel_max_diff;
    Vec3 gyro_avg;
    Vec3 gyro_max = imu_circular_buffer_[0].gyro;
    Vec3 gyro_min = imu_circular_buffer_[0].gyro;
    Vec3 gyro_max_diff;
    
    for (auto &measurement : imu_circular_buffer_) 
    {
      accel_avg += measurement.accel;
      gyro_avg += measurement.gyro;
      accel_max = Vec3::max(accel_max, measurement.accel);
      accel_min = Vec3::min(accel_min, measurement.accel);
      gyro_max = Vec3::max(gyro_max, measurement.gyro);
      gyro_min = Vec3::min(gyro_min, measurement.gyro);
    }
    accel_avg = accel_avg / static_cast<double>(imu_circular_buffer_.size());
    gyro_avg = gyro_avg / static_cast<double>(imu_circular_buffer_.size());

    accel_max_diff = Vec3::max(Vec3::abs(accel_max - accel_avg),
                                Vec3::abs(accel_avg - accel_min));
    gyro_max_diff = Vec3::max(Vec3::abs(gyro_max - gyro_avg),
                              Vec3::abs(gyro_avg - gyro_min));

    if (NULL != accel_avg_in) 
    {
      *accel_avg_in = accel_avg;
    }

    // Reset the flag and unlock the mutex  
    buffer_is_full_ = false;  
    lock.unlock();  //释放锁
    //返回平均值
    return accel_avg;
  }


private:
  double imu_rate_hz_;          /// IMU message publishing rate, in Hz. Currently 50Hz on Husky2
  double observation_period_s_; /// How long does the robot need to be stationary to be declared stationary?
  double imu_max_rate_x_;       /// Maximum allowed difference between measurement and average until considered moving
  double imu_max_rate_y_;       /// Maximum allowed difference between measurement and average until considered moving
  double imu_max_rate_z_;       /// Maximum allowed difference between measurement and average until considered moving
  double imu_max_accel_x_;      /// Maximum allowed difference between measurement and average until considered moving
  double imu_max_accel_y_;      /// Maximum allowed difference between measurement and average until considered moving
  double imu_max_accel_z_;      /// Maximum allowed difference between measurement and average until considered moving
  boost::circular_buffer<ImuMeasurement> imu_circular_buffer_;  /// Circular buffer where IMU messages are stored

};

class VisualizationMarker
{
public:
  VisualizationMarker()
  {
    ros::NodeHandle nh;
    // Set up ROS publishers and subscribers
    imu_sub_ = nh.subscribe("/livox/imu", 200, &VisualizationMarker::imuCallback, this);
    gravity_pub_ = nh.advertise<visualization_msgs::Marker>("gravity", 100); // Debug topic
    average_gravity_pub_ = nh.advertise<visualization_msgs::Marker>("average_gravity", 100); // Debug topic
    
    sample_time_ = ros::Time::now();
  }  


  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) 
  {

    meas_handler.addImuMeasurement(msg);

    Vec3 accel_avg = Vec3(0, 0, 0);
    Vec3 accel_avg_in;
    //错误：accel_avg =  meas_handler.getStatus(&accel_avg);自己返回自己会造成coredumped
    accel_avg =  meas_handler.getStatus(&accel_avg_in);
    ROS_INFO("444");
    double diff = ros::Time::now().toSec() - sample_time_.toSec();
    
    
    { // Publish raw accelerometer arrow visualization
      Vec3 gravity(msg->linear_acceleration.x,
                   msg->linear_acceleration.y,
                   msg->linear_acceleration.z);      
      std_msgs::ColorRGBA color;
      color.a = 0.5;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      ROS_INFO_STREAM("Real gravity: " << gravity.x << " " <<
                    gravity.y << " " <<
                    gravity.z);
      visualization_msgs::Marker gravity_arrow = makeArrow("gravity", gravity, color);
      gravity_pub_.publish(gravity_arrow);
    }
    
    {//publish avg acc
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      ROS_INFO_STREAM("Average gravity: " << accel_avg.x << " " <<
                    accel_avg.y << " " <<
                    accel_avg.z);
      visualization_msgs::Marker average_gravity_arrow = makeArrow("average_gravity", accel_avg, color);
      average_gravity_pub_.publish(average_gravity_arrow);
    }
   
  }


  visualization_msgs::Marker makeArrow(const std::string &marker_namespace, const Vec3 &gravity, std_msgs::ColorRGBA color) 
  { 
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.ns = marker_namespace;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0;
    marker.points.clear();
    
    //初始端
    geometry_msgs::Point origin, gravity_direction;
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    //末端
    gravity_direction.x = -gravity.x;
    gravity_direction.y = -gravity.y;
    gravity_direction.z = -gravity.z;
    
    marker.points.push_back(origin);
    marker.points.push_back(gravity_direction);
    marker.color = color;
    
    return marker;
  }


private:
  
  ros::Subscriber imu_sub_;
  ros::Publisher status_pub_;
  ros::Publisher gravity_pub_;
  ros::Publisher average_gravity_pub_;
  ros::Time sample_time_;
  // MeasurementHandler* meas_handler;
  MeasurementHandler meas_handler;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualization_gravity_node");
  ros::NodeHandle nh;
  VisualizationMarker vsg_node;
  ROS_INFO("1111");
  ros::spin();
  return 0;
}

