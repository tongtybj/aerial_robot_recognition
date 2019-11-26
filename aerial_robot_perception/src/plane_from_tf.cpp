// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PlaneFromTF
{
public:
  PlaneFromTF(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), plane_coeffients_in_cutting_frame_(4), plane_polygon_in_cutting_frame_(0)
  {
    nhp_.param("base_frame", base_frame_, std::string("unknown"));
    nhp_.param("cutting_frame", cutting_frame_, std::string("unknown"));

    plane_coeffients_in_cutting_frame_.at(2) = 1; // a = 0, b = 0, c = 1, d = 0
    if(nhp_.hasParam("plane_coeffients_in_cutting_frame"))
      {
        nhp_.getParam("plane_coeffients_in_cutting_frame", plane_coeffients_in_cutting_frame_); /* d */
      }

    geometry_msgs::Point point;
    plane_polygon_in_cutting_frame_.push_back(point);
    point.x = 1;
    point.y = 1;
    plane_polygon_in_cutting_frame_.push_back(point);
    point.x = 2;
    point.y = 0;
    plane_polygon_in_cutting_frame_.push_back(point);
    point.x = 1;
    point.y = -1;
    plane_polygon_in_cutting_frame_.push_back(point);

    XmlRpc::XmlRpcValue polygon_param;
    if(nhp_.hasParam("polygon_param"))
      {
        plane_polygon_in_cutting_frame_.resize(0);
        nhp_.getParam("polygon_param", polygon_param);

        for (int32_t i = 0; i < polygon_param.size(); ++i)
          {
            ROS_ASSERT(polygon_param[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            point.x = static_cast<double>(polygon_param[i][0]);
            point.y = static_cast<double>(polygon_param[i][1]);
            point.z = static_cast<double>(polygon_param[i][2]);
            plane_polygon_in_cutting_frame_.push_back(point);
          }
      }

    nhp_.param("no_bias", no_bias_, false); /* d */

    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);

    plane_polygon_pub_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>("tf_plane_polygon", 1);
    plane_coefficient_pub_ = nh_.advertise<jsk_recognition_msgs::ModelCoefficientsArray>("tf_plane_coefficient", 1);

    double pub_rate;
    nhp_.param("pub_rate", pub_rate, 1.0);
    timer_ = nhp_.createTimer(ros::Duration(1.0 / pub_rate), &PlaneFromTF::func,this);
  }

  ~PlaneFromTF(){}

  void func(const ros::TimerEvent & e)
  {
    tf2::Transform tf_base2cutting;
    geometry_msgs::TransformStamped pose_msg;
    try{
      pose_msg = tf_buff_.lookupTransform(base_frame_, cutting_frame_, ros::Time(0));
      tf2::convert(pose_msg.transform, tf_base2cutting);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }

    /* transform the plane coefficients from cutting frame to the base frame */
    tf2::Vector3 plane_coeff_abc_in_cutting_frame(plane_coeffients_in_cutting_frame_.at(0),
                                                  plane_coeffients_in_cutting_frame_.at(1),
                                                  plane_coeffients_in_cutting_frame_.at(2));

    tf2::Vector3 plane_coeff_abc_in_base_frame = tf_base2cutting.getBasis() * plane_coeff_abc_in_cutting_frame;
    double plane_coeff_d_in_base_frame = 0;
    if (!no_bias_) plane_coeff_d_in_base_frame = plane_coeff_abc_in_cutting_frame.dot(tf_base2cutting.inverse().getOrigin());

    jsk_recognition_msgs::PolygonArray polygon_msg;
    polygon_msg.header = pose_msg.header;
    polygon_msg.header.frame_id = base_frame_;
    geometry_msgs::PolygonStamped target_polygon;
    target_polygon.header = polygon_msg.header;
    for(auto& raw_point: plane_polygon_in_cutting_frame_)
      {
        tf2::Vector3 raw_point_v, point_v;
        tf2::fromMsg(raw_point, raw_point_v);
        if(no_bias_)
          point_v = tf_base2cutting.getBasis() * raw_point_v;
        else
          point_v = tf_base2cutting * raw_point_v;
        geometry_msgs::Point32 point;
        point.x = point_v.x();
        point.y = point_v.y();
        point.z = point_v.z();
        target_polygon.polygon.points.push_back(point);
      }

    polygon_msg.polygons.push_back(target_polygon);
    plane_polygon_pub_.publish(polygon_msg);

    jsk_recognition_msgs::ModelCoefficientsArray coefficients_msg;
    coefficients_msg.header = polygon_msg.header;
    pcl_msgs::ModelCoefficients target_coefficients;
    target_coefficients.header = polygon_msg.header;
    target_coefficients.values.push_back(plane_coeff_abc_in_base_frame.x()); // a x
    target_coefficients.values.push_back(plane_coeff_abc_in_base_frame.y()); // b y
    target_coefficients.values.push_back(plane_coeff_abc_in_base_frame.z()); // c z
    target_coefficients.values.push_back(plane_coeff_d_in_base_frame); // d
    coefficients_msg.coefficients.push_back(target_coefficients);
    plane_coefficient_pub_.publish(coefficients_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Timer timer_;
  ros::Publisher plane_polygon_pub_;
  ros::Publisher plane_coefficient_pub_;

  tf2_ros::Buffer tf_buff_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_ls_;

  bool no_bias_;
  std::vector<double> plane_coeffients_in_cutting_frame_;
  std::vector<geometry_msgs::Point> plane_polygon_in_cutting_frame_;
  std::string base_frame_, cutting_frame_;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "plane_from_tf");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  PlaneFromTF*  plane_from_tf = new PlaneFromTF(nh, nhp);
  ros::spin();

  delete plane_from_tf;
  return 0;
}
