/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <brain_box_msgs/BBLatency.h>
#include <brain_box_msgs/BBPose.h>
#include <brain_box_msgs/BBVisionStatus.h>
#include <boost/array.hpp>

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher bbpose_pub;
  ros::Publisher transform_pub; 
  ros::Publisher position_pub;
  ros::Publisher status_pub;
  ros::Publisher indicator_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;
  brain_box_msgs::BBVisionStatus status;

  double marker_size;
  int marker_id;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

public:
  ArucoSimple()
    : cam_info_received(false),
      nh("~"),
      it(nh)
  {
    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    bbpose_pub = nh.advertise<brain_box_msgs::BBPose>("bbpose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    status_pub = nh.advertise<brain_box_msgs::BBVisionStatus>("/status/vision", 100);
    indicator_pub = nh.advertise<std_msgs::Int32>("/indicator/green", 100);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 300);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
             marker_size, marker_id);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             reference_frame.c_str(), marker_frame.c_str());

    status.targets_acquired = 0;
  }

  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform)
  {
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(0.5),
                                       ros::Duration(0.01),
                                       &errMsg)
         )
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform( refFrame, childFrame,
                                     ros::Time(0),                  //get latest available
                                     transform);
      }
      catch ( const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }


  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      ros::Time in_time = ros::Time::now();
      ros::Time image_stamp = msg.get()->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        //detection results will go into "markers"
        markers.clear();
        //Ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        //for each marker, draw info and its boundaries in the image
        uint16_t targets_acquired = 0;
        ros::Time out_time;
        for(size_t i=0; i<markers.size(); ++i)
        {
          // only publishing the selected marker
          if(markers[i].id == marker_id)
          {
        	targets_acquired++;
            tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
            tf::StampedTransform cameraToReference;
            cameraToReference.setIdentity();

            if ( reference_frame != camera_frame )
            {
              getTransform(reference_frame,
                           camera_frame,
                           cameraToReference);
            }

            transform = 
              static_cast<tf::Transform>(cameraToReference) 
              * static_cast<tf::Transform>(rightToLeft) 
              * transform;

            tf::StampedTransform stampedTransform(transform, ros::Time::now(),
                                                  reference_frame, marker_frame);
            br.sendTransform(stampedTransform);
            brain_box_msgs::BBPose bbPoseMsg;
            tf::poseTFToMsg(transform, bbPoseMsg.pose);

            // transform to ned
            // TODO: move this out of aruco code to somewhere else
            double oldX = bbPoseMsg.pose.position.x;
            double oldY = bbPoseMsg.pose.position.y;
            double oldZ = bbPoseMsg.pose.position.z;
            bbPoseMsg.pose.position.x = oldZ;
            bbPoseMsg.pose.position.z = -oldY;
            bbPoseMsg.pose.position.y = -oldX;

            out_time = ros::Time::now();
            bbPoseMsg.header.frame_id = reference_frame;
            bbPoseMsg.header.stamp = image_stamp;
            // IP_ARUCO_POINTGREY_OUT=0
            bbPoseMsg.latency.image_pipeline_stamp0 = image_stamp;
            // IP_ARUCO_ARUCO_IN=3
            bbPoseMsg.latency.image_pipeline_stamp3 = in_time;
            // IP_ARUCO_ARUCO_OUT=4
            bbPoseMsg.latency.image_pipeline_stamp4 = out_time;
            bbpose_pub.publish(bbPoseMsg);


            geometry_msgs::PoseStamped poseMsg;
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = image_stamp;
            // convert to enu
            poseMsg.pose.position.x = bbPoseMsg.pose.position.y;
            poseMsg.pose.position.y = bbPoseMsg.pose.position.x;
            poseMsg.pose.position.z = -bbPoseMsg.pose.position.z;
            poseMsg.pose.orientation = bbPoseMsg.pose.orientation;
            pose_pub.publish(poseMsg);

//            geometry_msgs::TransformStamped transformMsg;
//            tf::transformStampedTFToMsg(stampedTransform, transformMsg);
//            transform_pub.publish(transformMsg);
//
//            geometry_msgs::Vector3Stamped positionMsg;
//            positionMsg.header = transformMsg.header;
//            positionMsg.vector = transformMsg.transform.translation;
//            position_pub.publish(positionMsg);
          }
        }

        if(targets_acquired != status.targets_acquired)
        {
        	ROS_INFO_STREAM("targets_acquired = " << targets_acquired);
        	status.targets_acquired = targets_acquired;
        }
        status.status = status.STATUS_GREEN;
        uint16_t latency_ms = (out_time.toNSec() - image_stamp.toNSec()) / 1000;
        status.latency_ave = latency_ms;
        status.latency_min = latency_ms;
        status.latency_max = latency_ms;
        status.fps_ave = 1000000 / latency_ms;
        status.fps_min = 1000000 / latency_ms;
        status.fps_max = 1000000 / latency_ms;
    	status_pub.publish(status);

    	if(targets_acquired > 0)
    	{
    		std_msgs::Int32 value;
    		value.data = 8;
    		indicator_pub.publish(value);
    	}
    	else
    	{
    		std_msgs::Int32 value;
    		value.data = 0;
    		indicator_pub.publish(value);
    	}

        if(image_pub.getNumSubscribers() > 0)
        {
			//draw a 3d cube in each marker if there is 3d info
			if(camParam.isValid() && marker_size!=-1)
			{
			  for(size_t i=0; i<markers.size(); ++i)
			  {
				markers[i].draw(inImage,cv::Scalar(0,0,255),2);
				CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
			  }
			}

            //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = ros::Time::now();
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if(debug_pub.getNumSubscribers() > 0)
        {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = ros::Time::now();
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
