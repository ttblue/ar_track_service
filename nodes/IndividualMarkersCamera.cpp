/*
  Software License Agreement (BSD License)

  Copyright (c) 2012, Scott Niekum
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  * Neither the name of the Willow Garage nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  author: Scott Niekum
*/


#include <fstream>
#include <sstream>
#include <string>
#include "CvTestbed.h"
#include "MarkerDetector.h"
#include "Shared.h"
#include <cv_bridge/CvBridge.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <tf/transform_listener.h>

using namespace alvar;
using namespace std;

bool init=true;
Camera *cam;
IplImage *capture_;
sensor_msgs::CvBridge bridge_;
image_transport::Subscriber cam_sub_;
ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
ar_track_alvar::AlvarMarkers arPoseMarkers_;
visualization_msgs::Marker rvizMarker_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;

int pub = 0;

double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string output_frame;
std::string unique_cam_name;
std::string calib_file;

void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);

// hard coded values
void setCameraInfo () {

  if (!cam->getCamInfo_) {
    int w, h;
    float k1, k2, k3, k4;
    float d1, d2, d3, d4;
      
    ifstream calib (calib_file.c_str());
    if (calib) {
      string line;

      getline(calib, line);
      istringstream in_wh(line);
      in_wh >> w;
      in_wh >> h;

      getline(calib, line);
      istringstream in_k(line);
      in_k >> k1;
      in_k >> k2;
      in_k >> k3;
      in_k >> k4;

      getline(calib, line);
      istringstream in_d(line);
      in_d >> d1;
      in_d >> d2;
      in_d >> d3;
      in_d >> d4;
      
      calib.close();
      
    } else {
      w = 1280;
      h = 720;

      k1 = 984.754843;
      k2 = 981.964366; 
      k3 = 648.881167;
      k4 = 354.697205;
      
      d1 = 0.000995;
      d2 = -0.055592;
      d3 = 0.001255;
      d4 = 0.001741;

    }
    cam->calib_x_res = w;
    cam->calib_y_res = h;
    cam->x_res = w;
    cam->y_res = h;
    
    cam->calib_K_data[0][0] = k1;
    cam->calib_K_data[1][1] = k2;
    cam->calib_K_data[0][2] = k3;
    cam->calib_K_data[1][2] = k4;
    
    cam->calib_D_data[0] = d1;
    cam->calib_D_data[1] = d2;
    cam->calib_D_data[2] = d3;
    cam->calib_D_data[3] = d4;

    cam->getCamInfo_ = true;
  } 
}



void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
  if(init){
    setCameraInfo();
    CvSize sz_ = cvSize (cam->x_res, cam->y_res);
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
    init = false;	
  }

  //If we've already gotten the cam info, then go ahead
  if(cam->getCamInfo_){
    try{

      capture_ = bridge_.imgMsgToCv (image_msg, "rgb8");
      marker_detector.Detect(capture_, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);

      arPoseMarkers_.markers.clear ();
      if (marker_detector.markers->size() > 0) {
	printf("\n--------------------------\n\n");
	pub = (pub+1)%10;
	std::cout<<pub<<std::endl;
      }
      for (size_t i=0; i<marker_detector.markers->size(); i++) 
	{
	  //Get the pose relative to the camera
	  int id = (*(marker_detector.markers))[i].GetId(); 

	  cout << "******* ID: " << id << endl;

	  Pose p = (*(marker_detector.markers))[i].pose;
	  double px = p.translation[0]/100.0;
	  double py = p.translation[1]/100.0;
	  double pz = p.translation[2]/100.0;
	  double qx = p.quaternion[1];
	  double qy = p.quaternion[2];
	  double qz = p.quaternion[3];
	  double qw = p.quaternion[0];

	  btQuaternion rotation (qx,qy,qz,qw);
	  btVector3 origin (px,py,pz);
	  btTransform t (rotation, origin);
	  btVector3 markerOrigin (0, 0, 0);
	  btTransform m (btQuaternion::getIdentity (), markerOrigin);
	  btTransform markerPose = t * m; // marker pose in the camera frame

	  //Publish the transform from the camera to the marker		
	  std::string markerFrame = "ar_marker";
	  std::stringstream out;
	  out << id;
	  std::string id_string = out.str();
	  markerFrame += id_string + unique_cam_name;
	  tf::StampedTransform camToMarker (t, image_msg->header.stamp, output_frame, markerFrame.c_str());
	  tf_broadcaster->sendTransform(camToMarker);
				
	  //Create the rviz visualization messages
	  tf::poseTFToMsg (markerPose, rvizMarker_.pose);
	  rvizMarker_.header.frame_id = output_frame;
	  rvizMarker_.header.stamp = image_msg->header.stamp;
	  rvizMarker_.id = id;

	  rvizMarker_.scale.x = 1.0 * marker_size/100.0;
	  rvizMarker_.scale.y = 1.0 * marker_size/100.0;
	  rvizMarker_.scale.z = 0.2 * marker_size/100.0;
	  rvizMarker_.ns = "basic_shapes";
	  rvizMarker_.type = visualization_msgs::Marker::CUBE;
	  rvizMarker_.action = visualization_msgs::Marker::ADD;
	  switch (id)
	    {
	    case 0:
	      rvizMarker_.color.r = 0.0f;
	      rvizMarker_.color.g = 0.0f;
	      rvizMarker_.color.b = 1.0f;
	      rvizMarker_.color.a = 1.0;
	      break;
	    case 1:
	      rvizMarker_.color.r = 1.0f;
	      rvizMarker_.color.g = 0.0f;
	      rvizMarker_.color.b = 0.0f;
	      rvizMarker_.color.a = 1.0;
	      break;
	    case 2:
	      rvizMarker_.color.r = 0.0f;
	      rvizMarker_.color.g = 1.0f;
	      rvizMarker_.color.b = 0.0f;
	      rvizMarker_.color.a = 1.0;
	      break;
	    case 3:
	      rvizMarker_.color.r = 0.0f;
	      rvizMarker_.color.g = 0.5f;
	      rvizMarker_.color.b = 0.5f;
	      rvizMarker_.color.a = 1.0;
	      break;
	    case 4:
	      rvizMarker_.color.r = 0.5f;
	      rvizMarker_.color.g = 0.5f;
	      rvizMarker_.color.b = 0.0;
	      rvizMarker_.color.a = 1.0;
	      break;
	    default:
	      rvizMarker_.color.r = 0.5f;
	      rvizMarker_.color.g = 0.0f;
	      rvizMarker_.color.b = 0.5f;
	      rvizMarker_.color.a = 1.0;
	      break;
	    }
	  rvizMarker_.lifetime = ros::Duration (1.0);
	  rvizMarkerPub_.publish (rvizMarker_);

	  //Create the pose marker messages
	  ar_track_alvar::AlvarMarker ar_pose_marker;
	  tf::poseTFToMsg (markerPose, ar_pose_marker.pose.pose);
	  ar_pose_marker.header.frame_id = output_frame;
	  ar_pose_marker.header.stamp = image_msg->header.stamp;
	  ar_pose_marker.id = id;
	  arPoseMarkers_.markers.push_back (ar_pose_marker);	
	}
      arPoseMarkers_.header.frame_id = output_frame;
      arPoseMarkers_.header.stamp = image_msg->header.stamp;
      arMarkerPub_.publish (arPoseMarkers_);
    }
    catch (sensor_msgs::CvBridgeException & e){
      ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    }
  }
}


int main(int argc, char *argv[])
{
  ros::init (argc, argv, "marker_detect");
  ros::NodeHandle n;
	
  if(argc < 6){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./individualMarkers <marker size in cm> <max new marker error> <max track error> <cam image topic> <output frame> [<calib_file>]" << endl;
    std::cout << std::endl;
    return 0;
  }

  // Get params from command line
  marker_size = atof(argv[1]);
  max_new_marker_error = atof(argv[2]);
  max_track_error = atof(argv[3]);
  cam_image_topic = argv[4];
  output_frame = argv[5];
  if (argc > 6) 
    calib_file = argv[6];
  //marker_detector.SetMarkerSize(marker_size);

  marker_detector.SetMarkerSize(3.8);
  /*
  // L gripper
  marker_detector.SetMarkerSizeForId(4, 6.5);
  marker_detector.SetMarkerSizeForId(13, 3.8);
  marker_detector.SetMarkerSizeForId(15, 6.5);
  marker_detector.SetMarkerSizeForId(3, 2.9);
  marker_detector.SetMarkerSizeForId(6, 2.9);
  marker_detector.SetMarkerSizeForId(10, 2.9);
  marker_detector.SetMarkerSizeForId(11, 2.9);*/
  marker_detector.SetMarkerSizeForId(1, 7.0);
  marker_detector.SetMarkerSizeForId(5, 7.0);
  //marker_detector.SetMarkerSizeForId(7, 16);

  // Camera calib
  marker_detector.SetMarkerSizeForId(0, 20.3);
  marker_detector.SetMarkerSizeForId(2, 20.3);
  marker_detector.SetMarkerSizeForId(3, 20.25);
  marker_detector.SetMarkerSizeForId(30, 20.3);
  marker_detector.SetMarkerSizeForId(31, 20.3);
  marker_detector.SetMarkerSizeForId(32, 20.3);
  marker_detector.SetMarkerSizeForId(33, 20.3);

  size_t i1,i2;
  i1 = cam_image_topic.find('/');
  i2 = cam_image_topic.find('/',1);
  if (i1 == i2) i1 = 0;
  else i1 = 1;
	
  unique_cam_name = "_" + cam_image_topic.substr(i1, i2-i1);

  cam = new Camera();
  tf_listener = new tf::TransformListener(n);
  tf_broadcaster = new tf::TransformBroadcaster();
  arMarkerPub_ = n.advertise < ar_track_alvar::AlvarMarkers > ("ar_pose_marker"+unique_cam_name, 0);
  rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker"+unique_cam_name, 0);
	
  //Give tf a chance to catch up before the camera callback starts asking for transforms
  ros::Duration(1.0).sleep();
  ros::spinOnce();	
	 
  ROS_INFO ("Subscribing to image topic");
  image_transport::ImageTransport it_(n);
  cam_sub_ = it_.subscribe (cam_image_topic, 1, &getCapCallback);

  ros::spin ();

  return 0;
}
