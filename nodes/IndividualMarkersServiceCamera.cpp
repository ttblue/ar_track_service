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
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include "ar_track_service/MarkerImagePositions.h"


using namespace alvar;
using namespace std;

bool init=true;
Camera *cam;
IplImage *capture_;
cv_bridge::CvImage m_;
sensor_msgs::CvBridge bridge_;
image_transport::Subscriber cam_sub_;
ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
ar_track_alvar::AlvarMarkers arPoseMarkers_;
visualization_msgs::Marker rvizMarker_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;

double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string output_frame;
std::string unique_cam_name;
std::string calib_file("");

void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);

void setCameraInfo () {

  if (!cam->getCamInfo_) {
    int w, h;
    float k1, k2, k3, k4;
    float d1, d2, d3, d4;
      
    calib_file = getenv("CAMERA_CALIB_DIR") + std::string("/") + calib_file;
    std::cout<<"Calibration file: "<<calib_file<<std::endl;
    ifstream calib;
    calib.open(calib_file.c_str());
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
      h = 1024;

      k1 = 1076.432831;
      k2 = 1073.943676; 
      k3 = 634.514167;
      k4 = 515.852599;
      
      d1 = 0.045975;
      d2 = -0.143820;
      d3 = 0.000710;
      d4 = 0.000672;

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

bool getCapCallback (ar_track_service::MarkerImagePositions::Request &req,
		     ar_track_service::MarkerImagePositions::Response &res) {
  if(init) {
    setCameraInfo();
    CvSize sz_ = cvSize (cam->x_res, cam->y_res);
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
    init = false;	
  }

  //If we've already gotten the cam info, then go ahead
  if(cam->getCamInfo_) {
    //capture_ = bridge_.imgMsgToCv (req.img, "rgb8");
    //    sensor_msg::Image copy_img;
    //memcpy(right_image_.data[0],  DSIF->getRImage(), image_size);
    sensor_msgs::ImageConstPtr img_ptr (new sensor_msgs::Image(req.img));
    capture_ = bridge_.imgMsgToCv (img_ptr, "rgb8");
    //    m_ = *cv_bridge::toCvCopy(req.img, "rgb8");
    //capture_ = new IplImage(m_.image);
    
    marker_detector.Detect(capture_, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);

    arPoseMarkers_.markers.clear ();
    printf("\n--------------------------\n\n");

    for (size_t i=0; i<marker_detector.markers->size(); i++) {
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

      cout <<"Vals: "<<px<<" "<<py<<" "<<pz<<endl;

      btQuaternion rotation (qx,qy,qz,qw);
      btVector3 origin (px,py,pz);
      btTransform t (rotation, origin);
      btVector3 markerOrigin (0, 0, 0);
      btTransform m (btQuaternion::getIdentity (), markerOrigin);
      btTransform markerPose = t * m; // marker pose in the camera frame

      //Create the pose marker messages
      ar_track_alvar::AlvarMarker ar_pose_marker;
      tf::poseTFToMsg (markerPose, ar_pose_marker.pose.pose);
      ar_pose_marker.header.frame_id = req.img.header.frame_id;
      ar_pose_marker.header.stamp = req.img.header.stamp;
      ar_pose_marker.id = id;
      res.markers.markers.push_back (ar_pose_marker);	
    }
  }
  return true;
}


int main(int argc, char *argv[])
{

  ros::init (argc, argv, "marker_image_detect_service");

  if (argc > 1)
    calib_file = argv[1];
  
  max_new_marker_error = 0.08;
  max_track_error = 0.2;

  marker_detector.SetMarkerSize(3.8);
  marker_detector.SetMarkerSizeForId(1, 7.0);
  marker_detector.SetMarkerSizeForId(5, 7.0);

  // Camera calib
  marker_detector.SetMarkerSizeForId(0, 20.3);
  marker_detector.SetMarkerSizeForId(2, 20.3);
  marker_detector.SetMarkerSizeForId(3, 20.25);
  marker_detector.SetMarkerSizeForId(30, 20.3);
  marker_detector.SetMarkerSizeForId(31, 20.3);
  marker_detector.SetMarkerSizeForId(32, 20.3);
  marker_detector.SetMarkerSizeForId(33, 20.3);


  ros::NodeHandle n;
  ros::ServiceServer markerService =
    n.advertiseService ("getImageMarkers", getCapCallback);

  cam = new Camera();

  std::cout<<"Spawned service. Ready for request."<<std::endl;	

  ros::Duration(1.0).sleep();
  ros::spin ();

  return 0;
}
