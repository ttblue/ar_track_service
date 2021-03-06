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


#include "CvTestbed.h"
#include "MarkerDetector.h"
#include "Shared.h"
#include <cv_bridge/CvBridge.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <tf/transform_listener.h>

#include "ar_track_service/MarkerPositions.h"

namespace gm=geometry_msgs;
namespace ata=ar_track_alvar;

typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

using namespace alvar;
using namespace std;
using boost::make_shared;

bool init=true;
Camera *cam;
IplImage *capture_;
sensor_msgs::CvBridge bridge_;
ar_track_alvar::AlvarMarkers arPoseMarkers_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;

double marker_size, marker_size_stand;
double max_new_marker_error;
double max_track_error;
bool use_image = false;



// Hard coded value for PR2 Kinect
// Will change then when I shift to pcl 1.7 and can get camera intrinsics
// Or if available some other way
void setCameraInfo () {
  if (!cam->getCamInfo_)
    {
      cam->calib_K_data[0][0] = 525;
      cam->calib_K_data[1][1] = 525;
      cam->calib_K_data[0][2] = 319.5;
      cam->calib_K_data[1][2] = 239.5;

      cam->getCamInfo_ = true;
    }
}

int PlaneFitPoseImprovement(int id, const ARCloud &corners_3D, ARCloud::Ptr selected_points, const ARCloud &cloud, Pose &p){

  ata::PlaneFitResult res = ata::fitPlane(selected_points);
  gm::PoseStamped pose;
  pose.header.stamp = cloud.header.stamp;
  pose.header.frame_id = cloud.header.frame_id;
  pose.pose.position = ata::centroid(*res.inliers);

  //Get 2 points that point forward in marker x direction   
  int i1,i2;
  if(isnan(corners_3D[0].x) || isnan(corners_3D[0].y) || isnan(corners_3D[0].z) || 
     isnan(corners_3D[1].x) || isnan(corners_3D[1].y) || isnan(corners_3D[1].z))
    {
      if(isnan(corners_3D[2].x) || isnan(corners_3D[2].y) || isnan(corners_3D[2].z) || 
	 isnan(corners_3D[3].x) || isnan(corners_3D[3].y) || isnan(corners_3D[3].z))
	{
	  return -1;
	}
      else{
	i1 = 3;
	i2 = 2;
      }	
    }
  else{
    i1 = 0;
    i2 = 1;
  }

  //Get 2 points the point forward in marker y direction   
  int i3,i4;
  if(isnan(corners_3D[0].x) || isnan(corners_3D[0].y) || isnan(corners_3D[0].z) || 
     isnan(corners_3D[3].x) || isnan(corners_3D[3].y) || isnan(corners_3D[3].z))
    {
      if(isnan(corners_3D[1].x) || isnan(corners_3D[1].y) || isnan(corners_3D[1].z) || 
	 isnan(corners_3D[2].x) || isnan(corners_3D[2].y) || isnan(corners_3D[2].z))
	{
	  return -1;
	}
      else{
	i3 = 1;
	i4 = 2;
      }	
    }
  else{
    i3 = 0;
    i4 = 3;
  }

  int succ;
  succ = ata::extractOrientation(res.coeffs, corners_3D[i1], corners_3D[i2], corners_3D[i3], corners_3D[i4], pose.pose.orientation);
  if(succ < 0) return -1;

  btMatrix3x3 mat;
  succ = ata::extractFrame(res.coeffs, corners_3D[i1], corners_3D[i2], corners_3D[i3], corners_3D[i4], mat);
  if(succ < 0) return -1;

  p.translation[0] = pose.pose.position.x * 100.0;
  p.translation[1] = pose.pose.position.y * 100.0;
  p.translation[2] = pose.pose.position.z * 100.0;
  p.quaternion[1] = pose.pose.orientation.x;
  p.quaternion[2] = pose.pose.orientation.y;
  p.quaternion[3] = pose.pose.orientation.z;
  p.quaternion[0] = pose.pose.orientation.w;

  return 0;
}

int pub = 0;
std::vector<size_t> GetMarkerPoses(IplImage *image, ARCloud &cloud, bool track) {

  std::vector<size_t> good_markers;
  //Detect and track the markers
  if (marker_detector.Detect(image, cam, track, false, max_new_marker_error,
			     max_track_error, CVSEQ, true))
    {
      printf("\n--------------------------\n\n");
      pub = (pub+1)%10;
      std::cout<<pub<<std::endl;

      for (size_t i=0; i<marker_detector.markers->size(); i++)
	{
	  vector<cv::Point> pixels;
	  Marker *m = &((*marker_detector.markers)[i]);
	  int id = m->GetId();

	  // Problems with marker 0
	  if (id == 0) continue;

	  cout << "******* ID: " << id << endl;

	  int resol = m->GetRes();
	  int ori = m->ros_orientation;

	  PointDouble pt[4];
	  pt[3] = m->ros_marker_points_img[0];
	  pt[2] = m->ros_marker_points_img[resol-1];
	  pt[0] = m->ros_marker_points_img[(resol*resol)-resol];
	  pt[1] = m->ros_marker_points_img[(resol*resol)-1];
	  
	  m->ros_corners_3D[0] = cloud(pt[0].x, pt[0].y);
	  m->ros_corners_3D[1] = cloud(pt[1].x, pt[1].y);
	  m->ros_corners_3D[2] = cloud(pt[2].x, pt[2].y);
	  m->ros_corners_3D[3] = cloud(pt[3].x, pt[3].y);

	  if(ori >= 0 && ori < 4){
	    while(ori != 0){
	      ARPoint temp = m->ros_corners_3D[0];
	      m->ros_corners_3D[0] = m->ros_corners_3D[1];
	      m->ros_corners_3D[1] = m->ros_corners_3D[2];
	      m->ros_corners_3D[2] = m->ros_corners_3D[3];
	      m->ros_corners_3D[3] = temp;
	      
	      PointDouble temp2 = pt[0];
	      pt[0] = pt[1];
	      pt[1] = pt[2];
	      pt[2] = pt[3];
	      pt[3] = temp2;
	      
	      ori--;
	    }
	  }
	  else
	    ROS_ERROR("FindMarkerBundles: Bad Orientation: %i for ID: %i", ori, id);

	  //Get the 3D marker points
	  BOOST_FOREACH (const PointDouble& p, m->ros_marker_points_img)
	    pixels.push_back(cv::Point(p.x, p.y));
	  ARCloud::Ptr selected_points = ata::filterCloud(cloud, pixels);

	  //Use the kinect data to find a plane and pose for the marker
	  int ret = PlaneFitPoseImprovement(i, m->ros_corners_3D, selected_points, cloud, m->pose);
	  if (ret != -1 || use_image)
	    good_markers.push_back(i);
	}
    }
  return good_markers;
}



bool getMarkersCallback (ar_track_service::MarkerPositions::Request &req,
			 ar_track_service::MarkerPositions::Response &res)
{
  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

  if(init){
    setCameraInfo();
    CvSize sz_ = cvSize (cam->x_res, cam->y_res);
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
    init = false;
  }

  //If we've already gotten the cam info, then go ahead
  if(cam->getCamInfo_){
    //Convert cloud to PCL
    ARCloud cloud;
    pcl::fromROSMsg(req.pc, cloud);
    //Get an OpenCV image from the cloud
    pcl::toROSMsg (cloud, *image_msg);
    image_msg->header.stamp = req.pc.header.stamp;
    image_msg->header.frame_id = req.pc.header.frame_id;

    //Convert the image
    capture_ = bridge_.imgMsgToCv (image_msg, "rgb8");

    //Use the kinect to improve the pose
    std::vector<size_t> good_markers = GetMarkerPoses(capture_, cloud, req.track);

    try {
      arPoseMarkers_.markers.clear ();
      for (size_t j=0; j<good_markers.size();j++)
	{
	  size_t i = good_markers[j];
	  //Get the pose relative to the camera
	  int id = (*(marker_detector.markers))[i].GetId();
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
	  btTransform markerPose (rotation, origin); // marker pose in the camera frame

	  //Create the pose marker messages
	  ar_track_alvar::AlvarMarker ar_pose_marker;
	  tf::poseTFToMsg (markerPose, ar_pose_marker.pose.pose);
	  ar_pose_marker.header.frame_id = req.pc.header.frame_id;
	  ar_pose_marker.header.stamp = image_msg->header.stamp;
	  ar_pose_marker.id = id;
	  res.markers.markers.push_back (ar_pose_marker);
	}
    }
    catch (sensor_msgs::CvBridgeException & e){
      ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
      return false;
    }
  }
  return true;
}



int main(int argc, char *argv[])
{
  ros::init (argc, argv, "marker_detect_service");
  // Change accordingly
  marker_size = 3.8;

  max_new_marker_error = 0.08;
  max_track_error = 0.2;

  marker_detector.SetMarkerSize(3.8);
  //marker_detector.SetMarkerSize(marker_size);
  /*/ L gripper
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

  ros::NodeHandle n;
  ros::ServiceServer markerService =
    n.advertiseService ("getMarkers", getMarkersCallback);


  cam = new Camera();

  std::cout<<"Spawned service. Ready for request."<<std::endl;

  ros::Duration(1.0).sleep();
  ros::spin ();

  return 0;
}
