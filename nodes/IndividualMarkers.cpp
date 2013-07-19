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

double marker_size;
double max_new_marker_error;
double max_track_error;



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
			isnan(corners_3D[3].x) || isnan(corners_3D[3].y) || isnan(corners_3D[3].z))
	{
		if(isnan(corners_3D[1].x) || isnan(corners_3D[1].y) || isnan(corners_3D[1].z) ||
				isnan(corners_3D[2].x) || isnan(corners_3D[2].y) || isnan(corners_3D[2].z))
		{
			return -1;
		}
		else{
			i1 = 1;
			i2 = 2;
		}
	}
	else{
		i1 = 0;
		i2 = 3;
	}

	//Get 2 points the point forward in marker y direction
	int i3,i4;
	if(isnan(corners_3D[0].x) || isnan(corners_3D[0].y) || isnan(corners_3D[0].z) ||
			isnan(corners_3D[1].x) || isnan(corners_3D[1].y) || isnan(corners_3D[1].z))
	{
		if(isnan(corners_3D[3].x) || isnan(corners_3D[3].y) || isnan(corners_3D[3].z) ||
				isnan(corners_3D[2].x) || isnan(corners_3D[2].y) || isnan(corners_3D[2].z))
		{
			return -1;
		}
		else{
			i3 = 2;
			i4 = 3;
		}
	}
	else{
		i3 = 1;
		i4 = 0;
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


void GetMarkerPoses(IplImage *image, ARCloud &cloud) {

	//Detect and track the markers
	if (marker_detector.Detect(image, cam, true, false, max_new_marker_error,
			max_track_error, CVSEQ, true))
	{
		printf("\n--------------------------\n\n");
		for (size_t i=0; i<marker_detector.markers->size(); i++)
		{
			vector<cv::Point> pixels;
			Marker *m = &((*marker_detector.markers)[i]);
			int id = m->GetId();
			cout << "******* ID: " << id << endl;

			int resol = m->GetRes();
			int ori = m->ros_orientation;

			PointDouble pt1, pt2, pt3, pt4;
			pt4 = m->ros_marker_points_img[0];
			pt3 = m->ros_marker_points_img[resol-1];
			pt1 = m->ros_marker_points_img[(resol*resol)-resol];
			pt2 = m->ros_marker_points_img[(resol*resol)-1];

			m->ros_corners_3D[0] = cloud(pt1.x, pt1.y);
			m->ros_corners_3D[1] = cloud(pt2.x, pt2.y);
			m->ros_corners_3D[2] = cloud(pt3.x, pt3.y);
			m->ros_corners_3D[3] = cloud(pt4.x, pt4.y);

			if(ori >= 0 && ori < 4){
				if(ori != 0){
					std::rotate(m->ros_corners_3D.begin(), m->ros_corners_3D.begin() + ori, m->ros_corners_3D.end());
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
		}
	}
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
		GetMarkerPoses(capture_, cloud);

		try {
			arPoseMarkers_.markers.clear ();
			for (size_t i=0; i<marker_detector.markers->size(); i++)
			{
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
				btTransform t (rotation, origin);
				btVector3 markerOrigin (0, 0, 0);
				btTransform m (btQuaternion::getIdentity (), markerOrigin);
				btTransform markerPose = t * m; // marker pose in the camera frame

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
	ros::init (argc, argv, "marker_detect");
	ros::NodeHandle n;
	ros::ServiceServer markerService =
			n.advertiseService ("getMarkers", getMarkersCallback);
	// Change accordingly
	marker_size = 3.8;

	// Not sure what these values are
	max_new_marker_error = 0.08;
	max_track_error = 0.2;
	marker_detector.SetMarkerSize(marker_size);

	cam = new Camera();

	std::cout<<"Spawned service. Ready for request."<<std::endl;

	ros::Duration(1.0).sleep();
	ros::spin ();

	return 0;
}
