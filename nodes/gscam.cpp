#include <stdlib.h>
#include <unistd.h>

#include <iostream>
extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>

#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>

//forward declarations
static gboolean processData(GstPad *pad, GstBuffer *buffer, gpointer u_data);
bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);

//globals
bool gstreamerPad, rosPad;
int width, height;
sensor_msgs::CameraInfo camera_info;

int main(int argc, char** argv) {

  ros::init(argc, argv, "gscam_publisher");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_pub;


  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  std::string gsconfig_rosparam = "";
  bool gsconfig_rosparam_defined = false;
  char *gsconfig_env = NULL;
  std::string config;

  gsconfig_rosparam_defined = nh.getParam("gscam_config",gsconfig_rosparam);
  ROS_INFO(gsconfig_rosparam.c_str());
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    ROS_FATAL( "Problem getting GSCAM_CONFIG environment variable and 'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline." );
    return false;
  } else if(gsconfig_env && gsconfig_rosparam_defined) {
    ROS_FATAL( "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. Please only define one." );
    return false;
  } else if(gsconfig_env) {
    config = std::string(gsconfig_env);
    ROS_INFO_STREAM("Using gstreamer config from env: \""<<gsconfig_env<<"\"");
  } else if(gsconfig_rosparam_defined) {
    config = gsconfig_rosparam;
    ROS_INFO_STREAM("Using gstreamer config from rosparam: \""<<gsconfig_rosparam<<"\"");
  }
  
  gst_init(0,0);
  std::cout << "Gstreamer Version: " << gst_version_string() << std::endl;

  GError *error = 0; //assignment to zero is a gst requirement
  GstElement *pipeline = gst_parse_launch(config.c_str(),&error);
  if (pipeline == NULL) {
    std::cout << error->message << std::endl;
    exit(-1);
  }

  bool sync_sink;
  bool preroll;
  bool use_gst_timestamps;
  std::string camera_info_url;
  std::string camera_name;

  // Get additional gscam configuration
  nh.param("sync_sink", sync_sink, true);
  nh.param("preroll", preroll, false);
  nh.param("use_gst_timestamps", use_gst_timestamps, false);
  
  // Get the camera parameters file
  nh.getParam("camera_info_url", camera_info_url);
  nh.getParam("camera_name", camera_name);



  GstElement * sink = gst_element_factory_make("appsink",NULL);
  GstCaps * caps = gst_caps_new_simple("video/x-raw-rgb", NULL);
  gst_app_sink_set_caps(GST_APP_SINK(sink), caps);
  gst_caps_unref(caps);

  gst_base_sink_set_sync(GST_BASE_SINK(sink), (sync_sink) ? TRUE : FALSE);

  if(GST_IS_PIPELINE(pipeline)) {
    GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline), GST_PAD_SRC);
    g_assert(outpad);
    GstElement *outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);


    if(!gst_bin_add(GST_BIN(pipeline), sink)) {
      fprintf(stderr, "gst_bin_add() failed\n"); // TODO: do some unref
      gst_object_unref(outelement);
      gst_object_unref(pipeline);
      return -1;
    }

    if(!gst_element_link(outelement, sink)) {
      fprintf(stderr, "GStreamer: cannot link outelement(\"%s\") -> sink\n", gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline);
      return -1;
    }

    gst_object_unref(outelement);
  } else {
    GstElement* launchpipe = pipeline;
    pipeline = gst_pipeline_new(NULL);
    g_assert(pipeline);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline), launchpipe, sink, NULL);

    if(!gst_element_link(launchpipe, sink)) {
      ROS_FATAL("GStreamer: cannot link launchpipe -> sink");
      gst_object_unref(pipeline);
      return -1;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  ros::Time now = ros::Time::now();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  double time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct)/1e6;
  ROS_INFO("Time offset: %.3f",time_offset_);

  gst_element_set_state(pipeline, GST_STATE_PAUSED);
  
  if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    ROS_FATAL("Failed to PAUSE stream, check your gstreamer configuration.");
    return -1;
  } else {
    ROS_DEBUG_STREAM("Stream is PAUSED.");
  }

  // We could probably do something with the camera name, check
  // errors or something, but at the moment, we don't care.
  std::string camera_name_;
  if (camera_calibration_parsers::readCalibrationIni(camera_info_url.c_str(), camera_name_, camera_info)) {
    ROS_INFO("Successfully read camera calibration.  Rerun camera calibrator if it is incorrect.");
  }
  else {
    ROS_ERROR("No camera_parameters.txt file found.  Use default file if no other is available.");
  }


  if (preroll) {
    ROS_DEBUG("Performing preroll...");
    //The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    //I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Failed to PLAY during preroll.");
      return -1 ;
    } else {
      ROS_DEBUG("Stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipeline, GST_STATE_PAUSED);

    if (gst_element_get_state(pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      ROS_ERROR("Failed to PAUSE.");
        return -1;
      } else {
        ROS_INFO("Stream is PAUSED in preroll.");
      }  
  }

  image_transport::ImageTransport it(nh_pub);
  image_transport::CameraPublisher pub = it.advertiseCamera(camera_name+std::string("/image_raw"), 1);

  ros::ServiceServer set_camera_info = nh.advertiseService("gscam/set_camera_info", setCameraInfo);

  // Get TF Frame
  std::string frame_id;
  if(!nh.getParam("frame_id",frame_id)){
    frame_id = "/camera_frame";
    ROS_WARN_STREAM("No camera frame_id set, using frame \""<<frame_id<<"\".");
    nh.setParam("frame_id",frame_id);
  }


  std::cout << "Processing..." << std::endl;

  //processVideo
  rosPad = false;
  gstreamerPad = true;
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  while(nh.ok()) {
    // This should block until a new frame is awake, this way, we'll run at the 
    // actual capture framerate of the device.
    GstBuffer* buf = gst_app_sink_pull_buffer(GST_APP_SINK(sink));
    GstClockTime bt = gst_element_get_base_time(pipeline);
    if (!buf) break;

    GstPad* pad = gst_element_get_static_pad(sink, "sink");
    const GstCaps *caps = gst_pad_get_negotiated_caps(pad);
    GstStructure *structure = gst_caps_get_structure(caps,0);
    gst_structure_get_int(structure,"width",&width);
    gst_structure_get_int(structure,"height",&height);

    sensor_msgs::Image msg;
    msg.width = width; 
    msg.height = height;
    if (use_gst_timestamps) {
      msg.header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->timestamp+bt)/1e6+time_offset_);
    } else {
      msg.header.stamp = ros::Time::now();
    }
    msg.header.frame_id = frame_id;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width*3;
    msg.data.resize(width*height*3);
    std::copy(buf->data, buf->data+(width*height*3), msg.data.begin());

    pub.publish(msg, camera_info);

    gst_buffer_unref(buf);

    ros::spinOnce();

  }

  //close out
  std::cout << "\nquitting..." << std::endl;
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);

  return 0;
}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {

  ROS_INFO("New camera info received");
  camera_info = req.camera_info;

  if (camera_calibration_parsers::writeCalibrationIni("../camera_parameters.txt", "gscam", camera_info)) {
    ROS_INFO("Camera information written to camera_parameters.txt");
    return true;
  }
  else {
    ROS_ERROR("Could not write camera_parameters.txt");
    return false;
  }
}
