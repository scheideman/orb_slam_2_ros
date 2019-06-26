#include "Node.h"

#include <iostream>

Node::Node (ORB_SLAM2::System* pSLAM, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) {
  name_of_node_ = ros::this_node::getName();
  orb_slam_ = pSLAM;
  node_handle_ = node_handle;
  min_observations_per_point_ = 2;

  //static parameters
  node_handle_.param(name_of_node_+"/publish_pointcloud", publish_pointcloud_param_, true);
  node_handle_.param(name_of_node_+"/publish_pose", publish_pose_param_, true);
  node_handle_.param(name_of_node_+"/publish_tf", publish_tf_param_, true);
  node_handle_.param(name_of_node_+"/publish_map_to_odom", publish_map_to_odom_param_, false);
  node_handle_.param<std::string>(name_of_node_+"/pointcloud_frame_id", map_frame_id_param_, "map");
  node_handle_.param<std::string>(name_of_node_+"/camera_frame_id", camera_frame_id_param_, "camera_link");
  node_handle_.param<std::string>(name_of_node_+"/robot_base_link_frame_id", robot_base_link_frame_id_param_, camera_frame_id_param_);


  //Setup dynamic reconfigure
  dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
  dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  dynamic_param_server_.setCallback(dynamic_param_callback);

  rendered_image_publisher_ = image_transport.advertise (name_of_node_+"/debug_image", 1);
  if (publish_pointcloud_param_) {
    map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points", 1);
    reference_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/reference_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1);
  }

  keyframe_publisher_ = node_handle_.advertise<geometry_msgs::PoseArray> (name_of_node_+"/keyframes",1);
  tracking_state_publisher_ = node_handle_.advertise<std_msgs::Int8> (name_of_node_+"/tracking_state",1);
  num_tracked_map_points_publisher_ = node_handle_.advertise<std_msgs::Int32> (name_of_node_+"/num_tracked_map_points",1);

  request_keyframes_service_ = node_handle_.advertiseService("request_keyframes", 
                                                    &Node::RequestKeyFrames, this);
  tfb_.reset(new tf2_ros::TransformBroadcaster());
  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));
}


Node::~Node () {

}


void Node::Update () {
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    if (publish_tf_param_){
      PublishPositionAsTransform (position);
    }

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped (position);
    }
  }

  PublishRenderedImage (orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetAllMapPoints());
    PublishReferencePoints(orb_slam_->GetReferenceMapPoints());
  }

  PublishTrackingState(orb_slam_->GetTrackingState());
  PublishTrackingState(orb_slam_->GetTrackedMapPoints().size());
  
}

void Node::PublishTrackingState (int tracking_state) {
  std_msgs::Int8 state;
  state.data = tracking_state;
  tracking_state_publisher_.publish(state);
}
void Node::PublishNumTrackedMapPoints (int num_tracked_map_points) {
  std_msgs::Int32 num_points;
  num_points.data = num_tracked_map_points;
  num_tracked_map_points_publisher_.publish(num_points);
}

void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_.publish (cloud);
}

void Node::PublishReferencePoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  reference_points_publisher_.publish (cloud);
}

void Node::PublishPositionAsTransform (cv::Mat position) {
  tf2::Transform tf2_transform = TransformFromMat (position);
  
  if(publish_map_to_odom_param_){
    PublishMapToOdomTransform(tf2_transform);
  }else{
    PublishMapToCameraTransform(tf2_transform);
  }
}

void Node::PublishMapToOdomTransform(tf2::Transform transform){
  try{
    geometry_msgs::PoseStamped odom_to_map;
    geometry_msgs::PoseStamped camera_to_map;

    camera_to_map.header.frame_id = camera_frame_id_param_;
    camera_to_map.header.stamp = current_frame_time_;
    tf2::toMsg(transform.inverse(), camera_to_map.pose);

    this->tf_->transform(camera_to_map, odom_to_map, "odom");

    tf2::Transform odom_to_map_tf;
    tf2::convert(odom_to_map.pose, odom_to_map_tf);

    geometry_msgs::TransformStamped out_tf_stamped;
    out_tf_stamped.header.frame_id = map_frame_id_param_;
    out_tf_stamped.header.stamp = current_frame_time_;
    out_tf_stamped.child_frame_id = "odom";
    tf2::convert(odom_to_map_tf.inverse(), out_tf_stamped.transform);
    tfb_->sendTransform(out_tf_stamped);
  }
  catch(tf2::TransformException)
  {
    ROS_DEBUG("Could not get map to odom transform");
  }
}
void Node::PublishMapToCameraTransform(tf2::Transform transform){
  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header.frame_id = map_frame_id_param_;
  tf_stamped.header.stamp = current_frame_time_;
  tf_stamped.child_frame_id = camera_frame_id_param_;
  tf2::convert(transform, tf_stamped.transform);
  tfb_->sendTransform(tf_stamped);
}

void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  tf2::Transform map_to_camera_tf = TransformFromMat (position);

  geometry_msgs::PoseStamped camera_to_map;
  geometry_msgs::PoseStamped base_link_to_map;

  tf2::toMsg(map_to_camera_tf.inverse(), camera_to_map.pose);
  camera_to_map.header.frame_id = camera_frame_id_param_;
  camera_to_map.header.stamp = current_frame_time_;

  this->tf_->transform(camera_to_map, base_link_to_map, robot_base_link_frame_id_param_);

  tf2::Transform base_link_to_map_tf;
  tf2::convert(base_link_to_map.pose, base_link_to_map_tf);

  geometry_msgs::PoseStamped map_to_base_link;
  tf2::toMsg(base_link_to_map_tf.inverse(), map_to_base_link.pose);
  // tf2::toMsg(map_to_camera_tf, map_to_base_link.pose);
  map_to_base_link.header.frame_id = robot_base_link_frame_id_param_;
  map_to_base_link.header.stamp = current_frame_time_;
  // map_to_base_link.header.stamp = ros::Time::now();

  pose_publisher_.publish(map_to_base_link);
}


void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}


tf2::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[3];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {//nObs isBad()
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, 3*sizeof(float));
    }
  }

  return cloud;
}


void Node::ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level) {
  orb_slam_->EnableLocalizationOnly (config.localize_only);
  min_observations_per_point_ = config.min_observations_for_ros_map;

  if (config.reset_map) {
    orb_slam_->Reset();
    config.reset_map = false;
  }

  orb_slam_->SetMinimumKeyFrames (config.min_num_kf_in_map);
}

bool Node::RequestKeyFrames(std_srvs::Empty::Request &request, 
                                             std_srvs::Empty::Response &response){
    std::vector<cv::Mat> keyframes = orb_slam_->GetAllSortedKeyFrames();    
    
    geometry_msgs::PoseArray poseArray = PoseVectorToPoseArray(keyframes, map_frame_id_param_);
    keyframe_publisher_.publish(poseArray);

    return true;
}

geometry_msgs::PoseArray Node::PoseVectorToPoseArray(vector<cv::Mat> poses, std::string frame_id){
  geometry_msgs::PoseArray poseArray;
  poseArray.header.stamp = ros::Time::now();
  poseArray.header.frame_id = frame_id;

  for(auto kf : poses){
    tf2::Transform tfT = TransformFromMat(kf);
    geometry_msgs::Pose poseMsg;
    tf2::toMsg(tfT, poseMsg);
    poseArray.poses.push_back(poseMsg);
  }

  return poseArray;
  
}
