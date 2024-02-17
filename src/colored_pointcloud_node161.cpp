#include <ros/ros.h>
#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include "colored_pointcloud/colored_pointcloud.h"
#include <sys/stat.h>
#include <sys/types.h> 
#include <cstdio>
#include <ctime>

#define YELLOW "\033[33m" /* Yellow */
#define GREEN "\033[32m"  /* Green */
#define REND "\033[0m" << std::endl

#define WARN (std::cout << YELLOW)
#define INFO (std::cout << GREEN)

ros::Publisher fused_image_pub, colored_cloud_showpub;
// colored_cloud_pub;

class RsCamFusion
{
  private:
    cv::Mat intrinsic;
    cv::Mat extrinsic;
    cv::Mat distcoeff;
    cv::Size imageSize;
    Eigen::Matrix4d transform, inv_transform; // L2C and C2L
    Eigen::Matrix4d tr_l2c, tr_c2l;
    Eigen::Matrix4d tr_l2ref, tr_ref2l;
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation vector
    cv::Mat rMat = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // Translation vector
    bool show_colored_cloud, save_data;
    std::string image_save_dir, cloud_save_dir, colored_cloud_save_dir;

    int color[21][3] = 
    {
        {255, 0, 0}, {255, 69, 0}, {255, 99, 71}, 
        {255, 140, 0}, {255, 165, 0}, {238, 173, 14},
        {255, 193, 37}, {255, 255, 0}, {255, 236, 139},
        {202, 255, 112}, {0, 255, 0}, {84, 255, 159},
        {127, 255, 212}, {0, 229, 238}, {152, 245, 255},
        {178, 223, 238}, {126, 192, 238}, {28, 134, 238},
        {0, 0, 255}, {72, 118, 255}, {122, 103, 238} 
    };
    float color_distance;   //step length to color the lidar points according to plane distance(z)
    int frame_count = 0;

  public:
    RsCamFusion(cv::Mat cam_intrinsic, cv::Mat lidar2cam_extrinsic, cv::Mat cam_distcoeff, cv::Size img_size, cv::Mat lidar2ref_extrinsic, bool ext_l2c, float color_dis, bool show_cloud, bool save)
    {
      intrinsic = cam_intrinsic;
      extrinsic = lidar2cam_extrinsic;
      distcoeff = cam_distcoeff;
      tr_l2c(0,0) = extrinsic.at<double>(0,0);
      tr_l2c(0,1) = extrinsic.at<double>(0,1);
      tr_l2c(0,2) = extrinsic.at<double>(0,2);
      tr_l2c(0,3) = extrinsic.at<double>(0,3);
      tr_l2c(1,0) = extrinsic.at<double>(1,0);
      tr_l2c(1,1) = extrinsic.at<double>(1,1);
      tr_l2c(1,2) = extrinsic.at<double>(1,2);
      tr_l2c(1,3) = extrinsic.at<double>(1,3);
      tr_l2c(2,0) = extrinsic.at<double>(2,0);
      tr_l2c(2,1) = extrinsic.at<double>(2,1);
      tr_l2c(2,2) = extrinsic.at<double>(2,2);
      tr_l2c(2,3) = extrinsic.at<double>(2,3);
      tr_l2c(3,0) = extrinsic.at<double>(3,0);
      tr_l2c(3,1) = extrinsic.at<double>(3,1);
      tr_l2c(3,2) = extrinsic.at<double>(3,2);
      tr_l2c(3,3) = extrinsic.at<double>(3,3);
      tr_c2l = tr_l2c.inverse();

      tr_l2ref(0,0) = lidar2ref_extrinsic.at<double>(0,0);
      tr_l2ref(0,1) = lidar2ref_extrinsic.at<double>(0,1);
      tr_l2ref(0,2) = lidar2ref_extrinsic.at<double>(0,2);
      tr_l2ref(0,3) = lidar2ref_extrinsic.at<double>(0,3);
      tr_l2ref(1,0) = lidar2ref_extrinsic.at<double>(1,0);
      tr_l2ref(1,1) = lidar2ref_extrinsic.at<double>(1,1);
      tr_l2ref(1,2) = lidar2ref_extrinsic.at<double>(1,2);
      tr_l2ref(1,3) = lidar2ref_extrinsic.at<double>(1,3);
      tr_l2ref(2,0) = lidar2ref_extrinsic.at<double>(2,0);
      tr_l2ref(2,1) = lidar2ref_extrinsic.at<double>(2,1);
      tr_l2ref(2,2) = lidar2ref_extrinsic.at<double>(2,2);
      tr_l2ref(2,3) = lidar2ref_extrinsic.at<double>(2,3);
      tr_l2ref(3,0) = lidar2ref_extrinsic.at<double>(3,0);
      tr_l2ref(3,1) = lidar2ref_extrinsic.at<double>(3,1);
      tr_l2ref(3,2) = lidar2ref_extrinsic.at<double>(3,2);
      tr_l2ref(3,3) = lidar2ref_extrinsic.at<double>(3,3);
      tr_ref2l = tr_l2ref.inverse();

      if(ext_l2c) transform = tr_l2c * tr_ref2l;
      else transform = tr_l2c.inverse() * tr_ref2l;
      inv_transform = transform.inverse();
      // inv_transform = transform;
      // transform = inv_transform.inverse();
      imageSize = img_size;
      color_distance = color_dis;
      show_colored_cloud = show_cloud;
      save_data = save;
      if(save_data)
      {
        time_t rawtime;
        struct tm *ptminfo;
        time(&rawtime);
        ptminfo = localtime(&rawtime);
        std::string currentdate = "/data/" + std::to_string(ptminfo->tm_year + 1900) + std::to_string(ptminfo->tm_mon + 1) 
                                          + std::to_string(ptminfo->tm_mday) + std::to_string(ptminfo->tm_hour) 
                                          + std::to_string(ptminfo->tm_min) + std::to_string(ptminfo->tm_sec);
        mkdir(currentdate.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        image_save_dir = currentdate + "/front_camera";
        mkdir(image_save_dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        cloud_save_dir = currentdate + "/rslidar_points";
        mkdir(cloud_save_dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        colored_cloud_save_dir = currentdate + "/colored_cloud";
        mkdir(colored_cloud_save_dir.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
      }
    }

    void callback(const sensor_msgs::ImageConstPtr input_image_msg,
                const sensor_msgs::PointCloud2ConstPtr input_cloud_msg)
    {
      cv::Mat input_image;
      cv::Mat undistorted_image;
      cv_bridge::CvImagePtr cv_ptr; 

      std_msgs::Header image_header = input_image_msg->header;
      std_msgs::Header cloud_header = input_cloud_msg->header;
      // INFO << image_header << REND;
      ROS_INFO("<<<<<<<< GET LIDAR & IMAGE");
    // sensor_msgs to cv image   
      try
      {
        cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
        // cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::TYPE_8UC3);

      }
      catch(cv_bridge::Exception e)
      {
        ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
        return;
      }
      input_image = cv_ptr->image;
      
      //sensor_msgs to pointxyzi
      pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*input_cloud_msg, *input_cloud_ptr);
      if (input_cloud_ptr->size() == 0)
      {
        WARN << "input cloud is empty, please check it out!" << REND;
      }

      //transform lidar points from lidar coordinate to camera coordiante
      pcl::transformPointCloud (*input_cloud_ptr, *transformed_cloud, transform);        //lidar coordinate(forward x+, left y+, up z+) 
                                                                                         //camera coordiante(right x+, down y+, forward z+) (3D-3D)  
                                                                                         //using the extrinsic matrix between this two coordinate system
      std::vector<cv::Point3d> lidar_points;
      std::vector<cv::Scalar> dis_color;
      std::vector<float> intensity;
      std::vector<cv::Point2d> imagePoints;
      
      //reserve the points in front of the camera(z>0)
      for(int i=0;i<=transformed_cloud->points.size();i++)
      {
          if(transformed_cloud->points[i].z>0)
          {
            lidar_points.push_back(cv::Point3d(transformed_cloud->points[i].x, transformed_cloud->points[i].y, transformed_cloud->points[i].z));
            int color_order = int(transformed_cloud->points[i].z / color_distance);
            if(color_order > 20)
            {
              color_order = 20;
            }
            dis_color.push_back(cv::Scalar(color[color_order][2], color[color_order][1], color[color_order][0])); // Scalar里顺序是bgr三通道 #by yao
            intensity.push_back(transformed_cloud->points[i].intensity);
          }
      }

      //project lidar points from the camera coordinate to the image coordinate(right x+, down y+)
      cv::projectPoints(lidar_points, rMat, tVec, intrinsic, distcoeff, imagePoints);          
      
      pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud (new pcl::PointCloud<PointXYZRGBI>);
      pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud_transback (new pcl::PointCloud<PointXYZRGBI>);
      cv::Mat image_to_show = input_image.clone();

      int valid_count=0;
      for(int i=0;i<imagePoints.size();i++)
      {
        // if(imagePoints[i].x>=0 && imagePoints[i].x<1920 && imagePoints[i].y>=0 && imagePoints[i].y<1200)
        if(imagePoints[i].x>=0 && imagePoints[i].x<imageSize.width && imagePoints[i].y>=0 && imagePoints[i].y<imageSize.height)
        {
          // cv::circle(image_to_show, imagePoints[i], 1, dis_color[i], 2, 8, 0);
          cv::circle(image_to_show, imagePoints[i], 1, dis_color[i], -1, 8, 0);           // #by yao
          PointXYZRGBI point;                                                             //reserve the lidar points in the range of image 
          point.x = lidar_points[i].x;                                                        //use 3D lidar points and RGB value of the corresponding pixels  
          point.y = lidar_points[i].y;                                                        //to create colored point clouds
          point.z = lidar_points[i].z;
          point.r = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[2];
          point.g = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[1];
          point.b = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[0];
          point.i = intensity[i];
          colored_cloud->points.push_back(point);  
          valid_count++;
        }
      }
      cout << "size valid count = " << valid_count << endl;
      //transform colored points from camera coordinate to lidar coordinate
      pcl::transformPointCloud (*colored_cloud, *colored_cloud_transback, inv_transform);      
      
      if(show_colored_cloud)
      {  
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_toshow (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i=0;i<colored_cloud_transback->points.size();i++)
        {
            pcl::PointXYZRGB point;                                                             
            point.x = colored_cloud_transback->points[i].x;                                                        
            point.y = colored_cloud_transback->points[i].y;                                                        
            point.z = colored_cloud_transback->points[i].z;
            point.r = colored_cloud_transback->points[i].r;
            point.g = colored_cloud_transback->points[i].g;
            point.b = colored_cloud_transback->points[i].b;
            colored_cloud_toshow->points.push_back (point);  
          }
        publishCloudtoShow(colored_cloud_showpub, cloud_header, colored_cloud_toshow);
      } 

      publishImage(fused_image_pub, image_header, image_to_show);
      if(save_data)
      {
        saveData(image_header, input_image, cloud_header, input_cloud_ptr, colored_cloud_transback);
      }
      frame_count = frame_count + 1;
    }

    void publishImage(const ros::Publisher& image_pub, const std_msgs::Header& header, const cv::Mat image)
    {
      cv_bridge::CvImage output_image;
      output_image.header.frame_id = header.frame_id;
      output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      output_image.image = image;
      image_pub.publish(output_image);
    } 
    
    void saveData(const std_msgs::Header& image_header, const cv::Mat image, const std_msgs::Header& cloud_header,
                      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud)
    {
      //timestamp: image_header.stamp.sec . image_header.stamp.nsec 
      std::string img_name = std::to_string(frame_count) + "_" + std::to_string(image_header.stamp.sec) + "_" + std::to_string(image_header.stamp.nsec) + ".jpg";
      std::string cloud_name = std::to_string(frame_count) + "_" + std::to_string(cloud_header.stamp.sec) + "_" + std::to_string(cloud_header.stamp.nsec) + ".pcd";
      std::string colored_cloud_name = "c_" + std::to_string(frame_count) + "_" + std::to_string(cloud_header.stamp.sec) + "_" + std::to_string(cloud_header.stamp.nsec) + ".pcd";
      cv::imwrite(image_save_dir + "/" + img_name, image);
      pcl::io::savePCDFileASCII(cloud_save_dir + "/" + cloud_name, *cloud);
      colored_cloud->width = cloud->size();
      colored_cloud->height = 1;
      colored_cloud->is_dense = false;
      colored_cloud->points.resize(cloud->width * cloud->height);
      pcl::io::savePCDFileASCII(colored_cloud_save_dir + "/" + colored_cloud_name, *colored_cloud);
    }

    void publishCloudtoShow(const ros::Publisher& cloudtoshow_pub, const std_msgs::Header& header,
                      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
    {
      sensor_msgs::PointCloud2 output_msg;
      pcl::toROSMsg(*cloud, output_msg);
      output_msg.header = header;
      cloudtoshow_pub.publish(output_msg);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "colored_pointcloud_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string config_path, file_name;
  std::string camera_topic, lidar_topic;
  float color_dis;
  bool show_cloud, save_data, ext_l2c;
  if (priv_nh.hasParam("calib_file_path") && priv_nh.hasParam("file_name"))
  {
    priv_nh.getParam("camera_topic", camera_topic);
    priv_nh.getParam("lidar_topic", lidar_topic);
    priv_nh.getParam("calib_file_path", config_path);
    priv_nh.getParam("file_name", file_name);
    priv_nh.getParam("color_distance", color_dis);
    priv_nh.getParam("show_colored_cloud", show_cloud);
    priv_nh.getParam("save_data", save_data);
    priv_nh.getParam("ext_l2c", ext_l2c);
  }
  else
  {
    WARN << "Config file is empty!" << REND;
    return 0;
  }
  
  INFO << "config path: " << config_path << REND;
  INFO << "config file: " << file_name << REND;

  std::string config_file_name = config_path + "/" + file_name;
  cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
  
  cv::Mat cam_intrinsic, lidar2cam_extrinsic, cam_distcoeff, lidar2ref_extrinsic;
  cv::Size img_size;
  fs_reader["CameraMat"] >> cam_intrinsic;
  fs_reader["CameraExtrinsicMat"] >> lidar2cam_extrinsic;
  fs_reader["LidarToRefExtrinsicMat"] >> lidar2ref_extrinsic;
  fs_reader["DistCoeff"] >> cam_distcoeff;
  fs_reader["ImageSize"] >> img_size;
  fs_reader.release();

  if(lidar2ref_extrinsic.empty())
  {
      WARN << "no lidar2ref extrinsic matrix! use defualt." << REND;
      lidar2ref_extrinsic = cv::Mat::eye(4, 4, CV_64FC1);
  }

  if (lidar_topic.empty() || camera_topic.empty())
  {
    WARN << "sensor topic is empty!" << REND;
    return 0;
  }
  cout << lidar2cam_extrinsic.inv() << endl;
  INFO << "Is LiDAR2Camear Extrinsic: " << ext_l2c << REND;
  INFO << "lidar topic: " << lidar_topic << REND;
  INFO << "camera topic: " << camera_topic << REND;
  INFO << "camera intrinsic matrix: " << cam_intrinsic << REND;
  INFO << "lidar2cam extrinsic matrix: " << lidar2cam_extrinsic << REND;
  INFO << "lidar2ref extrinsic matrix: " << lidar2ref_extrinsic << REND;

  
  RsCamFusion fusion(cam_intrinsic, lidar2cam_extrinsic, cam_distcoeff, img_size, lidar2ref_extrinsic, ext_l2c, color_dis, show_cloud, save_data); 
  message_filters::Subscriber<sensor_msgs::Image> camera_sub(nh, camera_topic, 30);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidar_topic, 10);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_sub, lidar_sub);
  sync.registerCallback(boost::bind(&RsCamFusion::callback, &fusion, _1, _2));

  fused_image_pub = nh.advertise<sensor_msgs::Image>("fused_image", 10);
  colored_cloud_showpub = nh.advertise<sensor_msgs::PointCloud2>("colored_cloud_toshow", 10);
  ros::spin();
  return 0;

}


