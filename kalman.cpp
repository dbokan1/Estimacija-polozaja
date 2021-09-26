#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include "std_msgs/Float64.h"

ros::Publisher pub,estimiran_x, mjereni_x,estimiran_y, mjereni_y,estimiran_z, mjereni_z;


  double T=0.03333;
  ros::WallTime start_, end_;

Eigen::MatrixXd P_(9,9); //Initial uncertainty
  Eigen::MatrixXd F_(9,9); //Linearized state approximation function
  Eigen::MatrixXd H_(3,9); //linearized measurement function
  Eigen::MatrixXd R_(3,3); //Measurement uncertainty
  Eigen::MatrixXd I_(9,9); //Identity matrix
  Eigen::MatrixXd Q_(9,9); //Mean of state function
  Eigen::MatrixXd x_(9,1); //Matrix of initial state variables
    Eigen::MatrixXd Z_(3,1); //Matrix of measurements
  

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-9;
    //ROS_INFO_STREAM("Exectution time (s): " << execution_time);
    T=execution_time;
    F_<<1, 0, 0, T, 0, 0,0.5*T*T,0,0,  //koristen model ravnomjerno ubrzanog kretanja
     0, 1, 0, 0, T, 0,0,0.5*T*T,0,
     0, 0, 1, 0, 0, T,0,0,0.5*T*T,
     0, 0, 0, 1, 0, 0,T,0,0,
     0, 0, 0, 0, 1, 0,0,T,0,
     0, 0, 0, 0, 0, 1,0,0,T,
     0, 0, 0, 0, 0, 0,1,0,0,
     0, 0, 0, 0, 0, 0,0,1,0,
     0, 0, 0, 0, 0, 0,0,0,1;
     
     
     
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::fromROSMsg(*cloud_msg, *rgb_cloud);
 pcl::PCLPointCloud2 cloud_filtered_ros;
 
 
 //filtriranje po z osi-odsjecanje nepotrebnog dijela snimka
 
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr zfilter(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::PassThrough<pcl::PointXYZRGB> filter;
 filter.setInputCloud(rgb_cloud);
 filter.setFilterFieldName("z");
 filter.setFilterLimits(0.45, 0.86);
// filter.setFilterLimitsNegative(true);
 filter.filter(*zfilter);
 
 
 //smanjenje broja tacaka radi lakseg rada
 pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voksel (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (zfilter);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*voksel);
  
 //filter 1 po boji
 pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;
 pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, 30));
 pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
 color_cond->addComparison (red_condition);
 color_filter.setInputCloud(voksel);
 color_filter.setCondition (color_cond);
 color_filter.filter(*cloud_filtered);
 
 //filter 2 po boji
 pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, 80));
 pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond2 (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
 color_cond2->addComparison (green_condition);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
 color_filter.setInputCloud(cloud_filtered);
 color_filter.setCondition (color_cond2);
 color_filter.filter(*cloud_filtered2);
 
 //racunanje centroida
 pcl::PointXYZRGB centroid; 
 pcl::computeCentroid (*cloud_filtered2, centroid);
 //ROS_INFO_STREAM("Mjerene koord:"<<centroid.x<<","<<centroid.y<<","<<centroid.z);
 //ROS_INFO_STREAM("velicina:"<<cloud_filtered2->size());

  Z_(0,0)=centroid.x;
  Z_(1,0)=centroid.y;
  Z_(2,0)=centroid.z;


//kalman INOVATION

Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;//3x3
Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); //9x3
Eigen::MatrixXd y = Z_ - (H_ * x_); //3x1


//update estimate and covariance

if(cloud_filtered2->size()>30){
x_ = x_ + (K * y);       
P_ = (I_ - (K * H_)) * P_;}


//predict state
x_ = (F_ * x_);
P_ = F_ * P_ * F_.transpose()+Q_;
 
//ROS_INFO_STREAM("Estim:"<<x_(0,0)<<","<<x_(1,0)<<","<<x_(2,0));

  //ucrtavanje estimacije u izlazni pointcloud
  pcl::PointXYZRGB centar;
  centar.x=x_(0,0);
  centar.y=x_(1,0);
  centar.z=x_(2,0);
  centar.r=200;
  centar.g=0;
  centar.b=0;
  
  cloud_filtered2->push_back(centar);
  sensor_msgs::PointCloud2 output;
  
  std_msgs::Float64 estx;
  estx.data=centar.x;
  
  std_msgs::Float64 mjerx;
  mjerx.data=centroid.x;
  
  std_msgs::Float64 esty;
  esty.data=centar.y;
  
  std_msgs::Float64 mjery;
  mjery.data=centroid.y;
  
   std_msgs::Float64 estz;
  estz.data=centar.z;
  
   std_msgs::Float64 mjerz;
  mjerz.data=centroid.z;
  
  pcl::toPCLPointCloud2(*cloud_filtered2, cloud_filtered_ros);
  pcl_conversions::fromPCL(cloud_filtered_ros, output);
  
  pub.publish(output);
  estimiran_x.publish(estx);
  mjereni_x.publish(mjerx);
  estimiran_y.publish(esty);
  mjereni_y.publish(mjery);
  estimiran_z.publish(estz);
  mjereni_z.publish(mjerz);
  
  start_ = ros::WallTime::now();
}

int
main (int argc, char** argv)
{
 F_<<1, 0, 0, T, 0, 0,0.5*T*T,0,0,
     0, 1, 0, 0, T, 0,0,0.5*T*T,0,
     0, 0, 1, 0, 0, T,0,0,0.5*T*T,
     0, 0, 0, 1, 0, 0,T,0,0,
     0, 0, 0, 0, 1, 0,0,T,0,
     0, 0, 0, 0, 0, 1,0,0,T,
     0, 0, 0, 0, 0, 0,1,0,0,
     0, 0, 0, 0, 0, 0,0,1,0,
     0, 0, 0, 0, 0, 0,0,0,1;
     
  H_<<1, 0, 0, 0, 0, 0,0,0,0,
     0, 1, 0, 0, 0, 0,0,0,0,
     0, 0, 1, 0, 0, 0,0,0,0; //linearrized measurement function
  
  
  P_<<0.01, 0, 0, 0, 0, 0,0,0,0,
 	0, 0.01, 0, 0, 0, 0,0,0,0,
  	0, 0, 0.01, 0, 0, 0,0,0,0,
  	0, 0, 0, 0.01, 0, 0,0,0,0,
  	0, 0, 0, 0, 0.01, 0,0,0,0,
  	0, 0, 0, 0, 0, 0.01,0,0,0,
  	0, 0, 0, 0, 0, 0,0.01,0,0,
  	0, 0, 0, 0, 0,0,0, 0.01,0,
  	0, 0, 0, 0, 0,0,0,0, 0.01; //Initial uncertainty
  		           
  R_<<1, 0, 0,
  	0, 1, 0,
  	0, 0, 1; //Measurement uncertainty
  Q_<<0.1, 0, 0, 0, 0, 0,0,0,0,
  	0, 0.1, 0, 0, 0, 0,0,0,0,
  	0, 0, 0.1, 0, 0, 0,0,0,0,
  	0, 0, 0, 0.1, 0, 0,0,0,0,
  	0, 0, 0, 0, 0.1, 0,0,0,0,
  	0, 0, 0, 0, 0, 0.1,0,0,0,
  	0, 0, 0, 0, 0,0, 0.1,0,0,
  	0, 0, 0, 0, 0,0,0, 0.1,0,
  	0, 0, 0, 0, 0,0,0,0,0.1; //State uncertainty
  
   I_<<1, 0, 0, 0, 0, 0,0,0,0,
      0, 1, 0, 0, 0, 0,0,0,0,
      0, 0, 1, 0, 0, 0,0,0,0,
      0, 0, 0, 1, 0, 0,0,0,0,
      0, 0, 0, 0, 1, 0,0,0,0,
      0, 0, 0, 0, 0, 1,0,0,0,
      0, 0, 0, 0, 0,0, 1,0,0,
      0, 0, 0, 0, 0,0,0, 1,0,
      0, 0, 0, 0, 0,0,0,0, 1; //Identity matrix
  		       
   x_<<0,0,0,0,0,0,0,0,0; //Matrix of initial state variables
   Z_<<0,0,0; 



  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  estimiran_x = nh.advertise<std_msgs::Float64> ("estimx", 1);
  mjereni_x = nh.advertise<std_msgs::Float64> ("mjerx", 1);
  estimiran_y = nh.advertise<std_msgs::Float64> ("estimy", 1);
  mjereni_y = nh.advertise<std_msgs::Float64> ("mjery", 1);
  estimiran_z = nh.advertise<std_msgs::Float64> ("estimz", 1);
  mjereni_z = nh.advertise<std_msgs::Float64> ("mjerz", 1);
  start_ = ros::WallTime::now();
  // Spin
  ros::spin ();
}





