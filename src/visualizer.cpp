#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>
#include <tf2_ros/buffer.h>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <pose_cnn/database_loader.h>
#include <dynamic_reconfigure/server.h>
#include <ycb_multicam_dataset/visualizerConfig.h>
#include <vision_msgs/Detection3DArray.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <mutex>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

std::string dataset_path_, models_dir_, scene_, camera_ = "";
int position_;
tf2_ros::Buffer tf_buffer_;
// vision_msgs::Detection3DArray ground_truth_;
DatabaseLoader * db_loader_;
std::map<std::string, geometry_msgs::PoseStamped> ground_truth_;
ros::Publisher gt_pub_;
ros::Publisher cloud_pub_;
ros::Publisher depth_img_pub_;
ros::Publisher depth_info_pub_;
ros::Publisher rgb_img_pub_;
ros::Publisher rgb_info_pub_;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>> cloud_buffer_;
std::vector<sensor_msgs::Image> rgb_img_buffer_;
std::vector<sensor_msgs::CameraInfo> rgb_info_buffer_;
std::vector<sensor_msgs::Image> depth_img_buffer_;
std::vector<sensor_msgs::CameraInfo> depth_info_buffer_;

tf::Transform offset_tf_;

std::mutex config_lock;

void init_tfs()
{
    tf_buffer_.clear();
    rosbag::Bag bag;
    try
    {
        bag.open(dataset_path_ + "/scenes/" + scene_ + "/snapshots/" + std::to_string(position_) + "/tf.bag", rosbag::bagmode::Read);
    } catch (rosbag::BagException e)
    {
        ROS_ERROR("Could not open Bagfile!");
    }
    std::vector<std::string> topics;
    topics.push_back("/tf");
    topics.push_back("/tf_static");

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::vector<geometry_msgs::TransformStamped> main_tf_msgs;
    std::vector<geometry_msgs::TransformStamped> aruco_tf_msgs;
    for(rosbag::MessageInstance const& m: rosbag::View(bag))
    {
        tf::tfMessage::ConstPtr tfs = m.instantiate<tf::tfMessage>();
        if (tfs != NULL)
        {
            if (m.getTopic() == "/tf_static")
            {
                for (auto tf : tfs->transforms)
                {
                    //clean slashes from word start static_tf
                    if (tf.child_frame_id.at(0) == '/')
                    {
                        tf.child_frame_id.erase(0, 1);
                    }
                    if (tf.header.frame_id.at(0) == '/')
                    {
                        tf.header.frame_id.erase(0, 1);
                    }
                    tf_buffer_.setTransform(tf, "", true);
                }
            }
            else
            {
                for (const auto& tf : tfs->transforms)
                {

                    if(tf.child_frame_id.find("aruco_ref") != std::string::npos)
                    {
                        tf_buffer_.setTransform(tf, "");
                    }
                    //we assume everything else is static.
                    else
                    {
                        tf_buffer_.setTransform(tf, "", true);
                    }

                }
            }
        }
    }
}

void read_bag_msgs()
{
    cloud_buffer_.clear();
    rgb_img_buffer_.clear();
    rgb_info_buffer_.clear();
    depth_img_buffer_.clear();
    depth_info_buffer_.clear();
    rosbag::Bag bag;
    bag.open(dataset_path_ + "/scenes/" + scene_ + "/snapshots/" + std::to_string(position_) + "/" + camera_ + "/snapshot.bag");

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if (m.getTopic() == "cloud")
        {
            sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud != NULL)
            {
                pcl::PCLPointCloud2 pcl_pc2;
                pcl_conversions::toPCL(*cloud, pcl_pc2);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all_rig_frame (new pcl::PointCloud<pcl::PointXYZRGB>);
                geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform("rig_link", cloud->header.frame_id, ros::Time(0));
                tf::StampedTransform transform_tf;
                tf::transformStampedMsgToTF(transform_msg, transform_tf);
                pcl_ros::transformPointCloud(*temp_cloud, *cloud_all_rig_frame, transform_tf);
                cloud_all_rig_frame->header.frame_id = "rig_link";

                cloud_buffer_.push_back(*cloud_all_rig_frame);
            }
        }
        if (m.getTopic() == "rgb_img")
        {
            sensor_msgs::Image::ConstPtr rgb_image = m.instantiate<sensor_msgs::Image>();
            if (rgb_image != NULL)
            {
                rgb_img_buffer_.push_back(*rgb_image);

            }
        }
        if (m.getTopic() == "rgb_info")
        {
            sensor_msgs::CameraInfo::ConstPtr rgb_info = m.instantiate<sensor_msgs::CameraInfo>();
            if (rgb_info != NULL)
            {
                rgb_info_buffer_.push_back(*rgb_info);

            }
        }
        if (m.getTopic() == "depth_img")
        {
            sensor_msgs::Image::ConstPtr depth_image = m.instantiate<sensor_msgs::Image>();
            if (depth_image != NULL)
            {
                depth_img_buffer_.push_back(*depth_image);

            }
        }
        if (m.getTopic() == "depth_info")
        {
            sensor_msgs::CameraInfo::ConstPtr depth_info = m.instantiate<sensor_msgs::CameraInfo>();
            if (depth_info != NULL)
            {
                depth_info_buffer_.push_back(*depth_info);

            }
        }
    }

    bag.close();
}

void read_ground_truth()
{
    ground_truth_.clear();
    if (boost::filesystem::is_regular_file(dataset_path_ + "/scenes/" + scene_ +"/recognition_truth.yaml"))
    {
        YAML::Node recog_gt = YAML::LoadFile(dataset_path_ + "/scenes/" + scene_ +"/recognition_truth.yaml");
        int index = 0;
        std::map<std::string, geometry_msgs::Pose> objects;
        for (const auto& detection : recog_gt)
        {
            std::string id = detection["class"].as<std::string>();
            geometry_msgs::Pose pose;
            pose.position.x = detection["position"]["x"].as<double>();
            pose.position.y = detection["position"]["y"].as<double>();
            pose.position.z = detection["position"]["z"].as<double>();
            pose.orientation.x = detection["orientation"]["x"].as<double>();
            pose.orientation.y = detection["orientation"]["y"].as<double>();
            pose.orientation.z = detection["orientation"]["z"].as<double>();
            pose.orientation.w = detection["orientation"]["w"].as<double>();
            objects[id] = pose;
        }
        for (auto& obj : objects)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = camera_ + "/aruco_node/aruco_ref";
            pose.header.stamp = ros::Time::now();
            pose.pose = obj.second;
            ground_truth_[obj.first] = pose;
            // insertObject(obj.first.c_str(), obj.second);
        }
    }
}

void dynamic_callback(ycb_multicam_dataset::visualizerConfig &config) {

    config_lock.lock();

    if (camera_ != config.camera)
    {
        camera_ = config.camera;
        if (position_ == config.position && scene_ == config.scene)
        {
            read_bag_msgs();
        }
    }

    if (position_ != config.position)
    {
        position_ = config.position;
        if (scene_ == config.scene)
        {
            init_tfs();
            read_bag_msgs();
        }
    }

    if (scene_ != config.scene)
    {
        if (boost::filesystem::is_directory(dataset_path_ + "/scenes/" + config.scene))
        {
            scene_ = config.scene;
        }
        else
        {
            ROS_WARN("Configured scene does not exist.");
        }
        init_tfs();
        read_ground_truth();
        read_bag_msgs();
    }



    tf::Vector3 pos;
    pos.setX(config.pos_offset_x);
    pos.setY(config.pos_offset_y);
    pos.setZ(config.pos_offset_z);

    tf::Quaternion q;
    q.setX(config.rot_offset_x);
    q.setY(config.rot_offset_y);
    q.setZ(config.rot_offset_z);
    q.setW(config.rot_offset_w);

    offset_tf_.setOrigin(pos);
    offset_tf_.setRotation(q);

    camera_ = config.camera;
    config_lock.unlock();
}

void publish_msgs()
{
    try {
        for (int i = 0; i < cloud_buffer_.size() ; i++)
        {
            cloud_pub_.publish(cloud_buffer_.at(i));
            rgb_img_pub_.publish(rgb_img_buffer_.at(i));
            rgb_info_pub_.publish(rgb_info_buffer_.at(i));
            depth_img_pub_.publish(depth_img_buffer_.at(i));
            depth_info_pub_.publish(depth_info_buffer_.at(i));
        }
    }
    catch (std::out_of_range)
    {
        ROS_ERROR("Unequal amount of msgs in bagfile!.");
    }
}

void publish_gt()
{
    while(ros::ok())
    {

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_all (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        config_lock.lock();
        //if no detections are found, use these standard values
        cloud_all->header.frame_id = camera_ + "/aruco_node/aruco_ref";
        pcl_conversions::toPCL(ros::Time::now(), cloud_all->header.stamp);
        for (auto object : ground_truth_)
        {
            cloud_all->header.frame_id = object.second.header.frame_id;
            cloud_all->header.seq = object.second.header.seq;
            pcl_conversions::toPCL(object.second.header.stamp, cloud_all->header.stamp);

            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            db_loader_->getCloud(object.first, cloud);

            tf::Transform tfTrans;

            tf::poseMsgToTF(object.second.pose, tfTrans);
            pcl_ros::transformPointCloud(*cloud, *cloud, tfTrans);
            *cloud_all += *cloud;
        }
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_all_cam_frame (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform("rig_link", "aruco_ref", ros::Time(0));
        tf::StampedTransform transform_tf;
        tf::transformStampedMsgToTF(transform_msg, transform_tf);
        // test offsets for positions by applying additional transformation configurable by dyn reconf
        transform_tf.setData(transform_tf*offset_tf_);
        pcl_ros::transformPointCloud(*cloud_all, *cloud_all_cam_frame, transform_tf);
        cloud_all_cam_frame->header.frame_id = "rig_link";
        gt_pub_.publish(cloud_all_cam_frame);

        publish_msgs();

        config_lock.unlock();

        ros::spinOnce();
    }
}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "ycb_multicam_dataset_visualizer");
    ros::NodeHandle n;


    if (argc > 1)
    {
        dataset_path_ = argv[1];
    }
    if (argc > 2)
    {
        models_dir_ = argv[2];
    }
    if (!boost::filesystem::is_directory(dataset_path_ + "/scenes/" + scene_) || !boost::filesystem::is_directory(models_dir_))
    {
        ROS_ERROR("Declared folders are not valid. Please create them or check correct path.");
        return 0;
    }
    DatabaseLoader db_loader(dataset_path_, models_dir_);
    db_loader_ = &db_loader;

    dynamic_reconfigure::Server<ycb_multicam_dataset::visualizerConfig> server;
    dynamic_reconfigure::Server<ycb_multicam_dataset::visualizerConfig>::CallbackType f;
    f = boost::bind(&dynamic_callback, _1);
    server.setCallback(f);

    cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("visualizer/depth/points", 1, true);
    depth_img_pub_ = n.advertise<sensor_msgs::Image>("visualizer/depth/image", 1, true);
    depth_info_pub_ = n.advertise<sensor_msgs::CameraInfo>("visualizer/depth/camera_info", 1, true);
    rgb_img_pub_ = n.advertise<sensor_msgs::Image>("visualizer/rgb/image", 1, true);
    rgb_info_pub_ = n.advertise<sensor_msgs::CameraInfo>("visualizer/rgb/camera_info", 1, true);
    gt_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>>("visualizer/ground_truth", 1, true);

    init_tfs();

    read_ground_truth();

    publish_gt();

    ros::shutdown();

    return 0;
}
