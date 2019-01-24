// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
// TFS
#include <tf2_ros/buffer.h>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf_conversions/tf_eigen.h>
// ycb_multicam_dataset
#include <ycb_multicam_dataset/model_loader.h>
#include <ycb_multicam_dataset/visualizerConfig.h>
// messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <vision_msgs/Detection3DArray.h>
//PCL
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

// misc
#include <mutex>
#include <boost/filesystem.hpp>

std::vector<std::string> cameras_({"astra", "basler_tof", "ensenso", "kinect2", "pico_flexx", "realsense_r200", "xtion"});

std::string dataset_path_, models_dir_, scene_;

std::vector<boost::shared_ptr<tf2_ros::Buffer>> tf_buffer_;

ycb_multicam_dataset::visualizerConfig config_;

bool remove_border_objects_;

ModelLoader * db_loader_;

ros::Publisher gt_pub_;
ros::Publisher debug_pub_;
ros::Publisher cloud_pub_;
ros::Publisher depth_img_pub_;
ros::Publisher depth_info_pub_;
ros::Publisher rgb_img_pub_;
ros::Publisher rgb_info_pub_;

boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

std::map<int, tf::Transform> position_offsets_;

std::map<std::string, std::map<int, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>>> cloud_buffer_;
std::map<std::string, std::map<int, std::vector<sensor_msgs::Image>>> rgb_img_buffer_;
std::map<std::string, std::map<int, std::vector<sensor_msgs::CameraInfo>>> rgb_info_buffer_;
std::map<std::string, std::map<int, std::vector<sensor_msgs::Image>>> depth_img_buffer_;
std::map<std::string, std::map<int, std::vector<sensor_msgs::CameraInfo>>> depth_info_buffer_;

std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> gt_cloud_;

void read_offsets();
void save_offsets();
void load_offsets_reconf(int pos);

void init_tfs(int position)
{
    tf_buffer_[position]->clear();
    rosbag::Bag bag;
    try
    {
        bag.open(dataset_path_ + "/scenes/" + scene_ + "/snapshots/" + std::to_string(position) + "/tf.bag", rosbag::bagmode::Read);
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
                    tf_buffer_[position]->setTransform(tf, "", true);
                }
            }
            else
            {
                for (const auto& tf : tfs->transforms)
                {

                    if(tf.child_frame_id.find("aruco_ref") != std::string::npos)
                    {
                        tf_buffer_[position]->setTransform(tf, "");
                    }
                    //we assume everything else is static.
                    else
                    {
                        tf_buffer_[position]->setTransform(tf, "", true);
                    }

                }
            }
        }
    }
}

void read_bag_msgs(std::string camera, int position)
{
    rosbag::Bag bag;
    bag.open(dataset_path_ + "/scenes/" + scene_ + "/snapshots/" + std::to_string(position) + "/" + camera + "/snapshot.bag");

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
                geometry_msgs::TransformStamped transform_msg = tf_buffer_[position]->lookupTransform("rig_link", cloud->header.frame_id, ros::Time(0));
                tf::StampedTransform transform_tf;
                tf::transformStampedMsgToTF(transform_msg, transform_tf);
                pcl_ros::transformPointCloud(*temp_cloud, *cloud_all_rig_frame, transform_tf);
                cloud_all_rig_frame->header.frame_id = "rig_link";

                cloud_buffer_[camera][position].push_back(*cloud_all_rig_frame);
            }
        }
        if (m.getTopic() == "rgb_img")
        {
            sensor_msgs::Image::ConstPtr rgb_image = m.instantiate<sensor_msgs::Image>();
            if (rgb_image != NULL)
            {
                rgb_img_buffer_[camera][position].push_back(*rgb_image);

            }
        }
        if (m.getTopic() == "rgb_info")
        {
            sensor_msgs::CameraInfo::ConstPtr rgb_info = m.instantiate<sensor_msgs::CameraInfo>();
            if (rgb_info != NULL)
            {
                rgb_info_buffer_[camera][position].push_back(*rgb_info);

            }
        }
        if (m.getTopic() == "depth_img")
        {
            sensor_msgs::Image::ConstPtr depth_image = m.instantiate<sensor_msgs::Image>();
            if (depth_image != NULL)
            {
                depth_img_buffer_[camera][position].push_back(*depth_image);

            }
        }
        if (m.getTopic() == "depth_info")
        {
            sensor_msgs::CameraInfo::ConstPtr depth_info = m.instantiate<sensor_msgs::CameraInfo>();
            if (depth_info != NULL)
            {
                depth_info_buffer_[camera][position].push_back(*depth_info);

            }
        }
    }

    bag.close();
}

void read_ground_truth(int position)
{
    std::map<std::string, geometry_msgs::PoseStamped> ground_truth;

    ground_truth.clear();
    if (boost::filesystem::is_regular_file(dataset_path_ + "/scenes/" + scene_ +"/recognition_truth.yaml"))
    {
        YAML::Node recog_gt = YAML::LoadFile(dataset_path_ + "/scenes/" + scene_ +"/recognition_truth.yaml");
        int index = 0;
        std::map<std::string, geometry_msgs::Pose> objects;
        for (const auto& detection : recog_gt)
        {
            std::string id = detection["class"].as<std::string>();
            if (remove_border_objects_ && (id.find("raceway") != std::string::npos || id == "getriebelager"))
            {
                continue;
            }
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
            pose.header.frame_id = "aruco_ref";
            pose.header.stamp = ros::Time::now();
            pose.pose = obj.second;
            ground_truth[obj.first] = pose;
        }


        gt_cloud_[position] = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //if no detections are found, use these standard values
        gt_cloud_[position]->header.frame_id = "aruco_ref";
        pcl_conversions::toPCL(ros::Time::now(), gt_cloud_[position]->header.stamp);
        for (auto object : ground_truth)
        {
            gt_cloud_[position]->header.frame_id = object.second.header.frame_id;
            gt_cloud_[position]->header.seq = object.second.header.seq;
            pcl_conversions::toPCL(object.second.header.stamp, gt_cloud_[position]->header.stamp);

            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            db_loader_->getCloud(object.first, cloud);

            tf::Transform tfTrans;

            tf::poseMsgToTF(object.second.pose, tfTrans);
            pcl_ros::transformPointCloud(*cloud, *cloud, tfTrans);
            *gt_cloud_[position] += *cloud;
        }
        geometry_msgs::TransformStamped transform_msg = tf_buffer_[position]->lookupTransform("rig_link", "aruco_ref", ros::Time(0));
        tf::StampedTransform transform_tf;
        tf::transformStampedMsgToTF(transform_msg, transform_tf);
        // test offsets for positions by applying additional transformation configurable by dyn reconf
        pcl_ros::transformPointCloud(*gt_cloud_[position], *gt_cloud_[position], transform_tf);
        gt_cloud_[position]->header.frame_id = "rig_link";
    }
}

void load_scene(std::string scene)
{
    ROS_INFO("Loading scene: %s", scene.c_str());
    scene_ = scene;
    cloud_buffer_.clear();
    rgb_img_buffer_.clear();
    rgb_info_buffer_.clear();
    depth_img_buffer_.clear();
    depth_info_buffer_.clear();
    tf_buffer_.clear();
    gt_cloud_.clear();

    for (int pos = 0 ; pos < 9 ; ++pos)
    {
        ROS_INFO("Loading position: %d", pos);
        tf_buffer_.push_back(boost::make_shared<tf2_ros::Buffer>());
        init_tfs(pos);
        read_ground_truth(pos);
        for (const auto& cam : cameras_)
        {
            ROS_INFO("Loading camera: %s", cam.c_str());
            read_bag_msgs(cam, pos);
        }
    }

}

void ransac(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_filtered, bool publish_debug = true)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (config_.ransac_distance);

    seg.setInputCloud (scene);
    seg.segment (*inliers, *coefficients);
    //extract points that are not in the plane
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (scene);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter (*scene_filtered);
    if (publish_debug)
    {
        // debug_pub_.publish(scene_filtered);
    }
}

void fit_scene_gt(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr scene, const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr gt, tf::Transform& tf_out, bool publish_debug = false)
{
    if (config_.icp_iterations == 0)
    {
        tf_out.setIdentity();
        return;
    }
    std::cout << config_.icp_iterations << std::endl;
    //for removeNaNFromPC call
    std::vector<int> dummy;

    // remove Normals from gt
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt_no_normal = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::copyPointCloud(*gt, *gt_no_normal);
    pcl::removeNaNFromPointCloud(*gt_no_normal, *gt_no_normal, dummy);

    //get area around the proposed gt
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_roi (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D (*gt_no_normal, min_pt, max_pt);

    pcl::CropBox<pcl::PointXYZRGB> boxFilter (true);
    boxFilter.setMin(Eigen::Vector4f(min_pt.x - 0.5, min_pt.y - 0.5, min_pt.z - 0.5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(max_pt.x + 0.5, max_pt.y + 0.5, max_pt.z + 0.5, 1.0));
    boxFilter.setInputCloud(scene);
    boxFilter.filter(*scene_roi);

    //downsample scene
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_ransac(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    ransac(scene_roi, scene_ransac, publish_debug);

    if (config_.voxel_grid_leaf_size != 0)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> scene_vg;
        scene_vg.setInputCloud(scene_ransac);
        scene_vg.setLeafSize(config_.voxel_grid_leaf_size, config_.voxel_grid_leaf_size, config_.voxel_grid_leaf_size);
        scene_vg.filter(*scene_filtered);
        pcl::removeNaNFromPointCloud(*scene_filtered, *scene_filtered, dummy);
    }
    else
    {
        pcl::removeNaNFromPointCloud(*scene_ransac, *scene_filtered, dummy);
    }


    debug_pub_.publish(scene_filtered);



    // ROS_INFO("Optimizing pose...");
    // ROS_INFO("Scene points: %lu, gt points: %lu", scene_filtered->size(), gt_no_normal->size());
    // ROS_INFO("With parameters:\n - MaxCorrespondenceDistance: %f\n"
    //                              "- MaxIterations: %d\n"
    //                              "- MaxTransformationEpsilon: %f\n"
    //                              "- MaxEuclideanFitnessEpsilon: %f\n ", config_.icp_max_correspondence_distance, config_.icp_iterations, config_.icp_transformation_epsilon, config_.icp_euclidean_fitness_epsilon);
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    icp.setUseReciprocalCorrespondences(false);
    icp.setMaxCorrespondenceDistance (config_.icp_max_correspondence_distance);
    icp.setMaximumIterations (config_.icp_iterations);
    icp.setTransformationEpsilon (config_.icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon (config_.icp_euclidean_fitness_epsilon);

    icp.setInputSource(scene_filtered);
    icp.setInputTarget(gt_no_normal);
    pcl::PointCloud<pcl::PointXYZRGB> final;
    icp.align(final);
    if (icp.hasConverged())
    {
        std::cout << "ICP has converged: " << (icp.hasConverged() ? "TRUE" : "FALSE") << " score: " << icp.getFitnessScore() << std::endl;
        switch (icp.getConvergeCriteria()->getConvergenceState())
        {
            case pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_NOT_CONVERGED:
                std::cout << "didnt converge" << std::endl;
                break;
            case pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_ITERATIONS:
                std::cout << "Max Iterations exceeded." << std::endl;
                break;
            case pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_TRANSFORM:
                std::cout << "Transform epsilon?." << std::endl;
                break;
            case pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_ABS_MSE:
                std::cout << "Abs MSE exceeded." << std::endl;
                break;
            case pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_REL_MSE:
                std::cout << "Rel MSE exceeded." << std::endl;
                break;
            case pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES:
                std::cout << "No correspondences found." << std::endl;
                break;
            default:
                std::cout << "well this didnt work" << std::endl;

        }
        Eigen::Matrix4f icp_mat = icp.getFinalTransformation();

        Eigen::Affine3d eigen_icp;
        eigen_icp.matrix() = icp_mat.cast<double>();
        tf::Transform tf_icp;
        tf_icp.setRotation(tf_icp.getRotation().normalize());
        tf::transformEigenToTF(eigen_icp, tf_icp);
        tf_out = tf_icp;
    }
    else
    {
        tf_out.setIdentity();
    }
}

void publish_icp_tfs()
{
    ROS_INFO("pub_icp");
    // for (int pos = 0; pos<9 ; ++pos)
    // {
    for (const auto& cam : cameras_)
    {
        ROS_INFO("ICP for TF: %d, %s", config_.position, cam.c_str());
        tf::Transform icp_tf;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_buffer_[cam][config_.position].at(0));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl_ros::transformPointCloud(*cloud_ptr, *cloud_transformed, position_offsets_[config_.position]);
        fit_scene_gt(cloud_transformed, gt_cloud_[config_.position], icp_tf);
        geometry_msgs::TransformStamped ros_tf;
        ros_tf.header.frame_id = "rig_link";
        ros_tf.child_frame_id = cam + std::to_string(config_.position);
        tf::transformTFToMsg(icp_tf, ros_tf.transform);
        tf_pub_->sendTransform(ros_tf);
    }
    // }
}

void dynamic_callback(ycb_multicam_dataset::visualizerConfig &config, uint32_t level)
{

    config_ = config;
    // scene changed
    if (level & 1)
    {
        ROS_INFO("Loading new scene: %s", config.scene.c_str());
        if (boost::filesystem::is_directory(dataset_path_ + "/scenes/" + config.scene))
        {
            load_scene(config.scene);
        }
        else
        {
            ROS_WARN("Configured scene does not exist.");
        }
    }

    // Offset changed

    if (level & 2)
    {
        tf::Vector3 pos;
        pos.setX(config.pos_offset_x);
        pos.setY(config.pos_offset_y);
        pos.setZ(config.pos_offset_z);

        tf::Quaternion q;
        q.setX(config.rot_offset_x);
        q.setY(config.rot_offset_y);
        q.setZ(config.rot_offset_z);
        q.setW(config.rot_offset_w);

        position_offsets_[config.position].setOrigin(pos);
        position_offsets_[config.position].setRotation(q);
        if (config.save_on_change)
        {
            save_offsets();
        }
        publish_icp_tfs();
    }

    // position changed
    if (level & 4)
    {
        load_offsets_reconf(config.position);
    }

    // load border objects changed
    if (level & 8)
    {
        ROS_INFO("border objects");
        remove_border_objects_ = config_.remove_border_objects;
        for (int pos = 0; pos < 9 ; pos++)
        {
            read_ground_truth(pos);
        }
    }

}

void publish_msgs()
{
    ROS_INFO("pub_msgs");
    // try {
        // for (int i = 0; i < cloud_buffer_[config_.camera][config_.position].size() ; i++)
        // {
    tf::Transform icp_tf;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_buffer_[config_.camera][config_.position].at(0));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl_ros::transformPointCloud(*cloud_ptr, *cloud_transformed, position_offsets_[config_.position]);
    fit_scene_gt(cloud_transformed, gt_cloud_[config_.position], icp_tf, true);
    std::cout << icp_tf.getOrigin().getX() << std::endl;
    std::cout << icp_tf.getOrigin().getY() << std::endl;
    std::cout << icp_tf.getOrigin().getZ() << std::endl;
    std::cout << icp_tf.getRotation().getX() << std::endl;
    std::cout << icp_tf.getRotation().getY() << std::endl;
    std::cout << icp_tf.getRotation().getZ() << std::endl;
    std::cout << icp_tf.getRotation().getW() << std::endl;

    pcl_ros::transformPointCloud(*cloud_transformed, *cloud_transformed, icp_tf);
    cloud_pub_.publish(cloud_transformed);

    rgb_img_pub_.publish(rgb_img_buffer_[config_.camera][config_.position].at(0));
    rgb_info_pub_.publish(rgb_info_buffer_[config_.camera][config_.position].at(0));
    depth_img_pub_.publish(depth_img_buffer_[config_.camera][config_.position].at(0));
    depth_info_pub_.publish(depth_info_buffer_[config_.camera][config_.position].at(0));
        // }
    // }
    // catch (std::out_of_range)
    // {
    //     ROS_ERROR("Unequal amount of msgs in bagfile!.");
    // }
}

void publish_gt()
{
    while(ros::ok())
    {
        gt_pub_.publish(gt_cloud_[config_.position]);

        publish_msgs();


        ros::spinOnce();
    }
}

void save_offsets()
{
    YAML::Node offsets;
    for (int pos = 0; pos < 9 ; ++pos)
    {
        offsets[pos]["position"]["x"] = position_offsets_[pos].getOrigin().getX();
        offsets[pos]["position"]["y"] = position_offsets_[pos].getOrigin().getY();
        offsets[pos]["position"]["z"] = position_offsets_[pos].getOrigin().getZ();
        offsets[pos]["orientation"]["x"] = position_offsets_[pos].getRotation().getX();
        offsets[pos]["orientation"]["y"] = position_offsets_[pos].getRotation().getY();
        offsets[pos]["orientation"]["z"] = position_offsets_[pos].getRotation().getZ();
        offsets[pos]["orientation"]["w"] = position_offsets_[pos].getRotation().getW();
    }
    std::ofstream fout(dataset_path_ + "/correction_offsets.yaml");
    fout << offsets;
}

void read_offsets()
{
    std::string path = dataset_path_ + "/correction_offsets.yaml";
    if (boost::filesystem::is_regular_file(path))
    {
        ROS_INFO("Reading position offsets from file: %s", path.c_str());
        YAML::Node offsets = YAML::LoadFile(path);
        int position = 0;
        std::map<std::string, geometry_msgs::Pose> objects;
        for (const auto& offset : offsets)
        {
            tf::Transform tf;
            tf::Vector3 origin;
            origin.setX(offset["position"]["x"].as<double>());
            origin.setY(offset["position"]["y"].as<double>());
            origin.setZ(offset["position"]["z"].as<double>());

            tf::Quaternion q;
            q.setX(offset["orientation"]["x"].as<double>());
            q.setY(offset["orientation"]["y"].as<double>());
            q.setZ(offset["orientation"]["z"].as<double>());
            q.setW(offset["orientation"]["w"].as<double>());

            tf.setOrigin(origin);
            tf.setRotation(q);
            position_offsets_[position] = tf;

            ++position;
        }
    }
    else
    {
        for (int pos = 0 ; pos < 9 ; ++pos)
        {
            // tf::Transform tf;
            // tf::Vector3 origin;
            // tf::Quaternion q;
            // tf.setOrigin(origin);
            // tf.setRotation(q);
            position_offsets_[pos] = tf::Transform();

        }
    }
}

void load_offsets_reconf(int pos)
{

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "pos_offset_x";
    double_param.value = position_offsets_[pos].getOrigin().getX();
    conf.doubles.push_back(double_param);
    double_param.name = "pos_offset_y";
    double_param.value = position_offsets_[pos].getOrigin().getY();
    conf.doubles.push_back(double_param);
    double_param.name = "pos_offset_z";
    double_param.value = position_offsets_[pos].getOrigin().getZ();
    conf.doubles.push_back(double_param);
    double_param.name = "rot_offset_x";
    double_param.value = position_offsets_[pos].getRotation().getX();
    conf.doubles.push_back(double_param);
    double_param.name = "rot_offset_y";
    double_param.value = position_offsets_[pos].getRotation().getY();
    conf.doubles.push_back(double_param);
    double_param.name = "rot_offset_z";
    double_param.value = position_offsets_[pos].getRotation().getZ();
    conf.doubles.push_back(double_param);
    double_param.name = "rot_offset_w";
    double_param.value = position_offsets_[pos].getRotation().getW();
    conf.doubles.push_back(double_param);

    srv_req.config = conf;
    ROS_INFO("call");
    ros::service::call("set_parameters", srv_req, srv_resp);
    ROS_INFO("called");
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
        ROS_INFO("Folder is: %s", (dataset_path_ + "/scenes/" + scene_).c_str());
        return 0;
    }


    cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("visualizer/depth/points", 1, true);
    debug_pub_ = n.advertise<sensor_msgs::PointCloud2>("visualizer/debug_cloud", 1, true);
    depth_img_pub_ = n.advertise<sensor_msgs::Image>("visualizer/depth/image", 1, true);
    depth_info_pub_ = n.advertise<sensor_msgs::CameraInfo>("visualizer/depth/camera_info", 1, true);
    rgb_img_pub_ = n.advertise<sensor_msgs::Image>("visualizer/rgb/image", 1, true);
    rgb_info_pub_ = n.advertise<sensor_msgs::CameraInfo>("visualizer/rgb/camera_info", 1, true);
    gt_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>>("visualizer/ground_truth", 1, true);
    tf_pub_ = boost::make_shared<tf2_ros::TransformBroadcaster>();

    ModelLoader db_loader(dataset_path_, models_dir_);
    db_loader_ = &db_loader;

    dynamic_reconfigure::Server<ycb_multicam_dataset::visualizerConfig> server;
    dynamic_reconfigure::Server<ycb_multicam_dataset::visualizerConfig>::CallbackType f;
    f = boost::bind(&dynamic_callback, _1, _2);
    server.setCallback(f);

    read_offsets();

    publish_gt();

    ros::shutdown();

    return 0;
}
