//ついにできた！！！
#include <ros/ros.h>
#include <server/youbot_server.h>
#include <behavior_tree_msgs/ComputePickTrajectoryAction.h>
#include <behavior_tree_msgs/FollowJointTrajectoryAction.h>
#include <behavior_tree_msgs/GripperAction.h>

#include "youbot_msgs/Object2Array.h"
#include <actionlib/client/simple_action_client.h>
#include "motion_planning/motion_plan_request.h"
#include "motion_plan/trajectory_function.h"
#include "motion_plan/generate_object.h"
#include <string>
#include "orientation_function.h"

#include "client/Gripper_client.h"
#include "client/Move_arm_1_client.h"
#include "client/Move_arm_2_client.h"
#include <Eigen/Core>
using namespace Eigen;

typedef actionlib::SimpleActionClient<behavior_tree_msgs::ComputePickTrajectoryAction> pick_c;
typedef actionlib::SimpleActionClient<behavior_tree_msgs::FollowJointTrajectoryAction> follow_c;

class Pick : public Youbot_Server
{
private:
    pick_c ac;
    follow_c ac_f;
    moveit_msgs::AttachedCollisionObject attached_object;
    moveit_msgs::CollisionObject collision_object;
    std::vector<trajectory_msgs::JointTrajectory> trajectory;
    moveit_msgs::AttachedCollisionObject grasped_object;
    bool success;
    std::string error_string;
    youbot_msgs::Object2Array base_object_array;
    Gripper_Client gripper;
    Move_Arm_1_Client move_arm_1;
    Move_Arm_2_Client move_arm_2;
    /////////////////////////////////toがT///////////////////////////////////////////////////////////////
    const Translation3d trans_youbot_map_to_mecanum_base = Translation3d(0.75, 0.75, 0);
    const AngleAxisd  rot_youbot_map_to_mecanum_base = AngleAxisd(DegToRad(-90), Vector3d::UnitZ());//軸の変換
    const Eigen::Affine3d affine_youbot_map_to_mecanum_base = trans_youbot_map_to_mecanum_base*rot_youbot_map_to_mecanum_base;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////affine_odom_to_base.inverse()
    ros::Subscriber basePoseSub;
    geometry_msgs::PoseStamped goal_pose_msg;


//ここがtrueじゃないと動かない
public:
    bool callback = true;
////////////////////////////////////
public:
    Pick() : ac("compute_pick", true), ac_f("follow_joint_trajectory", true)
    {
        ROS_INFO("Waiting for pick server to start.");
        ac.waitForServer();
        ac_f.waitForServer();
        ROS_INFO("pick server started.");

        // ノードハンドルを作成
        ros::NodeHandle nh;

        // サブスクライバを作成し、base_pose トピックを購読
        basePoseSub = nh.subscribe("/base_pose", 1, &Pick::odomObjectPoseCallback, this);
    }

    // odomObjectPose のコールバック関数
    void odomObjectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& basePoseMsg) {
        
        // base_pose からオドメトリ座標系の位置をEigen::Vector3dに変換
        Eigen::Vector3d base_position(basePoseMsg->pose.position.x, basePoseMsg->pose.position.y, basePoseMsg->pose.position.z);

        // 同次変換行列の逆行列を取得
        Eigen::Affine3d inverse_affine = affine_youbot_map_to_mecanum_base.inverse();

        // オドメトリ座標系からYoubot座標系に変換
        Eigen::Vector3d transformed_position = inverse_affine * base_position;

        // 変換後の位置情報を設定　ここにオフセット足してくといい2.15
        goal_pose_msg.pose.position.x = transformed_position.x() + 0.07;
        goal_pose_msg.pose.position.y = transformed_position.y();
        goal_pose_msg.pose.position.z = transformed_position.z() + 0.08; //youBotとメカナムの差

        // ゴールの位置が設定されたら、ここで何かしらの処理を実行する
        // 例えば、move_arm() 関数を呼び出すなど
        move_arm();
    }

    bool pick_plan()
    {
        //ここが移動前にグリッパーopen
        if(!gripper_is_open)
        {
            gripper.Gripper("open");
        }
        //ここまで
        ROS_INFO("Before sending goal.");
        behavior_tree_msgs::ComputePickTrajectoryGoal goal;
        goal.target_id = "a";
        goal.target_class_id = "bottle";
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        geometry_msgs::Pose goal_pose;

        // 以前の設定から変更
        goal_pose = goal_pose_msg.pose;

        collision_object = bottle(goal_pose, "base_link", "a", true);
        collision_object.subframe_names.push_back("0");
        collision_object.subframe_poses.push_back(goal_pose);

        collision_objects.push_back(collision_object);
        goal.collisions = collision_objects;
        goal.height_offset = 0;
        goal.depth_offset = 0;
        goal.pitch_offset = 0;
        ac.sendGoal(
            goal,
            boost::bind(&Pick::pick_planResultCallback, this, _1, _2),
            pick_c::SimpleActiveCallback(),
            pick_c::SimpleFeedbackCallback());

        bool finished_before_timeout = ac.waitForResult(ros::Duration());

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
        }
        if (trajectory.size() == 2)
            return true;
        else
        {
            ROS_INFO("trajectory num %ld", trajectory.size());
            return false;
        }
        ROS_INFO("After sending goal.");
    }

    bool follow_trajectory()
    {
        for (int i = 0; i < trajectory.size(); i++)
        {
            behavior_tree_msgs::FollowJointTrajectoryGoal goal;

            goal.trajectory = trajectory[i];
            ac_f.sendGoal(
                goal,
                boost::bind(&Pick::follow_trajectoryResultCallback, this, _1, _2),
                follow_c::SimpleActiveCallback(),
                follow_c::SimpleFeedbackCallback());

            bool finished_before_timeout = ac_f.waitForResult(ros::Duration());
            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac_f.getState();
                ROS_INFO("Action finished: %s", state.toString().c_str());
            }
            else
            {
                ROS_INFO("Action did not finish before the time out.");
            }
            if (error_string != "success")
                return false;
        }
        return true;
    }

    bool grasp()
    {
        if (!gripper_is_open)
        {
            gripper.Gripper("open");
        }
        bool grasp_success;
        gripper.Gripper("grasp", &grasp_success);
        // return true; //ここをgrasp_successにすると止まる。
        // return grasp_success;
        //////追加してみた
        if (grasp_success)
        {
            ROS_INFO("Grasping successful!");
        }
        else
        {
            ROS_INFO("Grasping failed!");
        }

        return grasp_success;
        ///////////////////////
    }
////////////////////////////////
    bool place()
    {
        if (gripper_is_open)
        {
            // グリッパーを閉じる
            gripper.Gripper("close");
        }

        // アームを対象物を置く位置に移動させる
        // move_arm_1.Move_arm_1(DegToRad(-60), DegToRad(50), DegToRad(30), DegToRad(0), DegToRad(0));
        // ここでアームが対象物を置く位置に到達するまで待つ
        ros::Duration(5.0).sleep();

        // グリッパーを開く
        gripper.Gripper("open");
        // ここでグリッパーが完全に開くまで待つ
        ros::Duration(2.0).sleep();

        // もしこの時点で対象物が成功裏に置かれたかどうかの確認が必要なら、適切な条件文を追加してください。
        bool place_success = true; // 例えば、常に成功したものとしていますが、実際の条件に応じて修正してください。

        return place_success;
    }
////////////////////////////////////



    bool move_arm()
    {
        move_arm_1.Move_arm_1(DegToRad(-10), DegToRad(0), DegToRad(45), DegToRad(45), DegToRad(0));
        return true;
    }

    bool turn_arm()
    {
        move_arm_1.Move_arm_1(DegToRad(-60), DegToRad(50), DegToRad(40), DegToRad(0), DegToRad(0));
        return true;
    }

    bool turnback_arm()
    {
        move_arm_1.Move_arm_1(DegToRad(-60), DegToRad(0), DegToRad(45), DegToRad(45), DegToRad(0));
        return true;
    }
    ////逆のアーム
    bool opposite_move_arm()
    {
        move_arm_2.Move_arm_2(DegToRad(-140), DegToRad(50), DegToRad(-80), DegToRad(-80), DegToRad(0));
        return true;
    }
    bool opposite_place()
    {
        move_arm_2.Move_arm_2(DegToRad(-120), DegToRad(-60), DegToRad(-30), DegToRad(-20), DegToRad(0));
        return true;
    }




    //cmdvel移動のやつ///////////////
    void stop_robot(ros::NodeHandle& n)
    {
        ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/youbot/cmd_vel", 10);
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            twist_pub.publish(twist);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void move_robot_x(ros::NodeHandle &n)
    {
        ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/youbot/cmd_vel", 10);
        geometry_msgs::Twist twist;
        twist.linear.x = 0.1;//マイナスにすれば逆方向
        // twist.linear.x = -0.1;
        twist.linear.y = 0;
        twist.angular.z = 0;

        ros::Rate loop_rate(10);
        for (int i = 0; i < 40; ++i)  // 1秒間動かす
        {
            twist_pub.publish(twist);
            loop_rate.sleep();
        }
    }

    void move_robot_y(ros::NodeHandle &n)
    {
        ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/youbot/cmd_vel", 10);
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = -0.1;
        // twist.linear.y = 0.1;
        twist.angular.z = 0;

        ros::Rate loop_rate(10);
        for (int i = 0; i < 20; ++i)  // 1秒間動かす
        {
            twist_pub.publish(twist);
            loop_rate.sleep();
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////


    void pick_planResultCallback(const actionlib::SimpleClientGoalState &state,
                                 const behavior_tree_msgs::ComputePickTrajectoryResultConstPtr &result)
    {
        trajectory = result->trajectories;
        grasped_object = result->grasped_object;

        ROS_INFO("Attached Collision Object:");
        ROS_INFO("Link Name: %s", grasped_object.link_name.c_str());

        ROS_INFO("Collision Object ID: %s", grasped_object.object.id.c_str());
        ROS_INFO("Collision Object Operation: %d", grasped_object.object.operation);
        for (int i = 0; i < grasped_object.object.primitive_poses.size(); i++)
        {
            ROS_INFO("Collision Object primitives :");
            ROS_INFO("%f", grasped_object.object.primitive_poses[i].position.x);
            ROS_INFO("%f", grasped_object.object.primitive_poses[i].position.y);
            ROS_INFO("%f", grasped_object.object.primitive_poses[i].position.z);
            ROS_INFO("%f", grasped_object.object.primitive_poses[i].orientation.x);
            ROS_INFO("%f", grasped_object.object.primitive_poses[i].orientation.y);
            ROS_INFO("%f", grasped_object.object.primitive_poses[i].orientation.z);
            ROS_INFO("%f", grasped_object.object.primitive_poses[i].orientation.w);
        }

        ROS_INFO("Touch Links:");
        for (const auto &touch_link : grasped_object.touch_links)
        {
            ROS_INFO("- %s", touch_link.c_str());
        }

        ROS_INFO("Detach Posture:");
        for (size_t i = 0; i < grasped_object.detach_posture.joint_names.size(); ++i)
        {
            ROS_INFO("- Joint Name: %s, Position: %f", grasped_object.detach_posture.joint_names[i].c_str(),
                     grasped_object.detach_posture.points[0].positions[i]);
        }

        ROS_INFO("Weight: %f", grasped_object.weight);

        ROS_INFO("--------------------------------------");
    }

    void follow_trajectoryResultCallback(const actionlib::SimpleClientGoalState &state,
                                         const behavior_tree_msgs::FollowJointTrajectoryResultConstPtr &result)
    {
        error_string = result->error_string;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick");
    ros::NodeHandle n;
    Pick pick;

    while (ros::ok())
    {
        if (pick.callback)
        {
            bool plan = pick.pick_plan();
            if (plan)
            {
                bool follow = pick.follow_trajectory();
                if (follow)
                {
                    bool grasp = pick.grasp();
                    if (grasp)
                    {
                        //これがいつものアームに関するやつ
                        bool move = pick.move_arm();
                        ros::Duration(2.0).sleep();
                        

                        //向きかえる
                        bool turn = pick.turn_arm();
                        ros::Duration(1.0).sleep();
                        bool place = pick.place();
                        //戻る
                        ros::Duration(2.0).sleep();
                        bool turnback = pick.turnback_arm();


                        /////ここから逆のアーム

                        ros::Duration(1.0).sleep();
                        //逆の手を動かす コメント外してね(双椀１)
                        // bool opposite_move = pick.opposite_move_arm();

                        // //逆の手でplace　コメント外してね(双椀２)
                        // ros::Duration(1.0).sleep();
                        // bool opposite_place = pick.opposite_place();

                        // //////ここから移動に関するやつ
                        // pick.move_robot_y(n);
                        // pick.move_robot_x(n);
                        // pick.stop_robot(n);



                        break;  // 処理が終わったらループを抜ける
                    }
                }

            }
        }
        ros::spinOnce();
    }

    return 0;
}