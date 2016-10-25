#include <ArmOperation/ArmOperation.h>

ArmOperation::ArmOperation(ros::NodeHandle &nodehandle) : nh_(nodehandle),  right_gripper(nodehandle), right_arm("right_arm"), left_arm("left_arm"),        display_publisher(nh_.advertise<moveit_msgs::DisplayTrajectory> ("/move_group/display_planned_path", 1, true))  {
  
    ROS_INFO("Reference right arm robot frame: %s", right_arm.getPlanningFrame().c_str());
    ROS_INFO("Reference right arm end-effector frame: %s", right_arm.getEndEffectorLink().c_str());
    ROS_INFO("Reference left arm robot frame: %s", left_arm.getPlanningFrame().c_str());
    ROS_INFO("Reference left arm end-effector frame: %s", left_arm.getEndEffectorLink().c_str());
    right_arm.setPlanningTime(5.0);
    pick_offset << 0, 0, 0.3;
    hold_offset << 0, -0.2, 0.3;
    pre_grab_offset << 0, -0.2, 0;
    grab_offset << 0, 0, 0;

    right_arm_back_pose.position.x = 0.48336029291;
    right_arm_back_pose.position.y = -0.345984422306;
    right_arm_back_pose.position.z = 0.442497286433;
    right_arm_back_pose.orientation.x = 1;
    right_arm_back_pose.orientation.y = 0;
    right_arm_back_pose.orientation.z = 0;
    right_arm_back_pose.orientation.w = 0;

    left_arm_back_pose.position.x = 0.356899870469;
    left_arm_back_pose.position.y = 0.553228163753;
    left_arm_back_pose.position.z = 0.333650371585;
    left_arm_back_pose.orientation.x = 1;
    left_arm_back_pose.orientation.y = 0;
    left_arm_back_pose.orientation.z = 0;
    left_arm_back_pose.orientation.w = 0;

    global_pose_offset.position.x = 0;
    global_pose_offset.position.y = 0;
    global_pose_offset.position.z = 0;
    global_pose_offset.orientation.x = 0;
    global_pose_offset.orientation.y = 0;
    global_pose_offset.orientation.z = 0;
    global_pose_offset.orientation.w = 0;

    global_joints_offset << 0,0,0,0,0,0,0;
    right_gripper.set_mode(BaxterGripper::OPEN_CLOSE);
}

bool ArmOperation::rightArmBack() {
    ROS_INFO("Move right arm back");
    ROS_INFO("Release gripper");
    rightRelease();
    return rightMove(right_arm_back_pose);
}

void ArmOperation::rightExecute() {
    ROS_INFO("executing right arm plan");
    right_arm.execute(right_plan);
}

geometry_msgs::Pose ArmOperation::rightGetPose() {
    geometry_msgs::PoseStamped arm_pose = right_arm.getCurrentPose();
    geometry_msgs::Pose pose = arm_pose.pose;
    return pose;
}

Vector7d ArmOperation::rightGetJoints() {
    Vector7d joints;
   
    std::vector<double> group_variable_values;
    right_arm.getCurrentState()->copyJointGroupPositions(right_arm. getCurrentState()->getRobotModel()->getJointModelGroup(right_arm.getName()), group_variable_values);
    for (int i = 0; i < 7; ++i)
    {
        joints[i] = group_variable_values[i];
    }
    return joints;
}

void ArmOperation::rightGrab() {
    ROS_INFO("close gripper");
    right_gripper.open();
}

void ArmOperation::rightRelease() {
    ROS_INFO("open gripper");
    right_gripper.close();
}

bool ArmOperation::rightPlan(geometry_msgs::Pose pose) {
    ROS_INFO("requesting a cartesian-space motion plan");
    pose = addPose(pose, global_pose_offset);
    right_arm.setPoseTarget(pose);
    return right_arm.plan(right_plan);
}

bool ArmOperation::rightPlan(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
    geometry_msgs::Pose pose;
    pose = normalToPose(plane_normal, major_axis, centroid);
    return rightPlan(pose);
}

bool ArmOperation::rightPlan(Vector7d joints) {
    ROS_INFO("requesting a joint-space motion plan");
    joints = joints + global_joints_offset;
    std::vector<double> group_variable_values(7);
    for (int i = 0; i < 7; ++i)
    {
        group_variable_values[i] = joints[i];
    }
    right_arm.setJointValueTarget(group_variable_values);
    return right_arm.plan(right_plan);
}

bool ArmOperation::rightPlanOffset(Vector3d offset) {
    geometry_msgs::Pose offset_pose = addPosOffset(rightGetPose(), offset);
    return rightPlan(offset_pose);
}

bool ArmOperation::rightMove(geometry_msgs::Pose pose) {
    if(rightPlan(pose)){
        rightExecute();
        return true;
    }
    return false;
}

bool ArmOperation::rightMove(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
    if(rightPlan(plane_normal, major_axis, centroid)){
        rightExecute();
        return false;
    }
    return false;
}

bool ArmOperation::rightMove(Vector7d joints) {
    if(rightPlan(joints)){
        rightExecute();
        return true;
    }
    return false;
}

bool ArmOperation::rightMoveOffset(Vector3d offset) {
    if(rightPlanOffset(offset)){
        rightExecute();
        return true;
    }
    return false;
}

void ArmOperation::rightShowPath() {
    ROS_INFO("Visualizing planed Path");
    display_trajectory.trajectory_start = right_plan.start_state_;
    display_trajectory.trajectory.push_back(right_plan.trajectory_);
    display_publisher.publish(display_trajectory);
   
}
bool ArmOperation::leftArmBack() {
    ROS_INFO("Move left arm back");
    return leftMove(left_arm_back_pose);
}
void ArmOperation::leftExecute() {
    left_arm.execute(left_plan);
}

geometry_msgs::Pose ArmOperation::leftGetPose() {
    geometry_msgs::PoseStamped arm_pose = left_arm.getCurrentPose();
    geometry_msgs::Pose pose = arm_pose.pose;
    return pose;
}

Vector7d ArmOperation::leftGetJoints() {
    Vector7d joints;    
    std::vector<double> group_variable_values;
    left_arm.getCurrentState()->copyJointGroupPositions(left_arm. getCurrentState()->getRobotModel()->getJointModelGroup(left_arm.getName()), group_variable_values);
    for (int i = 0; i < 7; ++i)
    {
        joints[i] = group_variable_values[i];
    }
    return joints;
}

bool ArmOperation::leftPlan(geometry_msgs::Pose pose) {
   
    pose = addPose(pose, global_pose_offset);
    left_arm.setPoseTarget(pose);
    return left_arm.plan(left_plan);
}

bool ArmOperation::leftPlan(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
    geometry_msgs::Pose pose;
    pose = normalToPose(plane_normal, major_axis, centroid);
    return leftPlan(pose);
}

bool ArmOperation::leftPlan(Vector7d joints) {
    
    joints = joints + global_joints_offset;
    std::vector<double> group_variable_values(7);
    for (int i = 0; i < 7; ++i)
    {
        group_variable_values[i] = joints[i];
    }
    left_arm.setJointValueTarget(group_variable_values);
    return left_arm.plan(left_plan);
}

bool ArmOperation::leftPlanOffset(Vector3d offset) {
    geometry_msgs::Pose offset_pose = addPosOffset(leftGetPose(), offset);
    return leftPlan(offset_pose);
}

bool ArmOperation::leftMove(geometry_msgs::Pose pose) {
    if(leftPlan(pose)){
        leftExecute();
        return true;
    }
    return false;
}

bool ArmOperation::leftMove(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
    if(leftPlan(plane_normal, major_axis, centroid)){
        leftExecute();
        return false;
    }
    return false;
}

bool ArmOperation::leftMove(Vector7d joints) {
    if(leftPlan(joints)){
        leftExecute();
        return true;
    }
    return false;
}

bool ArmOperation::leftMoveOffset(Vector3d offset) {
    if(leftPlanOffset(offset)){
        leftExecute();
        return true;
    }
    return false;
}

void ArmOperation::leftShowPath() {
    ROS_INFO("Visualizing planed Path");
    display_trajectory.trajectory_start = left_plan.start_state_;
    display_trajectory.trajectory.push_back(left_plan.trajectory_);
    display_publisher.publish(display_trajectory);
 
}
bool ArmOperation::ArmBack() {
    bool right = rightArmBack();
    bool left = leftArmBack();
    if ((right == true) && (left == true)) {
        return true;
    }
    return false;
}

bool ArmOperation::grabObject(geometry_msgs::Pose Object_pose) {
    Object_pose.orientation.x = -0.708866454238;
    Object_pose.orientation.y = 0.17589525363;
    Object_pose.orientation.z = 0.135739113146;
    Object_pose.orientation.w = 0.669435660067;
    geometry_msgs::Pose grab_pose = addPosOffset(Object_pose, grab_offset);
    geometry_msgs::Pose pre_grab_pose = addPosOffset(Object_pose, pre_grab_offset);
    geometry_msgs::Pose hold_pose = addPosOffset(Object_pose, hold_offset);
    geometry_msgs::Pose pick_pose = addPosOffset(Object_pose, pick_offset);


    ROS_INFO("Move to hold pose");
    if (!rightMove(hold_pose)) { return false; }
    ros::Duration(0.5).sleep();
    ROS_INFO("Move to pre grab pose");
    if (!rightMove(pre_grab_pose)) { return false; }
    ros::Duration(0.5).sleep();
    ROS_INFO("Move to grab pose");
    if (!rightMove(grab_pose)) { return false; }
    ros::Duration(0.5).sleep();
    ROS_INFO("Grab the Object");
    rightGrab();
    ros::Duration(0.5).sleep();
    ROS_INFO("Move to pick pose");
    if (!rightMove(pick_pose)) { return false; }
    ros::Duration(0.5).sleep();
    ROS_INFO("Move to hold pose");
    if (!rightMove(hold_pose)) { return false; }
    ros::Duration(0.5).sleep();
}
Eigen::Affine3d ArmOperation::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;

    Eigen::Vector3d Oe;

    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;

    return affine;
}

geometry_msgs::Pose ArmOperation::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); 
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

geometry_msgs::Pose ArmOperation::normalToPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
    geometry_msgs::Pose pose;
    Affine3d Affine_des_gripper;
    Vector3d xvec_des,yvec_des,zvec_des,origin_des;

    Matrix3d Rmat;
    for (int i=0;i<3;i++) {
        origin_des[i] = centroid[i]; 
        zvec_des[i] = -plane_normal[i]; 
        xvec_des[i] = major_axis[i];
    }

    yvec_des = zvec_des.cross(xvec_des);  
    Rmat.col(0)= xvec_des;
    Rmat.col(1)= yvec_des;
    Rmat.col(2)= zvec_des;
    Affine_des_gripper.linear()=Rmat;
    Affine_des_gripper.translation()=origin_des;

   
    pose = transformEigenAffine3dToPose(Affine_des_gripper);
    return pose;
}

geometry_msgs::Pose ArmOperation::addPosOffset(geometry_msgs::Pose pose, Vector3d offset) {
    geometry_msgs::Pose result;
    result.position.x=pose.position.x+offset[0];
    result.position.y=pose.position.y+offset[1];
    result.position.z=pose.position.z+offset[2];
    result.orientation=pose.orientation;
    return result;
}
geometry_msgs::Pose ArmOperation::subPosOffset(geometry_msgs::Pose pose, Vector3d offset) {
    geometry_msgs::Pose result;
    result.position.x=pose.position.x-offset[0];
    result.position.y=pose.position.y-offset[1];
    result.position.z=pose.position.z-offset[2];
    result.orientation=pose.orientation;
    return result;
}
geometry_msgs::Pose ArmOperation::addPose(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b) {
    geometry_msgs::Pose result;
    result.position.x=pose_a.position.x+pose_b.position.x;
    result.position.y=pose_a.position.y+pose_b.position.y;
    result.position.z=pose_a.position.z+pose_b.position.z;

    result.orientation.x=pose_a.orientation.x+pose_b.orientation.x;
    result.orientation.y=pose_a.orientation.y+pose_b.orientation.y;
    result.orientation.z=pose_a.orientation.z+pose_b.orientation.z;
    result.orientation.w=pose_a.orientation.w+pose_b.orientation.w;
    return result;
}

std::vector<double> ArmOperation::quat2euler(geometry_msgs::Quaternion quaternion) {
    double mData[4];
    std::vector<double> euler(3);
    const static double PI_OVER_2 = M_PI * 0.5;
    const static double EPSILON = 1e-10;
    double sqw, sqx, sqy, sqz;

    mData[0] = quaternion.x;
    mData[1] = quaternion.y;
    mData[2] = quaternion.z;
    mData[3] = quaternion.w;
   
    sqw = mData[3] * mData[3];
    sqx = mData[0] * mData[0];
    sqy = mData[1] * mData[1];
    sqz = mData[2] * mData[2];

    euler[1] = asin(2.0 * (mData[3] * mData[1] - mData[0] * mData[2]));
    if (PI_OVER_2 - fabs(euler[1]) > EPSILON) {
        euler[2] = atan2(2.0 * (mData[0] * mData[1] + mData[3] * mData[2]),
                         sqx - sqy - sqz + sqw);
        euler[0] = atan2(2.0 * (mData[3] * mData[0] + mData[1] * mData[2]),
                         sqw - sqx - sqy + sqz);
    } else {
      
        euler[2] = atan2(2 * mData[1] * mData[2] - 2 * mData[0] * mData[3],
                         2 * mData[0] * mData[2] + 2 * mData[1] * mData[3]);
        euler[0] = 0.0;

       
        if (euler[1] < 0)
            euler[2] = M_PI - euler[2];
    }
    return euler;
}
