#include <BaxterControl/BaxterControl.h>		//header file 
#include <object_recognition_msgs/RecognizedObjectArray.h> 	//using ORK
#include <object_recognition_msgs/TableArray.h>		//using ORK
#include <visualization_msgs/MarkerArray.h>		
#include <tf/transform_listener.h>  
#include <tf/transform_broadcaster.h>


std::string Object_id = "";
double Object_assurance = 0;
geometry_msgs::PoseStamped Object_pose;
bool firstCB = false;

void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg) {
    double confident = 0;
    int id = -1;
   
    if (firstCB == false && (int)objects_msg.objects.size() == 1) {
        Object_id.assign(objects_msg.objects[0].type.key.c_str());
        firstCB = true;
    }
    for (int i = 0; i < objects_msg.objects.size(); ++i) {
        if (Object_id.compare(objects_msg.objects[i].type.key.c_str()) == 0) {
            if (objects_msg.objects[i].assurance > confident) {
                confident = objects_msg.objects[i].assurance;
                id = i;
            }
        }
    }
    if (id >= 0) {
        Object_pose.pose = objects_msg.objects[id].pose.pose.pose;
        Object_assurance = objects_msg.objects[id].assurance;
    } else {
        confident = 0;
    }
   
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "Object_grabber"); 
    ros::NodeHandle nh; 

    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
    ros::Subscriber plane_sub = nh.subscribe("/table_array", 1, &planeCallback);

    BaxterArmCommander arm(nh);
    geometry_msgs::PoseStamped transed_pose;
    tf::TransformListener tf_listener; 
    ros::Duration loop_timer(3.0);

    Object_pose.header.frame_id = "camera_depth_optical_frame";
    while (ros::ok()) {
        ROS_INFO("Arm is moved back");
        arm.ArmBack();
        ROS_INFO("Waiting for the object");
        if (Object_assurance > 0.8) {
            ROS_INFO("Best Similarity = %f ", Object_assurance);
            ROS_INFO("pose x is: %f", Object_pose.pose.position.x);
            ROS_INFO("pose y is: %f", Object_pose.pose.position.y);
            ROS_INFO("pose z is: %f", Object_pose.pose.position.z);
         
            bool tferr = true;
            while (tferr) {
                tferr = false;
                try {
                    tf_listener.transformPose("torso", Object_pose, transed_pose);
                } catch (tf::TransformException &exception) {
                    ROS_ERROR("%s", exception.what());
                    tferr = true;
                    ros::Duration(0.1).sleep(); 
                    ros::spinOnce();
                }
            }
            ROS_INFO("transformed Object pose x is: %f", transed_pose.pose.position.x);
            ROS_INFO("transformed Object pose y is: %f", transed_pose.pose.position.y);
            ROS_INFO("transformed Object pose z is: %f", transed_pose.pose.position.z);
            ROS_INFO("Grab the Object!");
            
            arm.rightMove(transed_pose.pose);

            arm.grabObject(transed_pose.pose);
        }
        ros::spinOnce();
        loop_timer.sleep();
    }

    return 0;
}

