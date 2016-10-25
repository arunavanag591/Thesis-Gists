
#include <ObjectFinder/ObjectFinder.h>
ObjectFinder::ObjectFinder(ros::NodeHandle &nodehandle): nh_(nodehandle) {
    Object_confidence = 0;
    firstCB = false;
    object_sub = nh_.subscribe("/recognized_object_array", 1, &ObjectFinder::objectCallback, this);
    plane_sub = nh_.subscribe("/table_array", 1, &ObjectFinder::planeCallback, this);
    Object_pose.header.frame_id = "camera_depth_optical_frame";
    Object_id = "";
}

void ObjectFinder::objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg) {
    double confident = 0;
    int id = -1;
 

    if (!firstCB && (int)objects_msg.objects.size() == 1) {
        Object_id.assign(objects_msg.objects[0].type.key.c_str());
        Object_pose.header.frame_id = objects_msg.header.frame_id.c_str();
        firstCB = true;
    }
    for (int i = 0; i < objects_msg.objects.size(); ++i) {

        if (Object_id.compare(objects_msg.objects[i].type.key.c_str()) == 0) {
            if (objects_msg.objects[i].confidence > confident) {
                confident = objects_msg.objects[i].confidence;
                id = i;
            }
        }
    }
    if (id >= 0) {
        Object_pose.pose = objects_msg.objects[id].pose.pose.pose;
        Object_confidence = objects_msg.objects[id].confidence;
    } else {
        confident = 0;
    }
   
}


bool ObjectFinder::getObjectPoseKinect(geometry_msgs::PoseStamped &Object_pose, double &confidence) {
    ros::spinOnce();
    if (Object_confidence > 0.8) {
        Object_pose = this->Object_pose;
        confidence = this->Object_confidence;
        return true;
    }
    return false;
}

bool ObjectFinder::getObjectPoseTorso(geometry_msgs::PoseStamped &Object_pose, double &confidence) {
    if (getObjectPoseKinect(Object_pose, confidence)) {
        //stuff a goal message:
        bool tferr = true;
        while (tferr) {
            tferr = false;
            try {
                tf_listener.transformPose("torso", Object_pose, transed_pose);
            } catch (tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr = true;
                ros::Duration(0.1).sleep(); // sleep for half a second
                ros::spinOnce();
            }
        }
        Object_pose = transed_pose;
    }
    return false;
}
