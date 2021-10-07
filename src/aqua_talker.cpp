#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

#include "aqua_talker/sound_player.h"

int status_id;

void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9

    if (!status->status_list.empty()){
        actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
        status_id = goalStatus.status;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aqua_talker");
    ros::NodeHandle nh("~");
    ros::Rate looprate(1);

    ros::Subscriber move_base_status_sub;
    move_base_status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &navStatusCallBack);
    std::string sound1;
    std::string sound2;
    bool debug;
    nh.param("sound1", sound1, std::string("sound1.wav"));
    nh.param("sound2", sound2, std::string("sound2.wav"));
    nh.param("debug", debug, false);

    SoundPlayer player;
    ros::Duration(1).sleep();
    ros::spinOnce();

    status_id = -1;
    int pre_status_id = -1;
    bool is_update = true;
    while(ros::ok()) {
        std::cout << "Now status_id... " << status_id << std::endl;
        if (status_id != pre_status_id)
            is_update = true;
        if(!is_update)
            continue;
        
        switch (status_id)
        {
        case 0:
            std::cout << "hai" << std::endl;
            break;
        default:
            break;
        }

        looprate.sleep();
        ros::spinOnce();
    }

    return 0;
}