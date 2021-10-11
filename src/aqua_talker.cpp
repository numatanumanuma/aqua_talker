#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

#include "aqua_talker/sound_player.h"

int movebase_id;

ros::Time start_t;
ros::Duration limit_t;
bool is_loop;

namespace RobotBehaviors {
    enum State {
        PENDING,
        RUNNING,
        STOPPING,
        RECOVERY,
        CONFLICT
    };
}

void startTimer(double t) {
    start_t = ros::Time::now();
    is_loop = false;
    if (t < 0){
        is_loop = true;
    }
    limit_t = ros::Duration(t);
}
bool checkTimer() {
    if (is_loop)
        return false;
    ros::Duration t = ros::Time::now() - start_t;
    if (ros::Time::now() - start_t > limit_t) {
        return true;
    }else {
        return false;
    }
}

void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    movebase_id = 0;
    if (!status->status_list.empty()){
        actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
        movebase_id = goalStatus.status;
    }
}

/*
http://docs.ros.org/en/jade/api/actionlib_msgs/html/msg/GoalStatus.html
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
*/
int getStatus(){
    int status = RobotBehaviors::RUNNING;
    switch (movebase_id)
        {
        case 0: // PENDING
            status = RobotBehaviors::PENDING;
            break;
        case 1: // ACTIVE
            status = RobotBehaviors::STOPPING;
            break;
        case 2: // PREEMPTED
            status = RobotBehaviors::RUNNING;
            break;
        case 3: // SUCCEEDED
            status = RobotBehaviors::RUNNING;
            break;
        case 4: // ABORTED
            status = RobotBehaviors::STOPPING;
            break;
        case 5: // REJECTED
            status = RobotBehaviors::STOPPING;
            break;
        case 6: // PREEMPTING
            status = RobotBehaviors::STOPPING;
            break;
        case 7: // RECALLING
            status = RobotBehaviors::STOPPING;
            break;
        case 8: // RECALLED
            status = RobotBehaviors::STOPPING;
            break;
        case 9: // LOST
            status = RobotBehaviors::STOPPING;
            break;
        default:
            status = RobotBehaviors::PENDING;
            break;
        }
    std::cout << "Now movebase_id... " << movebase_id << std::endl;
    return status;
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
    ros::Duration(2).sleep();
    ros::spinOnce();

    int pre_status = -1;
    bool bgm_playing = false;
    while(ros::ok()) {
        int status = getStatus();
        if (status == pre_status){
            // 前回と同じstatusだった場合, スルー
            looprate.sleep();
            ros::spinOnce();
            continue;
        }
        if (bgm_playing){
            // play()でbgmをループで流しているときcancelしないと上書きされないときがある
            player.cancel();
        }
        bgm_playing = false;

        switch (status)
        {
        case RobotBehaviors::PENDING:
            player.say("pending");
            std::cout << "PENDING..." << std::endl;
            break;
        case RobotBehaviors::RUNNING:
            player.setSound(sound1);
            player.play();
            bgm_playing = true;
            // player.say("running");
            std::cout << "RUNNING!" << std::endl;
            break;
        case RobotBehaviors::STOPPING:
            player.setSound(sound2);
            player.playOnce();
            // player.say("stopping");
            std::cout << "STOPPING......" << std::endl; 
            break;       
        default:
            break;
        }
        pre_status = status;

        looprate.sleep();
        ros::spinOnce();
    }

    return 0;
}