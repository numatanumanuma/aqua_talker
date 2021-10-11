#include "aqua_talker/sound_player.h"
   
SoundPlayer::SoundPlayer(){
    ros::NodeHandle nh("~");
    timer_ = nh.createTimer(ros::Duration(0.05), &SoundPlayer::timerCallback, this);
    request_pub_ = nh.advertise<sound_play::SoundRequest>("/robotsound", 1);
    ros::Rate loop_rate(10);
    loop_rate.sleep(); //publish設定したあと少し待たないとバグる

    now_once_ = true;
    file_name_ = "";
    volume_ = 1.0;
}

SoundPlayer::~SoundPlayer(){

}

void SoundPlayer::timerCallback(const ros::TimerEvent&){
    
}

void SoundPlayer::setSound(std::string file_name) {
    file_name_ = file_name;
}

void SoundPlayer::setVolume(double volume) {
    if (volume < 0 || volume > 1.0){
        ROS_ERROR("0 as mute and 1.0 as 100％");
        return;
    }
    volume_ = volume;
}

void SoundPlayer::play() {
    now_once_ = false;
    publishGoal();
}

void SoundPlayer::playOnce() {
    now_once_ = true;
    publishGoal();
}

void SoundPlayer::say(std::string message){
    sound_play::SoundRequest request;
    request.sound = -3;
    request.command = 1;
    request.volume = volume_;
    request.arg = message;
    request.arg2 = "voice_kal_diphone";
    request_pub_.publish(request);
    ros::Rate loop_rate(10);
    loop_rate.sleep(); //publishしたあと少し待たないとバグる
    ROS_INFO("request Published");
}

void SoundPlayer::publishGoal() {
    if (file_name_.length() <= 0) {
        ROS_ERROR("not set sound file!");
        return;
    }

    sound_play::SoundRequest request;
    request.sound = -2;
    if (now_once_){
        request.command = 1;
    } else {
        request.command = 2;
    }
    request.volume = volume_;
    request.arg = file_name_;
    request_pub_.publish(request);
    ros::Rate loop_rate(10);
    loop_rate.sleep(); //publishしたあと少し待たないとバグる
    ROS_INFO("request Published");

}

void SoundPlayer::cancel() {
    sound_play::SoundRequest request;
    request.sound = -1;
    request.command = 0;
    request_pub_.publish(request);
    ros::Rate loop_rate(10);
    loop_rate.sleep(); //publishしたあと少し待たないとバグる
    ROS_INFO("cancel Published");
}