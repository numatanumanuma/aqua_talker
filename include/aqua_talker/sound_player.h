#ifndef Sound_PLAYER_H
#define Sound_PLAYER_H

#include <ros/ros.h>
#include <sound_play/SoundRequest.h>

class SoundPlayer{
public:
    SoundPlayer();
    ~SoundPlayer();
    void timerCallback(const ros::TimerEvent&);
    void setSound(std::string file_name);
    void setVolume(double volume);
    void play();
    void playOnce();
    void say(std::string message);
    void cancel();

    bool playing;
   
private:
    ros::Timer timer_;
    ros::Publisher request_pub_;

    void publishGoal();

    std::string file_name_;
    bool now_once_;
    double volume_;
};

#endif