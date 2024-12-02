#pragma once

/*
This class subscribes to a list of topics and process the incoming messages in chronological order (accounting for unordered receptions)
A safe time to wait before processing the messages is used to ensure that no messages lagging behind are not skipped.
*/

#include <ros/ros.h>

class MessageSequencer{
public:
    MessageSequencer(ros::NodeHandle nh, double safe_delay): 
        nh_(nh), safe_delay_(safe_delay){}

    template <typename MessageType>
    void subscriberCallback(const typename MessageType::ConstPtr& msg, std::function<void(const typename MessageType::ConstPtr&)> callback){
        pushCallback(msg->header.stamp, std::function<void()>(std::bind(callback, msg)));
    }

    template <typename MessageType>
    void attachSubscriber(const std::string& topic_name, int queue_size, std::function<void(const typename MessageType::ConstPtr&)> callback) {
        ROS_INFO_STREAM("Attaching subscriber to topic: " << topic_name << " with queue size " << queue_size);
        // Using a lambda to bind the callback and pass the correct template type
        auto cb = [this, callback](const typename MessageType::ConstPtr& msg) {
            this->subscriberCallback<MessageType>(msg, callback); // Forward to the template function
        };

        // Create a subscriber and store it
        ros::Subscriber sub = nh_.subscribe<MessageType>(topic_name, queue_size, cb);
        subscribers_.push_back(sub);
    }

    void pushCallback(ros::Time ts, std::function<void()> cb){
        if (ts > t_wall_){
            t_wall_ = ts;
            t_safe_ = t_wall_ - ros::Duration(safe_delay_);
        }

        callback_buffer_[ts] = cb;
    }

    void pollCallbacks(){
        // Check for safe measurements
        for (auto it = callback_buffer_.begin(); it != callback_buffer_.end();){
            ros::Time ts = it->first;
            if (ts < t_safe_){
                if (ts < t_head_){
                    ROS_WARN_STREAM("Timestamp before filter head");
                }
                else{
                    it->second();
                    t_head_ = ts;
                }

                callback_buffer_.erase(it++);
            }
            else{
                it++;
            }
        }
    }

    void clear(){
        callback_buffer_.clear();
    }

    void flushCallbacks(){
        for (auto it = callback_buffer_.begin(); it != callback_buffer_.end();){
            ros::Time ts = it->first;
            if (ts < t_head_){
                ROS_WARN_STREAM("Timestamp before filter head");
            }
            else{
                it->second();
                t_head_ = ts;
            }

            callback_buffer_.erase(it++);
        }
    }

private:
    ros::Time t_wall_;
    ros::Time t_head_;
    ros::Time t_safe_;
    double safe_delay_;

    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subscribers_;

    // Sorted key: Timestamp. Item: callback function
    std::map<ros::Time, std::function<void()>> callback_buffer_;
};