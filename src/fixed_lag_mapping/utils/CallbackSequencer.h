/*
This class is used to sequence callbacks in chronological order. 
This is handy when e.g. sensor messages are delayed because of hardware interfaces, or when working with asynchronous data input from ros topics.
*/

#pragma once

#include <ros/ros.h>

#include <functional>
#include <map>

class CallbackSequencer{
public:
    CallbackSequencer(): safe_delay_(0) {}
    CallbackSequencer(double safe_delay): safe_delay_(safe_delay) {}

    inline double getSafeTime() const { return ts_head_ - safe_delay_; }

    /*
    Add a callback to be sequenced according to its timestamp.
    */
    void addCallback(double ts, std::function<void()> cb){
        if (ts < getSafeTime()) // Discard if message if too late
            return;
        
        callback_buffer_[ts] = cb;
        if (ts > ts_head_){
            ts_head_ = ts;
            checkCallbackBuffer();
        }
    }

    /*
    Proccesss callbacks with stamped before safe time.
    */
    void checkCallbackBuffer(){
        const double t_safe = getSafeTime();

        // callback buffer iteration is chronological
        for (auto it = callback_buffer_.begin(); it != callback_buffer_.end();){
            double t_msg = it->first;

            if (t_msg > t_safe)
                break;
            
            it->second(); // Callback 
            it = callback_buffer_.erase(it);
        }
    }

private:
    std::map<double, std::function<void()>> callback_buffer_;   // Buffer for callbacks
    double ts_head_; 
    double safe_delay_;   // Difference between wall time and safe time.
};