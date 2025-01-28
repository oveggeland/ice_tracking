/*
This class is used to sequence callbacks in chronological order. 
This is handy when e.g. sensor messages are delayed because of hardware interfaces, or when working with asynchronous data input from ros topics.
*/

#pragma once

#include <functional>
#include <map>

class CallbackSequencer{
public:
    CallbackSequencer(){}
    CallbackSequencer(double safe_delay): safe_delay_(safe_delay) {}

    /*
    Add a callback to be sequenced according to its timestamp.
    */
    void addCallback(double ts, std::function<void()> cb){
        if (ts < t_head_) // Discard if message is too late
            return;
        callback_buffer_[ts] = cb;

        // Check for new valid callbacks
        checkCallbackBuffer(ts);
    }

    /*
    Check for callbacks that are safe to call (i.e. safe_time has passed).
    */
    void checkCallbackBuffer(double t_wall){
        double t_safe = t_wall - safe_delay_;

        // Process all messages stamped before safe time
        for (auto it = callback_buffer_.begin(); it != callback_buffer_.end();){
            double t_msg = it->first;

            if (t_msg > t_safe)
                break;
            
            it->second(); // Callback 
            it = callback_buffer_.erase(it);
            t_head_ = t_msg;
        }
    }

private:
    std::map<double, std::function<void()>> callback_buffer_;   // Buffer for callbacks
    
    // Timestamp management
    double t_head_ = 0.0;       // Timestamp of last call
    double safe_delay_ = 0.0;   // Delay between latest received timestamp and safetime
};