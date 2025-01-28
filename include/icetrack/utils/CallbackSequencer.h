/*
This class is used to sequence callbacks in chronological order. 
This is handy when e.g. sensor messages are delayed because of hardware interfaces, or when working with asynchronous data input from ros topics.*/

#include <functional>
#include <map>

class CallbackSequencer{
public:
    CallbackSequencer(){ }
    CallbackSequencer(double safe_delay): safe_delay_(safe_delay) {}

    void addCallback(double ts, std::function<void()> cb){
        if (ts < t_head_) // Is message even valid?
            return;
        callback_buffer_[ts] = cb;

        // Update safe time and check for valid callbacks
        if (ts - safe_delay_ > t_safe_){
            t_safe_ = ts - safe_delay_;
            checkCallbackBuffer();
        }
    }

    void checkCallbackBuffer(){
        for (auto it = callback_buffer_.begin(); it != callback_buffer_.end();){
            double t_msg = it->first;

            if (t_msg > t_safe_)
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
    double t_safe_ = 0.0;       // Only call functions with stamp before the safetime
    double safe_delay_ = 0.0;   // Delay between latest received timestamp and safetime
};