#pragma once

#include <stdint.h>
#include <sys/types.h>

#include "timeout.h"

class PriorityAck {

public:
    PriorityAck();
    virtual ~PriorityAck() {};

    void _set_priority(uint32_t msg_id);    // sets the priority level based on the incoming message type
    bool _reset_nice_value();               // resets the priority level to the value when this class was instaniated


    int _initial_nice_val;
    bool _timer_running;
    Timeout * _priority_timeout = nullptr;
    pid_t _this_pid;
};

