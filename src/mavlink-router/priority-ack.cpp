#include <functional>
#include <unistd.h>
#include <mutex>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <mavlink.h>

#include "mainloop.h"
#include "priority-ack.h"

PriorityAck::PriorityAck() {
    _initial_nice_val = getpriority(PRIO_PROCESS, getpid());
    _timer_running = false;
    _this_pid = getpid();
}

bool PriorityAck::_reset_nice_value() {

    if ((getpriority(PRIO_PROCESS, _this_pid) != _initial_nice_val)) {
        setpriority(PRIO_PROCESS, _this_pid, _initial_nice_val);
    }

    _timer_running = false;

    return false;
}

void PriorityAck::_set_priority(uint32_t msg_id) {

    // temporarily set the nice level of this process to -20 to keep
    // up with incoming data transmissions when receiving an acked
    // sequence of commands

    int cur_pri = getpriority(PRIO_PROCESS, _this_pid);

    if (msg_id == MAVLINK_MSG_ID_LOGGING_DATA_ACKED) {

        if (cur_pri > -20) {
            _initial_nice_val = cur_pri;
            setpriority(PRIO_PROCESS, _this_pid, -20);
        }

        if (_timer_running == false) {
            _timer_running = true;

            // add a 5 sec timeout for resetting the nice level
            // in case it doesn't get reset automatically

            _priority_timeout = Mainloop::get_instance().add_timeout(5000,
                                                                     std::bind(&PriorityAck::_reset_nice_value, this),
                                                                     this);
        }

    } else if (cur_pri != _initial_nice_val) {
        setpriority(PRIO_PROCESS, _this_pid, _initial_nice_val);
    }
}
