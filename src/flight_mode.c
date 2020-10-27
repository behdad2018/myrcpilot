#include <flight_mode.h>

bool mode_needs_mocap(flight_mode_t mode){
    if(mode == AUTONOMOUS || mode == OPEN_LOOP_DESCENT)
    {
        return true;
    }

    return false;
}