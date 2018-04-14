#pragma once

#include <RC_Channel/AP_RCSwitch.h>
#include "Rover.h"
#include "mode.h"

class AP_RCSwitch_Rover : public AP_RCSwitch
{

public:

    // this must be implemented for the AP_Scheduler functor to work:
    void read_aux_all() override {
        AP_RCSwitch::read_aux_all();
    }

    void reset_control_switch();

protected:

    void init_aux_function(aux_func_t ch_option, aux_switch_pos_t) override;
    void do_aux_function(aux_func_t ch_option, aux_switch_pos_t) override;

    bool in_rc_failsafe() const override;

private:

    void do_aux_function_change_mode(Mode &mode,
                                     const aux_switch_pos_t ch_flag);
};
