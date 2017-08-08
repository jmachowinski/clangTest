#pragma once

#include <state_machine/State.hpp>
#include "../startup/init/CrexMotionControl.hpp"
#include <lib_init/MLSProvider.hpp>

class ConnectMapper : public state_machine::State
{
protected:
    init::CrexMotionControl &motionControl;
    init::MLSPrecalculatedProvider &mapProvider;
public:
    ConnectMapper(State* success, State* failue, init::CrexMotionControl &motionControl, init::MLSPrecalculatedProvider &mapProvider);
    virtual void init();
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
};


