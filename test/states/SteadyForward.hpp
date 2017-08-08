#pragma once
#include <state_machine/State.hpp>
#include <lib_init/MotionControl2D.hpp>

class SteadyForward : public state_machine::State
{
    std::shared_ptr<init::MotionControl2D> control;
public:
    SteadyForward(std::shared_ptr<init::MotionControl2D> control, State* succ);
    virtual void init();
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
private:
};
