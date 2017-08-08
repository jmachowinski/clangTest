#pragma once
#include <state_machine/State.hpp>
#include <base/Time.hpp>

class Sleep : public state_machine::State
{
public:
    Sleep(State* success);
    
    void setDuration(const base::Time &duration);
    
    virtual void init() {};
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
private:
    base::Time start;
    base::Time duration;
    base::Time end;
};
