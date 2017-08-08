#pragma once

#include <state_machine/State.hpp>
#include <memory>

namespace init {
    class MLSProvider;
}

class GenerateMap : public state_machine::State
{
public:
    GenerateMap(State *successState, std::shared_ptr<init::MLSProvider> mapper);
    virtual ~GenerateMap();
    
    virtual void init();
    virtual void enter(const State *lastState1);
    virtual void executeFunction();
    virtual void exit();
    
protected:
    
    std::shared_ptr<init::MLSProvider> mapper;
    
    bool trigger;
    bool genMap();
};

