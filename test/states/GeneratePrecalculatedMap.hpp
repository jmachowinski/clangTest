#pragma once

#include <state_machine/State.hpp>
#include <memory>

namespace init {
    class MLSPrecalculatedProvider;
}

class GeneratePrecalculatedMap : public state_machine::State
{
public:
    GeneratePrecalculatedMap(State *successState, std::shared_ptr<init::MLSPrecalculatedProvider> mapper);
    virtual ~GeneratePrecalculatedMap();
    
    virtual void init();
    virtual void enter(const State *lastState1);
    virtual void executeFunction();
    virtual void exit();
    
protected:
    
    std::shared_ptr<init::MLSPrecalculatedProvider> mapper;
    
    bool trigger;
    bool genMap();
};

