#pragma once

#include <string>
#include <stdexcept>

template <class T>
class LazyProxy
{
    T *proxy;
    std::string name;
public:
    LazyProxy(const std::string &taskName) : proxy(nullptr), name(taskName)
    {
    };
    
    void init()
    {   
        if(!proxy)
            proxy = new T(name);
    }
    
    T& getProxy()
    {
        if(!proxy)
            throw std::runtime_error("Error, proxy for " + name + " has not been initialized");
        
        return *proxy;
    }
    
    T& operator* ()
    {
        if(!proxy)
            throw std::runtime_error("Error, proxy for " + name + " has not been initialized");
        
        return *proxy;
    }

    T* operator-> ()
    {
        if(!proxy)
            throw std::runtime_error("Error, proxy for " + name + " has not been initialized");
        
        return proxy;
    }
    
};