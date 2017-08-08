// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __ServiceRequester_hh__
#define __ServiceRequester_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
#endif



#ifndef __StdException_hh_EXTERNAL_GUARD__
#define __StdException_hh_EXTERNAL_GUARD__
#include "StdExceptionC.h"
#endif
#ifndef __ConfigurationInterface_hh_EXTERNAL_GUARD__
#define __ConfigurationInterface_hh_EXTERNAL_GUARD__
#include "ConfigurationInterfaceC.h"
#endif
#ifndef __OperationInterface_hh_EXTERNAL_GUARD__
#define __OperationInterface_hh_EXTERNAL_GUARD__
#include "OperationInterfaceC.h"
#endif
#ifndef __DataFlow_hh_EXTERNAL_GUARD__
#define __DataFlow_hh_EXTERNAL_GUARD__
#include "DataFlowC.h"
#endif
#ifndef __Service_hh_EXTERNAL_GUARD__
#define __Service_hh_EXTERNAL_GUARD__
#include "ServiceC.h"
#endif



#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





_CORBA_MODULE RTT

_CORBA_MODULE_BEG

  _CORBA_MODULE corba

  _CORBA_MODULE_BEG

    _CORBA_MODULE_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_CRequestNames;

    class CRequestNames_var;

    class CRequestNames : public _CORBA_Unbounded_Sequence_String {
    public:
      typedef CRequestNames_var _var_type;
      inline CRequestNames() {}
      inline CRequestNames(const CRequestNames& _s)
        : _CORBA_Unbounded_Sequence_String(_s) {}

      inline CRequestNames(_CORBA_ULong _max)
        : _CORBA_Unbounded_Sequence_String(_max) {}
      inline CRequestNames(_CORBA_ULong _max, _CORBA_ULong _len, char** _val, _CORBA_Boolean _rel=0)
        : _CORBA_Unbounded_Sequence_String(_max, _len, _val, _rel) {}

    

      inline CRequestNames& operator = (const CRequestNames& _s) {
        _CORBA_Unbounded_Sequence_String::operator=(_s);
        return *this;
      }
    };

    class CRequestNames_out;

    class CRequestNames_var {
    public:
      inline CRequestNames_var() : _pd_seq(0) {}
      inline CRequestNames_var(CRequestNames* _s) : _pd_seq(_s) {}
      inline CRequestNames_var(const CRequestNames_var& _s) {
        if( _s._pd_seq )  _pd_seq = new CRequestNames(*_s._pd_seq);
        else              _pd_seq = 0;
      }
      inline ~CRequestNames_var() { if( _pd_seq )  delete _pd_seq; }
        
      inline CRequestNames_var& operator = (CRequestNames* _s) {
        if( _pd_seq )  delete _pd_seq;
        _pd_seq = _s;
        return *this;
      }
      inline CRequestNames_var& operator = (const CRequestNames_var& _s) {
        if( _s._pd_seq ) {
          if( !_pd_seq )  _pd_seq = new CRequestNames;
          *_pd_seq = *_s._pd_seq;
        } else if( _pd_seq ) {
          delete _pd_seq;
          _pd_seq = 0;
        }
        return *this;
      }
      inline _CORBA_String_element operator [] (_CORBA_ULong _s) {
        return (*_pd_seq)[_s];
      }

    

      inline CRequestNames* operator -> () { return _pd_seq; }
      inline const CRequestNames* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
      inline operator CRequestNames& () const { return *_pd_seq; }
#else
      inline operator const CRequestNames& () const { return *_pd_seq; }
      inline operator CRequestNames& () { return *_pd_seq; }
#endif
        
      inline const CRequestNames& in() const { return *_pd_seq; }
      inline CRequestNames&       inout()    { return *_pd_seq; }
      inline CRequestNames*&      out() {
        if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
        return _pd_seq;
      }
      inline CRequestNames* _retn() { CRequestNames* tmp = _pd_seq; _pd_seq = 0; return tmp; }
        
      friend class CRequestNames_out;
      
    private:
      CRequestNames* _pd_seq;
    };

    class CRequestNames_out {
    public:
      inline CRequestNames_out(CRequestNames*& _s) : _data(_s) { _data = 0; }
      inline CRequestNames_out(CRequestNames_var& _s)
        : _data(_s._pd_seq) { _s = (CRequestNames*) 0; }
      inline CRequestNames_out(const CRequestNames_out& _s) : _data(_s._data) {}
      inline CRequestNames_out& operator = (const CRequestNames_out& _s) {
        _data = _s._data;
        return *this;
      }
      inline CRequestNames_out& operator = (CRequestNames* _s) {
        _data = _s;
        return *this;
      }
      inline operator CRequestNames*&()  { return _data; }
      inline CRequestNames*& ptr()       { return _data; }
      inline CRequestNames* operator->() { return _data; }

      inline _CORBA_String_element operator [] (_CORBA_ULong _i) {
        return (*_data)[_i];
      }

    

      CRequestNames*& _data;

    private:
      CRequestNames_out();
      CRequestNames_out& operator=(const CRequestNames_var&);
    };

    _CORBA_MODULE_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_COperationCallerNames;

    class COperationCallerNames_var;

    class COperationCallerNames : public _CORBA_Unbounded_Sequence_String {
    public:
      typedef COperationCallerNames_var _var_type;
      inline COperationCallerNames() {}
      inline COperationCallerNames(const COperationCallerNames& _s)
        : _CORBA_Unbounded_Sequence_String(_s) {}

      inline COperationCallerNames(_CORBA_ULong _max)
        : _CORBA_Unbounded_Sequence_String(_max) {}
      inline COperationCallerNames(_CORBA_ULong _max, _CORBA_ULong _len, char** _val, _CORBA_Boolean _rel=0)
        : _CORBA_Unbounded_Sequence_String(_max, _len, _val, _rel) {}

    

      inline COperationCallerNames& operator = (const COperationCallerNames& _s) {
        _CORBA_Unbounded_Sequence_String::operator=(_s);
        return *this;
      }
    };

    class COperationCallerNames_out;

    class COperationCallerNames_var {
    public:
      inline COperationCallerNames_var() : _pd_seq(0) {}
      inline COperationCallerNames_var(COperationCallerNames* _s) : _pd_seq(_s) {}
      inline COperationCallerNames_var(const COperationCallerNames_var& _s) {
        if( _s._pd_seq )  _pd_seq = new COperationCallerNames(*_s._pd_seq);
        else              _pd_seq = 0;
      }
      inline ~COperationCallerNames_var() { if( _pd_seq )  delete _pd_seq; }
        
      inline COperationCallerNames_var& operator = (COperationCallerNames* _s) {
        if( _pd_seq )  delete _pd_seq;
        _pd_seq = _s;
        return *this;
      }
      inline COperationCallerNames_var& operator = (const COperationCallerNames_var& _s) {
        if( _s._pd_seq ) {
          if( !_pd_seq )  _pd_seq = new COperationCallerNames;
          *_pd_seq = *_s._pd_seq;
        } else if( _pd_seq ) {
          delete _pd_seq;
          _pd_seq = 0;
        }
        return *this;
      }
      inline _CORBA_String_element operator [] (_CORBA_ULong _s) {
        return (*_pd_seq)[_s];
      }

    

      inline COperationCallerNames* operator -> () { return _pd_seq; }
      inline const COperationCallerNames* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
      inline operator COperationCallerNames& () const { return *_pd_seq; }
#else
      inline operator const COperationCallerNames& () const { return *_pd_seq; }
      inline operator COperationCallerNames& () { return *_pd_seq; }
#endif
        
      inline const COperationCallerNames& in() const { return *_pd_seq; }
      inline COperationCallerNames&       inout()    { return *_pd_seq; }
      inline COperationCallerNames*&      out() {
        if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
        return _pd_seq;
      }
      inline COperationCallerNames* _retn() { COperationCallerNames* tmp = _pd_seq; _pd_seq = 0; return tmp; }
        
      friend class COperationCallerNames_out;
      
    private:
      COperationCallerNames* _pd_seq;
    };

    class COperationCallerNames_out {
    public:
      inline COperationCallerNames_out(COperationCallerNames*& _s) : _data(_s) { _data = 0; }
      inline COperationCallerNames_out(COperationCallerNames_var& _s)
        : _data(_s._pd_seq) { _s = (COperationCallerNames*) 0; }
      inline COperationCallerNames_out(const COperationCallerNames_out& _s) : _data(_s._data) {}
      inline COperationCallerNames_out& operator = (const COperationCallerNames_out& _s) {
        _data = _s._data;
        return *this;
      }
      inline COperationCallerNames_out& operator = (COperationCallerNames* _s) {
        _data = _s;
        return *this;
      }
      inline operator COperationCallerNames*&()  { return _data; }
      inline COperationCallerNames*& ptr()       { return _data; }
      inline COperationCallerNames* operator->() { return _data; }

      inline _CORBA_String_element operator [] (_CORBA_ULong _i) {
        return (*_data)[_i];
      }

    

      COperationCallerNames*& _data;

    private:
      COperationCallerNames_out();
      COperationCallerNames_out& operator=(const COperationCallerNames_var&);
    };

#ifndef __RTT_mcorba_mCServiceRequester__
#define __RTT_mcorba_mCServiceRequester__

    class CServiceRequester;
    class _objref_CServiceRequester;
    class _impl_CServiceRequester;
    
    typedef _objref_CServiceRequester* CServiceRequester_ptr;
    typedef CServiceRequester_ptr CServiceRequesterRef;

    class CServiceRequester_Helper {
    public:
      typedef CServiceRequester_ptr _ptr_type;

      static _ptr_type _nil();
      static _CORBA_Boolean is_nil(_ptr_type);
      static void release(_ptr_type);
      static void duplicate(_ptr_type);
      static void marshalObjRef(_ptr_type, cdrStream&);
      static _ptr_type unmarshalObjRef(cdrStream&);
    };

    typedef _CORBA_ObjRef_Var<_objref_CServiceRequester, CServiceRequester_Helper> CServiceRequester_var;
    typedef _CORBA_ObjRef_OUT_arg<_objref_CServiceRequester,CServiceRequester_Helper > CServiceRequester_out;

#endif

    // interface CServiceRequester
    class CServiceRequester {
    public:
      // Declarations for this interface type.
      typedef CServiceRequester_ptr _ptr_type;
      typedef CServiceRequester_var _var_type;

      static _ptr_type _duplicate(_ptr_type);
      static _ptr_type _narrow(::CORBA::Object_ptr);
      static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
      
      static _ptr_type _nil();

      static inline void _marshalObjRef(_ptr_type, cdrStream&);

      static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
        omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
        if (o)
          return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
        else
          return _nil();
      }

      static _core_attr const char* _PD_repoId;

      // Other IDL defined within this scope.
      
    };

    class _objref_CServiceRequester :
      public virtual ::CORBA::Object,
      public virtual omniObjRef
    {
    public:
      char* getRequestName();
      CRequestNames* getRequestNames();
      COperationCallerNames* getOperationCallerNames();
      CServiceRequester_ptr getRequest(const char* name);
      ::CORBA::Boolean hasRequest(const char* name);
      ::CORBA::Boolean connectTo(::RTT::corba::CService_ptr svc);
      ::CORBA::Boolean connectCallerTo(const char* name, ::RTT::corba::CService_ptr svc);
      ::CORBA::Boolean ready();
      ::CORBA::Boolean disconnectCaller(const char* name);
      ::CORBA::Boolean callerReady(const char* name);
      void disconnect();

      inline _objref_CServiceRequester()  { _PR_setobj(0); }  // nil
      _objref_CServiceRequester(omniIOR*, omniIdentity*);

    protected:
      virtual ~_objref_CServiceRequester();

      
    private:
      virtual void* _ptrToObjRef(const char*);

      _objref_CServiceRequester(const _objref_CServiceRequester&);
      _objref_CServiceRequester& operator = (const _objref_CServiceRequester&);
      // not implemented

      friend class CServiceRequester;
    };

    class _pof_CServiceRequester : public _OMNI_NS(proxyObjectFactory) {
    public:
      inline _pof_CServiceRequester() : _OMNI_NS(proxyObjectFactory)(CServiceRequester::_PD_repoId) {}
      virtual ~_pof_CServiceRequester();

      virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
      virtual _CORBA_Boolean is_a(const char*) const;
    };

    class _impl_CServiceRequester :
      public virtual omniServant
    {
    public:
      virtual ~_impl_CServiceRequester();

      virtual char* getRequestName() = 0;
      virtual CRequestNames* getRequestNames() = 0;
      virtual COperationCallerNames* getOperationCallerNames() = 0;
      virtual CServiceRequester_ptr getRequest(const char* name) = 0;
      virtual ::CORBA::Boolean hasRequest(const char* name) = 0;
      virtual ::CORBA::Boolean connectTo(::RTT::corba::CService_ptr svc) = 0;
      virtual ::CORBA::Boolean connectCallerTo(const char* name, ::RTT::corba::CService_ptr svc) = 0;
      virtual ::CORBA::Boolean ready() = 0;
      virtual ::CORBA::Boolean disconnectCaller(const char* name) = 0;
      virtual ::CORBA::Boolean callerReady(const char* name) = 0;
      virtual void disconnect() = 0;
      
    public:  // Really protected, workaround for xlC
      virtual _CORBA_Boolean _dispatch(omniCallHandle&);

    private:
      virtual void* _ptrToInterface(const char*);
      virtual const char* _mostDerivedRepoId();
      
    };


    _CORBA_MODULE_VAR _dyn_attr const ::CORBA::TypeCode_ptr _tc_CServiceRequester;

  _CORBA_MODULE_END

_CORBA_MODULE_END



_CORBA_MODULE POA_RTT
_CORBA_MODULE_BEG

  _CORBA_MODULE corba
  _CORBA_MODULE_BEG

    class CServiceRequester :
      public virtual RTT::corba::_impl_CServiceRequester,
      public virtual ::PortableServer::ServantBase
    {
    public:
      virtual ~CServiceRequester();

      inline ::RTT::corba::CServiceRequester_ptr _this() {
        return (::RTT::corba::CServiceRequester_ptr) _do_this(::RTT::corba::CServiceRequester::_PD_repoId);
      }
    };

  _CORBA_MODULE_END

_CORBA_MODULE_END



_CORBA_MODULE OBV_RTT
_CORBA_MODULE_BEG

  _CORBA_MODULE corba
  _CORBA_MODULE_BEG

  _CORBA_MODULE_END

_CORBA_MODULE_END





#undef _core_attr
#undef _dyn_attr

void operator<<=(::CORBA::Any& _a, const RTT::corba::CRequestNames& _s);
void operator<<=(::CORBA::Any& _a, RTT::corba::CRequestNames* _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, RTT::corba::CRequestNames*& _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const RTT::corba::CRequestNames*& _sp);

void operator<<=(::CORBA::Any& _a, const RTT::corba::COperationCallerNames& _s);
void operator<<=(::CORBA::Any& _a, RTT::corba::COperationCallerNames* _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, RTT::corba::COperationCallerNames*& _sp);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, const RTT::corba::COperationCallerNames*& _sp);

void operator<<=(::CORBA::Any& _a, RTT::corba::CServiceRequester_ptr _s);
void operator<<=(::CORBA::Any& _a, RTT::corba::CServiceRequester_ptr* _s);
_CORBA_Boolean operator>>=(const ::CORBA::Any& _a, RTT::corba::CServiceRequester_ptr& _s);



inline void
RTT::corba::CServiceRequester::_marshalObjRef(::RTT::corba::CServiceRequester_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_ServiceRequester
#endif

#endif  // __ServiceRequester_hh__

