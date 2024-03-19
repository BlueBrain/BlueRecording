/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#define _pval pval
// clang-format off
#include "md1redef.h"
#include "section_fwd.hpp"
#include "nrniv_mf.h"
#include "md2redef.h"
// clang-format on
#include "neuron/cache/mechanism_range.hpp"
static constexpr auto number_of_datum_variables = 2;
static constexpr auto number_of_floating_point_variables = 6;
namespace {
template <typename T>
using _nrn_mechanism_std_vector = std::vector<T>;
using _nrn_model_sorted_token = neuron::model_sorted_token;
using _nrn_mechanism_cache_range = neuron::cache::MechanismRange<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_mechanism_cache_instance = neuron::cache::MechanismInstance<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_non_owning_id_without_container = neuron::container::non_owning_identifier_without_container;
template <typename T>
using _nrn_mechanism_field = neuron::mechanism::field<T>;
template <typename... Args>
void _nrn_mechanism_register_data_fields(Args&&... args) {
  neuron::mechanism::register_data_fields(std::forward<Args>(args)...);
}
}
 
#if !NRNGPU
#undef exp
#define exp hoc_Exp
#endif
 
#define nrn_init _nrn_init__VecStim
#define _nrn_initial _nrn_initial__VecStim
#define nrn_cur _nrn_cur__VecStim
#define _nrn_current _nrn_current__VecStim
#define nrn_jacob _nrn_jacob__VecStim
#define nrn_state _nrn_state__VecStim
#define _net_receive _net_receive__VecStim 
#define play play__VecStim 
#define restartEvent restartEvent__VecStim 
 
#define _threadargscomma_ _ml, _iml, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ Memb_list* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _internalthreadargsprotocomma_ _nrn_mechanism_cache_range* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _ml, _iml, _ppvar, _thread, _nt
#define _threadargsproto_ Memb_list* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, NrnThread* _nt
#define _internalthreadargsproto_ _nrn_mechanism_cache_range* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 
#define t _nt->_t
#define dt _nt->_dt
#define ping _ml->template fpfield<0>(_iml)
#define ping_columnindex 0
#define index _ml->template fpfield<1>(_iml)
#define index_columnindex 1
#define etime _ml->template fpfield<2>(_iml)
#define etime_columnindex 2
#define space _ml->template fpfield<3>(_iml)
#define space_columnindex 3
#define v _ml->template fpfield<4>(_iml)
#define v_columnindex 4
#define _tsav _ml->template fpfield<5>(_iml)
#define _tsav_columnindex 5
#define _nd_area *_ml->dptr_field<0>(_iml)
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_element(void*);
 static double _hoc_play(void*);
 static double _hoc_restartEvent(void*);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {0, 0}
};
 static Member_func _member_func[] = {
 {"loc", _hoc_loc_pnt},
 {"has_loc", _hoc_has_loc},
 {"get_loc", _hoc_get_loc_pnt},
 {"element", _hoc_element},
 {"play", _hoc_play},
 {"restartEvent", _hoc_restartEvent},
 {0, 0}
};
#define element element_VecStim
 extern double element( _internalthreadargsproto_ );
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"ping", "ms"},
 {"etime", "ms"},
 {0, 0}
};
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void nrn_init(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_state(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"VecStim",
 "ping",
 0,
 "index",
 "etime",
 0,
 0,
 0};
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 6);
 	/*initialize range parameters*/
 	ping = 1;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 6);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
 
#define _tqitem &(_ppvar[2])
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _VecStim_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nullptr, nullptr, nullptr, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"ping"} /* 0 */,
                                       _nrn_mechanism_field<double>{"index"} /* 1 */,
                                       _nrn_mechanism_field<double>{"etime"} /* 2 */,
                                       _nrn_mechanism_field<double>{"space"} /* 3 */,
                                       _nrn_mechanism_field<double>{"v"} /* 4 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 5 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<void*>{"_tqitem", "netsend"} /* 2 */);
  hoc_register_prop_size(_mechtype, 6, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "netsend");
 add_nrn_artcell(_mechtype, 2);
 add_nrn_has_net_event(_mechtype);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 VecStim VecStim.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int play(_internalthreadargsproto_);
static int restartEvent(_internalthreadargsproto_);
 
/*VERBATIM*/
#ifdef STIM_DEBUG
# define debug_printf(...) printf(__VA_ARGS__)
#else
# define debug_printf(...)
#endif
#if defined(NRN_VERSION_GTEQ)
#if NRN_VERSION_GTEQ(9,0,0)
#define NRN_VERSION_GTEQ_9_0_0
#endif
#endif
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  Prop* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _nrn_mechanism_cache_instance _ml_real{_pnt->_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
   _thread = nullptr; _nt = (NrnThread*)_pnt->_vnt;   _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  if (_tsav > t){ hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = nullptr;}
 {
   if ( _lflag  == 1.0 ) {
     
/*VERBATIM*/
        debug_printf("[VecStim] net_event(): index=%d, etime=%g, t=%g\n", (int)index - 1, etime, t);
 net_event ( _pnt, t ) ;
     if ( element ( _threadargs_ ) > 0.0 ) {
       if ( etime < t ) {
         printf ( "[VecStim] WARNING: spike time (%g ms) before current time (%g ms)\n" , etime , t ) ;
         }
       else {
         artcell_net_send ( _tqitem, _args, _pnt, t +  etime - t , 1.0 ) ;
         }
       }
     }
   else if ( _lflag  == 2.0 ) {
     if ( index  == - 2.0 ) {
       printf ( "[VecStim] Detected new time vector.\n" ) ;
       restartEvent ( _threadargs_ ) ;
       }
     artcell_net_send ( _tqitem, _args, _pnt, t +  ping , 2.0 ) ;
     }
   } }
 
static int  restartEvent ( _internalthreadargsproto_ ) {
   index = 0.0 ;
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
    while (element(_threadargs_) && etime < t) {}  // Ignore past events
    if (index > 0) {
        // Invoke low-level artcell_net_send, since generic NMODL net_send is only
        // available in INITIAL and NET_RECEIVE blocks. It takes an ABSOLUTE time instead
        debug_printf("[VecStim] restartEvent(): index=%d, etime=%g, t=%g\n", (int)index - 1, etime, t);
        #if defined(NRN_VERSION_GTEQ_9_0_0)
        artcell_net_send(_tqitem, (double*)0, _ppvar[1].get<Point_process*>(), etime, 1.0);
        #else
        artcell_net_send(_tqitem, (double*)0, (Point_process*)_ppvar[1]._pvoid, etime, 1.0);
        #endif
    }
#endif
  return 0; }
 
static double _hoc_restartEvent(void* _vptr) {
 double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _nrn_mechanism_cache_instance _ml_real{_p};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_p);
  _thread = _extcall_thread.data();
  _nt = static_cast<NrnThread*>(_pnt->_vnt);
 _r = 1.;
 restartEvent ( _threadargs_ );
 return(_r);
}
 
/*VERBATIM*/
#ifndef NRN_VERSION_GTEQ_8_2_0
extern double* vector_vec();
extern int vector_capacity();
extern void* vector_arg();
#endif
 
double element ( _internalthreadargsproto_ ) {
   double _lelement;
 
/*VERBATIM*/
    const int i = (int)index;
    IvocVect* const vv = *((IvocVect**)(&space));
    int size; double* px;
    if (i < 0 || vv == NULL)
        return 0;

    size = vector_capacity(vv);
    px = vector_vec(vv);
    if (i < size) {
        etime = px[i];
        index += 1.;
        debug_printf("[VecStim] element(): index=%d, etime=%g, t=%g\n", (int)index - 1, etime, t);
        return index;
    }
    index = -1;
    return 0;
 
return _lelement;
 }
 
static double _hoc_element(void* _vptr) {
 double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _nrn_mechanism_cache_instance _ml_real{_p};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_p);
  _thread = _extcall_thread.data();
  _nt = static_cast<NrnThread*>(_pnt->_vnt);
 _r =  element ( _threadargs_ );
 return(_r);
}
 
static int  play ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
    #ifndef CORENEURON_BUILD
    void** vv;
    vv = (void**)(&space);
    *vv = NULL;
    if (ifarg(1)) {
        *vv = vector_arg(1);
    }
    index = -2;
    #endif
  return 0; }
 
static double _hoc_play(void* _vptr) {
 double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _nrn_mechanism_cache_instance _ml_real{_p};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_p);
  _thread = _extcall_thread.data();
  _nt = static_cast<NrnThread*>(_pnt->_vnt);
 _r = 1.;
 play ( _threadargs_ );
 return(_r);
}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
   
/*VERBATIM*/
 #ifndef CORENEURON_BUILD
 // This Mechanism is not useful for CoreNeuron, since it has its own implementation
 // Therefore we should avoid even compiling it together, but for backwards compat keep guards
 index = 0.0 ;
   if ( element ( _threadargs_ ) ) {
     artcell_net_send ( _tqitem, nullptr, _ppvar[1].get<Point_process*>(), t +  etime - t , 1.0 ) ;
     }
   if ( ping > 0.0 ) {
     artcell_net_send ( _tqitem, nullptr, _ppvar[1].get<Point_process*>(), t +  ping , 2.0 ) ;
     }
   
/*VERBATIM*/
 #endif
 }

}
}

static void nrn_init(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _tsav = -1e20;
 initmodel(_threadargs_);
}
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{
} return _current;
}

static void nrn_state(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni;
_ni = _ml_arg->_nodeindices;
size_t _cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
for (size_t _iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _nd = _ml_arg->_nodelist[_iml];
 v=_v;
{
}}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "VecStim.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "/**\n"
  " * @file VecStim.mod\n"
  " * @brief\n"
  " * @author king\n"
  " * @date 2011-03-16\n"
  " * @remark Copyright \n"
  "\n"
  " BBP/EPFL 2005-2011; All rights reserved. Do not distribute without further notice.\n"
  " */\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  ": Vector stream of events\n"
  "NEURON {\n"
  "    THREADSAFE\n"
  "    ARTIFICIAL_CELL VecStim\n"
  "    RANGE ping, index, etime\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    ping = 1 (ms)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    index  : The index(+1) of the last retrieved element. See element()\n"
  "    etime (ms)\n"
  "    space\n"
  "}\n"
  "\n"
  "\n"
  "VERBATIM\n"
  "#ifdef STIM_DEBUG\n"
  "# define debug_printf(...) printf(__VA_ARGS__)\n"
  "#else\n"
  "# define debug_printf(...)\n"
  "#endif\n"
  "#if defined(NRN_VERSION_GTEQ)\n"
  "#if NRN_VERSION_GTEQ(9,0,0)\n"
  "#define NRN_VERSION_GTEQ_9_0_0\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "\n"
  "\n"
  "INITIAL {\n"
  "VERBATIM\n"
  " #ifndef CORENEURON_BUILD\n"
  " // This Mechanism is not useful for CoreNeuron, since it has its own implementation\n"
  " // Therefore we should avoid even compiling it together, but for backwards compat keep guards\n"
  "ENDVERBATIM\n"
  "    index = 0\n"
  "    if(element()) {\n"
  "        net_send(etime - t, 1)\n"
  "    }\n"
  "    if (ping > 0) {\n"
  "        net_send(ping, 2)\n"
  "    }\n"
  "VERBATIM\n"
  " #endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "NET_RECEIVE (w) {\n"
  "    if (flag == 1) {  : deliver event\n"
  "    VERBATIM\n"
  "        debug_printf(\"[VecStim] net_event(): index=%d, etime=%g, t=%g\\n\", (int)index - 1, etime, t);\n"
  "    ENDVERBATIM\n"
  "        net_event(t)\n"
  "\n"
  "        : schedule next event\n"
  "        if (element() > 0) {\n"
  "            if (etime < t) {\n"
  "                printf(\"[VecStim] WARNING: spike time (%g ms) before current time (%g ms)\\n\",etime,t)\n"
  "            } else {\n"
  "                net_send(etime - t, 1)\n"
  "            }\n"
  "        }\n"
  "    } else if (flag == 2) { : ping - reset index to 0\n"
  "        :printf(\"flag=2, etime=%g, t=%g, ping=%g, index=%g\\n\",etime,t,ping,index)\n"
  "        if (index == -2) { : play() has been called\n"
  "            printf(\"[VecStim] Detected new time vector.\\n\")\n"
  "            restartEvent()\n"
  "        }\n"
  "        net_send(ping, 2)\n"
  "    }\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/**\n"
  " * Resume the event delivery loop for NEURON restore.\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE restartEvent() {\n"
  "    index = 0\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "    while (element(_threadargs_) && etime < t) {}  // Ignore past events\n"
  "    if (index > 0) {\n"
  "        // Invoke low-level artcell_net_send, since generic NMODL net_send is only\n"
  "        // available in INITIAL and NET_RECEIVE blocks. It takes an ABSOLUTE time instead\n"
  "        debug_printf(\"[VecStim] restartEvent(): index=%d, etime=%g, t=%g\\n\", (int)index - 1, etime, t);\n"
  "        #if defined(NRN_VERSION_GTEQ_9_0_0)\n"
  "        artcell_net_send(_tqitem, (double*)0, _ppvar[1].get<Point_process*>(), etime, 1.0);\n"
  "        #else\n"
  "        artcell_net_send(_tqitem, (double*)0, (Point_process*)_ppvar[1]._pvoid, etime, 1.0);\n"
  "        #endif\n"
  "    }\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "VERBATIM\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "extern double* vector_vec();\n"
  "extern int vector_capacity();\n"
  "extern void* vector_arg();\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/**\n"
  " * \\brief Retrieves an element (spike time) from the source vector, store in etime.\n"
  " *\n"
  " * \\return The index+1 of the element (=~ true). Otherwise 0 (not initialized or end)\n"
  " *\n"
  " * NOTE: For back-compat index is incremented *after* the element is retrieved, making\n"
  " *   it like a base-1 indexing scheme, or representing the next elements index.\n"
  " */\n"
  "ENDCOMMENT\n"
  "FUNCTION element() {\n"
  "VERBATIM\n"
  "    const int i = (int)index;\n"
  "    IvocVect* const vv = *((IvocVect**)(&space));\n"
  "    int size; double* px;\n"
  "    if (i < 0 || vv == NULL)\n"
  "        return 0;\n"
  "\n"
  "    size = vector_capacity(vv);\n"
  "    px = vector_vec(vv);\n"
  "    if (i < size) {\n"
  "        etime = px[i];\n"
  "        index += 1.;\n"
  "        debug_printf(\"[VecStim] element(): index=%d, etime=%g, t=%g\\n\", (int)index - 1, etime, t);\n"
  "        return index;\n"
  "    }\n"
  "    index = -1;\n"
  "    return 0;\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "PROCEDURE play() {\n"
  "VERBATIM\n"
  "    #ifndef CORENEURON_BUILD\n"
  "    void** vv;\n"
  "    vv = (void**)(&space);\n"
  "    *vv = NULL;\n"
  "    if (ifarg(1)) {\n"
  "        *vv = vector_arg(1);\n"
  "    }\n"
  "    index = -2;\n"
  "    #endif\n"
  "ENDVERBATIM\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
