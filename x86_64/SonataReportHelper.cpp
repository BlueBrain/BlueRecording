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
static constexpr auto number_of_floating_point_variables = 4;
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
 
#define nrn_init _nrn_init__SonataReportHelper
#define _nrn_initial _nrn_initial__SonataReportHelper
#define nrn_cur _nrn_cur__SonataReportHelper
#define _nrn_current _nrn_current__SonataReportHelper
#define nrn_jacob _nrn_jacob__SonataReportHelper
#define nrn_state _nrn_state__SonataReportHelper
#define _net_receive _net_receive__SonataReportHelper 
#define add_spikes_population add_spikes_population__SonataReportHelper 
#define close_spikefile close_spikefile__SonataReportHelper 
#define create_spikefile create_spikefile__SonataReportHelper 
#define clear clear__SonataReportHelper 
#define disable_auto_flush disable_auto_flush__SonataReportHelper 
#define flush flush__SonataReportHelper 
#define make_comm make_comm__SonataReportHelper 
#define pre_savestate pre_savestate__SonataReportHelper 
#define prepare_datasets prepare_datasets__SonataReportHelper 
#define restorestate restorestate__SonataReportHelper 
#define restoretime restoretime__SonataReportHelper 
#define savestate savestate__SonataReportHelper 
#define set_max_buffer_size_hint set_max_buffer_size_hint__SonataReportHelper 
#define set_steps_to_buffer set_steps_to_buffer__SonataReportHelper 
#define write_spikes write_spikes__SonataReportHelper 
#define write_spike_populations write_spike_populations__SonataReportHelper 
 
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
#define activeStep _ml->template fpfield<0>(_iml)
#define activeStep_columnindex 0
#define initialStep _ml->template fpfield<1>(_iml)
#define initialStep_columnindex 1
#define v _ml->template fpfield<2>(_iml)
#define v_columnindex 2
#define _tsav _ml->template fpfield<3>(_iml)
#define _tsav_columnindex 3
#define _nd_area *_ml->dptr_field<0>(_iml)
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_add_spikes_population(void*);
 static double _hoc_close_spikefile(void*);
 static double _hoc_create_spikefile(void*);
 static double _hoc_clear(void*);
 static double _hoc_disable_auto_flush(void*);
 static double _hoc_flush(void*);
 static double _hoc_make_comm(void*);
 static double _hoc_pre_savestate(void*);
 static double _hoc_prepare_datasets(void*);
 static double _hoc_redirect(void*);
 static double _hoc_restorestate(void*);
 static double _hoc_restoretime(void*);
 static double _hoc_savestate(void*);
 static double _hoc_set_max_buffer_size_hint(void*);
 static double _hoc_set_steps_to_buffer(void*);
 static double _hoc_write_spikes(void*);
 static double _hoc_write_spike_populations(void*);
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
 {"add_spikes_population", _hoc_add_spikes_population},
 {"close_spikefile", _hoc_close_spikefile},
 {"create_spikefile", _hoc_create_spikefile},
 {"clear", _hoc_clear},
 {"disable_auto_flush", _hoc_disable_auto_flush},
 {"flush", _hoc_flush},
 {"make_comm", _hoc_make_comm},
 {"pre_savestate", _hoc_pre_savestate},
 {"prepare_datasets", _hoc_prepare_datasets},
 {"redirect", _hoc_redirect},
 {"restorestate", _hoc_restorestate},
 {"restoretime", _hoc_restoretime},
 {"savestate", _hoc_savestate},
 {"set_max_buffer_size_hint", _hoc_set_max_buffer_size_hint},
 {"set_steps_to_buffer", _hoc_set_steps_to_buffer},
 {"write_spikes", _hoc_write_spikes},
 {"write_spike_populations", _hoc_write_spike_populations},
 {0, 0}
};
#define redirect redirect_SonataReportHelper
 extern double redirect( _internalthreadargsproto_ );
 /* declare global and static user variables */
#define Dt Dt_SonataReportHelper
 double Dt = 0.1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"Dt_SonataReportHelper", "ms"},
 {0, 0}
};
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"Dt_SonataReportHelper", &Dt_SonataReportHelper},
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
 static void _constructor(Prop*);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"SonataReportHelper",
 "activeStep",
 "initialStep",
 0,
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
    assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	/*initialize range parameters*/
 	activeStep = 0;
 	initialStep = 0;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 if (!nrn_point_prop_) {_constructor(_prop);}
 
}
 static void _initlists();
 
#define _tqitem &(_ppvar[2])
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _SonataReportHelper_reg() {
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
                                       _nrn_mechanism_field<double>{"activeStep"} /* 0 */,
                                       _nrn_mechanism_field<double>{"initialStep"} /* 1 */,
                                       _nrn_mechanism_field<double>{"v"} /* 2 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<void*>{"_tqitem", "netsend"} /* 2 */);
  hoc_register_prop_size(_mechtype, 4, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "netsend");
 add_nrn_artcell(_mechtype, 2);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 SonataReportHelper SonataReportHelper.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int add_spikes_population(_internalthreadargsproto_);
static int close_spikefile(_internalthreadargsproto_);
static int create_spikefile(_internalthreadargsproto_);
static int clear(_internalthreadargsproto_);
static int disable_auto_flush(_internalthreadargsproto_);
static int flush(_internalthreadargsproto_);
static int make_comm(_internalthreadargsproto_);
static int pre_savestate(_internalthreadargsproto_);
static int prepare_datasets(_internalthreadargsproto_);
static int restorestate(_internalthreadargsproto_);
static int restoretime(_internalthreadargsproto_);
static int savestate(_internalthreadargsproto_);
static int set_max_buffer_size_hint(_internalthreadargsproto_);
static int set_steps_to_buffer(_internalthreadargsproto_);
static int write_spikes(_internalthreadargsproto_);
static int write_spike_populations(_internalthreadargsproto_);
 
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
#include <stdint.h>
#include <bbp/sonata/reports.h>
#include <mpi.h>
#ifndef NRN_VERSION_GTEQ_8_2_0
    extern int ifarg(int iarg);
    extern double* getarg(int iarg);
    extern double* vector_vec();
    extern int vector_capacity();
    extern void* vector_arg(int);
    extern void nrn_register_recalc_ptr_callback(void (*f)(void));
    extern double* nrn_recalc_ptr(double*);
#endif
#ifndef NRN_MECHANISM_DATA_IS_SOA
    void sonataRefreshPointers() { //callback function to update data locations before runtime
        sonata_refresh_pointers(nrn_recalc_ptr); //tell bin report library to update its pointers using nrn_recalc_ptr function
    }
#endif
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
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    sonata_record_data(activeStep);
    activeStep++;
#endif
#endif
 artcell_net_send ( _tqitem, _args, _pnt, t +  Dt , 1.0 ) ;
   } }
 
static int  make_comm ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    sonata_setup_communicators();
#endif
#endif
}
  return 0; }
 
static double _hoc_make_comm(void* _vptr) {
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
 make_comm ( _threadargs_ );
 return(_r);
}
 
static int  prepare_datasets ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    sonata_prepare_datasets();
#endif
#endif
}
  return 0; }
 
static double _hoc_prepare_datasets(void* _vptr) {
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
 prepare_datasets ( _threadargs_ );
 return(_r);
}
 
static int  disable_auto_flush ( _internalthreadargsproto_ ) {
    return 0; }
 
static double _hoc_disable_auto_flush(void* _vptr) {
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
 disable_auto_flush ( _threadargs_ );
 return(_r);
}
 
static int  set_steps_to_buffer ( _internalthreadargsproto_ ) {
    return 0; }
 
static double _hoc_set_steps_to_buffer(void* _vptr) {
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
 set_steps_to_buffer ( _threadargs_ );
 return(_r);
}
 
static int  set_max_buffer_size_hint ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    int buffer_size = (int) *getarg(1);
    sonata_set_max_buffer_size_hint(buffer_size);
#endif
#endif
  return 0; }
 
static double _hoc_set_max_buffer_size_hint(void* _vptr) {
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
 set_max_buffer_size_hint ( _threadargs_ );
 return(_r);
}
 
static int  flush ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    // Note: flush uses actual time (t) whereas recData uses timestep.  Should try to only use one or the other in the future
    sonata_flush( t );
#endif
#endif
  return 0; }
 
static double _hoc_flush(void* _vptr) {
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
 flush ( _threadargs_ );
 return(_r);
}
 
static int  pre_savestate ( _internalthreadargsproto_ ) {
    return 0; }
 
static double _hoc_pre_savestate(void* _vptr) {
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
 pre_savestate ( _threadargs_ );
 return(_r);
}
 
static int  savestate ( _internalthreadargsproto_ ) {
    return 0; }
 
static double _hoc_savestate(void* _vptr) {
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
 savestate ( _threadargs_ );
 return(_r);
}
 
static int  restoretime ( _internalthreadargsproto_ ) {
   initialStep = t / Dt ;
    return 0; }
 
static double _hoc_restoretime(void* _vptr) {
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
 restoretime ( _threadargs_ );
 return(_r);
}
 
static int  restorestate ( _internalthreadargsproto_ ) {
   activeStep = t / Dt ;
    return 0; }
 
static double _hoc_restorestate(void* _vptr) {
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
 restorestate ( _threadargs_ );
 return(_r);
}
 
double redirect ( _internalthreadargsproto_ ) {
   double _lredirect;
 
return _lredirect;
 }
 
static double _hoc_redirect(void* _vptr) {
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
 _r =  redirect ( _threadargs_ );
 return(_r);
}
 
static int  clear ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    sonata_clear();
#endif
#endif
  return 0; }
 
static double _hoc_clear(void* _vptr) {
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
 clear ( _threadargs_ );
 return(_r);
}
 
static int  create_spikefile ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    char output_dir[256] = ".";
    // output dir
    if (ifarg(1)) {
        sprintf(output_dir,"%s", gargstr(1));
    }
    char file_name[256] = "out";
    // file name
    if (ifarg(2)) {
        sprintf(file_name,"%s", gargstr(2));
    }
    sonata_create_spikefile(output_dir, file_name);
#endif
#endif
  return 0; }
 
static double _hoc_create_spikefile(void* _vptr) {
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
 create_spikefile ( _threadargs_ );
 return(_r);
}
 
static int  write_spike_populations ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    sonata_write_spike_populations();
#endif
#endif
  return 0; }
 
static double _hoc_write_spike_populations(void* _vptr) {
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
 write_spike_populations ( _threadargs_ );
 return(_r);
}
 
static int  close_spikefile ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    sonata_close_spikefile();
#endif
#endif
  return 0; }
 
static double _hoc_close_spikefile(void* _vptr) {
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
 close_spikefile ( _threadargs_ );
 return(_r);
}
 
static int  write_spikes ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB

    char output_dir[256] = ".";
    char population_name[256] = "All";
    char file_name[256] = "out";
    double *time = NULL, *gid = NULL;
    int num_spikes = 0;
    int num_gids = 0;
    IvocVect* v1;
    IvocVect* v2;

    // first vector is time of spikes
    if (ifarg(1)) {
        v1 = vector_arg(1);
        time = vector_vec(v1);
        num_spikes = vector_capacity(v1);
    }

    // second vector is associated gids
    if (ifarg(2)) {
        v2 = vector_arg(2);
        gid = vector_vec(v2);
        num_gids = vector_capacity(v2);
    }

    // output dir
    if (ifarg(3)) {
        sprintf(output_dir,"%s", gargstr(3));
    }

    if (ifarg(4)) {
        sprintf(population_name,"%s", gargstr(4));
    }

    int* int_gid = (int*)malloc(num_gids * sizeof(int));
    int i;
    for(i=0; i<num_spikes; ++i) {
        int_gid[i] = (int)gid[i];
    }
    sonata_create_spikefile(output_dir, file_name);
    sonata_add_spikes_population(population_name, 0, time, num_spikes, int_gid, num_gids);
    sonata_write_spike_populations();
    sonata_close_spikefile();
    free(int_gid);
#endif
#endif
  return 0; }
 
static double _hoc_write_spikes(void* _vptr) {
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
 write_spikes ( _threadargs_ );
 return(_r);
}
 
static int  add_spikes_population ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB

    char population_name[256] = "All";
    int population_offset = 0;
    double *time = NULL, *gid = NULL;
    int num_spikes = 0;
    int num_gids = 0;
    IvocVect* v1;
    IvocVect* v2;

    // first vector is time of spikes
    if (ifarg(1)) {
        v1 = vector_arg(1);
        time = vector_vec(v1);
        num_spikes = vector_capacity(v1);
    }

    // second vector is associated gids
    if (ifarg(2)) {
        v2 = vector_arg(2);
        gid = vector_vec(v2);
        num_gids = vector_capacity(v2);
    }

    if (ifarg(3)) {
        sprintf(population_name,"%s", gargstr(3));
    }

    if (ifarg(4)) {
        population_offset = (int) *getarg(4);
    }

    int* int_gid = (int*)malloc(num_gids * sizeof(int));
    int i;
    for(i=0; i<num_spikes; ++i) {
        int_gid[i] = (int)gid[i];
    }
    sonata_add_spikes_population(population_name, population_offset, time, num_spikes, int_gid, num_gids);
    free(int_gid);
#endif
#endif
  return 0; }
 
static double _hoc_add_spikes_population(void* _vptr) {
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
 add_spikes_population ( _threadargs_ );
 return(_r);
}
 
static void _constructor(Prop* _prop) {
  _nrn_mechanism_cache_instance _ml_real{_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  Datum *_ppvar{_nrn_mechanism_access_dparam(_prop)}, *_thread{};
  {
 {
   
/*VERBATIM*/
{
/**
* \param 1: Dt (double, optional). If not given no initializaton is performed
* \param 2: register_recalc_ptr (double, optional). By default will invoke
*    nrn_register_recalc_ptr_callback, which can be disabled by passing 0
*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    if( !ifarg(1) ) {
        return;
    }
    Dt = *getarg(1);
    sonata_set_atomic_step(Dt);
#ifndef NRN_MECHANISM_DATA_IS_SOA
    int register_recalc_ptr = 1;
    if( ifarg(2) ) {
        register_recalc_ptr = (int)*getarg(2);
    }
    if( register_recalc_ptr ) {
        nrn_register_recalc_ptr_callback( sonataRefreshPointers );
    }
#endif
#endif
#endif
}
 }
 
}
}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
   activeStep = initialStep ;
   artcell_net_send ( _tqitem, nullptr, _ppvar[1].get<Point_process*>(), t +  initialStep * Dt , 1.0 ) ;
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
    const char* nmodl_filename = "SonataReportHelper.mod";
    const char* nmodl_file_text = 
  "NEURON {\n"
  "        THREADSAFE\n"
  "        ARTIFICIAL_CELL SonataReportHelper\n"
  "        RANGE initialStep, activeStep\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "#include <stdint.h>\n"
  "#include <bbp/sonata/reports.h>\n"
  "#include <mpi.h>\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "    extern int ifarg(int iarg);\n"
  "    extern double* getarg(int iarg);\n"
  "    extern double* vector_vec();\n"
  "    extern int vector_capacity();\n"
  "    extern void* vector_arg(int);\n"
  "    extern void nrn_register_recalc_ptr_callback(void (*f)(void));\n"
  "    extern double* nrn_recalc_ptr(double*);\n"
  "#endif\n"
  "#ifndef NRN_MECHANISM_DATA_IS_SOA\n"
  "    void sonataRefreshPointers() { //callback function to update data locations before runtime\n"
  "        sonata_refresh_pointers(nrn_recalc_ptr); //tell bin report library to update its pointers using nrn_recalc_ptr function\n"
  "    }\n"
  "#endif\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "\n"
  "PARAMETER {\n"
  "    Dt = .1 (ms)\n"
  "    activeStep = 0\n"
  "    initialStep = 0\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    activeStep = initialStep\n"
  "    net_send(initialStep*Dt, 1)\n"
  "}\n"
  "\n"
  "\n"
  "NET_RECEIVE(w) {\n"
  "\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    sonata_record_data(activeStep);\n"
  "    activeStep++;\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "    net_send(Dt, 1)\n"
  "}\n"
  "\n"
  "CONSTRUCTOR  {\n"
  "VERBATIM {\n"
  "/**\n"
  "* \\param 1: Dt (double, optional). If not given no initializaton is performed\n"
  "* \\param 2: register_recalc_ptr (double, optional). By default will invoke\n"
  "*    nrn_register_recalc_ptr_callback, which can be disabled by passing 0\n"
  "*/\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    if( !ifarg(1) ) {\n"
  "        return;\n"
  "    }\n"
  "    Dt = *getarg(1);\n"
  "    sonata_set_atomic_step(Dt);\n"
  "#ifndef NRN_MECHANISM_DATA_IS_SOA\n"
  "    int register_recalc_ptr = 1;\n"
  "    if( ifarg(2) ) {\n"
  "        register_recalc_ptr = (int)*getarg(2);\n"
  "    }\n"
  "    if( register_recalc_ptr ) {\n"
  "        nrn_register_recalc_ptr_callback( sonataRefreshPointers );\n"
  "    }\n"
  "#endif\n"
  "#endif\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE make_comm() {\n"
  "VERBATIM\n"
  "{\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    sonata_setup_communicators();\n"
  "#endif\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE prepare_datasets() {\n"
  "VERBATIM\n"
  "{\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    sonata_prepare_datasets();\n"
  "#endif\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE disable_auto_flush() {\n"
  "}\n"
  "\n"
  "PROCEDURE set_steps_to_buffer() {\n"
  "}\n"
  "\n"
  "PROCEDURE set_max_buffer_size_hint() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    int buffer_size = (int) *getarg(1);\n"
  "    sonata_set_max_buffer_size_hint(buffer_size);\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE flush() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    // Note: flush uses actual time (t) whereas recData uses timestep.  Should try to only use one or the other in the future\n"
  "    sonata_flush( t );\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  ":Populate buffers from NEURON for savestate\n"
  ": @param SaveState object\n"
  "PROCEDURE pre_savestate() {\n"
  "}\n"
  "\n"
  ":Call ReportingLib for saving SaveState data using MPI I/O\n"
  "PROCEDURE savestate() {\n"
  "}\n"
  "\n"
  ": only restore global data for the purposes of getting the post retore time\n"
  "PROCEDURE restoretime() {\n"
  "    initialStep = t/Dt\n"
  "}\n"
  "\n"
  ": @param SaveState object\n"
  "PROCEDURE restorestate() {\n"
  "    activeStep = t/Dt\n"
  "}\n"
  "\n"
  "FUNCTION redirect() {\n"
  "}\n"
  "\n"
  "PROCEDURE clear() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    sonata_clear();\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE create_spikefile() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    char output_dir[256] = \".\";\n"
  "    // output dir\n"
  "    if (ifarg(1)) {\n"
  "        sprintf(output_dir,\"%s\", gargstr(1));\n"
  "    }\n"
  "    char file_name[256] = \"out\";\n"
  "    // file name\n"
  "    if (ifarg(2)) {\n"
  "        sprintf(file_name,\"%s\", gargstr(2));\n"
  "    }\n"
  "    sonata_create_spikefile(output_dir, file_name);\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE write_spike_populations() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    sonata_write_spike_populations();\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE close_spikefile() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    sonata_close_spikefile();\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE write_spikes() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "\n"
  "    char output_dir[256] = \".\";\n"
  "    char population_name[256] = \"All\";\n"
  "    char file_name[256] = \"out\";\n"
  "    double *time = NULL, *gid = NULL;\n"
  "    int num_spikes = 0;\n"
  "    int num_gids = 0;\n"
  "    IvocVect* v1;\n"
  "    IvocVect* v2;\n"
  "\n"
  "    // first vector is time of spikes\n"
  "    if (ifarg(1)) {\n"
  "        v1 = vector_arg(1);\n"
  "        time = vector_vec(v1);\n"
  "        num_spikes = vector_capacity(v1);\n"
  "    }\n"
  "\n"
  "    // second vector is associated gids\n"
  "    if (ifarg(2)) {\n"
  "        v2 = vector_arg(2);\n"
  "        gid = vector_vec(v2);\n"
  "        num_gids = vector_capacity(v2);\n"
  "    }\n"
  "\n"
  "    // output dir\n"
  "    if (ifarg(3)) {\n"
  "        sprintf(output_dir,\"%s\", gargstr(3));\n"
  "    }\n"
  "\n"
  "    if (ifarg(4)) {\n"
  "        sprintf(population_name,\"%s\", gargstr(4));\n"
  "    }\n"
  "\n"
  "    int* int_gid = (int*)malloc(num_gids * sizeof(int));\n"
  "    int i;\n"
  "    for(i=0; i<num_spikes; ++i) {\n"
  "        int_gid[i] = (int)gid[i];\n"
  "    }\n"
  "    sonata_create_spikefile(output_dir, file_name);\n"
  "    sonata_add_spikes_population(population_name, 0, time, num_spikes, int_gid, num_gids);\n"
  "    sonata_write_spike_populations();\n"
  "    sonata_close_spikefile();\n"
  "    free(int_gid);\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE add_spikes_population() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "\n"
  "    char population_name[256] = \"All\";\n"
  "    int population_offset = 0;\n"
  "    double *time = NULL, *gid = NULL;\n"
  "    int num_spikes = 0;\n"
  "    int num_gids = 0;\n"
  "    IvocVect* v1;\n"
  "    IvocVect* v2;\n"
  "\n"
  "    // first vector is time of spikes\n"
  "    if (ifarg(1)) {\n"
  "        v1 = vector_arg(1);\n"
  "        time = vector_vec(v1);\n"
  "        num_spikes = vector_capacity(v1);\n"
  "    }\n"
  "\n"
  "    // second vector is associated gids\n"
  "    if (ifarg(2)) {\n"
  "        v2 = vector_arg(2);\n"
  "        gid = vector_vec(v2);\n"
  "        num_gids = vector_capacity(v2);\n"
  "    }\n"
  "\n"
  "    if (ifarg(3)) {\n"
  "        sprintf(population_name,\"%s\", gargstr(3));\n"
  "    }\n"
  "\n"
  "    if (ifarg(4)) {\n"
  "        population_offset = (int) *getarg(4);\n"
  "    }\n"
  "\n"
  "    int* int_gid = (int*)malloc(num_gids * sizeof(int));\n"
  "    int i;\n"
  "    for(i=0; i<num_spikes; ++i) {\n"
  "        int_gid[i] = (int)gid[i];\n"
  "    }\n"
  "    sonata_add_spikes_population(population_name, population_offset, time, num_spikes, int_gid, num_gids);\n"
  "    free(int_gid);\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
