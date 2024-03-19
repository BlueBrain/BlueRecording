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
static constexpr auto number_of_datum_variables = 6;
static constexpr auto number_of_floating_point_variables = 10;
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
 
#define nrn_init _nrn_init__InhPoissonStim
#define _nrn_initial _nrn_initial__InhPoissonStim
#define nrn_cur _nrn_cur__InhPoissonStim
#define _nrn_current _nrn_current__InhPoissonStim
#define nrn_jacob _nrn_jacob__InhPoissonStim
#define nrn_state _nrn_state__InhPoissonStim
#define _net_receive _net_receive__InhPoissonStim 
#define generate_next_event generate_next_event__InhPoissonStim 
#define restartEvent restartEvent__InhPoissonStim 
#define setRate setRate__InhPoissonStim 
#define setTbins setTbins__InhPoissonStim 
#define setRNGs setRNGs__InhPoissonStim 
#define update_time update_time__InhPoissonStim 
 
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
#define duration _ml->template fpfield<0>(_iml)
#define duration_columnindex 0
#define rmax _ml->template fpfield<1>(_iml)
#define rmax_columnindex 1
#define index _ml->template fpfield<2>(_iml)
#define index_columnindex 2
#define curRate _ml->template fpfield<3>(_iml)
#define curRate_columnindex 3
#define start _ml->template fpfield<4>(_iml)
#define start_columnindex 4
#define event _ml->template fpfield<5>(_iml)
#define event_columnindex 5
#define usingR123 _ml->template fpfield<6>(_iml)
#define usingR123_columnindex 6
#define activeFlag _ml->template fpfield<7>(_iml)
#define activeFlag_columnindex 7
#define v _ml->template fpfield<8>(_iml)
#define v_columnindex 8
#define _tsav _ml->template fpfield<9>(_iml)
#define _tsav_columnindex 9
#define _nd_area *_ml->dptr_field<0>(_iml)
#define uniform_rng	*_ppvar[2].get<double*>()
#define _p_uniform_rng _ppvar[2].literal_value<void*>()
#define exp_rng	*_ppvar[3].get<double*>()
#define _p_exp_rng _ppvar[3].literal_value<void*>()
#define vecRate	*_ppvar[4].get<double*>()
#define _p_vecRate _ppvar[4].literal_value<void*>()
#define vecTbins	*_ppvar[5].get<double*>()
#define _p_vecTbins _ppvar[5].literal_value<void*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  2;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_erand(void*);
 static double _hoc_getPostRestoreFlag(void*);
 static double _hoc_generate_next_event(void*);
 static double _hoc_restartEvent(void*);
 static double _hoc_resumeEvent(void*);
 static double _hoc_setRate(void*);
 static double _hoc_setTbins(void*);
 static double _hoc_setRNGs(void*);
 static double _hoc_urand(void*);
 static double _hoc_update_time(void*);
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
 {"erand", _hoc_erand},
 {"getPostRestoreFlag", _hoc_getPostRestoreFlag},
 {"generate_next_event", _hoc_generate_next_event},
 {"restartEvent", _hoc_restartEvent},
 {"resumeEvent", _hoc_resumeEvent},
 {"setRate", _hoc_setRate},
 {"setTbins", _hoc_setTbins},
 {"setRNGs", _hoc_setRNGs},
 {"urand", _hoc_urand},
 {"update_time", _hoc_update_time},
 {0, 0}
};
#define erand erand_InhPoissonStim
#define getPostRestoreFlag getPostRestoreFlag_InhPoissonStim
#define resumeEvent resumeEvent_InhPoissonStim
#define urand urand_InhPoissonStim
 extern double erand( _internalthreadargsproto_ );
 extern double getPostRestoreFlag( _internalthreadargsproto_ );
 extern double resumeEvent( _internalthreadargsproto_ );
 extern double urand( _internalthreadargsproto_ );
 /* declare global and static user variables */
#define interval_min interval_min_InhPoissonStim
 double interval_min = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {"duration", 0, 1e+09},
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"duration", "ms"},
 {0, 0}
};
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"interval_min_InhPoissonStim", &interval_min_InhPoissonStim},
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
"InhPoissonStim",
 "duration",
 0,
 "rmax",
 0,
 0,
 "uniform_rng",
 "exp_rng",
 "vecRate",
 "vecTbins",
 0};
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 10);
 	/*initialize range parameters*/
 	duration = 1e+06;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 10);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
 
#define _tqitem &(_ppvar[6])
 static void _net_receive(Point_process*, double*, double);
 static void bbcore_write(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_write(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 static void bbcore_read(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_read(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _netstim_inhpoisson_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nullptr, nullptr, nullptr, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
   hoc_reg_bbcore_write(_mechtype, bbcore_write);
   hoc_reg_bbcore_read(_mechtype, bbcore_read);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"duration"} /* 0 */,
                                       _nrn_mechanism_field<double>{"rmax"} /* 1 */,
                                       _nrn_mechanism_field<double>{"index"} /* 2 */,
                                       _nrn_mechanism_field<double>{"curRate"} /* 3 */,
                                       _nrn_mechanism_field<double>{"start"} /* 4 */,
                                       _nrn_mechanism_field<double>{"event"} /* 5 */,
                                       _nrn_mechanism_field<double>{"usingR123"} /* 6 */,
                                       _nrn_mechanism_field<double>{"activeFlag"} /* 7 */,
                                       _nrn_mechanism_field<double>{"v"} /* 8 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 9 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"uniform_rng", "bbcorepointer"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"exp_rng", "bbcorepointer"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"vecRate", "bbcorepointer"} /* 4 */,
                                       _nrn_mechanism_field<double*>{"vecTbins", "bbcorepointer"} /* 5 */,
                                       _nrn_mechanism_field<void*>{"_tqitem", "netsend"} /* 6 */);
  hoc_register_prop_size(_mechtype, 10, 7);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 3, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 4, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 5, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 6, "netsend");
 add_nrn_artcell(_mechtype, 6);
 add_nrn_has_net_event(_mechtype);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 InhPoissonStim netstim_inhpoisson.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int generate_next_event(_internalthreadargsproto_);
static int restartEvent(_internalthreadargsproto_);
static int setRate(_internalthreadargsproto_);
static int setTbins(_internalthreadargsproto_);
static int setRNGs(_internalthreadargsproto_);
static int update_time(_internalthreadargsproto_);
 
/*VERBATIM*/
#if defined(NRN_VERSION_GTEQ)
#if NRN_VERSION_GTEQ(9,0,0)
#define NRN_VERSION_GTEQ_9_0_0
#endif
#endif

#ifndef NRN_VERSION_GTEQ_8_2_0
extern int ifarg(int iarg);
#ifndef CORENEURON_BUILD
extern double* vector_vec(void* vv);
extern void* vector_new1(int _i);
extern int vector_capacity(void* vv);
extern void* vector_arg(int iarg);
double nrn_random_pick(void* r);
#endif
void* nrn_random_arg(int argpos);
#define RANDCAST
#else
#define RANDCAST (Rand*)
#endif


#ifdef STIM_DEBUG
# define debug_printf(...) printf(__VA_ARGS__)
#else
# define debug_printf(...)
#endif

// constant used to indicate an event triggered after a restore to restart the main event loop
const int POST_RESTORE_RESTART_FLAG = -99;

 
/*VERBATIM*/
#include "nrnran123.h"
 
static int  generate_next_event ( _internalthreadargsproto_ ) {
   event = 1000.0 / rmax * erand ( _threadargs_ ) ;
   if ( event < 0.0 ) {
     event = 0.0 ;
     }
    return 0; }
 
static double _hoc_generate_next_event(void* _vptr) {
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
 generate_next_event ( _threadargs_ );
 return(_r);
}
 
static int  setRNGs ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
    if( ifarg(1) && hoc_is_double_arg(1) ) {
        nrnran123_State** pv = (nrnran123_State**)(&_p_exp_rng);

        if (*pv) {
            nrnran123_deletestream(*pv);
            *pv = (nrnran123_State*)0;
        }
        *pv = nrnran123_newstream3((uint32_t)*getarg(1), (uint32_t)*getarg(2), (uint32_t)*getarg(3));

        pv = (nrnran123_State**)(&_p_uniform_rng);
        if (*pv) {
            nrnran123_deletestream(*pv);
            *pv = (nrnran123_State*)0;
        }
        *pv = nrnran123_newstream3((uint32_t)*getarg(4), (uint32_t)*getarg(5), (uint32_t)*getarg(6));

        usingR123 = 1;
    } else if( ifarg(1) ) {
        void** pv = (void**)(&_p_exp_rng);
        *pv = nrn_random_arg(1);

        pv = (void**)(&_p_uniform_rng);
        *pv = nrn_random_arg(2);
        usingR123 = 0;
    } else {
        if( usingR123 ) {
            nrnran123_State** pv = (nrnran123_State**)(&_p_exp_rng);
            nrnran123_deletestream(*pv);
            *pv = (nrnran123_State*)0;
            pv = (nrnran123_State**)(&_p_uniform_rng);
            nrnran123_deletestream(*pv);
            *pv = (nrnran123_State*)0;
            //_p_exp_rng = (nrnran123_State*)0;
            //_p_uniform_rng = (nrnran123_State*)0;
        }
    }
#endif
}
  return 0; }
 
static double _hoc_setRNGs(void* _vptr) {
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
 setRNGs ( _threadargs_ );
 return(_r);
}
 
double urand ( _internalthreadargsproto_ ) {
   double _lurand;
 
/*VERBATIM*/
	if (_p_uniform_rng) {
		/*
		:Supports separate independent but reproducible streams for
		: each instance. However, the corresponding hoc Random
		: distribution MUST be set to Random.uniform(0,1)
		*/
            if( usingR123 ) {
		_lurand = nrnran123_dblpick((nrnran123_State*)_p_uniform_rng);
            } else {
#ifndef CORENEURON_BUILD
		_lurand = nrn_random_pick(RANDCAST _p_uniform_rng);
#endif
            }
	}else{
  	  hoc_execerror("multithread random in NetStim"," only via hoc Random");
	}
 
return _lurand;
 }
 
static double _hoc_urand(void* _vptr) {
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
 _r =  urand ( _threadargs_ );
 return(_r);
}
 
double erand ( _internalthreadargsproto_ ) {
   double _lerand;
 
/*VERBATIM*/
	if (_p_exp_rng) {
		/*
		:Supports separate independent but reproducible streams for
		: each instance. However, the corresponding hoc Random
		: distribution MUST be set to Random.negexp(1)
		*/
            if( usingR123 ) {
		_lerand = nrnran123_negexp((nrnran123_State*)_p_exp_rng);
            } else {
#ifndef CORENEURON_BUILD
		_lerand = nrn_random_pick(RANDCAST _p_exp_rng);
#endif
            }
	}else{
  	  hoc_execerror("multithread random in NetStim"," only via hoc Random");
	}
 
return _lerand;
 }
 
static double _hoc_erand(void* _vptr) {
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
 _r =  erand ( _threadargs_ );
 return(_r);
}
 
static int  setTbins ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
  #ifndef CORENEURON_BUILD
  IvocVect** vv;
  vv = (IvocVect**)(&_p_vecTbins);
  *vv = (IvocVect*)0;

  if (ifarg(1)) {
    *vv = vector_arg(1);

    /*int size = vector_capacity(*vv);
    int i;
    double* px = vector_vec(*vv);
    for (i=0;i<size;i++) {
      printf("%f ", px[i]);
    }*/
  }
  #endif
  return 0; }
 
static double _hoc_setTbins(void* _vptr) {
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
 setTbins ( _threadargs_ );
 return(_r);
}
 
static int  setRate ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
  #ifndef CORENEURON_BUILD

  IvocVect** vv;
  vv = (IvocVect**)(&_p_vecRate);
  *vv = (IvocVect*)0;

  if (ifarg(1)) {
    *vv = vector_arg(1);

    int size = vector_capacity(*vv);
    int i;
    double max=0.0;
    double* px = vector_vec(*vv);
    for (i=0;i<size;i++) {
    	if (px[i]>max) max = px[i];
    }

    curRate = px[0];
    rmax = max;

    activeFlag = activeFlag + 1;
  }
  #endif
  return 0; }
 
static double _hoc_setRate(void* _vptr) {
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
 setRate ( _threadargs_ );
 return(_r);
}
 
static int  update_time ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
  IvocVect* vv; int i, i_prev, size; double* px;
  i = (int)index;
  i_prev = i;

  if (i >= 0) { // are we disabled?
    vv = *((IvocVect**)(&_p_vecTbins));
    if (vv) {
      size = vector_capacity(vv);
      px = vector_vec(vv);
      /* advance to current tbins without exceeding array bounds */
      while ((i+1 < size) && (t>=px[i+1])) {
	index += 1.;
	i += 1;
      }
      /* did the index change? */
      if (i!=i_prev) {
        /* advance curRate to next vecRate if possible */
        IvocVect *vvRate = *((IvocVect**)(&_p_vecRate));
        if (vvRate && vector_capacity(vvRate)>i) {
          px = vector_vec(vvRate);
          curRate = px[i];
        }
        else curRate = 1.0;
      }

      /* have we hit last bin? ... disable time advancing leaving curRate as it is*/
      if (i==size)
        index = -1.;

    } else { /* no vecTbins, use some defaults */
      rmax = 1.0;
      curRate = 1.0;
      index = -1.; /* no vecTbins ... disable time advancing & Poisson unit rate. */
    }
  }

  return 0; }
 
static double _hoc_update_time(void* _vptr) {
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
 update_time ( _threadargs_ );
 return(_r);
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  Prop* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   _nrn_mechanism_cache_instance _ml_real{_pnt->_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
   _thread = nullptr; _nt = (NrnThread*)_pnt->_vnt;   _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  if (_tsav > t){ hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = nullptr;}
 {
   if ( _lflag  == POST_RESTORE_RESTART_FLAG ) {
     if ( t + event < start + duration ) {
       artcell_net_send ( _tqitem, _args, _pnt, t +  event , activeFlag ) ;
       }
     }
   else if ( activeFlag  == _lflag ) {
     update_time ( _threadargs_ ) ;
     generate_next_event ( _threadargs_ ) ;
     if ( t + event < start + duration ) {
       artcell_net_send ( _tqitem, _args, _pnt, t +  event , activeFlag ) ;
       }
     
/*VERBATIM*/
        double u = (double)urand(_threadargs_);
        //printf("InhPoisson: spike time at time %g urand=%g curRate=%g, rmax=%g, curRate/rmax=%g \n",t, u, curRate, rmax, curRate/rmax);
        if (u<curRate/rmax) {
            debug_printf("\nInhPoisson: Spike time t = %g [urand=%g curRate=%g, rmax=%g]\n",
                         t, u, curRate, rmax);
 net_event ( _pnt, t ) ;
     
/*VERBATIM*/
        }
 }
   } }
 
double getPostRestoreFlag ( _internalthreadargsproto_ ) {
   double _lgetPostRestoreFlag;
 
/*VERBATIM*/
    return POST_RESTORE_RESTART_FLAG;
 
return _lgetPostRestoreFlag;
 }
 
static double _hoc_getPostRestoreFlag(void* _vptr) {
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
 _r =  getPostRestoreFlag ( _threadargs_ );
 return(_r);
}
 
double resumeEvent ( _internalthreadargsproto_ ) {
   double _lresumeEvent;
 double _lelapsed_time ;
 _lelapsed_time = event ;
   while ( _lelapsed_time < t ) {
     update_time ( _threadargs_ ) ;
     generate_next_event ( _threadargs_ ) ;
     _lelapsed_time = _lelapsed_time + event ;
     }
   event = _lelapsed_time - t ;
   _lresumeEvent = _lelapsed_time ;
   
return _lresumeEvent;
 }
 
static double _hoc_resumeEvent(void* _vptr) {
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
 _r =  resumeEvent ( _threadargs_ );
 return(_r);
}
 
static int  restartEvent ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
    double etime = resumeEvent(_threadargs_);
    if (etime < start+duration) {
        debug_printf("InhPoisson: First event after resume at t = %6.3f\n", etime);
        #if defined(NRN_VERSION_GTEQ_9_0_0)
        artcell_net_send(_tqitem, (double*)0, _ppvar[1].get<Point_process*>(), etime, activeFlag);
        #else
        artcell_net_send(_tqitem, (double*)0, (Point_process*)_ppvar[1]._pvoid, etime, activeFlag);
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
static void bbcore_write(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
        uint32_t dsize = 0;
        if (_p_vecRate)
        {
          dsize = (uint32_t)vector_capacity((IvocVect*)_p_vecRate);
        }
        if (iArray) {
                uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
                nrnran123_State** pv = (nrnran123_State**)(&_p_exp_rng);
                nrnran123_getids3(*pv, ia, ia+1, ia+2);

                // for stream sequence
                char which;

                nrnran123_getseq(*pv, ia+3, &which);
                ia[4] = (int)which;

                ia = ia + 5;
                pv = (nrnran123_State**)(&_p_uniform_rng);
                nrnran123_getids3( *pv, ia, ia+1, ia+2);

                nrnran123_getseq(*pv, ia+3, &which);
                ia[4] = (int)which;

                ia = ia + 5;
                IvocVect* vec = (IvocVect*)_p_vecRate;
                ia[0] = dsize;

                double *da = dArray + *doffset;
                double *dv;
                if(dsize)
                {
                  dv = vector_vec(vec);
                }
                int iInt;
                for (iInt = 0; iInt < dsize; ++iInt)
                {
                  da[iInt] = dv[iInt];
                }

                vec = (IvocVect*)_p_vecTbins;
                da = dArray + *doffset + dsize;
                if(dsize)
                {
                  dv = vector_vec(vec);
                }
                for (iInt = 0; iInt < dsize; ++iInt)
                {
                  da[iInt] = dv[iInt];
                }
        }
        *ioffset += 11;
        *doffset += 2*dsize;

}

static void bbcore_read(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
        uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
        nrnran123_State** pv;
        if (ia[0] != 0 || ia[1] != 0)
        {
          pv = (nrnran123_State**)(&_p_exp_rng);
#if !NRNBBCORE
          if(*pv) {
              nrnran123_deletestream(*pv);
          }
#endif
          *pv = nrnran123_newstream3(ia[0], ia[1], ia[2] );
          nrnran123_setseq(*pv, ia[3], (char)ia[4]);
        }

        ia = ia + 5;
        if (ia[0] != 0 || ia[1] != 0)
        {
          pv = (nrnran123_State**)(&_p_uniform_rng);
#if !NRNBBCORE
          if(*pv) {
            nrnran123_deletestream(*pv);
          }
#endif
          *pv = nrnran123_newstream3(ia[0], ia[1], ia[2] );
          nrnran123_setseq(*pv, ia[2], (char)ia[3]);
        }

        ia = ia + 5;
        int dsize = ia[0];
        *ioffset += 11;

        double *da = dArray + *doffset;
        if(!_p_vecRate) {
          _p_vecRate = (double*)vector_new1(dsize);  /* works for dsize=0 */
        }
        assert(dsize == vector_capacity((IvocVect*)_p_vecRate));
        double *dv = vector_vec((IvocVect*)_p_vecRate);
        int iInt;
        for (iInt = 0; iInt < dsize; ++iInt)
        {
          dv[iInt] = da[iInt];
        }
        *doffset += dsize;

        da = dArray + *doffset;
        if(!_p_vecTbins) {
          _p_vecTbins = (double*)vector_new1(dsize);
        }
        assert(dsize == vector_capacity((IvocVect*)_p_vecTbins));
        dv = vector_vec((IvocVect*)_p_vecTbins);
        for (iInt = 0; iInt < dsize; ++iInt)
        {
          dv[iInt] = da[iInt];
        }
        *doffset += dsize;
}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
   index = 0. ;
   activeFlag = 0. ;
   
/*VERBATIM*/
   IvocVect *vvTbins = *((IvocVect**)(&_p_vecTbins));
   double* px;

   if (vvTbins && vector_capacity(vvTbins)>=1) {
     px = vector_vec(vvTbins);
     start = px[0];
     if (start < 0.0) start=0.0;
   }
   else start = 0.0;

   /* first event is at the start
   TODO: This should draw from a more appropriate dist
   that has the surrogate process starting a t=-inf
   */
   event = start;

   /* set curRate */
   IvocVect *vvRate = *((IvocVect**)(&_p_vecRate));
   px = vector_vec(vvRate);

   /* set rmax */
   rmax = 0.0;
   int i;
   for (i=0;i<vector_capacity(vvRate);i++) {
      if (px[i]>rmax) rmax = px[i];
   }

   if (vvRate && vector_capacity(vvRate)>0) {
     curRate = px[0];
   }
   else {
      curRate = 1.0;
      rmax = 1.0;
   }

   /** after discussion with michael : rng streams should be set 0
     * in initial block. this is to make sure if initial block is
     * get called multiple times then the simulation should give the
     * same results. Otherwise this is an issue in coreneuron because
     * finitialized is get called twice in coreneuron (once from
     * neurodamus and then in coreneuron. But in general, initial state
     * should be callable multiple times.
     */
   if (_p_uniform_rng && usingR123) {
     nrnran123_setseq((nrnran123_State*)_p_uniform_rng, 0, 0);
   }
   if (_p_exp_rng && usingR123) {
     nrnran123_setseq((nrnran123_State*)_p_exp_rng, 0, 0);
   }

 update_time ( _threadargs_ ) ;
   erand ( _threadargs_ ) ;
   generate_next_event ( _threadargs_ ) ;
   if ( t + event < start + duration ) {
     
/*VERBATIM*/
     debug_printf("InhPoisson: Initial event at t = %6.3f\n", t + event);
 artcell_net_send ( _tqitem, nullptr, _ppvar[1].get<Point_process*>(), t +  event , activeFlag ) ;
     }
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
    const char* nmodl_filename = "netstim_inhpoisson.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "/**\n"
  " * @file netstim_inhpoisson.mod\n"
  " * @brief Inhibitory poisson generator by the thinning method.\n"
  " * @author Eilif Muller\n"
  " * @date 2011-03-16\n"
  " * @remark Copyright \n"
  "\n"
  " BBP/EPFL 2005-2011; All rights reserved. Do not distribute without further notice.\n"
  " *  Based on vecstim.mod and netstim2.mod shipped with PyNN. See\n"
  " *   Muller, Buesing, Schemmel, Meier (2007). \"Spike-Frequency Adapting\n"
  " *   Neural Ensembles: Beyond Mean Adaptation and Renewal Theories\",\n"
  " *   Neural Computation 19:11, 2958-3010. doi:10.1162/neco.2007.19.11.2958\n"
  " */\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "THREADSAFE\n"
  "  ARTIFICIAL_CELL InhPoissonStim\n"
  "  RANGE rmax\n"
  "  RANGE duration\n"
  "  BBCOREPOINTER uniform_rng, exp_rng, vecRate, vecTbins\n"
  "  :THREADSAFE : only true if every instance has its own distinct Random\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "#if defined(NRN_VERSION_GTEQ)\n"
  "#if NRN_VERSION_GTEQ(9,0,0)\n"
  "#define NRN_VERSION_GTEQ_9_0_0\n"
  "#endif\n"
  "#endif\n"
  "\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "extern int ifarg(int iarg);\n"
  "#ifndef CORENEURON_BUILD\n"
  "extern double* vector_vec(void* vv);\n"
  "extern void* vector_new1(int _i);\n"
  "extern int vector_capacity(void* vv);\n"
  "extern void* vector_arg(int iarg);\n"
  "double nrn_random_pick(void* r);\n"
  "#endif\n"
  "void* nrn_random_arg(int argpos);\n"
  "#define RANDCAST\n"
  "#else\n"
  "#define RANDCAST (Rand*)\n"
  "#endif\n"
  "\n"
  "\n"
  "#ifdef STIM_DEBUG\n"
  "# define debug_printf(...) printf(__VA_ARGS__)\n"
  "#else\n"
  "# define debug_printf(...)\n"
  "#endif\n"
  "\n"
  "// constant used to indicate an event triggered after a restore to restart the main event loop\n"
  "const int POST_RESTORE_RESTART_FLAG = -99;\n"
  "\n"
  "ENDVERBATIM\n"
  "\n"
  "\n"
  "PARAMETER {\n"
  "  interval_min = 1.0  : average spike interval of surrogate Poisson process\n"
  "  duration	= 1e6 (ms) <0,1e9>   : duration of firing (msec)\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "#include \"nrnran123.h\"\n"
  "ENDVERBATIM\n"
  "\n"
  "ASSIGNED {\n"
  "   vecRate\n"
  "   vecTbins\n"
  "   index\n"
  "   curRate\n"
  "   start (ms)\n"
  "   event (ms)\n"
  "   uniform_rng\n"
  "   exp_rng\n"
  "   usingR123\n"
  "   rmax\n"
  "   activeFlag\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "   index = 0.\n"
  "   activeFlag = 0.\n"
  "\n"
  "   : determine start of spiking.\n"
  "   VERBATIM\n"
  "   IvocVect *vvTbins = *((IvocVect**)(&_p_vecTbins));\n"
  "   double* px;\n"
  "\n"
  "   if (vvTbins && vector_capacity(vvTbins)>=1) {\n"
  "     px = vector_vec(vvTbins);\n"
  "     start = px[0];\n"
  "     if (start < 0.0) start=0.0;\n"
  "   }\n"
  "   else start = 0.0;\n"
  "\n"
  "   /* first event is at the start\n"
  "   TODO: This should draw from a more appropriate dist\n"
  "   that has the surrogate process starting a t=-inf\n"
  "   */\n"
  "   event = start;\n"
  "\n"
  "   /* set curRate */\n"
  "   IvocVect *vvRate = *((IvocVect**)(&_p_vecRate));\n"
  "   px = vector_vec(vvRate);\n"
  "\n"
  "   /* set rmax */\n"
  "   rmax = 0.0;\n"
  "   int i;\n"
  "   for (i=0;i<vector_capacity(vvRate);i++) {\n"
  "      if (px[i]>rmax) rmax = px[i];\n"
  "   }\n"
  "\n"
  "   if (vvRate && vector_capacity(vvRate)>0) {\n"
  "     curRate = px[0];\n"
  "   }\n"
  "   else {\n"
  "      curRate = 1.0;\n"
  "      rmax = 1.0;\n"
  "   }\n"
  "\n"
  "   /** after discussion with michael : rng streams should be set 0\n"
  "     * in initial block. this is to make sure if initial block is\n"
  "     * get called multiple times then the simulation should give the\n"
  "     * same results. Otherwise this is an issue in coreneuron because\n"
  "     * finitialized is get called twice in coreneuron (once from\n"
  "     * neurodamus and then in coreneuron. But in general, initial state\n"
  "     * should be callable multiple times.\n"
  "     */\n"
  "   if (_p_uniform_rng && usingR123) {\n"
  "     nrnran123_setseq((nrnran123_State*)_p_uniform_rng, 0, 0);\n"
  "   }\n"
  "   if (_p_exp_rng && usingR123) {\n"
  "     nrnran123_setseq((nrnran123_State*)_p_exp_rng, 0, 0);\n"
  "   }\n"
  "\n"
  "   ENDVERBATIM\n"
  "   update_time()\n"
  "   erand() : for some reason, the first erand() call seems\n"
  "           : to give implausibly large values, so we discard it\n"
  "   generate_next_event()\n"
  "   : stop even producing surrogate events if we are past duration\n"
  "   if (t+event < start+duration) {\n"
  "VERBATIM\n"
  "     debug_printf(\"InhPoisson: Initial event at t = %6.3f\\n\", t + event);\n"
  "ENDVERBATIM\n"
  "     net_send(event, activeFlag )\n"
  "   }\n"
  "\n"
  "\n"
  "}\n"
  "\n"
  ": This procedure queues the next surrogate event in the\n"
  ": poisson process (rate=ramx) to be thinned.\n"
  "PROCEDURE generate_next_event() {\n"
  "	event = 1000.0/rmax*erand()\n"
  "	: but not earlier than 0\n"
  "	if (event < 0) {\n"
  "		event = 0\n"
  "	}\n"
  "}\n"
  "\n"
  ": Supports multiple rng types: mcellran4, random123\n"
  ": mcellran4:\n"
  ": 1st arg: exp_rng\n"
  ": 2nd arg: uniform_rng\n"
  ": random123\n"
  ": 3 exp seeds\n"
  ": 3 uniform seeds\n"
  "PROCEDURE setRNGs() {\n"
  "VERBATIM\n"
  "{\n"
  "#ifndef CORENEURON_BUILD\n"
  "    if( ifarg(1) && hoc_is_double_arg(1) ) {\n"
  "        nrnran123_State** pv = (nrnran123_State**)(&_p_exp_rng);\n"
  "\n"
  "        if (*pv) {\n"
  "            nrnran123_deletestream(*pv);\n"
  "            *pv = (nrnran123_State*)0;\n"
  "        }\n"
  "        *pv = nrnran123_newstream3((uint32_t)*getarg(1), (uint32_t)*getarg(2), (uint32_t)*getarg(3));\n"
  "\n"
  "        pv = (nrnran123_State**)(&_p_uniform_rng);\n"
  "        if (*pv) {\n"
  "            nrnran123_deletestream(*pv);\n"
  "            *pv = (nrnran123_State*)0;\n"
  "        }\n"
  "        *pv = nrnran123_newstream3((uint32_t)*getarg(4), (uint32_t)*getarg(5), (uint32_t)*getarg(6));\n"
  "\n"
  "        usingR123 = 1;\n"
  "    } else if( ifarg(1) ) {\n"
  "        void** pv = (void**)(&_p_exp_rng);\n"
  "        *pv = nrn_random_arg(1);\n"
  "\n"
  "        pv = (void**)(&_p_uniform_rng);\n"
  "        *pv = nrn_random_arg(2);\n"
  "        usingR123 = 0;\n"
  "    } else {\n"
  "        if( usingR123 ) {\n"
  "            nrnran123_State** pv = (nrnran123_State**)(&_p_exp_rng);\n"
  "            nrnran123_deletestream(*pv);\n"
  "            *pv = (nrnran123_State*)0;\n"
  "            pv = (nrnran123_State**)(&_p_uniform_rng);\n"
  "            nrnran123_deletestream(*pv);\n"
  "            *pv = (nrnran123_State*)0;\n"
  "            //_p_exp_rng = (nrnran123_State*)0;\n"
  "            //_p_uniform_rng = (nrnran123_State*)0;\n"
  "        }\n"
  "    }\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION urand() {\n"
  "VERBATIM\n"
  "	if (_p_uniform_rng) {\n"
  "		/*\n"
  "		:Supports separate independent but reproducible streams for\n"
  "		: each instance. However, the corresponding hoc Random\n"
  "		: distribution MUST be set to Random.uniform(0,1)\n"
  "		*/\n"
  "            if( usingR123 ) {\n"
  "		_lurand = nrnran123_dblpick((nrnran123_State*)_p_uniform_rng);\n"
  "            } else {\n"
  "#ifndef CORENEURON_BUILD\n"
  "		_lurand = nrn_random_pick(RANDCAST _p_uniform_rng);\n"
  "#endif\n"
  "            }\n"
  "	}else{\n"
  "  	  hoc_execerror(\"multithread random in NetStim\",\" only via hoc Random\");\n"
  "	}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "FUNCTION erand() {\n"
  "VERBATIM\n"
  "	if (_p_exp_rng) {\n"
  "		/*\n"
  "		:Supports separate independent but reproducible streams for\n"
  "		: each instance. However, the corresponding hoc Random\n"
  "		: distribution MUST be set to Random.negexp(1)\n"
  "		*/\n"
  "            if( usingR123 ) {\n"
  "		_lerand = nrnran123_negexp((nrnran123_State*)_p_exp_rng);\n"
  "            } else {\n"
  "#ifndef CORENEURON_BUILD\n"
  "		_lerand = nrn_random_pick(RANDCAST _p_exp_rng);\n"
  "#endif\n"
  "            }\n"
  "	}else{\n"
  "  	  hoc_execerror(\"multithread random in NetStim\",\" only via hoc Random\");\n"
  "	}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "\n"
  "\n"
  "PROCEDURE setTbins() {\n"
  "VERBATIM\n"
  "  #ifndef CORENEURON_BUILD\n"
  "  IvocVect** vv;\n"
  "  vv = (IvocVect**)(&_p_vecTbins);\n"
  "  *vv = (IvocVect*)0;\n"
  "\n"
  "  if (ifarg(1)) {\n"
  "    *vv = vector_arg(1);\n"
  "\n"
  "    /*int size = vector_capacity(*vv);\n"
  "    int i;\n"
  "    double* px = vector_vec(*vv);\n"
  "    for (i=0;i<size;i++) {\n"
  "      printf(\"%f \", px[i]);\n"
  "    }*/\n"
  "  }\n"
  "  #endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "PROCEDURE setRate() {\n"
  "VERBATIM\n"
  "  #ifndef CORENEURON_BUILD\n"
  "\n"
  "  IvocVect** vv;\n"
  "  vv = (IvocVect**)(&_p_vecRate);\n"
  "  *vv = (IvocVect*)0;\n"
  "\n"
  "  if (ifarg(1)) {\n"
  "    *vv = vector_arg(1);\n"
  "\n"
  "    int size = vector_capacity(*vv);\n"
  "    int i;\n"
  "    double max=0.0;\n"
  "    double* px = vector_vec(*vv);\n"
  "    for (i=0;i<size;i++) {\n"
  "    	if (px[i]>max) max = px[i];\n"
  "    }\n"
  "\n"
  "    curRate = px[0];\n"
  "    rmax = max;\n"
  "\n"
  "    activeFlag = activeFlag + 1;\n"
  "  }\n"
  "  #endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "PROCEDURE update_time() {\n"
  "VERBATIM\n"
  "  IvocVect* vv; int i, i_prev, size; double* px;\n"
  "  i = (int)index;\n"
  "  i_prev = i;\n"
  "\n"
  "  if (i >= 0) { // are we disabled?\n"
  "    vv = *((IvocVect**)(&_p_vecTbins));\n"
  "    if (vv) {\n"
  "      size = vector_capacity(vv);\n"
  "      px = vector_vec(vv);\n"
  "      /* advance to current tbins without exceeding array bounds */\n"
  "      while ((i+1 < size) && (t>=px[i+1])) {\n"
  "	index += 1.;\n"
  "	i += 1;\n"
  "      }\n"
  "      /* did the index change? */\n"
  "      if (i!=i_prev) {\n"
  "        /* advance curRate to next vecRate if possible */\n"
  "        IvocVect *vvRate = *((IvocVect**)(&_p_vecRate));\n"
  "        if (vvRate && vector_capacity(vvRate)>i) {\n"
  "          px = vector_vec(vvRate);\n"
  "          curRate = px[i];\n"
  "        }\n"
  "        else curRate = 1.0;\n"
  "      }\n"
  "\n"
  "      /* have we hit last bin? ... disable time advancing leaving curRate as it is*/\n"
  "      if (i==size)\n"
  "        index = -1.;\n"
  "\n"
  "    } else { /* no vecTbins, use some defaults */\n"
  "      rmax = 1.0;\n"
  "      curRate = 1.0;\n"
  "      index = -1.; /* no vecTbins ... disable time advancing & Poisson unit rate. */\n"
  "    }\n"
  "  }\n"
  "\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/**\n"
  " * Upon a net_receive, we do up to two things.  The first is to determine the next time this artificial cell triggers\n"
  " * and sending a self event.  Second, we check to see if the synapse coupled to this artificial cell should be activated.\n"
  " * This second task is not done if we have just completed a state restore and only wish to restart the self event triggers.\n"
  " *\n"
  " * @param flag >= 0 for Typical activation, POST_RESTORE_RESTART_FLAG for only restarting the self event triggers \n"
  " */\n"
  "ENDCOMMENT\n"
  "NET_RECEIVE (w) {\n"
  "    : Note - if we have restored a sim from a saved state.  We need to restart the queue, but do not generate a spike now\n"
  "    if ( flag == POST_RESTORE_RESTART_FLAG ) {\n"
  "        if (t+event < start+duration) {\n"
  "            net_send(event, activeFlag )\n"
  "        }\n"
  "    } else if( activeFlag == flag ) {\n"
  "        update_time()\n"
  "        generate_next_event()\n"
  "\n"
  "        : stop even producing surrogate events if we are past duration\n"
  "        if (t+event < start+duration) {\n"
  "            net_send(event, activeFlag )\n"
  "        }\n"
  "\n"
  "        : check if we trigger event on coupled synapse\n"
  "VERBATIM\n"
  "        double u = (double)urand(_threadargs_);\n"
  "        //printf(\"InhPoisson: spike time at time %g urand=%g curRate=%g, rmax=%g, curRate/rmax=%g \\n\",t, u, curRate, rmax, curRate/rmax);\n"
  "        if (u<curRate/rmax) {\n"
  "            debug_printf(\"\\nInhPoisson: Spike time t = %g [urand=%g curRate=%g, rmax=%g]\\n\",\n"
  "                         t, u, curRate, rmax);\n"
  "ENDVERBATIM\n"
  "            net_event(t)\n"
  "VERBATIM\n"
  "        }\n"
  "ENDVERBATIM\n"
  "    }\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/**\n"
  " * Supply the POST_RESTORE_RESTART_FLAG.  For example, so a hoc program can call a NetCon.event with the proper event value\n"
  " *\n"
  " * @DEPRECATED Consider using restartEvent() for resuming the event loop\n"
  " * @return POST_RESTORE_RESTART_FLAG value for entities that wish to use its value\n"
  " */\n"
  "ENDCOMMENT\n"
  "FUNCTION getPostRestoreFlag() {\n"
  "VERBATIM\n"
  "    return POST_RESTORE_RESTART_FLAG;\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/**\n"
  " * After a resume, send first event whose time is greater than the resume time.\n"
  " *\n"
  " * NOTE: Events generated right before the save time but scheduled for delivery afterwards\n"
  " *  will already be restored to the NetCon by the bbsavestate routines\n"
  " */\n"
  "ENDCOMMENT\n"
  "FUNCTION resumeEvent() {\n"
  "    LOCAL elapsed_time\n"
  "    : To be consistent with the previous run, it uses t=event as a starting point until it\n"
  "    : reaches an elapsed_time >= resume_t.\n"
  "    elapsed_time = event  : One event is always generated in the INITIAL block\n"
  "\n"
  "    while( elapsed_time < t ) {\n"
  "        update_time()\n"
  "        generate_next_event()\n"
  "        elapsed_time = elapsed_time + event\n"
  "    }\n"
  "    event = elapsed_time-t\n"
  "    resumeEvent = elapsed_time\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "/**\n"
  " * Restart the event loop after a NEURON restore. It will discard events in the past\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE restartEvent() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "    double etime = resumeEvent(_threadargs_);\n"
  "    if (etime < start+duration) {\n"
  "        debug_printf(\"InhPoisson: First event after resume at t = %6.3f\\n\", etime);\n"
  "        #if defined(NRN_VERSION_GTEQ_9_0_0)\n"
  "        artcell_net_send(_tqitem, (double*)0, _ppvar[1].get<Point_process*>(), etime, activeFlag);\n"
  "        #else\n"
  "        artcell_net_send(_tqitem, (double*)0, (Point_process*)_ppvar[1]._pvoid, etime, activeFlag);\n"
  "        #endif\n"
  "    }\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "VERBATIM\n"
  "static void bbcore_write(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {\n"
  "        uint32_t dsize = 0;\n"
  "        if (_p_vecRate)\n"
  "        {\n"
  "          dsize = (uint32_t)vector_capacity((IvocVect*)_p_vecRate);\n"
  "        }\n"
  "        if (iArray) {\n"
  "                uint32_t* ia = ((uint32_t*)iArray) + *ioffset;\n"
  "                nrnran123_State** pv = (nrnran123_State**)(&_p_exp_rng);\n"
  "                nrnran123_getids3(*pv, ia, ia+1, ia+2);\n"
  "\n"
  "                // for stream sequence\n"
  "                char which;\n"
  "\n"
  "                nrnran123_getseq(*pv, ia+3, &which);\n"
  "                ia[4] = (int)which;\n"
  "\n"
  "                ia = ia + 5;\n"
  "                pv = (nrnran123_State**)(&_p_uniform_rng);\n"
  "                nrnran123_getids3( *pv, ia, ia+1, ia+2);\n"
  "\n"
  "                nrnran123_getseq(*pv, ia+3, &which);\n"
  "                ia[4] = (int)which;\n"
  "\n"
  "                ia = ia + 5;\n"
  "                IvocVect* vec = (IvocVect*)_p_vecRate;\n"
  "                ia[0] = dsize;\n"
  "\n"
  "                double *da = dArray + *doffset;\n"
  "                double *dv;\n"
  "                if(dsize)\n"
  "                {\n"
  "                  dv = vector_vec(vec);\n"
  "                }\n"
  "                int iInt;\n"
  "                for (iInt = 0; iInt < dsize; ++iInt)\n"
  "                {\n"
  "                  da[iInt] = dv[iInt];\n"
  "                }\n"
  "\n"
  "                vec = (IvocVect*)_p_vecTbins;\n"
  "                da = dArray + *doffset + dsize;\n"
  "                if(dsize)\n"
  "                {\n"
  "                  dv = vector_vec(vec);\n"
  "                }\n"
  "                for (iInt = 0; iInt < dsize; ++iInt)\n"
  "                {\n"
  "                  da[iInt] = dv[iInt];\n"
  "                }\n"
  "        }\n"
  "        *ioffset += 11;\n"
  "        *doffset += 2*dsize;\n"
  "\n"
  "}\n"
  "\n"
  "static void bbcore_read(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {\n"
  "        uint32_t* ia = ((uint32_t*)iArray) + *ioffset;\n"
  "        nrnran123_State** pv;\n"
  "        if (ia[0] != 0 || ia[1] != 0)\n"
  "        {\n"
  "          pv = (nrnran123_State**)(&_p_exp_rng);\n"
  "#if !NRNBBCORE\n"
  "          if(*pv) {\n"
  "              nrnran123_deletestream(*pv);\n"
  "          }\n"
  "#endif\n"
  "          *pv = nrnran123_newstream3(ia[0], ia[1], ia[2] );\n"
  "          nrnran123_setseq(*pv, ia[3], (char)ia[4]);\n"
  "        }\n"
  "\n"
  "        ia = ia + 5;\n"
  "        if (ia[0] != 0 || ia[1] != 0)\n"
  "        {\n"
  "          pv = (nrnran123_State**)(&_p_uniform_rng);\n"
  "#if !NRNBBCORE\n"
  "          if(*pv) {\n"
  "            nrnran123_deletestream(*pv);\n"
  "          }\n"
  "#endif\n"
  "          *pv = nrnran123_newstream3(ia[0], ia[1], ia[2] );\n"
  "          nrnran123_setseq(*pv, ia[2], (char)ia[3]);\n"
  "        }\n"
  "\n"
  "        ia = ia + 5;\n"
  "        int dsize = ia[0];\n"
  "        *ioffset += 11;\n"
  "\n"
  "        double *da = dArray + *doffset;\n"
  "        if(!_p_vecRate) {\n"
  "          _p_vecRate = (double*)vector_new1(dsize);  /* works for dsize=0 */\n"
  "        }\n"
  "        assert(dsize == vector_capacity((IvocVect*)_p_vecRate));\n"
  "        double *dv = vector_vec((IvocVect*)_p_vecRate);\n"
  "        int iInt;\n"
  "        for (iInt = 0; iInt < dsize; ++iInt)\n"
  "        {\n"
  "          dv[iInt] = da[iInt];\n"
  "        }\n"
  "        *doffset += dsize;\n"
  "\n"
  "        da = dArray + *doffset;\n"
  "        if(!_p_vecTbins) {\n"
  "          _p_vecTbins = (double*)vector_new1(dsize);\n"
  "        }\n"
  "        assert(dsize == vector_capacity((IvocVect*)_p_vecTbins));\n"
  "        dv = vector_vec((IvocVect*)_p_vecTbins);\n"
  "        for (iInt = 0; iInt < dsize; ++iInt)\n"
  "        {\n"
  "          dv[iInt] = da[iInt];\n"
  "        }\n"
  "        *doffset += dsize;\n"
  "}\n"
  "ENDVERBATIM\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
