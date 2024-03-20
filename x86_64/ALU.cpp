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
static constexpr auto number_of_datum_variables = 3;
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
 
#define nrn_init _nrn_init__ALU
#define _nrn_initial _nrn_initial__ALU
#define nrn_cur _nrn_cur__ALU
#define _nrn_current _nrn_current__ALU
#define nrn_jacob _nrn_jacob__ALU
#define nrn_state _nrn_state__ALU
#define _net_receive _net_receive__ALU 
#define average average__ALU 
#define addvar addvar__ALU 
#define constant constant__ALU 
#define restartEvent restartEvent__ALU 
#define setop setop__ALU 
#define summation summation__ALU 
 
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
#define Dt _ml->template fpfield<0>(_iml)
#define Dt_columnindex 0
#define output _ml->template fpfield<1>(_iml)
#define output_columnindex 1
#define v _ml->template fpfield<2>(_iml)
#define v_columnindex 2
#define _tsav _ml->template fpfield<3>(_iml)
#define _tsav_columnindex 3
#define _nd_area *_ml->dptr_field<0>(_iml)
#define ptr	*_ppvar[2].get<double*>()
#define _p_ptr _ppvar[2].literal_value<void*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  2;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_average(void*);
 static double _hoc_addvar(void*);
 static double _hoc_constant(void*);
 static double _hoc_restartEvent(void*);
 static double _hoc_setop(void*);
 static double _hoc_summation(void*);
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
 {"average", _hoc_average},
 {"addvar", _hoc_addvar},
 {"constant", _hoc_constant},
 {"restartEvent", _hoc_restartEvent},
 {"setop", _hoc_setop},
 {"summation", _hoc_summation},
 {0, 0}
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"Dt", "ms"},
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
 static void _destructor(Prop*);
 static void _constructor(Prop*);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"ALU",
 "Dt",
 "output",
 0,
 0,
 0,
 "ptr",
 0};
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	/*initialize range parameters*/
 	Dt = 0.1;
 	output = 0;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 if (!nrn_point_prop_) {_constructor(_prop);}
 
}
 static void _initlists();
 
#define _tqitem &(_ppvar[3])
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

 extern "C" void _ALU_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nullptr, nullptr, nullptr, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 	register_destructor(_destructor);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
   hoc_reg_bbcore_write(_mechtype, bbcore_write);
   hoc_reg_bbcore_read(_mechtype, bbcore_read);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"Dt"} /* 0 */,
                                       _nrn_mechanism_field<double>{"output"} /* 1 */,
                                       _nrn_mechanism_field<double>{"v"} /* 2 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"ptr", "bbcorepointer"} /* 2 */,
                                       _nrn_mechanism_field<void*>{"_tqitem", "netsend"} /* 3 */);
  hoc_register_prop_size(_mechtype, 4, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 3, "netsend");
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 ALU ALU.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int average(_internalthreadargsproto_);
static int addvar(_internalthreadargsproto_);
static int constant(_internalthreadargsproto_);
static int restartEvent(_internalthreadargsproto_);
static int setop(_internalthreadargsproto_);
static int summation(_internalthreadargsproto_);
 
/*VERBATIM*/

#ifndef CORENEURON_BUILD
#if defined(NRN_VERSION_GTEQ)
#if NRN_VERSION_GTEQ(9,0,0)
#define NRN_VERSION_GTEQ_9_0_0
#endif
#endif
#ifndef NRN_VERSION_GTEQ_8_2_0
extern double* hoc_pgetarg(int iarg);
extern double* getarg(int iarg);
extern int ifarg(int iarg);
extern void nrn_register_recalc_ptr_callback(void (*f)());
extern Point_process* ob2pntproc(Object*);
extern double* nrn_recalc_ptr(double*);
#endif

#ifdef NRN_MECHANISM_DATA_IS_SOA
using handle_to_double = decltype(hoc_hgetarg<double>(42));
#else
typedef double* handle_to_double;
#endif
typedef struct {
    //! list of pointers to hoc variables
    handle_to_double* ptrs_;

    /*! list of scalars to apply to corresponding variables; useful for making units of variables
     * from different sources consistent (e.g. i current sources may be distributed, mA/cm^2, or point processes, nA)
     */
    double * scalars_;

    //! number of elements stored in the vectors
    int np_;

    //! number of slots allocated to the vectors
    int psize_;

    //! function pointer to execute when net_receive is triggered
#ifdef NRN_MECHANISM_DATA_IS_SOA
    int (*process)(_internalthreadargsproto_);
#else
    int (*process)(_threadargsproto_);
#endif
} Info;

#define INFOCAST Info** ip = (Info**)(&(_p_ptr))

#define dp double*

#ifndef NRN_MECHANISM_DATA_IS_SOA
static void recalcptr(Info* info, int cnt, double** old_vp, double* new_v) {
    int i;
    /*printf("recalcptr np_=%d %s\n", info->np_, info->path_);*/

}
static void recalc_ptr_callback() {
    Symbol* sym;
    int i;
    hoc_List* instances;
    hoc_Item* q;
    /*printf("ASCIIrecord.mod recalc_ptr_callback\n");*/
    /* hoc has a list of the ASCIIRecord instances */
    sym = hoc_lookup("ALU");
    #ifndef NRN_VERSION_GTEQ_8_2_0
    instances = sym->u.template->olist;
    #else
    instances = sym->u.ctemplate->olist;
    #endif
    ITERATE(q, instances) {
        Info* InfoPtr;
        Point_process* pnt;
        Object* o = OBJ(q);
        /*printf("callback for %s\n", hoc_object_name(o));*/
        pnt = ob2pntproc(o);
        Datum* _ppvar = pnt->_prop->dparam;
        INFOCAST;
        InfoPtr = *ip;
        for (i=0; i < InfoPtr->np_; ++i)
            InfoPtr->ptrs_[i] =  nrn_recalc_ptr(InfoPtr->ptrs_[i]);
    }
}
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
{
#ifndef CORENEURON_BUILD
    INFOCAST;
    Info* info = *ip;
    info->process(_threadargs_);
#endif
}
 net_send ( _tqitem, _args, _pnt, t +  Dt - 1e-5 , 1.0 ) ;
   } }
 
static int  restartEvent ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
    const double etime = *getarg(1);
    #if defined(NRN_VERSION_GTEQ_9_0_0)
    net_send(_tqitem, (double*)0, _ppvar[1].get<Point_process*>(), etime, 1.0);
    #else
    net_send(_tqitem, (double*)0, (Point_process*)_ppvar[1]._pvoid, etime, 1.0);
    #endif    
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
 
static int  addvar ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
    INFOCAST;
    Info* info = *ip;
    if (info->np_ >= info->psize_) {
        info->psize_ += 10;
#ifdef NRN_VERSION_GTEQ_9_0_0
        auto old_ptrs = info->ptrs_;
        info->ptrs_ = new handle_to_double[info->psize_]; hoc_malchk();
        std::copy(old_ptrs, old_ptrs+info->np_, info->ptrs_);
        delete[] old_ptrs;
#else
        info->ptrs_ = (handle_to_double*)hoc_Erealloc(info->ptrs_, info->psize_*sizeof(handle_to_double)); hoc_malchk();
#endif
        info->scalars_ = (double*) hoc_Erealloc(info->scalars_, info->psize_*sizeof(double)); hoc_malchk();
    }
#ifdef NRN_MECHANISM_DATA_IS_SOA
    handle_to_double var = hoc_hgetarg<double>(1);
#else
    handle_to_double var = hoc_pgetarg(1);
#endif
    info->ptrs_[info->np_] = var;
    if( ifarg(2)) {
        info->scalars_[info->np_] = *getarg(2);
    } else {
        info->scalars_[info->np_] = 1;
    }

    ++info->np_;
    //printf("I have %d values.. (new = %g * %g)\n", info->np_, *(info->ptrs_[info->np_-1]), info->scalars_[info->np_-1] );
#endif
}
  return 0; }
 
static double _hoc_addvar(void* _vptr) {
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
 addvar ( _threadargs_ );
 return(_r);
}
 
static int  constant ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
    INFOCAST;
    Info* info = *ip;
    if( info->np_ > 0 ) {
        output = info->scalars_[0];
    } else {
        output = 0;
    }
#endif
}
  return 0; }
 
static double _hoc_constant(void* _vptr) {
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
 constant ( _threadargs_ );
 return(_r);
}
 
static int  average ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
    INFOCAST;
    Info* info = *ip;
    int i;
    double n = 0;
    for (i=0; i < info->np_; ++i) {
      //  printf("%f", (*info->ptrs_[i] * info->scalars_[i]) );
        n += (*info->ptrs_[i] * info->scalars_[i]);
    }
    //printf("\n");
//    output = n/info->np_;
    if (info->np_ > 0)
      output = n/info->np_;
    else output = 0;
#endif
}
  return 0; }
 
static double _hoc_average(void* _vptr) {
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
 average ( _threadargs_ );
 return(_r);
}
 
static int  summation ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
    INFOCAST; Info* info = *ip;
    int i;
    double n = 0;
    for (i=0; i < info->np_; ++i) {
        //printf("%f = %f * %f\n", (*info->ptrs_[i] * info->scalars_[i]), *info->ptrs_[i], info->scalars_[i] );
        n += (*info->ptrs_[i] * info->scalars_[i]);
    }

    output = n;
#endif
}
  return 0; }
 
static double _hoc_summation(void* _vptr) {
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
 summation ( _threadargs_ );
 return(_r);
}
 
static int  setop ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
    INFOCAST; Info* info = *ip;

    char *opname = NULL;
    if (!hoc_is_str_arg(1)) {
        exit(0);
    }

    opname = gargstr(1);
    if( strcmp( opname, "summation" ) == 0 ) {
        info->process = &summation;
    } else if ( strcmp( opname, "average" ) == 0 ) {
        info->process = &average;
    } else if ( strcmp( opname, "constant" ) == 0 ) {
        info->process = &constant;
    } else {
        fprintf( stderr, "Error: unknown operation '%s' for ALU object.  Terminating.\n", opname );
        exit(0);
    }
#endif
}
  return 0; }
 
static double _hoc_setop(void* _vptr) {
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
 setop ( _threadargs_ );
 return(_r);
}
 
/*VERBATIM*/
/** not executed in coreneuron and hence need empty stubs only */
static void bbcore_write(double* x, int* d, int* xx, int* offset, _threadargsproto_) {
}
static void bbcore_read(double* x, int* d, int* xx, int* offset, _threadargsproto_) {
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
#ifndef CORENEURON_BUILD
#ifndef NRN_MECHANISM_DATA_IS_SOA
    static int first = 1;
    if (first) {
        first = 0;
        nrn_register_recalc_ptr_callback(recalc_ptr_callback);
    }
#endif

    INFOCAST;
    Info* info = (Info*)hoc_Emalloc(sizeof(Info)); hoc_malchk();
    info->psize_ = 10;
#ifdef NRN_VERSION_GTEQ_9_0_0
    info->ptrs_ = new handle_to_double[info->psize_]; hoc_malchk();
#else
    info->ptrs_ = (handle_to_double*)hoc_Ecalloc(info->psize_, sizeof(handle_to_double)); hoc_malchk();
#endif
    info->scalars_ = (double*)hoc_Ecalloc(info->psize_, sizeof(double)); hoc_malchk();
    info->np_ = 0;
    *ip = info;

    if (ifarg(2)) {
        Dt = *getarg(2);
    }

    //default operation is average
    info->process = &average;
#endif
}
 }
 
}
}
 
static void _destructor(Prop* _prop) {
  _nrn_mechanism_cache_instance _ml_real{_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  Datum *_ppvar{_nrn_mechanism_access_dparam(_prop)}, *_thread{};
  {
 {
   
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
    INFOCAST;
    Info* info = *ip;
#ifdef NRN_VERSION_GTEQ_9_0_0
    delete[] info->ptrs_;
#else
    free(info->ptrs_);
#endif
    free(info);
#endif
}
 }
 
}
}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
   net_send ( _tqitem, nullptr, _ppvar[1].get<Point_process*>(), t +  0.0 , 1.0 ) ;
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
   _v = _vec_v[_ni[_iml]];
 v = _v;
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
   _v = _vec_v[_ni[_iml]];
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
    const char* nmodl_filename = "ALU.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "If the local variable step method is used then the only variables that should\n"
  "be added are variables of the cell in which this ALU has been instantiated.\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    THREADSAFE\n"
  "    POINT_PROCESS ALU\n"
  "    BBCOREPOINTER ptr\n"
  "    RANGE Dt\n"
  "    RANGE output\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    Dt = .1 (ms)\n"
  "    output = 0\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    ptr\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    net_send(0, 1)\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "\n"
  "#ifndef CORENEURON_BUILD\n"
  "#if defined(NRN_VERSION_GTEQ)\n"
  "#if NRN_VERSION_GTEQ(9,0,0)\n"
  "#define NRN_VERSION_GTEQ_9_0_0\n"
  "#endif\n"
  "#endif\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "extern double* hoc_pgetarg(int iarg);\n"
  "extern double* getarg(int iarg);\n"
  "extern int ifarg(int iarg);\n"
  "extern void nrn_register_recalc_ptr_callback(void (*f)());\n"
  "extern Point_process* ob2pntproc(Object*);\n"
  "extern double* nrn_recalc_ptr(double*);\n"
  "#endif\n"
  "\n"
  "#ifdef NRN_MECHANISM_DATA_IS_SOA\n"
  "using handle_to_double = decltype(hoc_hgetarg<double>(42));\n"
  "#else\n"
  "typedef double* handle_to_double;\n"
  "#endif\n"
  "typedef struct {\n"
  "    //! list of pointers to hoc variables\n"
  "    handle_to_double* ptrs_;\n"
  "\n"
  "    /*! list of scalars to apply to corresponding variables; useful for making units of variables\n"
  "     * from different sources consistent (e.g. i current sources may be distributed, mA/cm^2, or point processes, nA)\n"
  "     */\n"
  "    double * scalars_;\n"
  "\n"
  "    //! number of elements stored in the vectors\n"
  "    int np_;\n"
  "\n"
  "    //! number of slots allocated to the vectors\n"
  "    int psize_;\n"
  "\n"
  "    //! function pointer to execute when net_receive is triggered\n"
  "#ifdef NRN_MECHANISM_DATA_IS_SOA\n"
  "    int (*process)(_internalthreadargsproto_);\n"
  "#else\n"
  "    int (*process)(_threadargsproto_);\n"
  "#endif\n"
  "} Info;\n"
  "\n"
  "#define INFOCAST Info** ip = (Info**)(&(_p_ptr))\n"
  "\n"
  "#define dp double*\n"
  "\n"
  "#ifndef NRN_MECHANISM_DATA_IS_SOA\n"
  "static void recalcptr(Info* info, int cnt, double** old_vp, double* new_v) {\n"
  "    int i;\n"
  "    /*printf(\"recalcptr np_=%d %s\\n\", info->np_, info->path_);*/\n"
  "\n"
  "}\n"
  "static void recalc_ptr_callback() {\n"
  "    Symbol* sym;\n"
  "    int i;\n"
  "    hoc_List* instances;\n"
  "    hoc_Item* q;\n"
  "    /*printf(\"ASCIIrecord.mod recalc_ptr_callback\\n\");*/\n"
  "    /* hoc has a list of the ASCIIRecord instances */\n"
  "    sym = hoc_lookup(\"ALU\");\n"
  "    #ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "    instances = sym->u.template->olist;\n"
  "    #else\n"
  "    instances = sym->u.ctemplate->olist;\n"
  "    #endif\n"
  "    ITERATE(q, instances) {\n"
  "        Info* InfoPtr;\n"
  "        Point_process* pnt;\n"
  "        Object* o = OBJ(q);\n"
  "        /*printf(\"callback for %s\\n\", hoc_object_name(o));*/\n"
  "        pnt = ob2pntproc(o);\n"
  "        Datum* _ppvar = pnt->_prop->dparam;\n"
  "        INFOCAST;\n"
  "        InfoPtr = *ip;\n"
  "        for (i=0; i < InfoPtr->np_; ++i)\n"
  "            InfoPtr->ptrs_[i] =  nrn_recalc_ptr(InfoPtr->ptrs_[i]);\n"
  "    }\n"
  "}\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "\n"
  "NET_RECEIVE(w) {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "    INFOCAST;\n"
  "    Info* info = *ip;\n"
  "    info->process(_threadargs_);\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "    net_send(Dt - 1e-5, 1)\n"
  "}\n"
  "\n"
  "CONSTRUCTOR {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef NRN_MECHANISM_DATA_IS_SOA\n"
  "    static int first = 1;\n"
  "    if (first) {\n"
  "        first = 0;\n"
  "        nrn_register_recalc_ptr_callback(recalc_ptr_callback);\n"
  "    }\n"
  "#endif\n"
  "\n"
  "    INFOCAST;\n"
  "    Info* info = (Info*)hoc_Emalloc(sizeof(Info)); hoc_malchk();\n"
  "    info->psize_ = 10;\n"
  "#ifdef NRN_VERSION_GTEQ_9_0_0\n"
  "    info->ptrs_ = new handle_to_double[info->psize_]; hoc_malchk();\n"
  "#else\n"
  "    info->ptrs_ = (handle_to_double*)hoc_Ecalloc(info->psize_, sizeof(handle_to_double)); hoc_malchk();\n"
  "#endif\n"
  "    info->scalars_ = (double*)hoc_Ecalloc(info->psize_, sizeof(double)); hoc_malchk();\n"
  "    info->np_ = 0;\n"
  "    *ip = info;\n"
  "\n"
  "    if (ifarg(2)) {\n"
  "        Dt = *getarg(2);\n"
  "    }\n"
  "\n"
  "    //default operation is average\n"
  "    info->process = &average;\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "DESTRUCTOR {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "    INFOCAST;\n"
  "    Info* info = *ip;\n"
  "#ifdef NRN_VERSION_GTEQ_9_0_0\n"
  "    delete[] info->ptrs_;\n"
  "#else\n"
  "    free(info->ptrs_);\n"
  "#endif\n"
  "    free(info);\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/**\n"
  " * Resume the event delivery loop for NEURON restore. Call from Hoc only (there's param)\n"
  " *\n"
  " * @param t The initial time\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE restartEvent() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "    const double etime = *getarg(1);\n"
  "    #if defined(NRN_VERSION_GTEQ_9_0_0)\n"
  "    net_send(_tqitem, (double*)0, _ppvar[1].get<Point_process*>(), etime, 1.0);\n"
  "    #else\n"
  "    net_send(_tqitem, (double*)0, (Point_process*)_ppvar[1]._pvoid, etime, 1.0);\n"
  "    #endif    \n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/*!\n"
  " * Include another variable in the arithmetic operation\n"
  " * @param variable pointers\n"
  " * @param scalar (optional, 1 by default)\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE addvar() { : double* pd\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "    INFOCAST;\n"
  "    Info* info = *ip;\n"
  "    if (info->np_ >= info->psize_) {\n"
  "        info->psize_ += 10;\n"
  "#ifdef NRN_VERSION_GTEQ_9_0_0\n"
  "        auto old_ptrs = info->ptrs_;\n"
  "        info->ptrs_ = new handle_to_double[info->psize_]; hoc_malchk();\n"
  "        std::copy(old_ptrs, old_ptrs+info->np_, info->ptrs_);\n"
  "        delete[] old_ptrs;\n"
  "#else\n"
  "        info->ptrs_ = (handle_to_double*)hoc_Erealloc(info->ptrs_, info->psize_*sizeof(handle_to_double)); hoc_malchk();\n"
  "#endif\n"
  "        info->scalars_ = (double*) hoc_Erealloc(info->scalars_, info->psize_*sizeof(double)); hoc_malchk();\n"
  "    }\n"
  "#ifdef NRN_MECHANISM_DATA_IS_SOA\n"
  "    handle_to_double var = hoc_hgetarg<double>(1);\n"
  "#else\n"
  "    handle_to_double var = hoc_pgetarg(1);\n"
  "#endif\n"
  "    info->ptrs_[info->np_] = var;\n"
  "    if( ifarg(2)) {\n"
  "        info->scalars_[info->np_] = *getarg(2);\n"
  "    } else {\n"
  "        info->scalars_[info->np_] = 1;\n"
  "    }\n"
  "\n"
  "    ++info->np_;\n"
  "    //printf(\"I have %d values.. (new = %g * %g)\\n\", info->np_, *(info->ptrs_[info->np_-1]), info->scalars_[info->np_-1] );\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "/*!\n"
  " * Ignore the ptr and instead just report the constant value.  This is a hack to allow reporting of the\n"
  " * area of a section.  A better solution should be created\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE constant() {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "    INFOCAST;\n"
  "    Info* info = *ip;\n"
  "    if( info->np_ > 0 ) {\n"
  "        output = info->scalars_[0];\n"
  "    } else {\n"
  "        output = 0;\n"
  "    }\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "COMMENT\n"
  "/*!\n"
  " * Take an average of all the variables assigned to this ALU object\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE average() {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "    INFOCAST;\n"
  "    Info* info = *ip;\n"
  "    int i;\n"
  "    double n = 0;\n"
  "    for (i=0; i < info->np_; ++i) {\n"
  "      //  printf(\"%f\", (*info->ptrs_[i] * info->scalars_[i]) );\n"
  "        n += (*info->ptrs_[i] * info->scalars_[i]);\n"
  "    }\n"
  "    //printf(\"\\n\");\n"
  "//    output = n/info->np_;\n"
  "    if (info->np_ > 0)\n"
  "      output = n/info->np_;\n"
  "    else output = 0;\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "/*!\n"
  " * Take a summation of all the variables assigned to this ALU object\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE summation() {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "    INFOCAST; Info* info = *ip;\n"
  "    int i;\n"
  "    double n = 0;\n"
  "    for (i=0; i < info->np_; ++i) {\n"
  "        //printf(\"%f = %f * %f\\n\", (*info->ptrs_[i] * info->scalars_[i]), *info->ptrs_[i], info->scalars_[i] );\n"
  "        n += (*info->ptrs_[i] * info->scalars_[i]);\n"
  "    }\n"
  "\n"
  "    output = n;\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "/*!\n"
  " * Set the operation performed when NET_RECEIVE block executes\n"
  " *\n"
  " * @param opname The name of the function to be executed\n"
  " */\n"
  "ENDCOMMENT\n"
  "PROCEDURE setop() {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "    INFOCAST; Info* info = *ip;\n"
  "\n"
  "    char *opname = NULL;\n"
  "    if (!hoc_is_str_arg(1)) {\n"
  "        exit(0);\n"
  "    }\n"
  "\n"
  "    opname = gargstr(1);\n"
  "    if( strcmp( opname, \"summation\" ) == 0 ) {\n"
  "        info->process = &summation;\n"
  "    } else if ( strcmp( opname, \"average\" ) == 0 ) {\n"
  "        info->process = &average;\n"
  "    } else if ( strcmp( opname, \"constant\" ) == 0 ) {\n"
  "        info->process = &constant;\n"
  "    } else {\n"
  "        fprintf( stderr, \"Error: unknown operation '%s' for ALU object.  Terminating.\\n\", opname );\n"
  "        exit(0);\n"
  "    }\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "/** not executed in coreneuron and hence need empty stubs only */\n"
  "static void bbcore_write(double* x, int* d, int* xx, int* offset, _threadargsproto_) {\n"
  "}\n"
  "static void bbcore_read(double* x, int* d, int* xx, int* offset, _threadargsproto_) {\n"
  "}\n"
  "ENDVERBATIM\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
