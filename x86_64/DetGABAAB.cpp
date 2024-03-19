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
static constexpr auto number_of_datum_variables = 4;
static constexpr auto number_of_floating_point_variables = 34;
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
 
#define nrn_init _nrn_init__DetGABAAB
#define _nrn_initial _nrn_initial__DetGABAAB
#define nrn_cur _nrn_cur__DetGABAAB
#define _nrn_current _nrn_current__DetGABAAB
#define nrn_jacob _nrn_jacob__DetGABAAB
#define nrn_state _nrn_state__DetGABAAB
#define _net_receive _net_receive__DetGABAAB 
#define state state__DetGABAAB 
#define setup_delay_vecs setup_delay_vecs__DetGABAAB 
 
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
#define tau_r_GABAA _ml->template fpfield<0>(_iml)
#define tau_r_GABAA_columnindex 0
#define tau_d_GABAA _ml->template fpfield<1>(_iml)
#define tau_d_GABAA_columnindex 1
#define tau_r_GABAB _ml->template fpfield<2>(_iml)
#define tau_r_GABAB_columnindex 2
#define tau_d_GABAB _ml->template fpfield<3>(_iml)
#define tau_d_GABAB_columnindex 3
#define Use _ml->template fpfield<4>(_iml)
#define Use_columnindex 4
#define Dep _ml->template fpfield<5>(_iml)
#define Dep_columnindex 5
#define Fac _ml->template fpfield<6>(_iml)
#define Fac_columnindex 6
#define e_GABAA _ml->template fpfield<7>(_iml)
#define e_GABAA_columnindex 7
#define e_GABAB _ml->template fpfield<8>(_iml)
#define e_GABAB_columnindex 8
#define u0 _ml->template fpfield<9>(_iml)
#define u0_columnindex 9
#define synapseID _ml->template fpfield<10>(_iml)
#define synapseID_columnindex 10
#define verboseLevel _ml->template fpfield<11>(_iml)
#define verboseLevel_columnindex 11
#define GABAB_ratio _ml->template fpfield<12>(_iml)
#define GABAB_ratio_columnindex 12
#define conductance _ml->template fpfield<13>(_iml)
#define conductance_columnindex 13
#define i _ml->template fpfield<14>(_iml)
#define i_columnindex 14
#define i_GABAA _ml->template fpfield<15>(_iml)
#define i_GABAA_columnindex 15
#define i_GABAB _ml->template fpfield<16>(_iml)
#define i_GABAB_columnindex 16
#define g_GABAA _ml->template fpfield<17>(_iml)
#define g_GABAA_columnindex 17
#define g_GABAB _ml->template fpfield<18>(_iml)
#define g_GABAB_columnindex 18
#define g _ml->template fpfield<19>(_iml)
#define g_columnindex 19
#define next_delay _ml->template fpfield<20>(_iml)
#define next_delay_columnindex 20
#define A_GABAA _ml->template fpfield<21>(_iml)
#define A_GABAA_columnindex 21
#define B_GABAA _ml->template fpfield<22>(_iml)
#define B_GABAA_columnindex 22
#define A_GABAB _ml->template fpfield<23>(_iml)
#define A_GABAB_columnindex 23
#define B_GABAB _ml->template fpfield<24>(_iml)
#define B_GABAB_columnindex 24
#define factor_GABAA _ml->template fpfield<25>(_iml)
#define factor_GABAA_columnindex 25
#define factor_GABAB _ml->template fpfield<26>(_iml)
#define factor_GABAB_columnindex 26
#define DA_GABAA _ml->template fpfield<27>(_iml)
#define DA_GABAA_columnindex 27
#define DB_GABAA _ml->template fpfield<28>(_iml)
#define DB_GABAA_columnindex 28
#define DA_GABAB _ml->template fpfield<29>(_iml)
#define DA_GABAB_columnindex 29
#define DB_GABAB _ml->template fpfield<30>(_iml)
#define DB_GABAB_columnindex 30
#define v _ml->template fpfield<31>(_iml)
#define v_columnindex 31
#define _g _ml->template fpfield<32>(_iml)
#define _g_columnindex 32
#define _tsav _ml->template fpfield<33>(_iml)
#define _tsav_columnindex 33
#define _nd_area *_ml->dptr_field<0>(_iml)
#define delay_times	*_ppvar[2].get<double*>()
#define _p_delay_times _ppvar[2].literal_value<void*>()
#define delay_weights	*_ppvar[3].get<double*>()
#define _p_delay_weights _ppvar[3].literal_value<void*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  2;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_setup_delay_vecs(void*);
 static double _hoc_toggleVerbose(void*);
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
 {"setup_delay_vecs", _hoc_setup_delay_vecs},
 {"toggleVerbose", _hoc_toggleVerbose},
 {0, 0}
};
#define toggleVerbose toggleVerbose_DetGABAAB
 extern double toggleVerbose( _internalthreadargsproto_ );
 /* declare global and static user variables */
#define gmax gmax_DetGABAAB
 double gmax = 0.001;
#define nc_type_param nc_type_param_DetGABAAB
 double nc_type_param = 7;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gmax_DetGABAAB", "uS"},
 {"tau_r_GABAA", "ms"},
 {"tau_d_GABAA", "ms"},
 {"tau_r_GABAB", "ms"},
 {"tau_d_GABAB", "ms"},
 {"Use", "1"},
 {"Dep", "ms"},
 {"Fac", "ms"},
 {"e_GABAA", "mV"},
 {"e_GABAB", "mV"},
 {"GABAB_ratio", "1"},
 {"i", "nA"},
 {"i_GABAA", "nA"},
 {"i_GABAB", "nA"},
 {"g_GABAA", "uS"},
 {"g_GABAB", "uS"},
 {"g", "uS"},
 {"next_delay", "ms"},
 {0, 0}
};
 static double A_GABAB0 = 0;
 static double A_GABAA0 = 0;
 static double B_GABAB0 = 0;
 static double B_GABAA0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"gmax_DetGABAAB", &gmax_DetGABAAB},
 {"nc_type_param_DetGABAAB", &nc_type_param_DetGABAAB},
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void nrn_init(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_state(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void nrn_cur(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_jacob(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[5].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"DetGABAAB",
 "tau_r_GABAA",
 "tau_d_GABAA",
 "tau_r_GABAB",
 "tau_d_GABAB",
 "Use",
 "Dep",
 "Fac",
 "e_GABAA",
 "e_GABAB",
 "u0",
 "synapseID",
 "verboseLevel",
 "GABAB_ratio",
 "conductance",
 0,
 "i",
 "i_GABAA",
 "i_GABAB",
 "g_GABAA",
 "g_GABAB",
 "g",
 "next_delay",
 0,
 "A_GABAA",
 "B_GABAA",
 "A_GABAB",
 "B_GABAB",
 0,
 "delay_times",
 "delay_weights",
 0};
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 6, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 34);
 	/*initialize range parameters*/
 	tau_r_GABAA = 0.2;
 	tau_d_GABAA = 8;
 	tau_r_GABAB = 3.5;
 	tau_d_GABAB = 260.9;
 	Use = 1;
 	Dep = 100;
 	Fac = 10;
 	e_GABAA = -80;
 	e_GABAB = -97;
 	u0 = 0;
 	synapseID = 0;
 	verboseLevel = 0;
 	GABAB_ratio = 0;
 	conductance = 0;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 34);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 
#define _tqitem &(_ppvar[4])
 static void _net_receive(Point_process*, double*, double);
 static void _net_init(Point_process*, double*, double);
 static void bbcore_write(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_write(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 static void bbcore_read(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_read(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _DetGABAAB_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
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
                                       _nrn_mechanism_field<double>{"tau_r_GABAA"} /* 0 */,
                                       _nrn_mechanism_field<double>{"tau_d_GABAA"} /* 1 */,
                                       _nrn_mechanism_field<double>{"tau_r_GABAB"} /* 2 */,
                                       _nrn_mechanism_field<double>{"tau_d_GABAB"} /* 3 */,
                                       _nrn_mechanism_field<double>{"Use"} /* 4 */,
                                       _nrn_mechanism_field<double>{"Dep"} /* 5 */,
                                       _nrn_mechanism_field<double>{"Fac"} /* 6 */,
                                       _nrn_mechanism_field<double>{"e_GABAA"} /* 7 */,
                                       _nrn_mechanism_field<double>{"e_GABAB"} /* 8 */,
                                       _nrn_mechanism_field<double>{"u0"} /* 9 */,
                                       _nrn_mechanism_field<double>{"synapseID"} /* 10 */,
                                       _nrn_mechanism_field<double>{"verboseLevel"} /* 11 */,
                                       _nrn_mechanism_field<double>{"GABAB_ratio"} /* 12 */,
                                       _nrn_mechanism_field<double>{"conductance"} /* 13 */,
                                       _nrn_mechanism_field<double>{"i"} /* 14 */,
                                       _nrn_mechanism_field<double>{"i_GABAA"} /* 15 */,
                                       _nrn_mechanism_field<double>{"i_GABAB"} /* 16 */,
                                       _nrn_mechanism_field<double>{"g_GABAA"} /* 17 */,
                                       _nrn_mechanism_field<double>{"g_GABAB"} /* 18 */,
                                       _nrn_mechanism_field<double>{"g"} /* 19 */,
                                       _nrn_mechanism_field<double>{"next_delay"} /* 20 */,
                                       _nrn_mechanism_field<double>{"A_GABAA"} /* 21 */,
                                       _nrn_mechanism_field<double>{"B_GABAA"} /* 22 */,
                                       _nrn_mechanism_field<double>{"A_GABAB"} /* 23 */,
                                       _nrn_mechanism_field<double>{"B_GABAB"} /* 24 */,
                                       _nrn_mechanism_field<double>{"factor_GABAA"} /* 25 */,
                                       _nrn_mechanism_field<double>{"factor_GABAB"} /* 26 */,
                                       _nrn_mechanism_field<double>{"DA_GABAA"} /* 27 */,
                                       _nrn_mechanism_field<double>{"DB_GABAA"} /* 28 */,
                                       _nrn_mechanism_field<double>{"DA_GABAB"} /* 29 */,
                                       _nrn_mechanism_field<double>{"DB_GABAB"} /* 30 */,
                                       _nrn_mechanism_field<double>{"v"} /* 31 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 32 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 33 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"delay_times", "bbcorepointer"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"delay_weights", "bbcorepointer"} /* 3 */,
                                       _nrn_mechanism_field<void*>{"_tqitem", "netsend"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 34, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 3, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 4, "netsend");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 8;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 DetGABAAB DetGABAAB.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "GABAA and GABAB receptor with presynaptic short-term plasticity";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int setup_delay_vecs(_internalthreadargsproto_);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[4], _dlist1[4];
 static int state(_internalthreadargsproto_);
 
/*VERBATIM*/

#ifndef CORENEURON_BUILD
extern int ifarg(int iarg);
#ifndef NRN_VERSION_GTEQ_8_2_0
extern void* vector_arg(int iarg);
extern double* vector_vec(void* vv);
extern int vector_capacity(void* vv);
#endif
#endif

 
static int  setup_delay_vecs ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
#ifndef CORENEURON_BUILD
    void** vv_delay_times = (void**)(&_p_delay_times);
    void** vv_delay_weights = (void**)(&_p_delay_weights);
    *vv_delay_times = (void*)NULL;
    *vv_delay_weights = (void*)NULL;
    if (ifarg(1)) {
        *vv_delay_times = vector_arg(1);
    }
    if (ifarg(2)) {
        *vv_delay_weights = vector_arg(2);
    }
#endif
  return 0; }
 
static double _hoc_setup_delay_vecs(void* _vptr) {
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
 setup_delay_vecs ( _threadargs_ );
 return(_r);
}
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   DA_GABAA = - A_GABAA / tau_r_GABAA ;
   DB_GABAA = - B_GABAA / tau_d_GABAA ;
   DA_GABAB = - A_GABAB / tau_r_GABAB ;
   DB_GABAB = - B_GABAB / tau_d_GABAB ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 DA_GABAA = DA_GABAA  / (1. - dt*( ( - 1.0 ) / tau_r_GABAA )) ;
 DB_GABAA = DB_GABAA  / (1. - dt*( ( - 1.0 ) / tau_d_GABAA )) ;
 DA_GABAB = DA_GABAB  / (1. - dt*( ( - 1.0 ) / tau_r_GABAB )) ;
 DB_GABAB = DB_GABAB  / (1. - dt*( ( - 1.0 ) / tau_d_GABAB )) ;
  return 0;
}
 /*END CVODE*/
 static int state (_internalthreadargsproto_) { {
    A_GABAA = A_GABAA + (1. - exp(dt*(( - 1.0 ) / tau_r_GABAA)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_r_GABAA ) - A_GABAA) ;
    B_GABAA = B_GABAA + (1. - exp(dt*(( - 1.0 ) / tau_d_GABAA)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_d_GABAA ) - B_GABAA) ;
    A_GABAB = A_GABAB + (1. - exp(dt*(( - 1.0 ) / tau_r_GABAB)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_r_GABAB ) - A_GABAB) ;
    B_GABAB = B_GABAB + (1. - exp(dt*(( - 1.0 ) / tau_d_GABAB)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_d_GABAB ) - B_GABAB) ;
   }
  return 0;
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
   double _lresult ;
 _args[1] = _args[0] ;
   _args[2] = _args[0] * GABAB_ratio ;
   if ( t < 0.0 ) {
     
/*VERBATIM*/
        return;
 }
   if ( _lflag  == 1.0 ) {
     
/*VERBATIM*/
        // setup self events for delayed connections to change weights
        IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));
        if (vv_delay_weights && vector_capacity(vv_delay_weights)>=next_delay) {
          double* weights_v = vector_vec(vv_delay_weights);
          double next_delay_weight = weights_v[(int)next_delay];
 _args[0] = conductance * next_delay_weight ;
     next_delay = next_delay + 1.0 ;
     
/*VERBATIM*/
        }
        return;
 }
   if ( Fac > 0.0 ) {
     _args[5] = _args[5] * exp ( - ( t - _args[6] ) / Fac ) ;
     }
   else {
     _args[5] = Use ;
     }
   if ( Fac > 0.0 ) {
     _args[5] = _args[5] + Use * ( 1.0 - _args[5] ) ;
     }
   _args[3] = 1.0 - ( 1.0 - _args[3] ) * exp ( - ( t - _args[6] ) / Dep ) ;
   _args[4] = _args[5] * _args[3] ;
   _args[3] = _args[3] - _args[5] * _args[3] ;
   if ( verboseLevel > 0.0 ) {
     printf ( "Synapse %f at time %g: R = %g Pr = %g erand = %g\n" , synapseID , t , _args[3] , _args[4] , _lresult ) ;
     }
   _args[6] = t ;
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A_GABAA;
    double __primary = (A_GABAA + _args[4] * _args[1] * factor_GABAA) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_r_GABAA ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_r_GABAA ) - __primary );
    A_GABAA += __primary;
  } else {
 A_GABAA = A_GABAA + _args[4] * _args[1] * factor_GABAA ;
     }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B_GABAA;
    double __primary = (B_GABAA + _args[4] * _args[1] * factor_GABAA) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_d_GABAA ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_d_GABAA ) - __primary );
    B_GABAA += __primary;
  } else {
 B_GABAA = B_GABAA + _args[4] * _args[1] * factor_GABAA ;
     }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A_GABAB;
    double __primary = (A_GABAB + _args[4] * _args[2] * factor_GABAB) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_r_GABAB ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_r_GABAB ) - __primary );
    A_GABAB += __primary;
  } else {
 A_GABAB = A_GABAB + _args[4] * _args[2] * factor_GABAB ;
     }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B_GABAB;
    double __primary = (B_GABAB + _args[4] * _args[2] * factor_GABAB) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_d_GABAB ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_d_GABAB ) - __primary );
    B_GABAB += __primary;
  } else {
 B_GABAB = B_GABAB + _args[4] * _args[2] * factor_GABAB ;
     }
 if ( verboseLevel > 0.0 ) {
     printf ( " vals %g %g %g %g\n" , A_GABAA , _args[1] , factor_GABAA , _args[0] ) ;
     }
   } }
 
static void _net_init(Point_process* _pnt, double* _args, double _lflag) {
     _nrn_mechanism_cache_instance _ml_real{_pnt->_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  Datum* _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  Datum* _thread = (Datum*)0;
  NrnThread* _nt = (NrnThread*)_pnt->_vnt;
 _args[3] = 1.0 ;
   _args[5] = u0 ;
   _args[6] = t ;
   if ( _args[7]  == 0.0 ) {
     
/*VERBATIM*/
            // setup self events for delayed connections to change weights
            IvocVect *vv_delay_times = *((IvocVect**)(&_p_delay_times));
            IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));
            if (vv_delay_times && vector_capacity(vv_delay_times)>=1) {
              double* deltm_el = vector_vec(vv_delay_times);
              int delay_times_idx;
              next_delay = 0;
              for(delay_times_idx = 0; delay_times_idx < vector_capacity(vv_delay_times); ++delay_times_idx) {
                double next_delay_t = deltm_el[delay_times_idx];
 net_send ( _tqitem, _args, _pnt, t +  next_delay_t , 1.0 ) ;
     
/*VERBATIM*/
              }
            }
 }
   }
 
double toggleVerbose ( _internalthreadargsproto_ ) {
   double _ltoggleVerbose;
 verboseLevel = 1.0 - verboseLevel ;
   
return _ltoggleVerbose;
 }
 
static double _hoc_toggleVerbose(void* _vptr) {
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
 _r =  toggleVerbose ( _threadargs_ );
 return(_r);
}
 
/*VERBATIM*/
static void bbcore_write(double* x, int* d, int* x_offset, int* d_offset, _threadargsproto_) {

  IvocVect *vv_delay_times = *((IvocVect**)(&_p_delay_times));
  IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));

  // serialize connection delay vectors
  if (vv_delay_times && vv_delay_weights &&
     (vector_capacity(vv_delay_times) >= 1) && (vector_capacity(vv_delay_weights) >= 1)) {
    if (d && x) {
      int* d_i = d + *d_offset;
      // store vector sizes for deserialization
      d_i[0] = vector_capacity(vv_delay_times);
      d_i[1] = vector_capacity(vv_delay_weights);

      double* delay_times_el = vector_vec(vv_delay_times);
      double* delay_weights_el = vector_vec(vv_delay_weights);
      double* x_i = x + *x_offset;
      int delay_vecs_idx;
      int x_idx = 0;
      for(delay_vecs_idx = 0; delay_vecs_idx < vector_capacity(vv_delay_times); ++delay_vecs_idx) {
         x_i[x_idx++] = delay_times_el[delay_vecs_idx];
         x_i[x_idx++] = delay_weights_el[delay_vecs_idx];
      }
    }
  } else {
    if (d) {
      int* d_i = d + *d_offset;
      d_i[0] = 0;
      d_i[1] = 0;
    }
  }
  // reserve space for delay connection vector sizes on serialization buffer
  *d_offset += 2;

  // reserve space for connection delay data on serialization buffer
  if (vv_delay_times && vv_delay_weights) {
    *x_offset += vector_capacity(vv_delay_times) + vector_capacity(vv_delay_weights);
  }
}

static void bbcore_read(double* x, int* d, int* x_offset, int* d_offset, _threadargsproto_) {
  assert(!_p_delay_times && !_p_delay_weights);

  // first get delay vector sizes
  int* d_i = d + *d_offset;
  int delay_times_sz = d_i[0];
  int delay_weights_sz = d_i[1];
  *d_offset += 2;

  if ((delay_times_sz > 0) && (delay_weights_sz > 0)) {
    double* x_i = x + *x_offset;

    // allocate vectors
    _p_delay_times = (double*)vector_new1(delay_times_sz);
    _p_delay_weights = (double*)vector_new1(delay_weights_sz);

    double* delay_times_el = vector_vec((IvocVect*)_p_delay_times);
    double* delay_weights_el = vector_vec((IvocVect*)_p_delay_weights);

    // copy data
    int x_idx;
    int vec_idx = 0;
    for(x_idx = 0; x_idx < delay_times_sz + delay_weights_sz; x_idx += 2) {
      delay_times_el[vec_idx] = x_i[x_idx];
      delay_weights_el[vec_idx++] = x_i[x_idx+1];
    }
    *x_offset += delay_times_sz + delay_weights_sz;
  }
}
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
   Datum* _ppvar;
   size_t _iml;   _nrn_mechanism_cache_range* _ml;   Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_threadargs_);
 }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  Datum* _ppvar;
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 4; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _ode_matsol1 (_threadargs_);
 }
 
static void _ode_matsol(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
   Datum* _ppvar;
   size_t _iml;   _nrn_mechanism_cache_range* _ml;   Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  A_GABAB = A_GABAB0;
  A_GABAA = A_GABAA0;
  B_GABAB = B_GABAB0;
  B_GABAA = B_GABAA0;
 {
   double _ltp_GABAA , _ltp_GABAB ;
 A_GABAA = 0.0 ;
   B_GABAA = 0.0 ;
   A_GABAB = 0.0 ;
   B_GABAB = 0.0 ;
   _ltp_GABAA = ( tau_r_GABAA * tau_d_GABAA ) / ( tau_d_GABAA - tau_r_GABAA ) * log ( tau_d_GABAA / tau_r_GABAA ) ;
   _ltp_GABAB = ( tau_r_GABAB * tau_d_GABAB ) / ( tau_d_GABAB - tau_r_GABAB ) * log ( tau_d_GABAB / tau_r_GABAB ) ;
   factor_GABAA = - exp ( - _ltp_GABAA / tau_r_GABAA ) + exp ( - _ltp_GABAA / tau_d_GABAA ) ;
   factor_GABAA = 1.0 / factor_GABAA ;
   factor_GABAB = - exp ( - _ltp_GABAB / tau_r_GABAB ) + exp ( - _ltp_GABAB / tau_d_GABAB ) ;
   factor_GABAB = 1.0 / factor_GABAB ;
   next_delay = - 1.0 ;
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
{ {
   g_GABAA = gmax * ( B_GABAA - A_GABAA ) ;
   g_GABAB = gmax * ( B_GABAB - A_GABAB ) ;
   g = g_GABAA + g_GABAB ;
   i_GABAA = g_GABAA * ( v - e_GABAA ) ;
   i_GABAB = g_GABAB * ( v - e_GABAB ) ;
   i = i_GABAA + i_GABAB ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_rhs = _nt->node_rhs_storage();
auto const _vec_sav_rhs = _nt->node_sav_rhs_storage();
auto const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ _rhs = _nrn_current(_threadargscomma_ _v);
 	}
 _g = (_g_local - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
	 _vec_rhs[_ni[_iml]] -= _rhs;
 
}
 
}

static void nrn_jacob(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_d = _nt->node_d_storage();
auto const _vec_sav_d = _nt->node_sav_d_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
  _vec_d[_ni[_iml]] += _g;
 
}
 
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
 {   state(_threadargs_);
  }}}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {A_GABAA_columnindex, 0};  _dlist1[0] = {DA_GABAA_columnindex, 0};
 _slist1[1] = {B_GABAA_columnindex, 0};  _dlist1[1] = {DB_GABAA_columnindex, 0};
 _slist1[2] = {A_GABAB_columnindex, 0};  _dlist1[2] = {DA_GABAB_columnindex, 0};
 _slist1[3] = {B_GABAB_columnindex, 0};  _dlist1[3] = {DB_GABAB_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "DetGABAAB.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "/**\n"
  " * @file DetGABAAB.mod\n"
  " * @brief Adapted from ProbGABAA_EMS.mod by Eilif, Michael and Srikanth\n"
  " * @author chindemi\n"
  " * @date 2014-05-25\n"
  " * @remark Copyright \n"
  "\n"
  " BBP/EPFL 2005-2014; All rights reserved. Do not distribute without further notice.\n"
  " */\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "TITLE GABAA and GABAB receptor with presynaptic short-term plasticity\n"
  "\n"
  "\n"
  "COMMENT\n"
  "GABAA and GABAB receptor conductance using a dual-exponential profile\n"
  "presynaptic short-term plasticity based on Fuhrmann et al. 2002, deterministic\n"
  "version.\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "NEURON {\n"
  "    THREADSAFE\n"
  "\n"
  "    POINT_PROCESS DetGABAAB\n"
  "    RANGE tau_r_GABAA, tau_d_GABAA, tau_r_GABAB, tau_d_GABAB\n"
  "    RANGE Use, u, Dep, Fac, u0, GABAB_ratio\n"
  "    RANGE i, i_GABAA, i_GABAB, g_GABAA, g_GABAB, g, e_GABAA, e_GABAB\n"
  "    NONSPECIFIC_CURRENT i\n"
  "    RANGE synapseID, verboseLevel\n"
  "    RANGE conductance\n"
  "    RANGE next_delay\n"
  "    BBCOREPOINTER delay_times, delay_weights\n"
  "    GLOBAL nc_type_param\n"
  "    : For debugging\n"
  "    :RANGE sgid, tgid\n"
  "}\n"
  "\n"
  "\n"
  "PARAMETER {\n"
  "    tau_r_GABAA  = 0.2   (ms) : dual-exponential conductance profile\n"
  "    tau_d_GABAA  = 8     (ms) : IMPORTANT: tau_r < tau_d\n"
  "    tau_r_GABAB  = 3.5   (ms) : dual-exponential conductance profile :Placeholder value from hippocampal recordings SR\n"
  "    tau_d_GABAB  = 260.9 (ms) : IMPORTANT: tau_r < tau_d  :Placeholder value from hippocampal recordings\n"
  "    Use          = 1.0   (1)  : Utilization of synaptic efficacy\n"
  "    Dep          = 100   (ms) : relaxation time constant from depression\n"
  "    Fac          = 10    (ms) :  relaxation time constant from facilitation\n"
  "    e_GABAA      = -80   (mV) : GABAA reversal potential\n"
  "    e_GABAB      = -97   (mV) : GABAB reversal potential\n"
  "    gmax         = .001  (uS) : weight conversion factor (from nS to uS)\n"
  "    u0           = 0          :initial value of u, which is the running value of release probability\n"
  "    synapseID    = 0\n"
  "    verboseLevel = 0\n"
  "    GABAB_ratio  = 0     (1)  : The ratio of GABAB to GABAA\n"
  "    conductance  = 0.0\n"
  "    nc_type_param = 7\n"
  "}\n"
  "\n"
  "\n"
  "VERBATIM\n"
  "\n"
  "#ifndef CORENEURON_BUILD\n"
  "extern int ifarg(int iarg);\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "extern void* vector_arg(int iarg);\n"
  "extern double* vector_vec(void* vv);\n"
  "extern int vector_capacity(void* vv);\n"
  "#endif\n"
  "#endif\n"
  "\n"
  "ENDVERBATIM\n"
  "\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    i (nA)\n"
  "    i_GABAA (nA)\n"
  "    i_GABAB (nA)\n"
  "    g_GABAA (uS)\n"
  "    g_GABAB (uS)\n"
  "    g (uS)\n"
  "    factor_GABAA\n"
  "    factor_GABAB\n"
  "\n"
  "    : stuff for delayed connections\n"
  "    delay_times\n"
  "    delay_weights\n"
  "    next_delay (ms)\n"
  "}\n"
  "\n"
  "PROCEDURE setup_delay_vecs() {\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "    void** vv_delay_times = (void**)(&_p_delay_times);\n"
  "    void** vv_delay_weights = (void**)(&_p_delay_weights);\n"
  "    *vv_delay_times = (void*)NULL;\n"
  "    *vv_delay_weights = (void*)NULL;\n"
  "    if (ifarg(1)) {\n"
  "        *vv_delay_times = vector_arg(1);\n"
  "    }\n"
  "    if (ifarg(2)) {\n"
  "        *vv_delay_weights = vector_arg(2);\n"
  "    }\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "STATE {\n"
  "    A_GABAA       : GABAA state variable to construct the dual-exponential profile - decays with conductance tau_r_GABAA\n"
  "    B_GABAA       : GABAA state variable to construct the dual-exponential profile - decays with conductance tau_d_GABAA\n"
  "    A_GABAB       : GABAB state variable to construct the dual-exponential profile - decays with conductance tau_r_GABAB\n"
  "    B_GABAB       : GABAB state variable to construct the dual-exponential profile - decays with conductance tau_d_GABAB\n"
  "}\n"
  "\n"
  "\n"
  "INITIAL{\n"
  "    LOCAL tp_GABAA, tp_GABAB\n"
  "\n"
  "    A_GABAA = 0\n"
  "    B_GABAA = 0\n"
  "\n"
  "    A_GABAB = 0\n"
  "    B_GABAB = 0\n"
  "\n"
  "    tp_GABAA = (tau_r_GABAA*tau_d_GABAA)/(tau_d_GABAA-tau_r_GABAA)*log(tau_d_GABAA/tau_r_GABAA) :time to peak of the conductance\n"
  "    tp_GABAB = (tau_r_GABAB*tau_d_GABAB)/(tau_d_GABAB-tau_r_GABAB)*log(tau_d_GABAB/tau_r_GABAB) :time to peak of the conductance\n"
  "\n"
  "    factor_GABAA = -exp(-tp_GABAA/tau_r_GABAA)+exp(-tp_GABAA/tau_d_GABAA) :GABAA Normalization factor - so that when t = tp_GABAA, gsyn = gpeak\n"
  "    factor_GABAA = 1/factor_GABAA\n"
  "\n"
  "    factor_GABAB = -exp(-tp_GABAB/tau_r_GABAB)+exp(-tp_GABAB/tau_d_GABAB) :GABAB Normalization factor - so that when t = tp_GABAB, gsyn = gpeak\n"
  "    factor_GABAB = 1/factor_GABAB\n"
  "\n"
  "    next_delay = -1\n"
  "\n"
  "}\n"
  "\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "    g_GABAA = gmax*(B_GABAA-A_GABAA) :compute time varying conductance as the difference of state variables B_GABAA and A_GABAA\n"
  "    g_GABAB = gmax*(B_GABAB-A_GABAB) :compute time varying conductance as the difference of state variables B_GABAB and A_GABAB\n"
  "    g = g_GABAA + g_GABAB\n"
  "    i_GABAA = g_GABAA*(v-e_GABAA) :compute the GABAA driving force based on the time varying conductance, membrane potential, and GABAA reversal\n"
  "    i_GABAB = g_GABAB*(v-e_GABAB) :compute the GABAB driving force based on the time varying conductance, membrane potential, and GABAB reversal\n"
  "    i = i_GABAA + i_GABAB\n"
  "}\n"
  "\n"
  "\n"
  "DERIVATIVE state{\n"
  "    A_GABAA' = -A_GABAA/tau_r_GABAA\n"
  "    B_GABAA' = -B_GABAA/tau_d_GABAA\n"
  "    A_GABAB' = -A_GABAB/tau_r_GABAB\n"
  "    B_GABAB' = -B_GABAB/tau_d_GABAB\n"
  "}\n"
  "\n"
  "\n"
  "NET_RECEIVE (weight, weight_GABAA, weight_GABAB, R, Pr, u, tsyn (ms), nc_type){\n"
  "    LOCAL result\n"
  "    weight_GABAA = weight\n"
  "    weight_GABAB = weight * GABAB_ratio\n"
  "\n"
  "    INITIAL{\n"
  "        R=1\n"
  "        u=u0\n"
  "        tsyn=t\n"
  "\n"
  "        if (nc_type == 0) {\n"
  "            : nc_type {\n"
  "            :   0 = presynaptic netcon\n"
  "            :   1 = spontmini netcon\n"
  "            :   2 = replay netcon\n"
  "            : }\n"
  "    VERBATIM\n"
  "            // setup self events for delayed connections to change weights\n"
  "            IvocVect *vv_delay_times = *((IvocVect**)(&_p_delay_times));\n"
  "            IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));\n"
  "            if (vv_delay_times && vector_capacity(vv_delay_times)>=1) {\n"
  "              double* deltm_el = vector_vec(vv_delay_times);\n"
  "              int delay_times_idx;\n"
  "              next_delay = 0;\n"
  "              for(delay_times_idx = 0; delay_times_idx < vector_capacity(vv_delay_times); ++delay_times_idx) {\n"
  "                double next_delay_t = deltm_el[delay_times_idx];\n"
  "    ENDVERBATIM\n"
  "                net_send(next_delay_t, 1)\n"
  "    VERBATIM\n"
  "              }\n"
  "            }\n"
  "    ENDVERBATIM\n"
  "        }\n"
  "    }\n"
  "\n"
  "    : Disable in case of t < 0 (in case of ForwardSkip) which causes numerical\n"
  "    : instability if synapses are activated.\n"
  "    if(t < 0 ) {\n"
  "    VERBATIM\n"
  "        return;\n"
  "    ENDVERBATIM\n"
  "    }\n"
  "\n"
  "    if (flag == 1) {\n"
  "        : self event to set next weight at delay\n"
  "    VERBATIM\n"
  "        // setup self events for delayed connections to change weights\n"
  "        IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));\n"
  "        if (vv_delay_weights && vector_capacity(vv_delay_weights)>=next_delay) {\n"
  "          double* weights_v = vector_vec(vv_delay_weights);\n"
  "          double next_delay_weight = weights_v[(int)next_delay];\n"
  "    ENDVERBATIM\n"
  "          weight = conductance*next_delay_weight\n"
  "          next_delay = next_delay + 1\n"
  "    VERBATIM\n"
  "        }\n"
  "        return;\n"
  "    ENDVERBATIM\n"
  "    }\n"
  "    : flag == 0, i.e. a spike has arrived\n"
  "\n"
  "    : calc u at event-\n"
  "    if (Fac > 0) {\n"
  "        u = u*exp(-(t - tsyn)/Fac) :update facilitation variable if Fac>0 Eq. 2 in Fuhrmann et al.\n"
  "    } else {\n"
  "        u = Use\n"
  "    }\n"
  "    if(Fac > 0){\n"
  "        u = u + Use*(1-u) :update facilitation variable if Fac>0 Eq. 2 in Fuhrmann et al.\n"
  "    }\n"
  "\n"
  "    R  = 1 - (1-R) * exp(-(t-tsyn)/Dep) :Probability R for a vesicle to be available for release, analogous to the pool of synaptic\n"
  "                                        :resources available for release in the deterministic model. Eq. 3 in Fuhrmann et al.\n"
  "    Pr  = u * R                         :Pr is calculated as R * u (running value of Use)\n"
  "    R  = R - u * R                      :update R as per Eq. 3 in Fuhrmann et al.\n"
  "\n"
  "    if( verboseLevel > 0 ) {\n"
  "        printf(\"Synapse %f at time %g: R = %g Pr = %g erand = %g\\n\", synapseID, t, R, Pr, result )\n"
  "    }\n"
  "\n"
  "    tsyn = t\n"
  "\n"
  "    A_GABAA = A_GABAA + Pr*weight_GABAA*factor_GABAA\n"
  "    B_GABAA = B_GABAA + Pr*weight_GABAA*factor_GABAA\n"
  "    A_GABAB = A_GABAB + Pr*weight_GABAB*factor_GABAB\n"
  "    B_GABAB = B_GABAB + Pr*weight_GABAB*factor_GABAB\n"
  "\n"
  "    if( verboseLevel > 0 ) {\n"
  "        printf( \" vals %g %g %g %g\\n\", A_GABAA, weight_GABAA, factor_GABAA, weight )\n"
  "    }\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION toggleVerbose() {\n"
  "    verboseLevel = 1-verboseLevel\n"
  "}\n"
  "\n"
  "\n"
  "VERBATIM\n"
  "static void bbcore_write(double* x, int* d, int* x_offset, int* d_offset, _threadargsproto_) {\n"
  "\n"
  "  IvocVect *vv_delay_times = *((IvocVect**)(&_p_delay_times));\n"
  "  IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));\n"
  "\n"
  "  // serialize connection delay vectors\n"
  "  if (vv_delay_times && vv_delay_weights &&\n"
  "     (vector_capacity(vv_delay_times) >= 1) && (vector_capacity(vv_delay_weights) >= 1)) {\n"
  "    if (d && x) {\n"
  "      int* d_i = d + *d_offset;\n"
  "      // store vector sizes for deserialization\n"
  "      d_i[0] = vector_capacity(vv_delay_times);\n"
  "      d_i[1] = vector_capacity(vv_delay_weights);\n"
  "\n"
  "      double* delay_times_el = vector_vec(vv_delay_times);\n"
  "      double* delay_weights_el = vector_vec(vv_delay_weights);\n"
  "      double* x_i = x + *x_offset;\n"
  "      int delay_vecs_idx;\n"
  "      int x_idx = 0;\n"
  "      for(delay_vecs_idx = 0; delay_vecs_idx < vector_capacity(vv_delay_times); ++delay_vecs_idx) {\n"
  "         x_i[x_idx++] = delay_times_el[delay_vecs_idx];\n"
  "         x_i[x_idx++] = delay_weights_el[delay_vecs_idx];\n"
  "      }\n"
  "    }\n"
  "  } else {\n"
  "    if (d) {\n"
  "      int* d_i = d + *d_offset;\n"
  "      d_i[0] = 0;\n"
  "      d_i[1] = 0;\n"
  "    }\n"
  "  }\n"
  "  // reserve space for delay connection vector sizes on serialization buffer\n"
  "  *d_offset += 2;\n"
  "\n"
  "  // reserve space for connection delay data on serialization buffer\n"
  "  if (vv_delay_times && vv_delay_weights) {\n"
  "    *x_offset += vector_capacity(vv_delay_times) + vector_capacity(vv_delay_weights);\n"
  "  }\n"
  "}\n"
  "\n"
  "static void bbcore_read(double* x, int* d, int* x_offset, int* d_offset, _threadargsproto_) {\n"
  "  assert(!_p_delay_times && !_p_delay_weights);\n"
  "\n"
  "  // first get delay vector sizes\n"
  "  int* d_i = d + *d_offset;\n"
  "  int delay_times_sz = d_i[0];\n"
  "  int delay_weights_sz = d_i[1];\n"
  "  *d_offset += 2;\n"
  "\n"
  "  if ((delay_times_sz > 0) && (delay_weights_sz > 0)) {\n"
  "    double* x_i = x + *x_offset;\n"
  "\n"
  "    // allocate vectors\n"
  "    _p_delay_times = (double*)vector_new1(delay_times_sz);\n"
  "    _p_delay_weights = (double*)vector_new1(delay_weights_sz);\n"
  "\n"
  "    double* delay_times_el = vector_vec((IvocVect*)_p_delay_times);\n"
  "    double* delay_weights_el = vector_vec((IvocVect*)_p_delay_weights);\n"
  "\n"
  "    // copy data\n"
  "    int x_idx;\n"
  "    int vec_idx = 0;\n"
  "    for(x_idx = 0; x_idx < delay_times_sz + delay_weights_sz; x_idx += 2) {\n"
  "      delay_times_el[vec_idx] = x_i[x_idx];\n"
  "      delay_weights_el[vec_idx++] = x_i[x_idx+1];\n"
  "    }\n"
  "    *x_offset += delay_times_sz + delay_weights_sz;\n"
  "  }\n"
  "}\n"
  "ENDVERBATIM\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
