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
static constexpr auto number_of_datum_variables = 5;
static constexpr auto number_of_floating_point_variables = 20;
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
 
#define nrn_init _nrn_init__Nap_Et2
#define _nrn_initial _nrn_initial__Nap_Et2
#define nrn_cur _nrn_cur__Nap_Et2
#define _nrn_current _nrn_current__Nap_Et2
#define nrn_jacob _nrn_jacob__Nap_Et2
#define nrn_state _nrn_state__Nap_Et2
#define _net_receive _net_receive__Nap_Et2 
#define rates rates__Nap_Et2 
#define states states__Nap_Et2 
 
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
#define gNap_Et2bar _ml->template fpfield<0>(_iml)
#define gNap_Et2bar_columnindex 0
#define ina _ml->template fpfield<1>(_iml)
#define ina_columnindex 1
#define gNap_Et2 _ml->template fpfield<2>(_iml)
#define gNap_Et2_columnindex 2
#define m _ml->template fpfield<3>(_iml)
#define m_columnindex 3
#define h _ml->template fpfield<4>(_iml)
#define h_columnindex 4
#define ttxo _ml->template fpfield<5>(_iml)
#define ttxo_columnindex 5
#define ttxi _ml->template fpfield<6>(_iml)
#define ttxi_columnindex 6
#define ena _ml->template fpfield<7>(_iml)
#define ena_columnindex 7
#define mInf _ml->template fpfield<8>(_iml)
#define mInf_columnindex 8
#define mTau _ml->template fpfield<9>(_iml)
#define mTau_columnindex 9
#define mAlpha _ml->template fpfield<10>(_iml)
#define mAlpha_columnindex 10
#define mBeta _ml->template fpfield<11>(_iml)
#define mBeta_columnindex 11
#define hInf _ml->template fpfield<12>(_iml)
#define hInf_columnindex 12
#define hTau _ml->template fpfield<13>(_iml)
#define hTau_columnindex 13
#define hAlpha _ml->template fpfield<14>(_iml)
#define hAlpha_columnindex 14
#define hBeta _ml->template fpfield<15>(_iml)
#define hBeta_columnindex 15
#define Dm _ml->template fpfield<16>(_iml)
#define Dm_columnindex 16
#define Dh _ml->template fpfield<17>(_iml)
#define Dh_columnindex 17
#define v _ml->template fpfield<18>(_iml)
#define v_columnindex 18
#define _g _ml->template fpfield<19>(_iml)
#define _g_columnindex 19
#define _ion_ena *(_ml->dptr_field<0>(_iml))
#define _p_ion_ena static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ina *(_ml->dptr_field<1>(_iml))
#define _p_ion_ina static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dinadv *(_ml->dptr_field<2>(_iml))
#define _ion_ttxo *(_ml->dptr_field<3>(_iml))
#define _p_ion_ttxo static_cast<neuron::container::data_handle<double>>(_ppvar[3])
#define _ion_ttxi *(_ml->dptr_field<4>(_iml))
#define _p_ion_ttxi static_cast<neuron::container::data_handle<double>>(_ppvar[4])
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_rates(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 _prop_id = _nrn_get_prop_id(_prop);
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {"setdata_Nap_Et2", _hoc_setdata},
 {"rates_Nap_Et2", _hoc_rates},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_rates(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"rates", _npy_rates},
 {0, 0}
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gNap_Et2bar_Nap_Et2", "S/cm2"},
 {"ina_Nap_Et2", "mA/cm2"},
 {"gNap_Et2_Nap_Et2", "S/cm2"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double h0 = 0;
 static double m0 = 0;
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
 static void nrn_cur(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_jacob(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[5].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"Nap_Et2",
 "gNap_Et2bar_Nap_Et2",
 0,
 "ina_Nap_Et2",
 "gNap_Et2_Nap_Et2",
 0,
 "m_Nap_Et2",
 "h_Nap_Et2",
 0,
 0};
 static Symbol* _na_sym;
 static Symbol* _ttx_sym;
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 6, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 20);
 	/*initialize range parameters*/
 	gNap_Et2bar = 1e-05;
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 20);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* ena */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ina */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dinadv */
 prop_ion = need_memb(_ttx_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[3] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* ttxo */
 	_ppvar[4] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* ttxi */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _Nap_Et2_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("ttx", 1.0);
 	_na_sym = hoc_lookup("na_ion");
 	_ttx_sym = hoc_lookup("ttx_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gNap_Et2bar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"ina"} /* 1 */,
                                       _nrn_mechanism_field<double>{"gNap_Et2"} /* 2 */,
                                       _nrn_mechanism_field<double>{"m"} /* 3 */,
                                       _nrn_mechanism_field<double>{"h"} /* 4 */,
                                       _nrn_mechanism_field<double>{"ttxo"} /* 5 */,
                                       _nrn_mechanism_field<double>{"ttxi"} /* 6 */,
                                       _nrn_mechanism_field<double>{"ena"} /* 7 */,
                                       _nrn_mechanism_field<double>{"mInf"} /* 8 */,
                                       _nrn_mechanism_field<double>{"mTau"} /* 9 */,
                                       _nrn_mechanism_field<double>{"mAlpha"} /* 10 */,
                                       _nrn_mechanism_field<double>{"mBeta"} /* 11 */,
                                       _nrn_mechanism_field<double>{"hInf"} /* 12 */,
                                       _nrn_mechanism_field<double>{"hTau"} /* 13 */,
                                       _nrn_mechanism_field<double>{"hAlpha"} /* 14 */,
                                       _nrn_mechanism_field<double>{"hBeta"} /* 15 */,
                                       _nrn_mechanism_field<double>{"Dm"} /* 16 */,
                                       _nrn_mechanism_field<double>{"Dh"} /* 17 */,
                                       _nrn_mechanism_field<double>{"v"} /* 18 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 19 */,
                                       _nrn_mechanism_field<double*>{"_ion_ena", "na_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ina", "na_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dinadv", "na_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"_ion_ttxo", "ttx_ion"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_ion_ttxi", "ttx_ion"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 20, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ttx_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ttx_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 Nap_Et2 Nap_Et2.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_internalthreadargsproto_);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int states(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   if ( ttxi  == 0.015625  && ttxo > 1e-12 ) {
     mInf = 0.0 ;
     mTau = 1e-12 ;
     hInf = 1.0 ;
     hTau = 1e-12 ;
     }
   else {
     rates ( _threadargs_ ) ;
     }
   Dm = ( mInf - m ) / mTau ;
   Dh = ( hInf - h ) / hTau ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 if ( ttxi  == 0.015625  && ttxo > 1e-12 ) {
   mInf = 0.0 ;
   mTau = 1e-12 ;
   hInf = 1.0 ;
   hTau = 1e-12 ;
   }
 else {
   rates ( _threadargs_ ) ;
   }
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / mTau )) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / hTau )) ;
  return 0;
}
 /*END CVODE*/
 static int states (_internalthreadargsproto_) { {
   if ( ttxi  == 0.015625  && ttxo > 1e-12 ) {
     mInf = 0.0 ;
     mTau = 1e-12 ;
     hInf = 1.0 ;
     hTau = 1e-12 ;
     }
   else {
     rates ( _threadargs_ ) ;
     }
    m = m + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / mTau)))*(- ( ( ( mInf ) ) / mTau ) / ( ( ( ( - 1.0 ) ) ) / mTau ) - m) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / hTau)))*(- ( ( ( hInf ) ) / hTau ) / ( ( ( ( - 1.0 ) ) ) / hTau ) - h) ;
   }
  return 0;
}
 
static int  rates ( _internalthreadargsproto_ ) {
   double _lqt ;
 _lqt = pow( 2.3 , ( ( 34.0 - 21.0 ) / 10.0 ) ) ;
    mInf = 1.0 / ( 1.0 + exp ( ( v - - 52.6 ) / - 4.6 ) ) ;
   if ( v  == - 38.0 ) {
     v = v + 0.0001 ;
     }
   mAlpha = ( 0.182 * ( v - - 38.0 ) ) / ( 1.0 - ( exp ( - ( v - - 38.0 ) / 6.0 ) ) ) ;
   mBeta = ( 0.124 * ( - v - 38.0 ) ) / ( 1.0 - ( exp ( - ( - v - 38.0 ) / 6.0 ) ) ) ;
   mTau = 6.0 * ( 1.0 / ( mAlpha + mBeta ) ) / _lqt ;
   if ( v  == - 17.0 ) {
     v = v + 0.0001 ;
     }
   if ( v  == - 64.4 ) {
     v = v + 0.0001 ;
     }
   hInf = 1.0 / ( 1.0 + exp ( ( v - - 48.8 ) / 10.0 ) ) ;
   hAlpha = - 2.88e-6 * ( v + 17.0 ) / ( 1.0 - exp ( ( v + 17.0 ) / 4.63 ) ) ;
   hBeta = 6.94e-6 * ( v + 64.4 ) / ( 1.0 - exp ( - ( v + 64.4 ) / 2.63 ) ) ;
   hTau = ( 1.0 / ( hAlpha + hBeta ) ) / _lqt ;
     return 0; }
 
static void _hoc_rates(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  Prop* _local_prop = _prop_id ? _extcall_prop : nullptr;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r = 1.;
 rates ( _threadargs_ );
 hoc_retpushx(_r);
}
 
static double _npy_rates(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r = 1.;
 rates ( _threadargs_ );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
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
  ena = _ion_ena;
  ttxo = _ion_ttxo;
  ttxi = _ion_ttxi;
     _ode_spec1 (_threadargs_);
  }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  Datum* _ppvar;
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 2; ++_i) {
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
  ena = _ion_ena;
  ttxo = _ion_ttxo;
  ttxi = _ion_ttxi;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  h = h0;
  m = m0;
 {
   if ( ttxi  == 0.015625  && ttxo > 1e-12 ) {
     mInf = 0.0 ;
     mTau = 1e-12 ;
     hInf = 1.0 ;
     hTau = 1e-12 ;
     }
   else {
     rates ( _threadargs_ ) ;
     }
   m = mInf ;
   h = hInf ;
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
   _v = _vec_v[_ni[_iml]];
 v = _v;
  ena = _ion_ena;
  ttxo = _ion_ttxo;
  ttxi = _ion_ttxi;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   gNap_Et2 = gNap_Et2bar * m * m * m * h ;
   ina = gNap_Et2 * ( v - ena ) ;
   }
 _current += ina;

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
  ena = _ion_ena;
  ttxo = _ion_ttxo;
  ttxi = _ion_ttxi;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dina;
  _dina = ina;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dinadv += (_dina - ina)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ina += ina ;
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
  ena = _ion_ena;
  ttxo = _ion_ttxo;
  ttxi = _ion_ttxi;
 {   states(_threadargs_);
  } }}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {m_columnindex, 0};  _dlist1[0] = {Dm_columnindex, 0};
 _slist1[1] = {h_columnindex, 0};  _dlist1[1] = {Dh_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "Nap_Et2.mod";
    const char* nmodl_file_text = 
  ":Comment : mtau deduced from text (said to be 6 times faster than for NaTa)\n"
  ":Comment : so I used the equations from NaT and multiplied by 6\n"
  ":Reference : Modeled according to kinetics derived from Magistretti & Alonso 1999\n"
  ":Comment: corrected rates using q10 = 2.3, target temperature 34, orginal 21\n"
  "\n"
  ": Adapted by Werner Van Geit @ BBP, 2015 (with help from M.Hines):\n"
  ": channel detects TTX concentration set by TTXDynamicsSwitch.mod\n"
  "\n"
  ": LJP: OK, cell-attached modes bath and pipette solution almost identical ljp < 1mV \"this procedure can be considered as valid in the case of a cell-attached or inside-out patch measurement provided that the pipette contains a solution identical to that of the bath\" Erwin Neher, 1992\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX Nap_Et2\n"
  "	USEION na READ ena WRITE ina\n"
  "	USEION ttx READ ttxo, ttxi VALENCE 1\n"
  "	RANGE gNap_Et2bar, gNap_Et2, ina\n"
  "}\n"
  "\n"
  "UNITS	{\n"
  "	(S) = (siemens)\n"
  "	(mV) = (millivolt)\n"
  "	(mA) = (milliamp)\n"
  "}\n"
  "\n"
  "PARAMETER	{\n"
  "	gNap_Et2bar = 0.00001 (S/cm2)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	ttxo (mM)\n"
  "	ttxi (mM)\n"
  "	v	(mV)\n"
  "	ena	(mV)\n"
  "	ina	(mA/cm2)\n"
  "	gNap_Et2	(S/cm2)\n"
  "	mInf\n"
  "	mTau\n"
  "	mAlpha\n"
  "	mBeta\n"
  "	hInf\n"
  "	hTau\n"
  "	hAlpha\n"
  "	hBeta\n"
  "}\n"
  "\n"
  "STATE	{\n"
  "	m\n"
  "	h\n"
  "}\n"
  "\n"
  "BREAKPOINT	{\n"
  "	SOLVE states METHOD cnexp\n"
  "	gNap_Et2 = gNap_Et2bar*m*m*m*h\n"
  "	ina = gNap_Et2*(v-ena)\n"
  "}\n"
  "\n"
  "DERIVATIVE states	{\n"
  "	if (ttxi == 0.015625 && ttxo > 1e-12) {\n"
  "		mInf = 0.0\n"
  "		mTau = 1e-12\n"
  "		hInf = 1.0\n"
  "		hTau = 1e-12\n"
  "	} else {\n"
  "		rates()\n"
  "	}\n"
  "	m' = (mInf-m)/mTau\n"
  "	h' = (hInf-h)/hTau\n"
  "}\n"
  "\n"
  "INITIAL{\n"
  "	if (ttxi == 0.015625 && ttxo > 1e-12) {\n"
  "		mInf = 0.0\n"
  "		mTau = 1e-12\n"
  "		hInf = 1.0\n"
  "		hTau = 1e-12\n"
  "	} else {\n"
  "		rates()\n"
  "	}\n"
  "	m = mInf\n"
  "	h = hInf\n"
  "}\n"
  "\n"
  "PROCEDURE rates(){\n"
  "  LOCAL qt\n"
  "  qt = 2.3^((34-21)/10)\n"
  "\n"
  "	UNITSOFF\n"
  "		mInf = 1.0/(1+exp((v- -52.6)/-4.6))\n"
  "    if(v == -38){\n"
  "    	v = v+0.0001\n"
  "    }\n"
  "		mAlpha = (0.182 * (v- -38))/(1-(exp(-(v- -38)/6)))\n"
  "		mBeta  = (0.124 * (-v -38))/(1-(exp(-(-v -38)/6)))\n"
  "		mTau = 6*(1/(mAlpha + mBeta))/qt\n"
  "\n"
  "  	if(v == -17){\n"
  "   		v = v + 0.0001\n"
  "  	}\n"
  "    if(v == -64.4){\n"
  "      v = v+0.0001\n"
  "    }\n"
  "\n"
  "		hInf = 1.0/(1+exp((v- -48.8)/10))\n"
  "    hAlpha = -2.88e-6 * (v + 17) / (1 - exp((v + 17)/4.63))\n"
  "    hBeta = 6.94e-6 * (v + 64.4) / (1 - exp(-(v + 64.4)/2.63))\n"
  "		hTau = (1/(hAlpha + hBeta))/qt\n"
  "	UNITSON\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
