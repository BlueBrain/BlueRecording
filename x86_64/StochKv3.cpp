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
static constexpr auto number_of_floating_point_variables = 40;
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
 
#define nrn_init _nrn_init__StochKv3
#define _nrn_initial _nrn_initial__StochKv3
#define nrn_cur _nrn_cur__StochKv3
#define _nrn_current _nrn_current__StochKv3
#define nrn_jacob _nrn_jacob__StochKv3
#define nrn_state _nrn_state__StochKv3
#define _net_receive _net_receive__StochKv3 
#define ChkProb ChkProb__StochKv3 
#define _f_trates _f_trates__StochKv3 
#define setRNG setRNG__StochKv3 
#define states states__StochKv3 
#define trates trates__StochKv3 
 
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
#define gamma _ml->template fpfield<0>(_iml)
#define gamma_columnindex 0
#define eta _ml->template fpfield<1>(_iml)
#define eta_columnindex 1
#define gkbar _ml->template fpfield<2>(_iml)
#define gkbar_columnindex 2
#define deterministic _ml->template fpfield<3>(_iml)
#define deterministic_columnindex 3
#define an _ml->template fpfield<4>(_iml)
#define an_columnindex 4
#define bn _ml->template fpfield<5>(_iml)
#define bn_columnindex 5
#define al _ml->template fpfield<6>(_iml)
#define al_columnindex 6
#define bl _ml->template fpfield<7>(_iml)
#define bl_columnindex 7
#define ik _ml->template fpfield<8>(_iml)
#define ik_columnindex 8
#define gk _ml->template fpfield<9>(_iml)
#define gk_columnindex 9
#define ninf _ml->template fpfield<10>(_iml)
#define ninf_columnindex 10
#define ntau _ml->template fpfield<11>(_iml)
#define ntau_columnindex 11
#define linf _ml->template fpfield<12>(_iml)
#define linf_columnindex 12
#define ltau _ml->template fpfield<13>(_iml)
#define ltau_columnindex 13
#define N _ml->template fpfield<14>(_iml)
#define N_columnindex 14
#define P_an _ml->template fpfield<15>(_iml)
#define P_an_columnindex 15
#define P_bn _ml->template fpfield<16>(_iml)
#define P_bn_columnindex 16
#define P_al _ml->template fpfield<17>(_iml)
#define P_al_columnindex 17
#define P_bl _ml->template fpfield<18>(_iml)
#define P_bl_columnindex 18
#define n _ml->template fpfield<19>(_iml)
#define n_columnindex 19
#define l _ml->template fpfield<20>(_iml)
#define l_columnindex 20
#define ek _ml->template fpfield<21>(_iml)
#define ek_columnindex 21
#define scale_dens _ml->template fpfield<22>(_iml)
#define scale_dens_columnindex 22
#define usingR123 _ml->template fpfield<23>(_iml)
#define usingR123_columnindex 23
#define Dn _ml->template fpfield<24>(_iml)
#define Dn_columnindex 24
#define Dl _ml->template fpfield<25>(_iml)
#define Dl_columnindex 25
#define N0L0 _ml->template fpfield<26>(_iml)
#define N0L0_columnindex 26
#define N1L0 _ml->template fpfield<27>(_iml)
#define N1L0_columnindex 27
#define N0L1 _ml->template fpfield<28>(_iml)
#define N0L1_columnindex 28
#define N1L1 _ml->template fpfield<29>(_iml)
#define N1L1_columnindex 29
#define n0l0_n1l0 _ml->template fpfield<30>(_iml)
#define n0l0_n1l0_columnindex 30
#define n0l0_n0l1 _ml->template fpfield<31>(_iml)
#define n0l0_n0l1_columnindex 31
#define n1l0_n1l1 _ml->template fpfield<32>(_iml)
#define n1l0_n1l1_columnindex 32
#define n1l0_n0l0 _ml->template fpfield<33>(_iml)
#define n1l0_n0l0_columnindex 33
#define n0l1_n1l1 _ml->template fpfield<34>(_iml)
#define n0l1_n1l1_columnindex 34
#define n0l1_n0l0 _ml->template fpfield<35>(_iml)
#define n0l1_n0l0_columnindex 35
#define n1l1_n0l1 _ml->template fpfield<36>(_iml)
#define n1l1_n0l1_columnindex 36
#define n1l1_n1l0 _ml->template fpfield<37>(_iml)
#define n1l1_n1l0_columnindex 37
#define v _ml->template fpfield<38>(_iml)
#define v_columnindex 38
#define _g _ml->template fpfield<39>(_iml)
#define _g_columnindex 39
#define _ion_ek *(_ml->dptr_field<0>(_iml))
#define _p_ion_ek static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ik *(_ml->dptr_field<1>(_iml))
#define _p_ion_ik static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dikdv *(_ml->dptr_field<2>(_iml))
#define rng	*_ppvar[3].get<double*>()
#define _p_rng _ppvar[3].literal_value<void*>()
#define area	*_ppvar[4].get<double*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  3;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_BnlDev(void);
 static void _hoc_ChkProb(void);
 static void _hoc_bbsavestate(void);
 static void _hoc_brand(void);
 static void _hoc_setRNG(void);
 static void _hoc_strap(void);
 static void _hoc_trates(void);
 static void _hoc_urand(void);
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
 {"setdata_StochKv3", _hoc_setdata},
 {"BnlDev_StochKv3", _hoc_BnlDev},
 {"ChkProb_StochKv3", _hoc_ChkProb},
 {"bbsavestate_StochKv3", _hoc_bbsavestate},
 {"brand_StochKv3", _hoc_brand},
 {"setRNG_StochKv3", _hoc_setRNG},
 {"strap_StochKv3", _hoc_strap},
 {"trates_StochKv3", _hoc_trates},
 {"urand_StochKv3", _hoc_urand},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_BnlDev(Prop*);
 static double _npy_ChkProb(Prop*);
 static double _npy_bbsavestate(Prop*);
 static double _npy_brand(Prop*);
 static double _npy_setRNG(Prop*);
 static double _npy_strap(Prop*);
 static double _npy_trates(Prop*);
 static double _npy_urand(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"BnlDev", _npy_BnlDev},
 {"ChkProb", _npy_ChkProb},
 {"bbsavestate", _npy_bbsavestate},
 {"brand", _npy_brand},
 {"setRNG", _npy_setRNG},
 {"strap", _npy_strap},
 {"trates", _npy_trates},
 {"urand", _npy_urand},
 {0, 0}
};
#define BnlDev BnlDev_StochKv3
#define bbsavestate bbsavestate_StochKv3
#define brand brand_StochKv3
#define strap strap_StochKv3
#define urand urand_StochKv3
 extern double BnlDev( _internalthreadargsprotocomma_ double , double );
 extern double bbsavestate( _internalthreadargsproto_ );
 extern double brand( _internalthreadargsprotocomma_ double , double );
 extern double strap( _internalthreadargsprotocomma_ double );
 extern double urand( _internalthreadargsproto_ );
 
static void _check_trates(_internalthreadargsproto_); 
static void _check_table_thread(_threadargsprotocomma_ int _type, _nrn_model_sorted_token const& _sorted_token) {
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml, _type};
  {
    auto* const _ml = &_lmr;
   _check_trates(_threadargs_);
   }
}
 /* declare global and static user variables */
#define usetable usetable_StochKv3
 double usetable = 1;
#define vmax vmax_StochKv3
 double vmax = 100;
#define vmin vmin_StochKv3
 double vmin = -120;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {"usetable_StochKv3", 0, 1},
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"vmin_StochKv3", "mV"},
 {"vmax_StochKv3", "mV"},
 {"gamma_StochKv3", "pS"},
 {"eta_StochKv3", "1/um2"},
 {"gkbar_StochKv3", "S/cm2"},
 {"an_StochKv3", "/ms"},
 {"bn_StochKv3", "/ms"},
 {"al_StochKv3", "/ms"},
 {"bl_StochKv3", "/ms"},
 {"ik_StochKv3", "mA/cm2"},
 {"gk_StochKv3", "S/cm2"},
 {"ntau_StochKv3", "ms"},
 {"ltau_StochKv3", "ms"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double l0 = 0;
 static double n0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"vmin_StochKv3", &vmin_StochKv3},
 {"vmax_StochKv3", &vmax_StochKv3},
 {"usetable_StochKv3", &usetable_StochKv3},
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
"StochKv3",
 "gamma_StochKv3",
 "eta_StochKv3",
 "gkbar_StochKv3",
 "deterministic_StochKv3",
 0,
 "an_StochKv3",
 "bn_StochKv3",
 "al_StochKv3",
 "bl_StochKv3",
 "ik_StochKv3",
 "gk_StochKv3",
 "ninf_StochKv3",
 "ntau_StochKv3",
 "linf_StochKv3",
 "ltau_StochKv3",
 "N_StochKv3",
 "P_an_StochKv3",
 "P_bn_StochKv3",
 "P_al_StochKv3",
 "P_bl_StochKv3",
 0,
 "n_StochKv3",
 "l_StochKv3",
 0,
 "rng_StochKv3",
 0};
 extern Node* nrn_alloc_node_;
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 6, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 40);
 	/*initialize range parameters*/
 	gamma = 50;
 	eta = 0;
 	gkbar = 0.01;
 	deterministic = 0;
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 40);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 	_ppvar[4] = _nrn_mechanism_get_area_handle(nrn_alloc_node_);
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* ek */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ik */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 static void bbcore_write(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_write(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 static void bbcore_read(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_read(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _StochKv3_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_table_reg(_mechtype, _check_table_thread);
   hoc_reg_bbcore_write(_mechtype, bbcore_write);
   hoc_reg_bbcore_read(_mechtype, bbcore_read);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gamma"} /* 0 */,
                                       _nrn_mechanism_field<double>{"eta"} /* 1 */,
                                       _nrn_mechanism_field<double>{"gkbar"} /* 2 */,
                                       _nrn_mechanism_field<double>{"deterministic"} /* 3 */,
                                       _nrn_mechanism_field<double>{"an"} /* 4 */,
                                       _nrn_mechanism_field<double>{"bn"} /* 5 */,
                                       _nrn_mechanism_field<double>{"al"} /* 6 */,
                                       _nrn_mechanism_field<double>{"bl"} /* 7 */,
                                       _nrn_mechanism_field<double>{"ik"} /* 8 */,
                                       _nrn_mechanism_field<double>{"gk"} /* 9 */,
                                       _nrn_mechanism_field<double>{"ninf"} /* 10 */,
                                       _nrn_mechanism_field<double>{"ntau"} /* 11 */,
                                       _nrn_mechanism_field<double>{"linf"} /* 12 */,
                                       _nrn_mechanism_field<double>{"ltau"} /* 13 */,
                                       _nrn_mechanism_field<double>{"N"} /* 14 */,
                                       _nrn_mechanism_field<double>{"P_an"} /* 15 */,
                                       _nrn_mechanism_field<double>{"P_bn"} /* 16 */,
                                       _nrn_mechanism_field<double>{"P_al"} /* 17 */,
                                       _nrn_mechanism_field<double>{"P_bl"} /* 18 */,
                                       _nrn_mechanism_field<double>{"n"} /* 19 */,
                                       _nrn_mechanism_field<double>{"l"} /* 20 */,
                                       _nrn_mechanism_field<double>{"ek"} /* 21 */,
                                       _nrn_mechanism_field<double>{"scale_dens"} /* 22 */,
                                       _nrn_mechanism_field<double>{"usingR123"} /* 23 */,
                                       _nrn_mechanism_field<double>{"Dn"} /* 24 */,
                                       _nrn_mechanism_field<double>{"Dl"} /* 25 */,
                                       _nrn_mechanism_field<double>{"N0L0"} /* 26 */,
                                       _nrn_mechanism_field<double>{"N1L0"} /* 27 */,
                                       _nrn_mechanism_field<double>{"N0L1"} /* 28 */,
                                       _nrn_mechanism_field<double>{"N1L1"} /* 29 */,
                                       _nrn_mechanism_field<double>{"n0l0_n1l0"} /* 30 */,
                                       _nrn_mechanism_field<double>{"n0l0_n0l1"} /* 31 */,
                                       _nrn_mechanism_field<double>{"n1l0_n1l1"} /* 32 */,
                                       _nrn_mechanism_field<double>{"n1l0_n0l0"} /* 33 */,
                                       _nrn_mechanism_field<double>{"n0l1_n1l1"} /* 34 */,
                                       _nrn_mechanism_field<double>{"n0l1_n0l0"} /* 35 */,
                                       _nrn_mechanism_field<double>{"n1l1_n0l1"} /* 36 */,
                                       _nrn_mechanism_field<double>{"n1l1_n1l0"} /* 37 */,
                                       _nrn_mechanism_field<double>{"v"} /* 38 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 39 */,
                                       _nrn_mechanism_field<double*>{"_ion_ek", "k_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ik", "k_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dikdv", "k_ion"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"rng", "bbcorepointer"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"area", "area"} /* 4 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 5 */);
  hoc_register_prop_size(_mechtype, 40, 6);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 5, "cvodeieq");
  hoc_register_dparam_semantics(_mechtype, 4, "area");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 StochKv3 StochKv3.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double *_t_ntau;
 static double *_t_ltau;
 static double *_t_ninf;
 static double *_t_linf;
 static double *_t_al;
 static double *_t_bl;
 static double *_t_an;
 static double *_t_bn;
static int _reset;
static const char *modelname = "StochKv3.mod";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int ChkProb(_internalthreadargsprotocomma_ double);
static int _f_trates(_internalthreadargsprotocomma_ double);
static int setRNG(_internalthreadargsproto_);
static int trates(_internalthreadargsprotocomma_ double);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static void _n_trates(_internalthreadargsprotocomma_ double _lv);
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int states(_internalthreadargsproto_);
 
/*VERBATIM*/
#ifndef NRN_VERSION_GTEQ_8_2_0
#include "nrnran123.h"
extern int cvode_active_;

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef CORENEURON_BUILD
double nrn_random_pick(void* r);
void* nrn_random_arg(int argpos);
#endif
#define RANDCAST
#else
#define RANDCAST (Rand*)
#endif

 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   trates ( _threadargscomma_ v ) ;
   Dl = al - ( al + bl ) * l ;
   Dn = an - ( an + bn ) * n ;
   if ( deterministic  || dt > 1.0 ) {
     N1L1 = n * l * N ;
     N1L0 = n * ( 1.0 - l ) * N ;
     N0L1 = ( 1.0 - n ) * l * N ;
     }
   else {
     N1L1 = floor ( N1L1 + 0.5 ) ;
     N1L0 = floor ( N1L0 + 0.5 ) ;
     N0L1 = floor ( N0L1 + 0.5 ) ;
     N0L0 = N - N1L1 - N1L0 - N0L1 ;
     P_an = strap ( _threadargscomma_ an * dt ) ;
     P_bn = strap ( _threadargscomma_ bn * dt ) ;
     ChkProb ( _threadargscomma_ P_an ) ;
     ChkProb ( _threadargscomma_ P_bn ) ;
     n0l0_n1l0 = BnlDev ( _threadargscomma_ P_an , N0L0 ) ;
     n0l1_n1l1 = BnlDev ( _threadargscomma_ P_an , N0L1 ) ;
     n1l1_n0l1 = BnlDev ( _threadargscomma_ P_bn , N1L1 ) ;
     n1l0_n0l0 = BnlDev ( _threadargscomma_ P_bn , N1L0 ) ;
     N0L0 = strap ( _threadargscomma_ N0L0 - n0l0_n1l0 + n1l0_n0l0 ) ;
     N1L0 = strap ( _threadargscomma_ N1L0 - n1l0_n0l0 + n0l0_n1l0 ) ;
     N0L1 = strap ( _threadargscomma_ N0L1 - n0l1_n1l1 + n1l1_n0l1 ) ;
     N1L1 = strap ( _threadargscomma_ N1L1 - n1l1_n0l1 + n0l1_n1l1 ) ;
     P_al = strap ( _threadargscomma_ al * dt ) ;
     P_bl = strap ( _threadargscomma_ bl * dt ) ;
     ChkProb ( _threadargscomma_ P_al ) ;
     ChkProb ( _threadargscomma_ P_bl ) ;
     n0l0_n0l1 = BnlDev ( _threadargscomma_ P_al , N0L0 - n0l0_n1l0 ) ;
     n1l0_n1l1 = BnlDev ( _threadargscomma_ P_al , N1L0 - n1l0_n0l0 ) ;
     n0l1_n0l0 = BnlDev ( _threadargscomma_ P_bl , N0L1 - n0l1_n1l1 ) ;
     n1l1_n1l0 = BnlDev ( _threadargscomma_ P_bl , N1L1 - n1l1_n0l1 ) ;
     N0L0 = strap ( _threadargscomma_ N0L0 - n0l0_n0l1 + n0l1_n0l0 ) ;
     N1L0 = strap ( _threadargscomma_ N1L0 - n1l0_n1l1 + n1l1_n1l0 ) ;
     N0L1 = strap ( _threadargscomma_ N0L1 - n0l1_n0l0 + n0l0_n0l1 ) ;
     N1L1 = strap ( _threadargscomma_ N1L1 - n1l1_n1l0 + n1l0_n1l1 ) ;
     }
   N0L0 = N - N1L1 - N1L0 - N0L1 ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 trates ( _threadargscomma_ v ) ;
 Dl = Dl  / (1. - dt*( ( - ( ( al + bl ) )*( 1.0 ) ) )) ;
 Dn = Dn  / (1. - dt*( ( - ( ( an + bn ) )*( 1.0 ) ) )) ;
 if ( deterministic  || dt > 1.0 ) {
   N1L1 = n * l * N ;
   N1L0 = n * ( 1.0 - l ) * N ;
   N0L1 = ( 1.0 - n ) * l * N ;
   }
 else {
   N1L1 = floor ( N1L1 + 0.5 ) ;
   N1L0 = floor ( N1L0 + 0.5 ) ;
   N0L1 = floor ( N0L1 + 0.5 ) ;
   N0L0 = N - N1L1 - N1L0 - N0L1 ;
   P_an = strap ( _threadargscomma_ an * dt ) ;
   P_bn = strap ( _threadargscomma_ bn * dt ) ;
   ChkProb ( _threadargscomma_ P_an ) ;
   ChkProb ( _threadargscomma_ P_bn ) ;
   n0l0_n1l0 = BnlDev ( _threadargscomma_ P_an , N0L0 ) ;
   n0l1_n1l1 = BnlDev ( _threadargscomma_ P_an , N0L1 ) ;
   n1l1_n0l1 = BnlDev ( _threadargscomma_ P_bn , N1L1 ) ;
   n1l0_n0l0 = BnlDev ( _threadargscomma_ P_bn , N1L0 ) ;
   N0L0 = strap ( _threadargscomma_ N0L0 - n0l0_n1l0 + n1l0_n0l0 ) ;
   N1L0 = strap ( _threadargscomma_ N1L0 - n1l0_n0l0 + n0l0_n1l0 ) ;
   N0L1 = strap ( _threadargscomma_ N0L1 - n0l1_n1l1 + n1l1_n0l1 ) ;
   N1L1 = strap ( _threadargscomma_ N1L1 - n1l1_n0l1 + n0l1_n1l1 ) ;
   P_al = strap ( _threadargscomma_ al * dt ) ;
   P_bl = strap ( _threadargscomma_ bl * dt ) ;
   ChkProb ( _threadargscomma_ P_al ) ;
   ChkProb ( _threadargscomma_ P_bl ) ;
   n0l0_n0l1 = BnlDev ( _threadargscomma_ P_al , N0L0 - n0l0_n1l0 ) ;
   n1l0_n1l1 = BnlDev ( _threadargscomma_ P_al , N1L0 - n1l0_n0l0 ) ;
   n0l1_n0l0 = BnlDev ( _threadargscomma_ P_bl , N0L1 - n0l1_n1l1 ) ;
   n1l1_n1l0 = BnlDev ( _threadargscomma_ P_bl , N1L1 - n1l1_n0l1 ) ;
   N0L0 = strap ( _threadargscomma_ N0L0 - n0l0_n0l1 + n0l1_n0l0 ) ;
   N1L0 = strap ( _threadargscomma_ N1L0 - n1l0_n1l1 + n1l1_n1l0 ) ;
   N0L1 = strap ( _threadargscomma_ N0L1 - n0l1_n0l0 + n0l0_n0l1 ) ;
   N1L1 = strap ( _threadargscomma_ N1L1 - n1l1_n1l0 + n1l0_n1l1 ) ;
   }
 N0L0 = N - N1L1 - N1L0 - N0L1 ;
  return 0;
}
 /*END CVODE*/
 static int states (_internalthreadargsproto_) { {
   trates ( _threadargscomma_ v ) ;
    l = l + (1. - exp(dt*(( - ( ( al + bl ) )*( 1.0 ) ))))*(- ( al ) / ( ( - ( ( al + bl ) )*( 1.0 ) ) ) - l) ;
    n = n + (1. - exp(dt*(( - ( ( an + bn ) )*( 1.0 ) ))))*(- ( an ) / ( ( - ( ( an + bn ) )*( 1.0 ) ) ) - n) ;
   if ( deterministic  || dt > 1.0 ) {
     N1L1 = n * l * N ;
     N1L0 = n * ( 1.0 - l ) * N ;
     N0L1 = ( 1.0 - n ) * l * N ;
     }
   else {
     N1L1 = floor ( N1L1 + 0.5 ) ;
     N1L0 = floor ( N1L0 + 0.5 ) ;
     N0L1 = floor ( N0L1 + 0.5 ) ;
     N0L0 = N - N1L1 - N1L0 - N0L1 ;
     P_an = strap ( _threadargscomma_ an * dt ) ;
     P_bn = strap ( _threadargscomma_ bn * dt ) ;
     ChkProb ( _threadargscomma_ P_an ) ;
     ChkProb ( _threadargscomma_ P_bn ) ;
     n0l0_n1l0 = BnlDev ( _threadargscomma_ P_an , N0L0 ) ;
     n0l1_n1l1 = BnlDev ( _threadargscomma_ P_an , N0L1 ) ;
     n1l1_n0l1 = BnlDev ( _threadargscomma_ P_bn , N1L1 ) ;
     n1l0_n0l0 = BnlDev ( _threadargscomma_ P_bn , N1L0 ) ;
     N0L0 = strap ( _threadargscomma_ N0L0 - n0l0_n1l0 + n1l0_n0l0 ) ;
     N1L0 = strap ( _threadargscomma_ N1L0 - n1l0_n0l0 + n0l0_n1l0 ) ;
     N0L1 = strap ( _threadargscomma_ N0L1 - n0l1_n1l1 + n1l1_n0l1 ) ;
     N1L1 = strap ( _threadargscomma_ N1L1 - n1l1_n0l1 + n0l1_n1l1 ) ;
     P_al = strap ( _threadargscomma_ al * dt ) ;
     P_bl = strap ( _threadargscomma_ bl * dt ) ;
     ChkProb ( _threadargscomma_ P_al ) ;
     ChkProb ( _threadargscomma_ P_bl ) ;
     n0l0_n0l1 = BnlDev ( _threadargscomma_ P_al , N0L0 - n0l0_n1l0 ) ;
     n1l0_n1l1 = BnlDev ( _threadargscomma_ P_al , N1L0 - n1l0_n0l0 ) ;
     n0l1_n0l0 = BnlDev ( _threadargscomma_ P_bl , N0L1 - n0l1_n1l1 ) ;
     n1l1_n1l0 = BnlDev ( _threadargscomma_ P_bl , N1L1 - n1l1_n0l1 ) ;
     N0L0 = strap ( _threadargscomma_ N0L0 - n0l0_n0l1 + n0l1_n0l0 ) ;
     N1L0 = strap ( _threadargscomma_ N1L0 - n1l0_n1l1 + n1l1_n1l0 ) ;
     N0L1 = strap ( _threadargscomma_ N0L1 - n0l1_n0l0 + n0l0_n0l1 ) ;
     N1L1 = strap ( _threadargscomma_ N1L1 - n1l1_n1l0 + n1l0_n1l1 ) ;
     }
   N0L0 = N - N1L1 - N1L0 - N0L1 ;
   }
  return 0;
}
 static double _mfac_trates, _tmin_trates;
  static void _check_trates(_internalthreadargsproto_) {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  static double _sav_dt;
  if (!usetable) {return;}
  if (_sav_dt != dt) { _maktable = 1;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_trates =  vmin ;
   _tmax =  vmax ;
   _dx = (_tmax - _tmin_trates)/199.; _mfac_trates = 1./_dx;
   for (_i=0, _x=_tmin_trates; _i < 200; _x += _dx, _i++) {
    _f_trates(_threadargscomma_ _x);
    _t_ntau[_i] = ntau;
    _t_ltau[_i] = ltau;
    _t_ninf[_i] = ninf;
    _t_linf[_i] = linf;
    _t_al[_i] = al;
    _t_bl[_i] = bl;
    _t_an[_i] = an;
    _t_bn[_i] = bn;
   }
   _sav_dt = dt;
  }
 }

 static int trates(_internalthreadargsprotocomma_ double _lv) { 
#if 0
_check_trates(_threadargs_);
#endif
 _n_trates(_threadargscomma_ _lv);
 return 0;
 }

 static void _n_trates(_internalthreadargsprotocomma_ double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 _f_trates(_threadargscomma_ _lv); return; 
}
 _xi = _mfac_trates * (_lv - _tmin_trates);
 if (std::isnan(_xi)) {
  ntau = _xi;
  ltau = _xi;
  ninf = _xi;
  linf = _xi;
  al = _xi;
  bl = _xi;
  an = _xi;
  bn = _xi;
  return;
 }
 if (_xi <= 0.) {
 ntau = _t_ntau[0];
 ltau = _t_ltau[0];
 ninf = _t_ninf[0];
 linf = _t_linf[0];
 al = _t_al[0];
 bl = _t_bl[0];
 an = _t_an[0];
 bn = _t_bn[0];
 return; }
 if (_xi >= 199.) {
 ntau = _t_ntau[199];
 ltau = _t_ltau[199];
 ninf = _t_ninf[199];
 linf = _t_linf[199];
 al = _t_al[199];
 bl = _t_bl[199];
 an = _t_an[199];
 bn = _t_bn[199];
 return; }
 _i = (int) _xi;
 _theta = _xi - (double)_i;
 ntau = _t_ntau[_i] + _theta*(_t_ntau[_i+1] - _t_ntau[_i]);
 ltau = _t_ltau[_i] + _theta*(_t_ltau[_i+1] - _t_ltau[_i]);
 ninf = _t_ninf[_i] + _theta*(_t_ninf[_i+1] - _t_ninf[_i]);
 linf = _t_linf[_i] + _theta*(_t_linf[_i+1] - _t_linf[_i]);
 al = _t_al[_i] + _theta*(_t_al[_i+1] - _t_al[_i]);
 bl = _t_bl[_i] + _theta*(_t_bl[_i+1] - _t_bl[_i]);
 an = _t_an[_i] + _theta*(_t_an[_i+1] - _t_an[_i]);
 bn = _t_bn[_i] + _theta*(_t_bn[_i+1] - _t_bn[_i]);
 }

 
static int  _f_trates ( _internalthreadargsprotocomma_ double _lv ) {
   _lv = _lv + 10.0 ;
   linf = 1.0 / ( 1.0 + exp ( ( - 30.0 - _lv ) / 10.0 ) ) ;
   ltau = 0.346 * exp ( - _lv / ( 18.272 ) ) + 2.09 ;
   ninf = 1.0 / ( 1.0 + exp ( 0.0878 * ( _lv + 55.1 ) ) ) ;
   ntau = 2.1 * exp ( - _lv / 21.2 ) + 4.627 ;
   _lv = _lv - 10.0 ;
   al = linf / ltau ;
   bl = 1.0 / ltau - al ;
   an = ninf / ntau ;
   bn = 1.0 / ntau - an ;
    return 0; }
 
static void _hoc_trates(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for trates_StochKv3. Requires prior call to setdata_StochKv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 
#if 1
 _check_trates(_threadargs_);
#endif
 _r = 1.;
 trates ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_trates(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 
#if 1
 _check_trates(_threadargs_);
#endif
 _r = 1.;
 trates ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
double strap ( _internalthreadargsprotocomma_ double _lx ) {
   double _lstrap;
 if ( _lx < 0.0 ) {
     _lstrap = 0.0 ;
     }
   else {
     _lstrap = _lx ;
     }
   
return _lstrap;
 }
 
static void _hoc_strap(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  Prop* _local_prop = _prop_id ? _extcall_prop : nullptr;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  strap ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_strap(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  strap ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
static int  ChkProb ( _internalthreadargsprotocomma_ double _lp ) {
   if ( _lp < 0.0  || _lp > 1.0 ) {
     
/*VERBATIM*/
    fprintf(stderr, "StochKv2.mod:ChkProb: argument not a probability.\n");
 }
    return 0; }
 
static void _hoc_ChkProb(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for ChkProb_StochKv3. Requires prior call to setdata_StochKv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r = 1.;
 ChkProb ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_ChkProb(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r = 1.;
 ChkProb ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
static int  setRNG ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
    // For compatibility, allow for either MCellRan4 or Random123.  Distinguish by the arg types
    // Object => MCellRan4, seeds (double) => Random123
#ifndef CORENEURON_BUILD
    usingR123 = 0;
    if( ifarg(1) && hoc_is_double_arg(1) ) {
        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
        uint32_t a2 = 0;
        uint32_t a3 = 0;

        if (*pv) {
            nrnran123_deletestream(*pv);
            *pv = (nrnran123_State*)0;
        }
        if (ifarg(2)) {
            a2 = (uint32_t)*getarg(2);
        }
        if (ifarg(3)) {
            a3 = (uint32_t)*getarg(3);
        }
        *pv = nrnran123_newstream3((uint32_t)*getarg(1), a2, a3);
        usingR123 = 1;
    } else if( ifarg(1) ) {
        void** pv = (void**)(&_p_rng);
        *pv = nrn_random_arg(1);
    } else {
        void** pv = (void**)(&_p_rng);
        *pv = (void*)0;
    }
#endif
  return 0; }
 
static void _hoc_setRNG(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for setRNG_StochKv3. Requires prior call to setdata_StochKv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r = 1.;
 setRNG ( _threadargs_ );
 hoc_retpushx(_r);
}
 
static double _npy_setRNG(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r = 1.;
 setRNG ( _threadargs_ );
 return(_r);
}
 
double urand ( _internalthreadargsproto_ ) {
   double _lurand;
 
/*VERBATIM*/
    double value = 0.0;
    if( usingR123 ) {
        value = nrnran123_dblpick((nrnran123_State*)_p_rng);
    } else if (_p_rng) {
#ifndef CORENEURON_BUILD
        value = nrn_random_pick(RANDCAST _p_rng);
#endif
    } else {
        // see BBPBGLIB-972
        value = 0.0;
    }
    _lurand = value;
 
return _lurand;
 }
 
static void _hoc_urand(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for urand_StochKv3. Requires prior call to setdata_StochKv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  urand ( _threadargs_ );
 hoc_retpushx(_r);
}
 
static double _npy_urand(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  urand ( _threadargs_ );
 return(_r);
}
 
/*VERBATIM*/
static void bbcore_write(double* x, int* d, int* xx, int* offset, _threadargsproto_) {
    if (d) {
        uint32_t* di = ((uint32_t*)d) + *offset;
      // temporary just enough to see how much space is being used
      if (!_p_rng) {
        di[0] = 0; di[1] = 0, di[2] = 0;
      }else{
        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
        nrnran123_getids3(*pv, di, di+1, di+2);
        // write stream sequence
        char which;
        nrnran123_getseq(*pv, di+3, &which);
        di[4] = (int)which;
      }
      //printf("StochKv3.mod %p: bbcore_write offset=%d %d %d\n", _p, *offset, d?di[0]:-1, d?di[1]:-1);
    }
    *offset += 5;
}
static void bbcore_read(double* x, int* d, int* xx, int* offset, _threadargsproto_) {
    uint32_t* di = ((uint32_t*)d) + *offset;
        if (di[0] != 0 || di[1] != 0|| di[2] != 0)
        {
      nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
#if !NRNBBCORE
      if(*pv) {
          nrnran123_deletestream(*pv);
      }
#endif
      *pv = nrnran123_newstream3(di[0], di[1], di[2]);
      nrnran123_setseq(*pv, di[3], (char)di[4]);
        }
      //printf("StochKv3.mod %p: bbcore_read offset=%d %d %d\n", _p, *offset, di[0], di[1]);
    *offset += 5;
}
 
double brand ( _internalthreadargsprotocomma_ double _lP , double _lN ) {
   double _lbrand;
 
/*VERBATIM*/
        /*
        :Supports separate independent but reproducible streams for
        : each instance. However, the corresponding hoc Random
        : distribution MUST be set to Random.uniform(0,1)
        */

        // Should probably be optimized
        double value = 0.0;
        int i;
        for (i = 0; i < _lN; i++) {
           if (urand(_threadargs_) < _lP) {
              value = value + 1;
           }
        }
        return(value);

 _lbrand = value ;
   
return _lbrand;
 }
 
static void _hoc_brand(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for brand_StochKv3. Requires prior call to setdata_StochKv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  brand ( _threadargscomma_ *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
static double _npy_brand(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  brand ( _threadargscomma_ *getarg(1) , *getarg(2) );
 return(_r);
}
 
/*VERBATIM*/
#define        PI 3.141592654
#define        r_ia     16807
#define        r_im     2147483647
#define        r_am     (1.0/r_im)
#define        r_iq     127773
#define        r_ir     2836
#define        r_ntab   32
#define        r_ndiv   (1+(r_im-1)/r_ntab)
#define        r_eps    1.2e-7
#define        r_rnmx   (1.0-r_eps)
 
/*VERBATIM*/
/* ---------------------------------------------------------------- */
/* gammln - compute natural log of gamma function of xx */
static double
gammln(double xx)
{
    double x,tmp,ser;
    static double cof[6]={76.18009173,-86.50532033,24.01409822,
        -1.231739516,0.120858003e-2,-0.536382e-5};
    int j;
    x=xx-1.0;
    tmp=x+5.5;
    tmp -= (x+0.5)*log(tmp);
    ser=1.0;
    for (j=0;j<=5;j++) {
        x += 1.0;
        ser += cof[j]/x;
    }
    return -tmp+log(2.50662827465*ser);
}
 
double BnlDev ( _internalthreadargsprotocomma_ double _lppr , double _lnnr ) {
   double _lBnlDev;
 
/*VERBATIM*/
        int j;
        double am,em,g,angle,p,bnl,sq,bt,y;
        double pc,plog,pclog,en,oldg;

        /* prepare to always ignore errors within this routine */

        p=(_lppr <= 0.5 ? _lppr : 1.0-_lppr);
        am=_lnnr*p;
        if (_lnnr < 25) {
            bnl=0.0;
            for (j=1;j<=_lnnr;j++)
                if (urand(_threadargs_) < p) bnl += 1.0;
        }
        else if (am < 1.0) {
            g=exp(-am);
            bt=1.0;
            for (j=0;j<=_lnnr;j++) {
                bt *= urand(_threadargs_);
                if (bt < g) break;
            }
            bnl=(j <= _lnnr ? j : _lnnr);
        }
        else {
            {
                en=_lnnr;
                oldg=gammln(en+1.0);
            }
            {
                pc=1.0-p;
                plog=log(p);
                pclog=log(pc);
            }
            sq=sqrt(2.0*am*pc);
            do {
                do {
                    angle=PI*urand(_threadargs_);
                    angle=PI*urand(_threadargs_);
                    y=tan(angle);
                    em=sq*y+am;
                } while (em < 0.0 || em >= (en+1.0));
                em=floor(em);
                    bt=1.2*sq*(1.0+y*y)*exp(oldg-gammln(em+1.0) -
                    gammln(en-em+1.0)+em*plog+(en-em)*pclog);
            } while (urand(_threadargs_) > bt);
            bnl=em;
        }
        if (p != _lppr) bnl=_lnnr-bnl;

        /* recover error if changed during this routine, thus ignoring
            any errors during this routine */


        return bnl;

 _lBnlDev = bnl ;
   
return _lBnlDev;
 }
 
static void _hoc_BnlDev(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for BnlDev_StochKv3. Requires prior call to setdata_StochKv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  BnlDev ( _threadargscomma_ *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
static double _npy_BnlDev(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  BnlDev ( _threadargscomma_ *getarg(1) , *getarg(2) );
 return(_r);
}
 
double bbsavestate ( _internalthreadargsproto_ ) {
   double _lbbsavestate;
 _lbbsavestate = 0.0 ;
   
/*VERBATIM*/
 #ifndef CORENEURON_BUILD
        // TODO: since N0,N1 are no longer state variables, they will need to be written using this callback
        //  provided that it is the version that supports multivalue writing
        /* first arg is direction (-1 get info, 0 save, 1 restore), second is value*/
        double *xdir, *xval;
        #ifndef NRN_VERSION_GTEQ_8_2_0
        double *hoc_pgetarg();
        long nrn_get_random_sequence(void* r);
        void nrn_set_random_sequence(void* r, int val);
        #endif
        xdir = hoc_pgetarg(1);
        xval = hoc_pgetarg(2);
        if (_p_rng) {
                // tell how many items need saving
                if (*xdir == -1.) {
                    if( usingR123 ) {
                        *xdir = 2.0;
                    } else {
                        *xdir = 1.0;
                    }
                    return 0.0;
                }
                else if (*xdir == 0.) {
                    if( usingR123 ) {
                        uint32_t seq;
                        char which;
                        nrnran123_getseq( (nrnran123_State*)_p_rng, &seq, &which );
                        xval[0] = (double) seq;
                        xval[1] = (double) which;
                    } else {
                        xval[0] = (double)nrn_get_random_sequence(RANDCAST _p_rng);
                    }
                } else{
                    if( usingR123 ) {
                        nrnran123_setseq( (nrnran123_State*)_p_rng, (uint32_t)xval[0], (char)xval[1] );
                    } else {
                        nrn_set_random_sequence(RANDCAST _p_rng, (long)(xval[0]));
                    }
                }
        }

        // TODO: check for random123 and get the seq values
#endif
 
return _lbbsavestate;
 }
 
static void _hoc_bbsavestate(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for bbsavestate_StochKv3. Requires prior call to setdata_StochKv3 and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  bbsavestate ( _threadargs_ );
 hoc_retpushx(_r);
}
 
static double _npy_bbsavestate(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
_nt = nrn_threads;
 _r =  bbsavestate ( _threadargs_ );
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
  ek = _ion_ek;
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
  ek = _ion_ek;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  l = l0;
  n = n0;
 {
   
/*VERBATIM*/
    if (cvode_active_ && !deterministic) {
        hoc_execerror("StochKv2 with deterministic=0", "cannot be used with cvode");
    }

    if( usingR123 ) {
        nrnran123_setseq((nrnran123_State*)_p_rng, 0, 0);
    }
 eta = ( gkbar / gamma ) * ( 10000.0 ) ;
   trates ( _threadargscomma_ v ) ;
   n = ninf ;
   l = linf ;
   scale_dens = gamma / area ;
   N = floor ( eta * area + 0.5 ) ;
   N1L1 = n * l * N ;
   N1L0 = n * ( 1.0 - l ) * N ;
   N0L1 = ( 1.0 - n ) * l * N ;
   if (  ! deterministic ) {
     N1L1 = floor ( N1L1 + 0.5 ) ;
     N1L0 = floor ( N1L0 + 0.5 ) ;
     N0L1 = floor ( N0L1 + 0.5 ) ;
     }
   N0L0 = N - N1L1 - N1L0 - N0L1 ;
   n0l0_n1l0 = 0.0 ;
   n0l0_n0l1 = 0.0 ;
   n1l0_n1l1 = 0.0 ;
   n1l0_n0l0 = 0.0 ;
   n0l1_n1l1 = 0.0 ;
   n0l1_n0l0 = 0.0 ;
   n1l1_n0l1 = 0.0 ;
   n1l1_n1l0 = 0.0 ;
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

#if 0
 _check_trates(_threadargs_);
#endif
   _v = _vec_v[_ni[_iml]];
 v = _v;
  ek = _ion_ek;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   gk = ( strap ( _threadargscomma_ N1L1 ) * scale_dens ) * ( 0.0001 ) ;
   ik = gk * ( v - ek ) ;
   }
 _current += ik;

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
  ek = _ion_ek;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ik += ik ;
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
  ek = _ion_ek;
 {   states(_threadargs_);
  } }}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {l_columnindex, 0};  _dlist1[0] = {Dl_columnindex, 0};
 _slist1[1] = {n_columnindex, 0};  _dlist1[1] = {Dn_columnindex, 0};
   _t_ntau = makevector(200*sizeof(double));
   _t_ltau = makevector(200*sizeof(double));
   _t_ninf = makevector(200*sizeof(double));
   _t_linf = makevector(200*sizeof(double));
   _t_al = makevector(200*sizeof(double));
   _t_bl = makevector(200*sizeof(double));
   _t_an = makevector(200*sizeof(double));
   _t_bn = makevector(200*sizeof(double));
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "StochKv3.mod";
    const char* nmodl_file_text = 
  "TITLE StochKv3.mod\n"
  "\n"
  "COMMENT\n"
  "----------------------------------------------------------------\n"
  "Stochastic inactivating channel using values reported in Mendonca et al. 2016.\n"
  "(Modified from https://senselab.med.yale.edu/modeldb/showmodel.cshtml?model=125385&file=/Sbpap_code/mod/skaprox.mod)\n"
  "\n"
  "The model keeps track on the number of channels in each state, and\n"
  "uses a binomial distribution to update these number.\n"
  "\n"
  "Jan 1999, Mickey London, Hebrew University, mikilon@lobster.ls.huji.ac.il\n"
  "        Peter N. Steinmetz, Caltech, peter@klab.caltech.edu\n"
  "14 Sep 99 PNS. Added deterministic flag.\n"
  "19 May 2002 Kamran Diba.  Changed gamma and deterministic from GLOBAL to RANGE.\n"
  "23 Nov 2011 Werner Van Geit @ BBP. Changed the file so that it can use the neuron random number generator. Tuned voltage dependence\n"
  "16 Mar 2016 James G King @ BBP.  Incorporate modifications suggested by Michael Hines to improve stiching to deterministic mode, thread safety, and using Random123\n"
  "26 Sep 2016 Christian Roessert @ BBP. Adding inactivation, changing dynamics to values reported in Mendonca et al. 2016\n"
  ": LJP: OK, whole-cell patch, corrected by 10 mV (Mendonca et al. 2016)\n"
  "\n"
  "----------------------------------------------------------------\n"
  "ENDCOMMENT\n"
  "\n"
  "INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX StochKv3\n"
  "    THREADSAFE\n"
  "    USEION k READ ek WRITE ik\n"
  "    RANGE N, eta, gk, gamma, deterministic, gkbar, ik\n"
  "    RANGE N0, N1, n0_n1, n1_n0\n"
  "    RANGE ninf, linf, ltau, ntau, an, bn, al, bl\n"
  "    RANGE P_an, P_bn, P_al, P_bl\n"
  "    GLOBAL vmin, vmax\n"
  "    BBCOREPOINTER rng\n"
  "    :POINTER rng\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "    (mA) = (milliamp)\n"
  "    (mV) = (millivolt)\n"
  "    (pS) = (picosiemens)\n"
  "    (S) = (siemens)\n"
  "    (um) = (micron)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    v           (mV)\n"
  "    dt      (ms)\n"
  "    area    (um2)\n"
  "\n"
  "    gamma  =  50      (pS)\n"
  "    eta              (1/um2)\n"
  "    gkbar = .01      (S/cm2)\n"
  "\n"
  "    deterministic = 0   : if non-zero, will use deterministic version\n"
  "    vmin = -120 (mV)    : range to construct tables for\n"
  "    vmax = 100  (mV)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    an      (/ms)\n"
  "    bn      (/ms)\n"
  "    al      (/ms)\n"
  "    bl      (/ms)\n"
  "    ik      (mA/cm2)\n"
  "    gk      (S/cm2)\n"
  "    ek      (mV)\n"
  "    ninf        : steady-state value\n"
  "    ntau (ms)   : time constant for relaxation\n"
  "    linf        : steady-state value\n"
  "    ltau (ms)   : time constant for relaxation\n"
  "\n"
  "    N\n"
  "    scale_dens (pS/um2)\n"
  "    P_an     : probability of one channel making alpha n transition\n"
  "    P_bn     : probability of one channel making beta n transition\n"
  "    P_al     : probability of one channel making alpha l transition\n"
  "    P_bl     : probability of one channel making beta l transition\n"
  "\n"
  "    rng\n"
  "    usingR123\n"
  "}\n"
  "\n"
  "\n"
  "STATE {\n"
  "    n l  : state variable of deterministic description\n"
  "}\n"
  "ASSIGNED {\n"
  "    N0L0 N1L0 N0L1 N1L1     : N states populations (These currently will not be saved via the bbsavestate functionality.  Would need to be STATE again)\n"
  "    n0l0_n1l0 n0l0_n0l1     : number of channels moving from one state to the other\n"
  "    n1l0_n1l1 n1l0_n0l0\n"
  "    n0l1_n1l1 n0l1_n0l0\n"
  "    n1l1_n0l1 n1l1_n1l0\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "The Verbatim block is needed to generate random nos. from a uniform distribution between 0 and 1\n"
  "for comparison with Pr to decide whether to activate the synapse or not\n"
  "ENDCOMMENT\n"
  "\n"
  "VERBATIM\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "#include \"nrnran123.h\"\n"
  "extern int cvode_active_;\n"
  "\n"
  "#include <stdlib.h>\n"
  "#include <stdio.h>\n"
  "#include <math.h>\n"
  "\n"
  "#ifndef CORENEURON_BUILD\n"
  "double nrn_random_pick(void* r);\n"
  "void* nrn_random_arg(int argpos);\n"
  "#endif\n"
  "#define RANDCAST\n"
  "#else\n"
  "#define RANDCAST (Rand*)\n"
  "#endif\n"
  "\n"
  "ENDVERBATIM\n"
  ": ----------------------------------------------------------------\n"
  ": initialization\n"
  "INITIAL {\n"
  "    VERBATIM\n"
  "    if (cvode_active_ && !deterministic) {\n"
  "        hoc_execerror(\"StochKv2 with deterministic=0\", \"cannot be used with cvode\");\n"
  "    }\n"
  "\n"
  "    if( usingR123 ) {\n"
  "        nrnran123_setseq((nrnran123_State*)_p_rng, 0, 0);\n"
  "    }\n"
  "    ENDVERBATIM\n"
  "\n"
  "    eta = (gkbar / gamma) * (10000)\n"
  "    trates(v)\n"
  "    n=ninf\n"
  "    l=linf\n"
  "    scale_dens = gamma/area\n"
  "    N = floor(eta*area + 0.5)\n"
  "\n"
  "    N1L1 = n*l*N\n"
  "    N1L0 = n*(1-l)*N\n"
  "    N0L1 = (1-n)*l*N\n"
  "    if( !deterministic) {\n"
  "        N1L1 = floor(N1L1 + 0.5)\n"
  "        N1L0 = floor(N1L0 + 0.5)\n"
  "        N0L1 = floor(N0L1 + 0.5)\n"
  "    }\n"
  "    N0L0 = N - N1L1 - N1L0 - N0L1  : put rest into non-conducting state\n"
  "\n"
  "    n0l0_n1l0 = 0\n"
  "    n0l0_n0l1 = 0\n"
  "    n1l0_n1l1 = 0\n"
  "    n1l0_n0l0 = 0\n"
  "    n0l1_n1l1 = 0\n"
  "    n0l1_n0l0 = 0\n"
  "    n1l1_n0l1 = 0\n"
  "    n1l1_n1l0 = 0\n"
  "}\n"
  "\n"
  ": ----------------------------------------------------------------\n"
  ": Breakpoint for each integration step\n"
  "BREAKPOINT {\n"
  "  SOLVE states METHOD cnexp\n"
  "\n"
  "  gk = (strap(N1L1) * scale_dens) * (0.0001)\n"
  "\n"
  "  ik = gk * (v - ek)\n"
  "}\n"
  "\n"
  "\n"
  ": ----------------------------------------------------------------\n"
  ": states - updates number of channels in each state\n"
  "DERIVATIVE states {\n"
  "\n"
  "    trates(v)\n"
  "\n"
  "    l' = al - (al + bl)*l\n"
  "    n' = an - (an + bn)*n\n"
  "\n"
  "    if (deterministic || dt > 1) { : ForwardSkip is also deterministic\n"
  "\n"
  "      N1L1 = n*l*N\n"
  "      N1L0 = n*(1-l)*N\n"
  "      N0L1 = (1-n)*l*N\n"
  "\n"
  "    }else{\n"
  "\n"
  "      : ensure that N0 is an integer for when transitioning from deterministic mode to stochastic mode\n"
  "      N1L1 = floor(N1L1 + 0.5)\n"
  "      N1L0 = floor(N1L0 + 0.5)\n"
  "      N0L1 = floor(N0L1 + 0.5)\n"
  "      N0L0 = N - N1L1 - N1L0 - N0L1\n"
  "\n"
  "      P_an = strap(an*dt)\n"
  "      P_bn = strap(bn*dt)\n"
  "      : check that will represent probabilities when used\n"
  "      ChkProb(P_an)\n"
  "      ChkProb(P_bn)\n"
  "\n"
  "      : n gate transitions\n"
  "      n0l0_n1l0 = BnlDev(P_an, N0L0)\n"
  "      n0l1_n1l1 = BnlDev(P_an, N0L1)\n"
  "      n1l1_n0l1 = BnlDev(P_bn, N1L1)\n"
  "      n1l0_n0l0 = BnlDev(P_bn, N1L0)\n"
  "\n"
  "      : move the channels\n"
  "      N0L0 = strap(N0L0 - n0l0_n1l0 + n1l0_n0l0)\n"
  "      N1L0 = strap(N1L0 - n1l0_n0l0 + n0l0_n1l0)\n"
  "      N0L1 = strap(N0L1 - n0l1_n1l1 + n1l1_n0l1)\n"
  "      N1L1 = strap(N1L1 - n1l1_n0l1 + n0l1_n1l1)\n"
  "\n"
  "      : probabilities of making l gate transitions\n"
  "      P_al = strap(al*dt)\n"
  "      P_bl  = strap(bl*dt)\n"
  "      : check that will represent probabilities when used\n"
  "      ChkProb(P_al)\n"
  "      ChkProb(P_bl)\n"
  "\n"
  "      : number making l gate transitions\n"
  "      n0l0_n0l1 = BnlDev(P_al,N0L0-n0l0_n1l0)\n"
  "      n1l0_n1l1 = BnlDev(P_al,N1L0-n1l0_n0l0)\n"
  "      n0l1_n0l0 = BnlDev(P_bl,N0L1-n0l1_n1l1)\n"
  "      n1l1_n1l0 = BnlDev(P_bl,N1L1-n1l1_n0l1)\n"
  "\n"
  "      : move the channels\n"
  "      N0L0 = strap(N0L0 - n0l0_n0l1  + n0l1_n0l0)\n"
  "      N1L0 = strap(N1L0 - n1l0_n1l1  + n1l1_n1l0)\n"
  "      N0L1 = strap(N0L1 - n0l1_n0l0  + n0l0_n0l1)\n"
  "      N1L1 = strap(N1L1 - n1l1_n1l0  + n1l0_n1l1)\n"
  "\n"
  "    }\n"
  "\n"
  "    N0L0 = N - N1L1 - N1L0 - N0L1  : put rest into non-conducting state\n"
  "}\n"
  "\n"
  ": ----------------------------------------------------------------\n"
  ": trates - compute rates, using table if possible\n"
  "PROCEDURE trates(v (mV)) {\n"
  "    TABLE ntau,ltau,ninf,linf,al,bl,an,bn\n"
  "    DEPEND dt\n"
  "    FROM vmin TO vmax WITH 199\n"
  "\n"
  "    v = v + 10\n"
  "    linf = 1/(1+exp((-30(mV)-v)/10(mV)))\n"
  "    ltau = 0.346(ms)*exp(-v/(18.272(mV)))+2.09(ms)\n"
  "\n"
  "    ninf = 1/(1+exp(0.0878(1/mV)*(v+55.1(mV))))\n"
  "    ntau = 2.1(ms)*exp(-v/21.2(mV))+4.627(ms)\n"
  "    v = v - 10\n"
  "\n"
  "    al = linf/ltau\n"
  "    bl = 1/ltau - al\n"
  "    an = ninf/ntau\n"
  "    bn = 1/ntau - an\n"
  "}\n"
  "\n"
  ": ----------------------------------------------------------------\n"
  ": sign trap - trap for negative values and replace with zero\n"
  "FUNCTION strap(x) {\n"
  "    if (x < 0) {\n"
  "        strap = 0\n"
  ": This function gets executed in the DERIVATIVE and BREAKPOINT blocks\n"
  ": which should get vectorized and fprintf doesn't allow this\n"
  ": Also Intel classic compilers generate invalid code when #pragma omp simd is used\n"
  ":VERBATIM\n"
  ":        fprintf (stderr,\"skv.mod:strap: negative state\");\n"
  ":ENDVERBATIM\n"
  "    } else {\n"
  "        strap = x\n"
  "    }\n"
  "}\n"
  "\n"
  ": ----------------------------------------------------------------\n"
  ": ChkProb - Check that number represents a probability\n"
  "PROCEDURE ChkProb(p) {\n"
  "\n"
  "  if (p < 0.0 || p > 1.0) {\n"
  "    VERBATIM\n"
  "    fprintf(stderr, \"StochKv2.mod:ChkProb: argument not a probability.\\n\");\n"
  "    ENDVERBATIM\n"
  "  }\n"
  "\n"
  "}\n"
  "\n"
  "PROCEDURE setRNG() {\n"
  "\n"
  "VERBATIM\n"
  "    // For compatibility, allow for either MCellRan4 or Random123.  Distinguish by the arg types\n"
  "    // Object => MCellRan4, seeds (double) => Random123\n"
  "#ifndef CORENEURON_BUILD\n"
  "    usingR123 = 0;\n"
  "    if( ifarg(1) && hoc_is_double_arg(1) ) {\n"
  "        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);\n"
  "        uint32_t a2 = 0;\n"
  "        uint32_t a3 = 0;\n"
  "\n"
  "        if (*pv) {\n"
  "            nrnran123_deletestream(*pv);\n"
  "            *pv = (nrnran123_State*)0;\n"
  "        }\n"
  "        if (ifarg(2)) {\n"
  "            a2 = (uint32_t)*getarg(2);\n"
  "        }\n"
  "        if (ifarg(3)) {\n"
  "            a3 = (uint32_t)*getarg(3);\n"
  "        }\n"
  "        *pv = nrnran123_newstream3((uint32_t)*getarg(1), a2, a3);\n"
  "        usingR123 = 1;\n"
  "    } else if( ifarg(1) ) {\n"
  "        void** pv = (void**)(&_p_rng);\n"
  "        *pv = nrn_random_arg(1);\n"
  "    } else {\n"
  "        void** pv = (void**)(&_p_rng);\n"
  "        *pv = (void*)0;\n"
  "    }\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "FUNCTION urand() {\n"
  "\n"
  "VERBATIM\n"
  "    double value = 0.0;\n"
  "    if( usingR123 ) {\n"
  "        value = nrnran123_dblpick((nrnran123_State*)_p_rng);\n"
  "    } else if (_p_rng) {\n"
  "#ifndef CORENEURON_BUILD\n"
  "        value = nrn_random_pick(RANDCAST _p_rng);\n"
  "#endif\n"
  "    } else {\n"
  "        // see BBPBGLIB-972\n"
  "        value = 0.0;\n"
  "    }\n"
  "    _lurand = value;\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "static void bbcore_write(double* x, int* d, int* xx, int* offset, _threadargsproto_) {\n"
  "    if (d) {\n"
  "        uint32_t* di = ((uint32_t*)d) + *offset;\n"
  "      // temporary just enough to see how much space is being used\n"
  "      if (!_p_rng) {\n"
  "        di[0] = 0; di[1] = 0, di[2] = 0;\n"
  "      }else{\n"
  "        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);\n"
  "        nrnran123_getids3(*pv, di, di+1, di+2);\n"
  "        // write stream sequence\n"
  "        char which;\n"
  "        nrnran123_getseq(*pv, di+3, &which);\n"
  "        di[4] = (int)which;\n"
  "      }\n"
  "      //printf(\"StochKv3.mod %p: bbcore_write offset=%d %d %d\\n\", _p, *offset, d?di[0]:-1, d?di[1]:-1);\n"
  "    }\n"
  "    *offset += 5;\n"
  "}\n"
  "static void bbcore_read(double* x, int* d, int* xx, int* offset, _threadargsproto_) {\n"
  "    uint32_t* di = ((uint32_t*)d) + *offset;\n"
  "        if (di[0] != 0 || di[1] != 0|| di[2] != 0)\n"
  "        {\n"
  "      nrnran123_State** pv = (nrnran123_State**)(&_p_rng);\n"
  "#if !NRNBBCORE\n"
  "      if(*pv) {\n"
  "          nrnran123_deletestream(*pv);\n"
  "      }\n"
  "#endif\n"
  "      *pv = nrnran123_newstream3(di[0], di[1], di[2]);\n"
  "      nrnran123_setseq(*pv, di[3], (char)di[4]);\n"
  "        }\n"
  "      //printf(\"StochKv3.mod %p: bbcore_read offset=%d %d %d\\n\", _p, *offset, di[0], di[1]);\n"
  "    *offset += 5;\n"
  "}\n"
  "ENDVERBATIM\n"
  "\n"
  ": Returns random numbers drawn from a binomial distribution\n"
  "FUNCTION brand(P, N) {\n"
  "\n"
  "VERBATIM\n"
  "        /*\n"
  "        :Supports separate independent but reproducible streams for\n"
  "        : each instance. However, the corresponding hoc Random\n"
  "        : distribution MUST be set to Random.uniform(0,1)\n"
  "        */\n"
  "\n"
  "        // Should probably be optimized\n"
  "        double value = 0.0;\n"
  "        int i;\n"
  "        for (i = 0; i < _lN; i++) {\n"
  "           if (urand(_threadargs_) < _lP) {\n"
  "              value = value + 1;\n"
  "           }\n"
  "        }\n"
  "        return(value);\n"
  "\n"
  "ENDVERBATIM\n"
  "\n"
  "        brand = value\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "#define        PI 3.141592654\n"
  "#define        r_ia     16807\n"
  "#define        r_im     2147483647\n"
  "#define        r_am     (1.0/r_im)\n"
  "#define        r_iq     127773\n"
  "#define        r_ir     2836\n"
  "#define        r_ntab   32\n"
  "#define        r_ndiv   (1+(r_im-1)/r_ntab)\n"
  "#define        r_eps    1.2e-7\n"
  "#define        r_rnmx   (1.0-r_eps)\n"
  "ENDVERBATIM\n"
  "\n"
  "VERBATIM\n"
  "/* ---------------------------------------------------------------- */\n"
  "/* gammln - compute natural log of gamma function of xx */\n"
  "static double\n"
  "gammln(double xx)\n"
  "{\n"
  "    double x,tmp,ser;\n"
  "    static double cof[6]={76.18009173,-86.50532033,24.01409822,\n"
  "        -1.231739516,0.120858003e-2,-0.536382e-5};\n"
  "    int j;\n"
  "    x=xx-1.0;\n"
  "    tmp=x+5.5;\n"
  "    tmp -= (x+0.5)*log(tmp);\n"
  "    ser=1.0;\n"
  "    for (j=0;j<=5;j++) {\n"
  "        x += 1.0;\n"
  "        ser += cof[j]/x;\n"
  "    }\n"
  "    return -tmp+log(2.50662827465*ser);\n"
  "}\n"
  "ENDVERBATIM\n"
  "\n"
  "\n"
  ": ----------------------------------------------------------------\n"
  ": BnlDev - draw a uniform deviate from the generator\n"
  "FUNCTION BnlDev (ppr, nnr) {\n"
  "\n"
  "VERBATIM\n"
  "        int j;\n"
  "        double am,em,g,angle,p,bnl,sq,bt,y;\n"
  "        double pc,plog,pclog,en,oldg;\n"
  "\n"
  "        /* prepare to always ignore errors within this routine */\n"
  "\n"
  "        p=(_lppr <= 0.5 ? _lppr : 1.0-_lppr);\n"
  "        am=_lnnr*p;\n"
  "        if (_lnnr < 25) {\n"
  "            bnl=0.0;\n"
  "            for (j=1;j<=_lnnr;j++)\n"
  "                if (urand(_threadargs_) < p) bnl += 1.0;\n"
  "        }\n"
  "        else if (am < 1.0) {\n"
  "            g=exp(-am);\n"
  "            bt=1.0;\n"
  "            for (j=0;j<=_lnnr;j++) {\n"
  "                bt *= urand(_threadargs_);\n"
  "                if (bt < g) break;\n"
  "            }\n"
  "            bnl=(j <= _lnnr ? j : _lnnr);\n"
  "        }\n"
  "        else {\n"
  "            {\n"
  "                en=_lnnr;\n"
  "                oldg=gammln(en+1.0);\n"
  "            }\n"
  "            {\n"
  "                pc=1.0-p;\n"
  "                plog=log(p);\n"
  "                pclog=log(pc);\n"
  "            }\n"
  "            sq=sqrt(2.0*am*pc);\n"
  "            do {\n"
  "                do {\n"
  "                    angle=PI*urand(_threadargs_);\n"
  "                    angle=PI*urand(_threadargs_);\n"
  "                    y=tan(angle);\n"
  "                    em=sq*y+am;\n"
  "                } while (em < 0.0 || em >= (en+1.0));\n"
  "                em=floor(em);\n"
  "                    bt=1.2*sq*(1.0+y*y)*exp(oldg-gammln(em+1.0) -\n"
  "                    gammln(en-em+1.0)+em*plog+(en-em)*pclog);\n"
  "            } while (urand(_threadargs_) > bt);\n"
  "            bnl=em;\n"
  "        }\n"
  "        if (p != _lppr) bnl=_lnnr-bnl;\n"
  "\n"
  "        /* recover error if changed during this routine, thus ignoring\n"
  "            any errors during this routine */\n"
  "\n"
  "\n"
  "        return bnl;\n"
  "\n"
  "    ENDVERBATIM\n"
  "    BnlDev = bnl\n"
  "}\n"
  "\n"
  "FUNCTION bbsavestate() {\n"
  "        bbsavestate = 0\n"
  "VERBATIM\n"
  " #ifndef CORENEURON_BUILD\n"
  "        // TODO: since N0,N1 are no longer state variables, they will need to be written using this callback\n"
  "        //  provided that it is the version that supports multivalue writing\n"
  "        /* first arg is direction (-1 get info, 0 save, 1 restore), second is value*/\n"
  "        double *xdir, *xval;\n"
  "        #ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "        double *hoc_pgetarg();\n"
  "        long nrn_get_random_sequence(void* r);\n"
  "        void nrn_set_random_sequence(void* r, int val);\n"
  "        #endif\n"
  "        xdir = hoc_pgetarg(1);\n"
  "        xval = hoc_pgetarg(2);\n"
  "        if (_p_rng) {\n"
  "                // tell how many items need saving\n"
  "                if (*xdir == -1.) {\n"
  "                    if( usingR123 ) {\n"
  "                        *xdir = 2.0;\n"
  "                    } else {\n"
  "                        *xdir = 1.0;\n"
  "                    }\n"
  "                    return 0.0;\n"
  "                }\n"
  "                else if (*xdir == 0.) {\n"
  "                    if( usingR123 ) {\n"
  "                        uint32_t seq;\n"
  "                        char which;\n"
  "                        nrnran123_getseq( (nrnran123_State*)_p_rng, &seq, &which );\n"
  "                        xval[0] = (double) seq;\n"
  "                        xval[1] = (double) which;\n"
  "                    } else {\n"
  "                        xval[0] = (double)nrn_get_random_sequence(RANDCAST _p_rng);\n"
  "                    }\n"
  "                } else{\n"
  "                    if( usingR123 ) {\n"
  "                        nrnran123_setseq( (nrnran123_State*)_p_rng, (uint32_t)xval[0], (char)xval[1] );\n"
  "                    } else {\n"
  "                        nrn_set_random_sequence(RANDCAST _p_rng, (long)(xval[0]));\n"
  "                    }\n"
  "                }\n"
  "        }\n"
  "\n"
  "        // TODO: check for random123 and get the seq values\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
