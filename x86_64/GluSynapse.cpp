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
static constexpr auto number_of_floating_point_variables = 54;
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
 
#define nrn_init _nrn_init__GluSynapse
#define _nrn_initial _nrn_initial__GluSynapse
#define nrn_cur _nrn_cur__GluSynapse
#define _nrn_current _nrn_current__GluSynapse
#define nrn_jacob _nrn_jacob__GluSynapse
#define nrn_state _nrn_state__GluSynapse
#define _net_receive _net_receive__GluSynapse 
#define clearRNG clearRNG__GluSynapse 
#define setRNG setRNG__GluSynapse 
#define state state__GluSynapse 
#define setup_delay_vecs setup_delay_vecs__GluSynapse 
 
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
#define tau_d_AMPA _ml->template fpfield<0>(_iml)
#define tau_d_AMPA_columnindex 0
#define gmax0_AMPA _ml->template fpfield<1>(_iml)
#define gmax0_AMPA_columnindex 1
#define gmax_d_AMPA _ml->template fpfield<2>(_iml)
#define gmax_d_AMPA_columnindex 2
#define gmax_p_AMPA _ml->template fpfield<3>(_iml)
#define gmax_p_AMPA_columnindex 3
#define gmax_NMDA _ml->template fpfield<4>(_iml)
#define gmax_NMDA_columnindex 4
#define Use _ml->template fpfield<5>(_iml)
#define Use_columnindex 5
#define Dep _ml->template fpfield<6>(_iml)
#define Dep_columnindex 6
#define Fac _ml->template fpfield<7>(_iml)
#define Fac_columnindex 7
#define Nrrp _ml->template fpfield<8>(_iml)
#define Nrrp_columnindex 8
#define Use_d _ml->template fpfield<9>(_iml)
#define Use_d_columnindex 9
#define Use_p _ml->template fpfield<10>(_iml)
#define Use_p_columnindex 10
#define volume_CR _ml->template fpfield<11>(_iml)
#define volume_CR_columnindex 11
#define theta_d_GB _ml->template fpfield<12>(_iml)
#define theta_d_GB_columnindex 12
#define theta_p_GB _ml->template fpfield<13>(_iml)
#define theta_p_GB_columnindex 13
#define rho0_GB _ml->template fpfield<14>(_iml)
#define rho0_GB_columnindex 14
#define synapseID _ml->template fpfield<15>(_iml)
#define synapseID_columnindex 15
#define verbose _ml->template fpfield<16>(_iml)
#define verbose_columnindex 16
#define selected_for_report _ml->template fpfield<17>(_iml)
#define selected_for_report_columnindex 17
#define conductance _ml->template fpfield<18>(_iml)
#define conductance_columnindex 18
#define g_AMPA _ml->template fpfield<19>(_iml)
#define g_AMPA_columnindex 19
#define g_NMDA _ml->template fpfield<20>(_iml)
#define g_NMDA_columnindex 20
#define ica_NMDA _ml->template fpfield<21>(_iml)
#define ica_NMDA_columnindex 21
#define ica_VDCC _ml->template fpfield<22>(_iml)
#define ica_VDCC_columnindex 22
#define dep_GB _ml->template fpfield<23>(_iml)
#define dep_GB_columnindex 23
#define pot_GB _ml->template fpfield<24>(_iml)
#define pot_GB_columnindex 24
#define vsyn _ml->template fpfield<25>(_iml)
#define vsyn_columnindex 25
#define i _ml->template fpfield<26>(_iml)
#define i_columnindex 26
#define next_delay _ml->template fpfield<27>(_iml)
#define next_delay_columnindex 27
#define A_AMPA _ml->template fpfield<28>(_iml)
#define A_AMPA_columnindex 28
#define B_AMPA _ml->template fpfield<29>(_iml)
#define B_AMPA_columnindex 29
#define gmax_AMPA _ml->template fpfield<30>(_iml)
#define gmax_AMPA_columnindex 30
#define A_NMDA _ml->template fpfield<31>(_iml)
#define A_NMDA_columnindex 31
#define B_NMDA _ml->template fpfield<32>(_iml)
#define B_NMDA_columnindex 32
#define Use_GB _ml->template fpfield<33>(_iml)
#define Use_GB_columnindex 33
#define m_VDCC _ml->template fpfield<34>(_iml)
#define m_VDCC_columnindex 34
#define h_VDCC _ml->template fpfield<35>(_iml)
#define h_VDCC_columnindex 35
#define cai_CR _ml->template fpfield<36>(_iml)
#define cai_CR_columnindex 36
#define rho_GB _ml->template fpfield<37>(_iml)
#define rho_GB_columnindex 37
#define effcai_GB _ml->template fpfield<38>(_iml)
#define effcai_GB_columnindex 38
#define usingR123 _ml->template fpfield<39>(_iml)
#define usingR123_columnindex 39
#define DA_AMPA _ml->template fpfield<40>(_iml)
#define DA_AMPA_columnindex 40
#define DB_AMPA _ml->template fpfield<41>(_iml)
#define DB_AMPA_columnindex 41
#define Dgmax_AMPA _ml->template fpfield<42>(_iml)
#define Dgmax_AMPA_columnindex 42
#define DA_NMDA _ml->template fpfield<43>(_iml)
#define DA_NMDA_columnindex 43
#define DB_NMDA _ml->template fpfield<44>(_iml)
#define DB_NMDA_columnindex 44
#define DUse_GB _ml->template fpfield<45>(_iml)
#define DUse_GB_columnindex 45
#define Dm_VDCC _ml->template fpfield<46>(_iml)
#define Dm_VDCC_columnindex 46
#define Dh_VDCC _ml->template fpfield<47>(_iml)
#define Dh_VDCC_columnindex 47
#define Dcai_CR _ml->template fpfield<48>(_iml)
#define Dcai_CR_columnindex 48
#define Drho_GB _ml->template fpfield<49>(_iml)
#define Drho_GB_columnindex 49
#define Deffcai_GB _ml->template fpfield<50>(_iml)
#define Deffcai_GB_columnindex 50
#define v _ml->template fpfield<51>(_iml)
#define v_columnindex 51
#define _g _ml->template fpfield<52>(_iml)
#define _g_columnindex 52
#define _tsav _ml->template fpfield<53>(_iml)
#define _tsav_columnindex 53
#define _nd_area *_ml->dptr_field<0>(_iml)
#define rng	*_ppvar[2].get<double*>()
#define _p_rng _ppvar[2].literal_value<void*>()
#define delay_times	*_ppvar[3].get<double*>()
#define _p_delay_times _ppvar[3].literal_value<void*>()
#define delay_weights	*_ppvar[4].get<double*>()
#define _p_delay_weights _ppvar[4].literal_value<void*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  2;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static double _hoc_bbsavestate(void*);
 static double _hoc_brand(void*);
 static double _hoc_clearRNG(void*);
 static double _hoc_nernst(void*);
 static double _hoc_setRNG(void*);
 static double _hoc_setup_delay_vecs(void*);
 static double _hoc_urand(void*);
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
 {"bbsavestate", _hoc_bbsavestate},
 {"brand", _hoc_brand},
 {"clearRNG", _hoc_clearRNG},
 {"nernst", _hoc_nernst},
 {"setRNG", _hoc_setRNG},
 {"setup_delay_vecs", _hoc_setup_delay_vecs},
 {"urand", _hoc_urand},
 {0, 0}
};
#define bbsavestate bbsavestate_GluSynapse
#define brand brand_GluSynapse
#define nernst nernst_GluSynapse
#define urand urand_GluSynapse
 extern double bbsavestate( _internalthreadargsproto_ );
 extern double brand( _internalthreadargsprotocomma_ double , double );
 extern double nernst( _internalthreadargsprotocomma_ double , double , double );
 extern double urand( _internalthreadargsproto_ );
 /* declare global and static user variables */
#define E_NMDA E_NMDA_GluSynapse
 double E_NMDA = -3;
#define E_AMPA E_AMPA_GluSynapse
 double E_AMPA = 0;
#define cao_CR cao_CR_GluSynapse
 double cao_CR = 2;
#define gamma_p_GB gamma_p_GB_GluSynapse
 double gamma_p_GB = 450;
#define gamma_d_GB gamma_d_GB_GluSynapse
 double gamma_d_GB = 100;
#define gamma_ca_CR gamma_ca_CR_GluSynapse
 double gamma_ca_CR = 0.04;
#define gca_bar_VDCC gca_bar_VDCC_GluSynapse
 double gca_bar_VDCC = 0.0744;
#define htau_VDCC htau_VDCC_GluSynapse
 double htau_VDCC = 27;
#define init_depleted init_depleted_GluSynapse
 double init_depleted = 0;
#define kh_VDCC kh_VDCC_GluSynapse
 double kh_VDCC = -9.2;
#define km_VDCC km_VDCC_GluSynapse
 double km_VDCC = 9.5;
#define ljp_VDCC ljp_VDCC_GluSynapse
 double ljp_VDCC = 0;
#define minis_single_vesicle minis_single_vesicle_GluSynapse
 double minis_single_vesicle = 0;
#define min_ca_CR min_ca_CR_GluSynapse
 double min_ca_CR = 7e-05;
#define mtau_VDCC mtau_VDCC_GluSynapse
 double mtau_VDCC = 1;
#define mg mg_GluSynapse
 double mg = 1;
#define nc_type_param nc_type_param_GluSynapse
 double nc_type_param = 5;
#define rho_star_GB rho_star_GB_GluSynapse
 double rho_star_GB = 0.5;
#define slope_NMDA slope_NMDA_GluSynapse
 double slope_NMDA = 0.072;
#define scale_NMDA scale_NMDA_GluSynapse
 double scale_NMDA = 2.552;
#define tau_effca_GB tau_effca_GB_GluSynapse
 double tau_effca_GB = 200;
#define tau_exp_GB tau_exp_GB_GluSynapse
 double tau_exp_GB = 100;
#define tau_ind_GB tau_ind_GB_GluSynapse
 double tau_ind_GB = 70;
#define tau_ca_CR tau_ca_CR_GluSynapse
 double tau_ca_CR = 12;
#define tau_d_NMDA tau_d_NMDA_GluSynapse
 double tau_d_NMDA = 70;
#define tau_r_NMDA tau_r_NMDA_GluSynapse
 double tau_r_NMDA = 0.29;
#define tau_r_AMPA tau_r_AMPA_GluSynapse
 double tau_r_AMPA = 0.2;
#define vhh_VDCC vhh_VDCC_GluSynapse
 double vhh_VDCC = -39;
#define vhm_VDCC vhm_VDCC_GluSynapse
 double vhm_VDCC = -5.9;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"tau_r_AMPA_GluSynapse", "ms"},
 {"E_AMPA_GluSynapse", "mV"},
 {"mg_GluSynapse", "mM"},
 {"scale_NMDA_GluSynapse", "mM"},
 {"slope_NMDA_GluSynapse", "/mV"},
 {"tau_r_NMDA_GluSynapse", "ms"},
 {"tau_d_NMDA_GluSynapse", "ms"},
 {"E_NMDA_GluSynapse", "mV"},
 {"gca_bar_VDCC_GluSynapse", "nS/um2"},
 {"ljp_VDCC_GluSynapse", "mV"},
 {"vhm_VDCC_GluSynapse", "mV"},
 {"km_VDCC_GluSynapse", "mV"},
 {"vhh_VDCC_GluSynapse", "mV"},
 {"kh_VDCC_GluSynapse", "mV"},
 {"mtau_VDCC_GluSynapse", "ms"},
 {"htau_VDCC_GluSynapse", "ms"},
 {"gamma_ca_CR_GluSynapse", "1"},
 {"tau_ca_CR_GluSynapse", "ms"},
 {"min_ca_CR_GluSynapse", "mM"},
 {"cao_CR_GluSynapse", "mM"},
 {"rho_star_GB_GluSynapse", "1"},
 {"tau_ind_GB_GluSynapse", "s"},
 {"tau_exp_GB_GluSynapse", "s"},
 {"tau_effca_GB_GluSynapse", "ms"},
 {"gamma_d_GB_GluSynapse", "1"},
 {"gamma_p_GB_GluSynapse", "1"},
 {"tau_d_AMPA", "ms"},
 {"gmax0_AMPA", "nS"},
 {"gmax_d_AMPA", "nS"},
 {"gmax_p_AMPA", "nS"},
 {"gmax_NMDA", "nS"},
 {"Use", "1"},
 {"Dep", "ms"},
 {"Fac", "ms"},
 {"Nrrp", "1"},
 {"Use_d", "1"},
 {"Use_p", "1"},
 {"volume_CR", "um3"},
 {"theta_d_GB", "us/liter"},
 {"theta_p_GB", "us/liter"},
 {"rho0_GB", "1"},
 {"A_AMPA", "1"},
 {"B_AMPA", "1"},
 {"gmax_AMPA", "nS"},
 {"A_NMDA", "1"},
 {"B_NMDA", "1"},
 {"Use_GB", "1"},
 {"m_VDCC", "1"},
 {"h_VDCC", "1"},
 {"cai_CR", "mM"},
 {"rho_GB", "1"},
 {"effcai_GB", "us/liter"},
 {"g_AMPA", "uS"},
 {"g_NMDA", "uS"},
 {"ica_NMDA", "nA"},
 {"ica_VDCC", "nA"},
 {"dep_GB", "1"},
 {"pot_GB", "1"},
 {"vsyn", "mV"},
 {"i", "nA"},
 {"next_delay", "ms"},
 {0, 0}
};
 static double A_NMDA0 = 0;
 static double A_AMPA0 = 0;
 static double B_NMDA0 = 0;
 static double B_AMPA0 = 0;
 static double Use_GB0 = 0;
 static double cai_CR0 = 0;
 static double delta_t = 0.01;
 static double effcai_GB0 = 0;
 static double gmax_AMPA0 = 0;
 static double h_VDCC0 = 0;
 static double m_VDCC0 = 0;
 static double rho_GB0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"tau_r_AMPA_GluSynapse", &tau_r_AMPA_GluSynapse},
 {"E_AMPA_GluSynapse", &E_AMPA_GluSynapse},
 {"mg_GluSynapse", &mg_GluSynapse},
 {"scale_NMDA_GluSynapse", &scale_NMDA_GluSynapse},
 {"slope_NMDA_GluSynapse", &slope_NMDA_GluSynapse},
 {"tau_r_NMDA_GluSynapse", &tau_r_NMDA_GluSynapse},
 {"tau_d_NMDA_GluSynapse", &tau_d_NMDA_GluSynapse},
 {"E_NMDA_GluSynapse", &E_NMDA_GluSynapse},
 {"gca_bar_VDCC_GluSynapse", &gca_bar_VDCC_GluSynapse},
 {"ljp_VDCC_GluSynapse", &ljp_VDCC_GluSynapse},
 {"vhm_VDCC_GluSynapse", &vhm_VDCC_GluSynapse},
 {"km_VDCC_GluSynapse", &km_VDCC_GluSynapse},
 {"vhh_VDCC_GluSynapse", &vhh_VDCC_GluSynapse},
 {"kh_VDCC_GluSynapse", &kh_VDCC_GluSynapse},
 {"mtau_VDCC_GluSynapse", &mtau_VDCC_GluSynapse},
 {"htau_VDCC_GluSynapse", &htau_VDCC_GluSynapse},
 {"gamma_ca_CR_GluSynapse", &gamma_ca_CR_GluSynapse},
 {"tau_ca_CR_GluSynapse", &tau_ca_CR_GluSynapse},
 {"min_ca_CR_GluSynapse", &min_ca_CR_GluSynapse},
 {"cao_CR_GluSynapse", &cao_CR_GluSynapse},
 {"rho_star_GB_GluSynapse", &rho_star_GB_GluSynapse},
 {"tau_ind_GB_GluSynapse", &tau_ind_GB_GluSynapse},
 {"tau_exp_GB_GluSynapse", &tau_exp_GB_GluSynapse},
 {"tau_effca_GB_GluSynapse", &tau_effca_GB_GluSynapse},
 {"gamma_d_GB_GluSynapse", &gamma_d_GB_GluSynapse},
 {"gamma_p_GB_GluSynapse", &gamma_p_GB_GluSynapse},
 {"nc_type_param_GluSynapse", &nc_type_param_GluSynapse},
 {"minis_single_vesicle_GluSynapse", &minis_single_vesicle_GluSynapse},
 {"init_depleted_GluSynapse", &init_depleted_GluSynapse},
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
 
#define _watch_array _ppvar + 6 
 static void _watch_alloc(Datum*);
 extern void hoc_reg_watch_allocate(int, void(*)(Datum*)); static void _hoc_destroy_pnt(void* _vptr) {
   Prop* _prop = ((Point_process*)_vptr)->_prop;
   if (_prop) { _nrn_free_watch(_nrn_mechanism_access_dparam(_prop), 6, 5);}
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[11].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"GluSynapse",
 "tau_d_AMPA",
 "gmax0_AMPA",
 "gmax_d_AMPA",
 "gmax_p_AMPA",
 "gmax_NMDA",
 "Use",
 "Dep",
 "Fac",
 "Nrrp",
 "Use_d",
 "Use_p",
 "volume_CR",
 "theta_d_GB",
 "theta_p_GB",
 "rho0_GB",
 "synapseID",
 "verbose",
 "selected_for_report",
 "conductance",
 0,
 "g_AMPA",
 "g_NMDA",
 "ica_NMDA",
 "ica_VDCC",
 "dep_GB",
 "pot_GB",
 "vsyn",
 "i",
 "next_delay",
 0,
 "A_AMPA",
 "B_AMPA",
 "gmax_AMPA",
 "A_NMDA",
 "B_NMDA",
 "Use_GB",
 "m_VDCC",
 "h_VDCC",
 "cai_CR",
 "rho_GB",
 "effcai_GB",
 0,
 "rng",
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
   _ppvar = nrn_prop_datum_alloc(_mechtype, 12, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 54);
 	/*initialize range parameters*/
 	tau_d_AMPA = 1.7;
 	gmax0_AMPA = 1;
 	gmax_d_AMPA = 1;
 	gmax_p_AMPA = 2;
 	gmax_NMDA = 0.55;
 	Use = 0.5;
 	Dep = 100;
 	Fac = 10;
 	Nrrp = 1;
 	Use_d = 0.2;
 	Use_p = 0.8;
 	volume_CR = 0.087;
 	theta_d_GB = 0.006;
 	theta_p_GB = 0.001;
 	rho0_GB = 0;
 	synapseID = 0;
 	verbose = 0;
 	selected_for_report = 0;
 	conductance = 0;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 54);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {"cai_CR", 1e-06},
 {"effcai_GB", 0.001},
 {0, 0}
};
 
#define _tqitem &(_ppvar[5])
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

 extern "C" void _GluSynapse_reg() {
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
                                       _nrn_mechanism_field<double>{"tau_d_AMPA"} /* 0 */,
                                       _nrn_mechanism_field<double>{"gmax0_AMPA"} /* 1 */,
                                       _nrn_mechanism_field<double>{"gmax_d_AMPA"} /* 2 */,
                                       _nrn_mechanism_field<double>{"gmax_p_AMPA"} /* 3 */,
                                       _nrn_mechanism_field<double>{"gmax_NMDA"} /* 4 */,
                                       _nrn_mechanism_field<double>{"Use"} /* 5 */,
                                       _nrn_mechanism_field<double>{"Dep"} /* 6 */,
                                       _nrn_mechanism_field<double>{"Fac"} /* 7 */,
                                       _nrn_mechanism_field<double>{"Nrrp"} /* 8 */,
                                       _nrn_mechanism_field<double>{"Use_d"} /* 9 */,
                                       _nrn_mechanism_field<double>{"Use_p"} /* 10 */,
                                       _nrn_mechanism_field<double>{"volume_CR"} /* 11 */,
                                       _nrn_mechanism_field<double>{"theta_d_GB"} /* 12 */,
                                       _nrn_mechanism_field<double>{"theta_p_GB"} /* 13 */,
                                       _nrn_mechanism_field<double>{"rho0_GB"} /* 14 */,
                                       _nrn_mechanism_field<double>{"synapseID"} /* 15 */,
                                       _nrn_mechanism_field<double>{"verbose"} /* 16 */,
                                       _nrn_mechanism_field<double>{"selected_for_report"} /* 17 */,
                                       _nrn_mechanism_field<double>{"conductance"} /* 18 */,
                                       _nrn_mechanism_field<double>{"g_AMPA"} /* 19 */,
                                       _nrn_mechanism_field<double>{"g_NMDA"} /* 20 */,
                                       _nrn_mechanism_field<double>{"ica_NMDA"} /* 21 */,
                                       _nrn_mechanism_field<double>{"ica_VDCC"} /* 22 */,
                                       _nrn_mechanism_field<double>{"dep_GB"} /* 23 */,
                                       _nrn_mechanism_field<double>{"pot_GB"} /* 24 */,
                                       _nrn_mechanism_field<double>{"vsyn"} /* 25 */,
                                       _nrn_mechanism_field<double>{"i"} /* 26 */,
                                       _nrn_mechanism_field<double>{"next_delay"} /* 27 */,
                                       _nrn_mechanism_field<double>{"A_AMPA"} /* 28 */,
                                       _nrn_mechanism_field<double>{"B_AMPA"} /* 29 */,
                                       _nrn_mechanism_field<double>{"gmax_AMPA"} /* 30 */,
                                       _nrn_mechanism_field<double>{"A_NMDA"} /* 31 */,
                                       _nrn_mechanism_field<double>{"B_NMDA"} /* 32 */,
                                       _nrn_mechanism_field<double>{"Use_GB"} /* 33 */,
                                       _nrn_mechanism_field<double>{"m_VDCC"} /* 34 */,
                                       _nrn_mechanism_field<double>{"h_VDCC"} /* 35 */,
                                       _nrn_mechanism_field<double>{"cai_CR"} /* 36 */,
                                       _nrn_mechanism_field<double>{"rho_GB"} /* 37 */,
                                       _nrn_mechanism_field<double>{"effcai_GB"} /* 38 */,
                                       _nrn_mechanism_field<double>{"usingR123"} /* 39 */,
                                       _nrn_mechanism_field<double>{"DA_AMPA"} /* 40 */,
                                       _nrn_mechanism_field<double>{"DB_AMPA"} /* 41 */,
                                       _nrn_mechanism_field<double>{"Dgmax_AMPA"} /* 42 */,
                                       _nrn_mechanism_field<double>{"DA_NMDA"} /* 43 */,
                                       _nrn_mechanism_field<double>{"DB_NMDA"} /* 44 */,
                                       _nrn_mechanism_field<double>{"DUse_GB"} /* 45 */,
                                       _nrn_mechanism_field<double>{"Dm_VDCC"} /* 46 */,
                                       _nrn_mechanism_field<double>{"Dh_VDCC"} /* 47 */,
                                       _nrn_mechanism_field<double>{"Dcai_CR"} /* 48 */,
                                       _nrn_mechanism_field<double>{"Drho_GB"} /* 49 */,
                                       _nrn_mechanism_field<double>{"Deffcai_GB"} /* 50 */,
                                       _nrn_mechanism_field<double>{"v"} /* 51 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 52 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 53 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"rng", "bbcorepointer"} /* 2 */,
                                       _nrn_mechanism_field<double*>{"delay_times", "bbcorepointer"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"delay_weights", "bbcorepointer"} /* 4 */,
                                       _nrn_mechanism_field<void*>{"_tqitem", "netsend"} /* 5 */,
                                       _nrn_mechanism_field<void*>{"_watch_array", "watch"} /* 6 */,
                                       _nrn_mechanism_field<void*>{"_watch_array", "watch"} /* 7 */,
                                       _nrn_mechanism_field<void*>{"_watch_array", "watch"} /* 8 */,
                                       _nrn_mechanism_field<void*>{"_watch_array", "watch"} /* 9 */,
                                       _nrn_mechanism_field<void*>{"_watch_array", "watch"} /* 10 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 11 */);
  hoc_register_prop_size(_mechtype, 54, 12);
  hoc_reg_watch_allocate(_mechtype, _watch_alloc);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 3, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 4, "bbcorepointer");
  hoc_register_dparam_semantics(_mechtype, 5, "netsend");
  hoc_register_dparam_semantics(_mechtype, 6, "watch");
  hoc_register_dparam_semantics(_mechtype, 7, "watch");
  hoc_register_dparam_semantics(_mechtype, 8, "watch");
  hoc_register_dparam_semantics(_mechtype, 9, "watch");
  hoc_register_dparam_semantics(_mechtype, 10, "watch");
  hoc_register_dparam_semantics(_mechtype, 11, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 6;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 GluSynapse GluSynapse.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 0x1.78e555060882cp+16;
 static double PI = 0x1.921fb54442d18p+1;
 static double R = 0x1.0a1013e8990bep+3;
static int _reset;
static const char *modelname = "Glutamatergic synapse";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int clearRNG(_internalthreadargsproto_);
static int setRNG(_internalthreadargsproto_);
static int setup_delay_vecs(_internalthreadargsproto_);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static double *_temp1;
 static neuron::container::field_index _slist1[11], _dlist1[11];
 static int state(_internalthreadargsproto_);
 
/*VERBATIM*/
/**
 * This Verbatim block is needed to generate random numbers from a uniform
 * distribution U(0, 1).
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifndef NRN_VERSION_GTEQ_8_2_0
#include "nrnran123.h"
double nrn_random_pick(void* r);
void* nrn_random_arg(int argpos);

#ifndef CORENEURON_BUILD
extern int ifarg(int iarg);

extern void* vector_arg(int iarg);
extern double* vector_vec(void* vv);
extern int vector_capacity(void* vv);
#endif
#define RANDCAST
#else
#define RANDCAST (Rand*)
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
   double _lminf_VDCC , _lhinf_VDCC ;
 DA_AMPA = - A_AMPA / tau_r_AMPA ;
   DB_AMPA = - B_AMPA / tau_d_AMPA ;
   Dgmax_AMPA = ( gmax_d_AMPA + rho_GB * ( gmax_p_AMPA - gmax_d_AMPA ) - gmax_AMPA ) / ( ( 1e3 ) * tau_exp_GB ) ;
   DA_NMDA = - A_NMDA / tau_r_NMDA ;
   DB_NMDA = - B_NMDA / tau_d_NMDA ;
   DUse_GB = ( Use_d + rho_GB * ( Use_p - Use_d ) - Use_GB ) / ( ( 1e3 ) * tau_exp_GB ) ;
   _lminf_VDCC = 1.0 / ( 1.0 + exp ( ( ( vhm_VDCC - ljp_VDCC ) - v ) / km_VDCC ) ) ;
   _lhinf_VDCC = 1.0 / ( 1.0 + exp ( ( ( vhh_VDCC - ljp_VDCC ) - v ) / kh_VDCC ) ) ;
   Dm_VDCC = ( _lminf_VDCC - m_VDCC ) / mtau_VDCC ;
   Dh_VDCC = ( _lhinf_VDCC - h_VDCC ) / htau_VDCC ;
   Dcai_CR = - ( 1e-9 ) * ( ica_NMDA + ica_VDCC ) * gamma_ca_CR / ( ( 1e-15 ) * volume_CR * 2.0 * FARADAY ) - ( cai_CR - min_ca_CR ) / tau_ca_CR ;
   Deffcai_GB = - effcai_GB / tau_effca_GB + ( cai_CR - min_ca_CR ) ;
   Drho_GB = ( - rho_GB * ( 1.0 - rho_GB ) * ( rho_star_GB - rho_GB ) + pot_GB * gamma_p_GB * ( 1.0 - rho_GB ) - dep_GB * gamma_d_GB * rho_GB ) / ( ( 1e3 ) * tau_ind_GB ) ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 double _lminf_VDCC , _lhinf_VDCC ;
 DA_AMPA = DA_AMPA  / (1. - dt*( ( - 1.0 ) / tau_r_AMPA )) ;
 DB_AMPA = DB_AMPA  / (1. - dt*( ( - 1.0 ) / tau_d_AMPA )) ;
 Dgmax_AMPA = Dgmax_AMPA  / (1. - dt*( ( ( ( - 1.0 ) ) ) / ( ( 1e3 ) * tau_exp_GB ) )) ;
 DA_NMDA = DA_NMDA  / (1. - dt*( ( - 1.0 ) / tau_r_NMDA )) ;
 DB_NMDA = DB_NMDA  / (1. - dt*( ( - 1.0 ) / tau_d_NMDA )) ;
 DUse_GB = DUse_GB  / (1. - dt*( ( ( ( - 1.0 ) ) ) / ( ( 1e3 ) * tau_exp_GB ) )) ;
 _lminf_VDCC = 1.0 / ( 1.0 + exp ( ( ( vhm_VDCC - ljp_VDCC ) - v ) / km_VDCC ) ) ;
 _lhinf_VDCC = 1.0 / ( 1.0 + exp ( ( ( vhh_VDCC - ljp_VDCC ) - v ) / kh_VDCC ) ) ;
 Dm_VDCC = Dm_VDCC  / (1. - dt*( ( ( ( - 1.0 ) ) ) / mtau_VDCC )) ;
 Dh_VDCC = Dh_VDCC  / (1. - dt*( ( ( ( - 1.0 ) ) ) / htau_VDCC )) ;
 Dcai_CR = Dcai_CR  / (1. - dt*( ( - ( ( 1.0 ) ) / tau_ca_CR ) )) ;
 Deffcai_GB = Deffcai_GB  / (1. - dt*( ( - 1.0 ) / tau_effca_GB )) ;
 Drho_GB = Drho_GB  / (1. - dt*( ( ( (( (( - 1.0 )*( ( 1.0 - rho_GB ) ) + ( - rho_GB )*( ( ( - 1.0 ) ) )) )*( ( rho_star_GB - rho_GB ) ) + ( - rho_GB * ( 1.0 - rho_GB ) )*( ( ( - 1.0 ) ) )) + ( pot_GB * gamma_p_GB )*( ( ( - 1.0 ) ) ) - ( dep_GB * gamma_d_GB )*( 1.0 ) ) ) / ( ( 1e3 ) * tau_ind_GB ) )) ;
  return 0;
}
 /*END CVODE*/
 
static int state (_internalthreadargsproto_) {
  int _reset=0;
  int error = 0;
 {
   double _lminf_VDCC , _lhinf_VDCC ;
 DA_AMPA = - A_AMPA / tau_r_AMPA ;
   DB_AMPA = - B_AMPA / tau_d_AMPA ;
   Dgmax_AMPA = ( gmax_d_AMPA + rho_GB * ( gmax_p_AMPA - gmax_d_AMPA ) - gmax_AMPA ) / ( ( 1e3 ) * tau_exp_GB ) ;
   DA_NMDA = - A_NMDA / tau_r_NMDA ;
   DB_NMDA = - B_NMDA / tau_d_NMDA ;
   DUse_GB = ( Use_d + rho_GB * ( Use_p - Use_d ) - Use_GB ) / ( ( 1e3 ) * tau_exp_GB ) ;
   _lminf_VDCC = 1.0 / ( 1.0 + exp ( ( ( vhm_VDCC - ljp_VDCC ) - v ) / km_VDCC ) ) ;
   _lhinf_VDCC = 1.0 / ( 1.0 + exp ( ( ( vhh_VDCC - ljp_VDCC ) - v ) / kh_VDCC ) ) ;
   Dm_VDCC = ( _lminf_VDCC - m_VDCC ) / mtau_VDCC ;
   Dh_VDCC = ( _lhinf_VDCC - h_VDCC ) / htau_VDCC ;
   Dcai_CR = - ( 1e-9 ) * ( ica_NMDA + ica_VDCC ) * gamma_ca_CR / ( ( 1e-15 ) * volume_CR * 2.0 * FARADAY ) - ( cai_CR - min_ca_CR ) / tau_ca_CR ;
   Deffcai_GB = - effcai_GB / tau_effca_GB + ( cai_CR - min_ca_CR ) ;
   Drho_GB = ( - rho_GB * ( 1.0 - rho_GB ) * ( rho_star_GB - rho_GB ) + pot_GB * gamma_p_GB * ( 1.0 - rho_GB ) - dep_GB * gamma_d_GB * rho_GB ) / ( ( 1e3 ) * tau_ind_GB ) ;
   }
 return _reset;}
 
static double _watch1_cond(Point_process* _pnt) {
   Datum* _ppvar; Datum* _thread{};
  NrnThread* _nt{static_cast<NrnThread*>(_pnt->_vnt)};
   auto* const _prop = _pnt->_prop;
  _nrn_mechanism_cache_instance _ml_real{_prop};
  auto* const _ml = &_ml_real;
  size_t _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  v = NODEV(_pnt->node);
	return  ( effcai_GB ) - ( theta_d_GB ) ;
}
 
static double _watch2_cond(Point_process* _pnt) {
   Datum* _ppvar; Datum* _thread{};
  NrnThread* _nt{static_cast<NrnThread*>(_pnt->_vnt)};
   auto* const _prop = _pnt->_prop;
  _nrn_mechanism_cache_instance _ml_real{_prop};
  auto* const _ml = &_ml_real;
  size_t _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  v = NODEV(_pnt->node);
	return  -( ( effcai_GB ) - ( theta_d_GB ) ) ;
}
 
static double _watch3_cond(Point_process* _pnt) {
   Datum* _ppvar; Datum* _thread{};
  NrnThread* _nt{static_cast<NrnThread*>(_pnt->_vnt)};
   auto* const _prop = _pnt->_prop;
  _nrn_mechanism_cache_instance _ml_real{_prop};
  auto* const _ml = &_ml_real;
  size_t _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  v = NODEV(_pnt->node);
	return  ( effcai_GB ) - ( theta_p_GB ) ;
}
 
static double _watch4_cond(Point_process* _pnt) {
   Datum* _ppvar; Datum* _thread{};
  NrnThread* _nt{static_cast<NrnThread*>(_pnt->_vnt)};
   auto* const _prop = _pnt->_prop;
  _nrn_mechanism_cache_instance _ml_real{_prop};
  auto* const _ml = &_ml_real;
  size_t _iml{};
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  v = NODEV(_pnt->node);
	return  -( ( effcai_GB ) - ( theta_p_GB ) ) ;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{  Prop* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   int _watch_rm = 0;
   _nrn_mechanism_cache_instance _ml_real{_pnt->_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
   _thread = nullptr; _nt = (NrnThread*)_pnt->_vnt;   _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  if (_tsav > t){ hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = nullptr;}
 {
   double _lp_rec , _lreleased , _ltp , _lfactor , _lrec ;
 if ( verbose > 0.0 ) {
      printf ( "Time = %g ms, incoming spike at synapse %g\n" , t , synapseID ) ;
      }
   if ( _lflag  == 0.0 ) {
     if ( _args[0] <= 0.0 ) {
       if ( verbose > 0.0 ) {
         printf ( "Inactive synapse, weight = %g\n" , _args[0] ) ;
         }
       }
     else {
       if ( verbose > 0.0 ) {
         printf ( "Flag 0, Regular spike\n" ) ;
         }
       _args[1] = Use_GB + _args[1] * ( 1.0 - Use_GB ) * exp ( - ( t - _args[2] ) / Fac ) ;
       if ( verbose > 0.0 ) {
         printf ( "\tVesicle release probability = %g\n" , _args[1] ) ;
         }
       _lp_rec = 1.0 - exp ( - ( t - _args[2] ) / Dep ) ;
       if ( verbose > 0.0 ) {
         printf ( "\tVesicle recovery probability = %g\n" , _lp_rec ) ;
         }
       if ( verbose > 0.0 ) {
         printf ( "\tVesicle available before recovery = %g\n" , _args[3] ) ;
         }
       _args[3] = _args[3] + brand ( _threadargscomma_ _args[4] , _lp_rec ) ;
       if ( verbose > 0.0 ) {
         printf ( "\tVesicles available after recovery = %g\n" , _args[3] ) ;
         }
       _lrec = _args[3] ;
       if ( _lrec > 1.0  && minis_single_vesicle  && _args[5]  == 1.0 ) {
         _lrec = 1.0 ;
         }
       _lreleased = brand ( _threadargscomma_ _lrec , _args[1] ) ;
       if ( verbose > 0.0 ) {
         printf ( "\tReleased %g vesicles out of %g\n" , _lreleased , _args[3] ) ;
         }
       _args[3] = _args[3] - _lreleased ;
       _args[4] = Nrrp - _args[3] ;
       if ( verbose > 0.0 ) {
         printf ( "\tFinal vesicle count, Recovered = %g, Unrecovered = %g, Nrrp = %g\n" , _args[3] , _args[4] , Nrrp ) ;
         }
       _ltp = ( tau_r_AMPA * tau_d_AMPA ) / ( tau_d_AMPA - tau_r_AMPA ) * log ( tau_d_AMPA / tau_r_AMPA ) ;
       _lfactor = 1.0 / ( - exp ( - _ltp / tau_r_AMPA ) + exp ( - _ltp / tau_d_AMPA ) ) ;
         if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for general derivimplicit and KINETIC case */
    int __i, __neq = 11;
    double __state = A_AMPA;
    double __primary_delta = (A_AMPA + _lreleased / Nrrp * _lfactor) - __state;
    double __dtsav = dt;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _dlist1[__i]) = 0.0;
    }
    _ml->data(_iml, _dlist1[0]) = __primary_delta;
    dt *= 0.5;
    v = NODEV(_pnt->node);
#if NRN_VECTORIZED
    _thread = _nt->_ml_list[_mechtype]->_thread;
#endif
    _ode_matsol_instance1(_threadargs_);
    dt = __dtsav;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _slist1[__i]) += _ml->data(_iml, _dlist1[__i]);
    }
  } else {
 A_AMPA = A_AMPA + _lreleased / Nrrp * _lfactor ;
         }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for general derivimplicit and KINETIC case */
    int __i, __neq = 11;
    double __state = B_AMPA;
    double __primary_delta = (B_AMPA + _lreleased / Nrrp * _lfactor) - __state;
    double __dtsav = dt;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _dlist1[__i]) = 0.0;
    }
    _ml->data(_iml, _dlist1[1]) = __primary_delta;
    dt *= 0.5;
    v = NODEV(_pnt->node);
#if NRN_VECTORIZED
    _thread = _nt->_ml_list[_mechtype]->_thread;
#endif
    _ode_matsol_instance1(_threadargs_);
    dt = __dtsav;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _slist1[__i]) += _ml->data(_iml, _dlist1[__i]);
    }
  } else {
 B_AMPA = B_AMPA + _lreleased / Nrrp * _lfactor ;
         }
 _ltp = ( tau_r_NMDA * tau_d_NMDA ) / ( tau_d_NMDA - tau_r_NMDA ) * log ( tau_d_NMDA / tau_r_NMDA ) ;
       _lfactor = 1.0 / ( - exp ( - _ltp / tau_r_NMDA ) + exp ( - _ltp / tau_d_NMDA ) ) ;
         if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for general derivimplicit and KINETIC case */
    int __i, __neq = 11;
    double __state = A_NMDA;
    double __primary_delta = (A_NMDA + _lreleased / Nrrp * _lfactor) - __state;
    double __dtsav = dt;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _dlist1[__i]) = 0.0;
    }
    _ml->data(_iml, _dlist1[3]) = __primary_delta;
    dt *= 0.5;
    v = NODEV(_pnt->node);
#if NRN_VECTORIZED
    _thread = _nt->_ml_list[_mechtype]->_thread;
#endif
    _ode_matsol_instance1(_threadargs_);
    dt = __dtsav;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _slist1[__i]) += _ml->data(_iml, _dlist1[__i]);
    }
  } else {
 A_NMDA = A_NMDA + _lreleased / Nrrp * _lfactor ;
         }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for general derivimplicit and KINETIC case */
    int __i, __neq = 11;
    double __state = B_NMDA;
    double __primary_delta = (B_NMDA + _lreleased / Nrrp * _lfactor) - __state;
    double __dtsav = dt;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _dlist1[__i]) = 0.0;
    }
    _ml->data(_iml, _dlist1[4]) = __primary_delta;
    dt *= 0.5;
    v = NODEV(_pnt->node);
#if NRN_VECTORIZED
    _thread = _nt->_ml_list[_mechtype]->_thread;
#endif
    _ode_matsol_instance1(_threadargs_);
    dt = __dtsav;
    for (__i = 0; __i < __neq; ++__i) {
      _ml->data(_iml, _slist1[__i]) += _ml->data(_iml, _dlist1[__i]);
    }
  } else {
 B_NMDA = B_NMDA + _lreleased / Nrrp * _lfactor ;
         }
 _args[2] = t ;
       }
     }
   else if ( _lflag  == 1.0 ) {
     if ( verbose > 0.0 ) {
       printf ( "Flag 1, Initialize watchers\n" ) ;
       }
       _nrn_watch_activate(_watch_array, _watch1_cond, 1, _pnt, _watch_rm++, 2.0);
   _nrn_watch_activate(_watch_array, _watch2_cond, 2, _pnt, _watch_rm++, 3.0);
   _nrn_watch_activate(_watch_array, _watch3_cond, 3, _pnt, _watch_rm++, 4.0);
   _nrn_watch_activate(_watch_array, _watch4_cond, 4, _pnt, _watch_rm++, 5.0);
 }
   else if ( _lflag  == 2.0 ) {
     if ( verbose > 0.0 ) {
       printf ( "Flag 2, Activate depression mechanisms\n" ) ;
       }
     dep_GB = 1.0 ;
     }
   else if ( _lflag  == 3.0 ) {
     if ( verbose > 0.0 ) {
       printf ( "Flag 3, Deactivate depression mechanisms\n" ) ;
       }
     dep_GB = 0.0 ;
     }
   else if ( _lflag  == 4.0 ) {
     if ( verbose > 0.0 ) {
       printf ( "Flag 4, Activate potentiation mechanisms\n" ) ;
       }
     pot_GB = 1.0 ;
     }
   else if ( _lflag  == 5.0 ) {
     if ( verbose > 0.0 ) {
       printf ( "Flag 5, Deactivate potentiation mechanisms\n" ) ;
       }
     pot_GB = 0.0 ;
     }
   else if ( _lflag  == 10.0 ) {
     
/*VERBATIM*/
        IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));
        if (vv_delay_weights && vector_capacity(vv_delay_weights)>=next_delay) {
            double* weights_v = vector_vec(vv_delay_weights);
            double next_delay_weight = weights_v[(int)next_delay];
 _args[0] = conductance * next_delay_weight ;
     next_delay = next_delay + 1.0 ;
     
/*VERBATIM*/
        }
 }
   } }
 
static void _net_init(Point_process* _pnt, double* _args, double _lflag) {
     _nrn_mechanism_cache_instance _ml_real{_pnt->_prop};
  auto* const _ml = &_ml_real;
  size_t const _iml{};
  Datum* _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  Datum* _thread = (Datum*)0;
  NrnThread* _nt = (NrnThread*)_pnt->_vnt;
 _args[0] = 1.0 ;
   _args[1] = 0.0 ;
   _args[2] = 0.0 ;
   if ( init_depleted ) {
     _args[3] = 0.0 ;
     _args[4] = Nrrp ;
     }
   else {
     _args[3] = Nrrp ;
     _args[4] = 0.0 ;
     }
   if ( _args[5]  == 0.0 ) {
     
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
 net_send ( _tqitem, _args, _pnt, t +  next_delay_t , 10.0 ) ;
     
/*VERBATIM*/
                }
            }
 }
   }
 
double nernst ( _internalthreadargsprotocomma_ double _lci , double _lco , double _lz ) {
   double _lnernst;
 _lnernst = ( 1000.0 ) * R * ( celsius + 273.15 ) / ( _lz * FARADAY ) * log ( _lco / _lci ) ;
   if ( verbose > 1.0 ) {
      printf ( "nernst:%g R:%g temperature (c):%g \n" , _lnernst , R , celsius ) ;
      }
   
return _lnernst;
 }
 
static double _hoc_nernst(void* _vptr) {
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
 _r =  nernst ( _threadargscomma_ *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
static int  setRNG ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
    #ifndef CORENEURON_BUILD
    // For compatibility, allow for either MCellRan4 or Random123
    // Distinguish by the arg types
    // Object => MCellRan4, seeds (double) => Random123
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
    } else if( ifarg(1) ) {   // not a double, so assume hoc object type
        void** pv = (void**)(&_p_rng);
        *pv = nrn_random_arg(1);
    } else {  // no arg, so clear pointer
        void** pv = (void**)(&_p_rng);
        *pv = (void*)0;
    }
    #endif
  return 0; }
 
static double _hoc_setRNG(void* _vptr) {
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
 setRNG ( _threadargs_ );
 return(_r);
}
 
static int  clearRNG ( _internalthreadargsproto_ ) {
   
/*VERBATIM*/
    #ifndef CORENEURON_BUILD
    if (usingR123) {
        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
        if (*pv) {
            nrnran123_deletestream(*pv);
            *pv = (nrnran123_State*)0;
        }
    } else {
        void** pv = (void**)(&_p_rng);
        if (*pv) {
            *pv = (void*)0;
        }
    }
    #endif
  return 0; }
 
static double _hoc_clearRNG(void* _vptr) {
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
 clearRNG ( _threadargs_ );
 return(_r);
}
 
double urand ( _internalthreadargsproto_ ) {
   double _lurand;
 
/*VERBATIM*/
    double value;
    if ( usingR123 ) {
        value = nrnran123_dblpick((nrnran123_State*)_p_rng);
    } else if (_p_rng) {
        #ifndef CORENEURON_BUILD
        value = nrn_random_pick(RANDCAST _p_rng);
        #endif
    } else {
        value = 0.0;
    }
    _lurand = value;
 
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
 
double brand ( _internalthreadargsprotocomma_ double _ln , double _lp ) {
   double _lbrand;
 double _lresult , _lcount , _lsuccess ;
 _lsuccess = 0.0 ;
   {int  _lcount ;for ( _lcount = 0 ; _lcount <= ( ((int) _ln ) - 1 ) ; _lcount ++ ) {
     _lresult = urand ( _threadargs_ ) ;
     if ( _lresult <= _lp ) {
       _lsuccess = _lsuccess + 1.0 ;
       }
     } }
   _lbrand = _lsuccess ;
   
return _lbrand;
 }
 
static double _hoc_brand(void* _vptr) {
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
 _r =  brand ( _threadargscomma_ *getarg(1) , *getarg(2) );
 return(_r);
}
 
double bbsavestate ( _internalthreadargsproto_ ) {
   double _lbbsavestate;
 _lbbsavestate = 0.0 ;
   
/*VERBATIM*/
    #ifndef CORENEURON_BUILD
        /* first arg is direction (0 save, 1 restore), second is array*/
        /* if first arg is -1, fill xdir with the size of the array */
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
            if (*xdir == -1) {  // count items
                if( usingR123 ) {
                    *xdir = 2.0;
                } else {
                    *xdir = 1.0;
                }
                return 0.0;
            } else if(*xdir ==0 ) {  // save
                if( usingR123 ) {
                    uint32_t seq;
                    char which;
                    nrnran123_getseq( (nrnran123_State*)_p_rng, &seq, &which );
                    xval[0] = (double) seq;
                    xval[1] = (double) which;
                } else {
                    xval[0] = (double)nrn_get_random_sequence(RANDCAST _p_rng);
                }
            } else {  // restore
                if( usingR123 ) {
                    nrnran123_setseq( (nrnran123_State*)_p_rng, (uint32_t)xval[0], (char)xval[1] );
                } else {
                    nrn_set_random_sequence(RANDCAST _p_rng, (long)(xval[0]));
                }
            }
        }
    #endif
 
return _lbbsavestate;
 }
 
static double _hoc_bbsavestate(void* _vptr) {
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
 _r =  bbsavestate ( _threadargs_ );
 return(_r);
}
 
/*VERBATIM*/
static void bbcore_write(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    IvocVect *vv_delay_times = *((IvocVect**)(&_p_delay_times));
    IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));

    // make sure offset array non-null
    if (iArray) {
        // get handle to random123 instance
        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
        // get location for storing ids
        uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
        // retrieve/store identifier seeds
        nrnran123_getids3(*pv, ia, ia+1, ia+2);
        // retrieve/store stream sequence
        char which;
        nrnran123_getseq(*pv, ia+3, &which);
        ia[4] = (int)which;
    }

    // increment integer offset (2 identifier), no double data
    *ioffset += 5;
    *doffset += 0;

    // serialize connection delay vectors
    if (vv_delay_times && vv_delay_weights &&
       (vector_capacity(vv_delay_times) >= 1) && (vector_capacity(vv_delay_weights) >= 1)) {
        if (iArray) {
            uint32_t* di = ((uint32_t*)iArray) + *ioffset;
            // store vector sizes for deserialization
            di[0] = vector_capacity(vv_delay_times);
            di[1] = vector_capacity(vv_delay_weights);
        }
        if (dArray) {
            double* delay_times_el = vector_vec(vv_delay_times);
            double* delay_weights_el = vector_vec(vv_delay_weights);
            double* x_i = dArray + *doffset;
            int delay_vecs_idx;
            int x_idx = 0;
            for(delay_vecs_idx = 0; delay_vecs_idx < vector_capacity(vv_delay_times); ++delay_vecs_idx) {
                 x_i[x_idx++] = delay_times_el[delay_vecs_idx];
                 x_i[x_idx++] = delay_weights_el[delay_vecs_idx];
            }
        }
        // reserve space for connection delay data on serialization buffer
        *doffset += vector_capacity(vv_delay_times) + vector_capacity(vv_delay_weights);
    } else {
        if (iArray) {
            uint32_t* di = ((uint32_t*)iArray) + *ioffset;
            di[0] = 0;
            di[1] = 0;
        }
    }
    // reserve space for delay vectors (may be 0)
    *ioffset += 2;
}

static void bbcore_read(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    // make sure it's not previously set
    assert(!_p_rng);
    assert(!_p_delay_times && !_p_delay_weights);

    uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
    // make sure non-zero identifier seeds
    if (ia[0] != 0 || ia[1] != 0 || ia[2] != 0) {
        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
        // get new stream
        *pv = nrnran123_newstream3(ia[0], ia[1], ia[2]);
        // restore sequence
        nrnran123_setseq(*pv, ia[3], (char)ia[4]);
    }
    // increment intger offset (2 identifiers), no double data
    *ioffset += 5;

    int delay_times_sz = iArray[5];
    int delay_weights_sz = iArray[6];
    *ioffset += 2;

    if ((delay_times_sz > 0) && (delay_weights_sz > 0)) {
        double* x_i = dArray + *doffset;

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
        *doffset += delay_times_sz + delay_weights_sz;
    }
}
 
static void _watch_alloc(Datum* _ppvar) {
  auto* _pnt = _ppvar[1].get<Point_process*>();
   _nrn_watch_allocate(_watch_array, _watch1_cond, 1, _pnt, 2.0);
   _nrn_watch_allocate(_watch_array, _watch2_cond, 2, _pnt, 3.0);
   _nrn_watch_allocate(_watch_array, _watch3_cond, 3, _pnt, 4.0);
   _nrn_watch_allocate(_watch_array, _watch4_cond, 4, _pnt, 5.0);
 }

 
static int _ode_count(int _type){ return 11;}
 
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
  for (int _i=0; _i < 11; ++_i) {
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
  A_NMDA = A_NMDA0;
  A_AMPA = A_AMPA0;
  B_NMDA = B_NMDA0;
  B_AMPA = B_AMPA0;
  Use_GB = Use_GB0;
  cai_CR = cai_CR0;
  effcai_GB = effcai_GB0;
  gmax_AMPA = gmax_AMPA0;
  h_VDCC = h_VDCC0;
  m_VDCC = m_VDCC0;
  rho_GB = rho_GB0;
 {
   A_AMPA = 0.0 ;
   B_AMPA = 0.0 ;
   gmax_AMPA = gmax0_AMPA ;
   A_NMDA = 0.0 ;
   B_NMDA = 0.0 ;
   Use_GB = Use ;
   cai_CR = min_ca_CR ;
   rho_GB = rho0_GB ;
   effcai_GB = 0.0 ;
   dep_GB = 0.0 ;
   pot_GB = 0.0 ;
   next_delay = - 1.0 ;
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
{ {
   double _lEca_syn , _lmggate , _li_AMPA , _li_NMDA , _lPf_NMDA , _lgca_bar_abs_VDCC , _lgca_VDCC ;
 g_AMPA = ( 1e-3 ) * gmax_AMPA * ( B_AMPA - A_AMPA ) ;
   _li_AMPA = g_AMPA * ( v - E_AMPA ) ;
   _lmggate = 1.0 / ( 1.0 + exp ( - slope_NMDA * v ) * ( mg / scale_NMDA ) ) ;
   g_NMDA = ( 1e-3 ) * gmax_NMDA * _lmggate * ( B_NMDA - A_NMDA ) ;
   _li_NMDA = g_NMDA * ( v - E_NMDA ) ;
   _lPf_NMDA = ( 4.0 * cao_CR ) / ( 4.0 * cao_CR + ( 1.0 / 1.38 ) * 120.0 ) * 0.6 ;
   ica_NMDA = _lPf_NMDA * g_NMDA * ( v - 40.0 ) ;
   _lgca_bar_abs_VDCC = gca_bar_VDCC * 4.0 * PI * pow( ( 3.0 / 4.0 * volume_CR * 1.0 / PI ) , ( 2.0 / 3.0 ) ) ;
   _lgca_VDCC = ( 1e-3 ) * _lgca_bar_abs_VDCC * m_VDCC * m_VDCC * h_VDCC ;
   _lEca_syn = nernst ( _threadargscomma_ cai_CR , cao_CR , 2.0 ) ;
   ica_VDCC = _lgca_VDCC * ( v - _lEca_syn ) ;
   vsyn = v ;
   i = _li_AMPA + _li_NMDA + ica_VDCC ;
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
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
_ni = _ml_arg->_nodeindices;
size_t _cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
for (size_t _iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _nd = _ml_arg->_nodelist[_iml];
   _v = _vec_v[_ni[_iml]];
 v=_v;
{
 {   euler_thread(11, _slist1, _dlist1, neuron::scopmath::row_view{_ml, _iml}, state, _ml, _iml, _ppvar, _thread, _nt);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 11; ++_i) {
      _ml->data(_iml, _slist1[_i]) += dt*_ml->data(_iml, _dlist1[_i]);
    }}
 }}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {A_AMPA_columnindex, 0};  _dlist1[0] = {DA_AMPA_columnindex, 0};
 _slist1[1] = {B_AMPA_columnindex, 0};  _dlist1[1] = {DB_AMPA_columnindex, 0};
 _slist1[2] = {gmax_AMPA_columnindex, 0};  _dlist1[2] = {Dgmax_AMPA_columnindex, 0};
 _slist1[3] = {A_NMDA_columnindex, 0};  _dlist1[3] = {DA_NMDA_columnindex, 0};
 _slist1[4] = {B_NMDA_columnindex, 0};  _dlist1[4] = {DB_NMDA_columnindex, 0};
 _slist1[5] = {Use_GB_columnindex, 0};  _dlist1[5] = {DUse_GB_columnindex, 0};
 _slist1[6] = {m_VDCC_columnindex, 0};  _dlist1[6] = {Dm_VDCC_columnindex, 0};
 _slist1[7] = {h_VDCC_columnindex, 0};  _dlist1[7] = {Dh_VDCC_columnindex, 0};
 _slist1[8] = {cai_CR_columnindex, 0};  _dlist1[8] = {Dcai_CR_columnindex, 0};
 _slist1[9] = {effcai_GB_columnindex, 0};  _dlist1[9] = {Deffcai_GB_columnindex, 0};
 _slist1[10] = {rho_GB_columnindex, 0};  _dlist1[10] = {Drho_GB_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "GluSynapse.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "/**\n"
  " * @file GluSynapse.mod\n"
  " * @brief Probabilistic synapse featuring long-term plasticity\n"
  " * @author king, chindemi, rossert\n"
  " * @date 2021-05-19\n"
  " * @version 1.0.1\n"
  " * @remark Copyright BBP/EPFL 2005-2021; All rights reserved.\n"
  "           Do not distribute without further notice.\n"
  " */\n"
  " Glutamatergic synapse model featuring:\n"
  "1) AMPA receptor with a dual-exponential conductance profile.\n"
  "2) NMDA receptor  with a dual-exponential conductance profile and magnesium\n"
  "   block as described in Jahr and Stevens 1990.\n"
  "3) Tsodyks-Markram presynaptic short-term plasticity as Barros et al. 2019.\n"
  "   Implementation based on the work of Eilif Muller, Michael Reimann and\n"
  "   Srikanth Ramaswamy (Blue Brain Project, August 2011), who introduced the\n"
  "   2-state Markov model of vesicle release. The new model is an extension of\n"
  "   Fuhrmann et al. 2002, motivated by the following constraints:\n"
  "        a) No consumption on failure\n"
  "        b) No release until recovery\n"
  "        c) Same ensemble averaged trace as canonical Tsodyks-Markram using same\n"
  "           parameters determined from experiment.\n"
  "   For a pre-synaptic spike or external spontaneous release trigger event, the\n"
  "   synapse will only release if it is in the recovered state, and with\n"
  "   probability u (which follows facilitation dynamics). If it releases, it will\n"
  "   transition to the unrecovered state. Recovery is as a Poisson process with\n"
  "   rate 1/Dep.\n"
  "   John Rahmon and Giuseppe Chindemi introduced multi-vesicular release as an\n"
  "   extension of the 2-state Markov model of vesicle release described above\n"
  "   (Blue Brain Project, February 2017).\n"
  "4) NMDAR-mediated calcium current. Fractional calcium current Pf_NMDA from\n"
  "   Schneggenburger et al. 1993. Fractional NMDAR conductance treated as a\n"
  "   calcium-only permeable channel with Erev = 40 mV independent of extracellular\n"
  "   calcium concentration (see Jahr and Stevens 1993). Implemented by Christian\n"
  "   Rossert and Giuseppe Chindemi (Blue Brain Project, 2016).\n"
  "5) Spine volume.\n"
  "6) VDCC.\n"
  "7) Postsynaptic calcium dynamics.\n"
  "8) Long-term synaptic plasticity. Calcium-based STDP model based on Graupner and\n"
  "   Brunel 2012.\n"
  "Model implementation, optimization and simulation curated by James King (Blue\n"
  "Brain Project, 2021).\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "TITLE Glutamatergic synapse\n"
  "\n"
  "NEURON {\n"
  "    THREADSAFE\n"
  "    POINT_PROCESS GluSynapse\n"
  "    : AMPA Receptor\n"
  "    GLOBAL tau_r_AMPA, E_AMPA\n"
  "    RANGE tau_d_AMPA, gmax0_AMPA, gmax_d_AMPA, gmax_p_AMPA, g_AMPA\n"
  "    : NMDA Receptor\n"
  "    GLOBAL scale_NMDA, slope_NMDA\n"
  "    GLOBAL tau_r_NMDA, tau_d_NMDA, E_NMDA\n"
  "    RANGE gmax_NMDA, g_NMDA\n"
  "    GLOBAL mg\n"
  "    : Stochastic Tsodyks-Markram Multi-Vesicular Release\n"
  "    RANGE Use, Dep, Fac, Nrrp\n"
  "    RANGE Use_d, Use_p\n"
  "    BBCOREPOINTER rng\n"
  "    : NMDAR-mediated calcium current\n"
  "    RANGE ica_NMDA\n"
  "    : Spine\n"
  "    RANGE volume_CR\n"
  "    : VDCC (R-type)\n"
  "    GLOBAL ljp_VDCC, vhm_VDCC, km_VDCC, mtau_VDCC, vhh_VDCC, kh_VDCC, htau_VDCC, gca_bar_VDCC\n"
  "    RANGE ica_VDCC\n"
  "    : Postsynaptic Ca2+ dynamics\n"
  "    GLOBAL gamma_ca_CR, tau_ca_CR, min_ca_CR, cao_CR\n"
  "    : Long-term synaptic plasticity\n"
  "    GLOBAL rho_star_GB, tau_ind_GB, tau_exp_GB, tau_effca_GB\n"
  "    GLOBAL gamma_d_GB, gamma_p_GB\n"
  "    RANGE theta_d_GB, theta_p_GB, rho0_GB, dep_GB, pot_GB\n"
  "    : Misc\n"
  "    RANGE vsyn, synapseID, selected_for_report, verbose\n"
  "    NONSPECIFIC_CURRENT i\n"
  "    RANGE conductance\n"
  "    RANGE next_delay\n"
  "    BBCOREPOINTER delay_times, delay_weights\n"
  "    GLOBAL nc_type_param\n"
  "    GLOBAL minis_single_vesicle\n"
  "    GLOBAL init_depleted\n"
  "    : For debugging\n"
  "    :RANGE sgid, tgid\n"
  "}\n"
  "\n"
  "\n"
  "UNITS {\n"
  "    (nA)    = (nanoamp)\n"
  "    (mV)    = (millivolt)\n"
  "    (uS)    = (microsiemens)\n"
  "    (nS)    = (nanosiemens)\n"
  "    (pS)    = (picosiemens)\n"
  "    (umho)  = (micromho)\n"
  "    (um)    = (micrometers)\n"
  "    (mM)    = (milli/liter)\n"
  "    (uM)    = (micro/liter)\n"
  "    FARADAY = (faraday) (coulomb)\n"
  "    PI      = (pi)      (1)\n"
  "    R       = (k-mole)  (joule/degC)\n"
  "}\n"
  "\n"
  "\n"
  "PARAMETER {\n"
  "    celsius                     (degC)\n"
  "    : AMPA Receptor\n"
  "    tau_r_AMPA      = 0.2       (ms)        : Tau rise, dual-exponential conductance profile\n"
  "    tau_d_AMPA      = 1.7       (ms)        : Tau decay, IMPORTANT: tau_r < tau_d\n"
  "    E_AMPA          = 0         (mV)        : Reversal potential\n"
  "    gmax0_AMPA      = 1.0       (nS)        : Initial peak conductance\n"
  "    gmax_d_AMPA     = 1.0       (nS)        : Peak conductance in the depressed state\n"
  "    gmax_p_AMPA     = 2.0       (nS)        : Peak conductance in the potentitated state\n"
  "    : NMDA Receptor\n"
  "    mg              = 1         (mM)        : Extracellular magnesium concentration\n"
  "    scale_NMDA      = 2.552     (mM)        : Scale of the mg block (Vargas-Caballero and Robinson 2003)\n"
  "    slope_NMDA      = 0.072     (/mV)       : Slope of the ma block (Vargas-Caballero and Robinson 2003)\n"
  "    tau_r_NMDA      = 0.29      (ms)        : Tau rise, dual-exponential conductance profile\n"
  "    tau_d_NMDA      = 70        (ms)        : Tau decay, IMPORTANT: tau_r < tau_d\n"
  "    E_NMDA          = -3        (mV)        : Reversal potential (Vargas-Caballero and Robinson 2003)\n"
  "    gmax_NMDA       = 0.55      (nS)        : Peak conductance\n"
  "    : Stochastic Tsodyks-Markram Multi-Vesicular Release\n"
  "    Use             = 0.5       (1)         : Initial utilization of synaptic efficacy\n"
  "    Dep             = 100       (ms)        : Relaxation time constant from depression\n"
  "    Fac             = 10        (ms)        : Relaxation time constant from facilitation\n"
  "    Nrrp            = 1         (1)         : Number of release sites for given contact\n"
  "    Use_d           = 0.2       (1)         : Depressed Use\n"
  "    Use_p           = 0.8       (1)         : Potentiated Use\n"
  "    : Spine\n"
  "    volume_CR       = 0.087     (um3)       : From spine data by Ruth Benavides-Piccione (unpublished)\n"
  "    : VDCC (R-type)\n"
  "    gca_bar_VDCC    = 0.0744    (nS/um2)    : Density spines: 20 um-2 (Sabatini 2000), unitary conductance VGCC 3.72 pS (Bartol 2015)\n"
  "    ljp_VDCC        = 0         (mV)\n"
  "    vhm_VDCC        = -5.9      (mV)        : v 1/2 for act, Magee and Johnston 1995 (corrected for m*m)\n"
  "    km_VDCC         = 9.5       (mV)        : act slope, Magee and Johnston 1995 (corrected for m*m)\n"
  "    vhh_VDCC        = -39       (mV)        : v 1/2 for inact, Magee and Johnston 1995\n"
  "    kh_VDCC         = -9.2      (mV)        : inact, Magee and Johnston 1995\n"
  "    mtau_VDCC       = 1         (ms)        : max time constant (guess)\n"
  "    htau_VDCC       = 27        (ms)        : max time constant 100*0.27\n"
  "    : Postsynaptic Ca2+ dynamics\n"
  "    gamma_ca_CR     = 0.04      (1)         : Percent of free calcium (not buffered), Sabatini et al 2002: kappa_e = 24+-11 (also 14 (2-31) or 22 (18-33))\n"
  "    tau_ca_CR       = 12        (ms)        : Rate of removal of calcium, Sabatini et al 2002: 14ms (12-20ms)\n"
  "    min_ca_CR       = 70e-6     (mM)        : Sabatini et al 2002: 70+-29 nM, per AP: 1.1 (0.6-8.2) uM = 1100 e-6 mM = 1100 nM\n"
  "    cao_CR          = 2.0       (mM)        : Extracellular calcium concentration in slices\n"
  "    : Long-term synaptic plasticity\n"
  "    rho_star_GB     = 0.5       (1)\n"
  "    tau_ind_GB      = 70        (s)\n"
  "    tau_exp_GB      = 100       (s)\n"
  "    tau_effca_GB    = 200       (ms)\n"
  "    gamma_d_GB      = 100       (1)\n"
  "    gamma_p_GB      = 450       (1)\n"
  "    theta_d_GB      = 0.006     (us/liter)\n"
  "    theta_p_GB      = 0.001     (us/liter)\n"
  "    rho0_GB         = 0         (1)\n"
  "    : Misc\n"
  "    synapseID       = 0\n"
  "    verbose         = 0\n"
  "    selected_for_report = 0\n"
  "    conductance     = 0.0\n"
  "    nc_type_param = 5\n"
  "    minis_single_vesicle = 0   :// 0 -> no limit (old behavior)\n"
  "    init_depleted = 0          :// 0 -> init full (old behavior)\n"
  "    :sgid = -1\n"
  "    :tgid = -1\n"
  "}\n"
  "\n"
  "\n"
  "VERBATIM\n"
  "/**\n"
  " * This Verbatim block is needed to generate random numbers from a uniform\n"
  " * distribution U(0, 1).\n"
  " */\n"
  "#include <stdlib.h>\n"
  "#include <stdio.h>\n"
  "#include <math.h>\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "#include \"nrnran123.h\"\n"
  "double nrn_random_pick(void* r);\n"
  "void* nrn_random_arg(int argpos);\n"
  "\n"
  "#ifndef CORENEURON_BUILD\n"
  "extern int ifarg(int iarg);\n"
  "\n"
  "extern void* vector_arg(int iarg);\n"
  "extern double* vector_vec(void* vv);\n"
  "extern int vector_capacity(void* vv);\n"
  "#endif\n"
  "#define RANDCAST\n"
  "#else\n"
  "#define RANDCAST (Rand*)\n"
  "#endif\n"
  "\n"
  "\n"
  "ENDVERBATIM\n"
  "\n"
  "\n"
  "ASSIGNED {\n"
  "    g_AMPA          (uS)    : AMPA Receptor\n"
  "    g_NMDA          (uS)    : NMDA Receptor\n"
  "    : Stochastic Tsodyks-Markram Multi-Vesicular Release\n"
  "    rng                     : Random Number Generator\n"
  "    usingR123               : TEMPORARY until mcellran4 completely deprecated\n"
  "    ica_NMDA        (nA)    : NMDAR-mediated calcium current\n"
  "    ica_VDCC        (nA)    : VDCC (R-type)\n"
  "    : Long-term synaptic plasticity\n"
  "    dep_GB          (1)\n"
  "    pot_GB          (1)\n"
  "    : Misc\n"
  "    v               (mV)\n"
  "    vsyn            (mV)\n"
  "    i               (nA)\n"
  "\n"
  "    : stuff for delayed connections\n"
  "    delay_times\n"
  "    delay_weights\n"
  "    next_delay (ms)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    : AMPA Receptor\n"
  "    A_AMPA      (1)\n"
  "    B_AMPA      (1)\n"
  "    gmax_AMPA   (nS)\n"
  "    : NMDA Receptor\n"
  "    A_NMDA      (1)\n"
  "    B_NMDA      (1)\n"
  "    : Stochastic Tsodyks-Markram Multi-Vesicular Release\n"
  "    Use_GB      (1)\n"
  "    : VDCC (R-type)\n"
  "    m_VDCC      (1)\n"
  "    h_VDCC      (1)\n"
  "    : Postsynaptic Ca2+ dynamics\n"
  "    cai_CR      (mM)        <1e-6>\n"
  "    : Long-term synaptic plasticity\n"
  "    rho_GB      (1)\n"
  "    effcai_GB   (us/liter)  <1e-3>\n"
  "}\n"
  "\n"
  "INITIAL{\n"
  "    : AMPA Receptor\n"
  "    A_AMPA      = 0\n"
  "    B_AMPA      = 0\n"
  "    gmax_AMPA   = gmax0_AMPA\n"
  "    : NMDA Receptor\n"
  "    A_NMDA      = 0\n"
  "    B_NMDA      = 0\n"
  "    : Stochastic Tsodyks-Markram Multi-Vesicular Release\n"
  "    Use_GB      = Use\n"
  "    : Postsynaptic Ca2+ dynamics\n"
  "    cai_CR      = min_ca_CR\n"
  "    : Long-term synaptic plasticity\n"
  "    rho_GB      = rho0_GB\n"
  "    effcai_GB   = 0\n"
  "    dep_GB      = 0\n"
  "    pot_GB      = 0\n"
  "    : Delayed connection\n"
  "    next_delay = -1\n"
  "\n"
  "    : Initialize watchers\n"
  "    net_send(0, 1)\n"
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
  "BREAKPOINT {\n"
  "    LOCAL Eca_syn, mggate, i_AMPA, i_NMDA, Pf_NMDA, gca_bar_abs_VDCC, gca_VDCC\n"
  "    SOLVE state METHOD euler\n"
  "    : AMPA Receptor\n"
  "    g_AMPA = (1e-3)*gmax_AMPA*(B_AMPA - A_AMPA)\n"
  "    i_AMPA = g_AMPA*(v-E_AMPA)\n"
  "    : NMDA Receptor\n"
  "    mggate = 1 / (1 + exp(-slope_NMDA*v) * (mg/scale_NMDA))\n"
  "    g_NMDA = (1e-3)*gmax_NMDA*mggate*(B_NMDA - A_NMDA)\n"
  "    i_NMDA = g_NMDA*(v - E_NMDA)\n"
  "    : NMDAR-mediated calcium current\n"
  "    Pf_NMDA  = (4*cao_CR) / (4*cao_CR + (1/1.38) * 120 (mM)) * 0.6\n"
  "    ica_NMDA = Pf_NMDA*g_NMDA*(v-40.0)\n"
  "    : VDCC (R-type), assuming sphere for spine head\n"
  "    gca_bar_abs_VDCC = gca_bar_VDCC * 4(um2)*PI*(3(1/um3)/4*volume_CR*1/PI)^(2/3)\n"
  "    gca_VDCC = (1e-3) * gca_bar_abs_VDCC * m_VDCC * m_VDCC * h_VDCC\n"
  "    Eca_syn = nernst(cai_CR, cao_CR, 2)\n"
  "    ica_VDCC = gca_VDCC*(v-Eca_syn)\n"
  "    : Update synaptic voltage (for recording convenience)\n"
  "    vsyn = v\n"
  "    : Update current\n"
  "    i = i_AMPA + i_NMDA + ica_VDCC\n"
  "}\n"
  "\n"
  "\n"
  "DERIVATIVE state {\n"
  "    LOCAL minf_VDCC, hinf_VDCC\n"
  "    : AMPA Receptor\n"
  "    A_AMPA'      = - A_AMPA/tau_r_AMPA\n"
  "    B_AMPA'      = - B_AMPA/tau_d_AMPA\n"
  "    gmax_AMPA'   = (gmax_d_AMPA + rho_GB*(gmax_p_AMPA - gmax_d_AMPA) - gmax_AMPA) / ((1e3)*tau_exp_GB)\n"
  "    : NMDA Receptor\n"
  "    A_NMDA'      = - A_NMDA/tau_r_NMDA\n"
  "    B_NMDA'      = - B_NMDA/tau_d_NMDA\n"
  "    : Stochastic Tsodyks-Markram Multi-Vesicular Release\n"
  "    Use_GB'         = (Use_d + rho_GB*(Use_p - Use_d) - Use_GB) / ((1e3)*tau_exp_GB)\n"
  "    : VDCC (R-type)\n"
  "    minf_VDCC    = 1 / (1 + exp(((vhm_VDCC - ljp_VDCC) - v) / km_VDCC))\n"
  "    hinf_VDCC    = 1 / (1 + exp(((vhh_VDCC - ljp_VDCC) - v) / kh_VDCC))\n"
  "    m_VDCC'      = (minf_VDCC-m_VDCC)/mtau_VDCC\n"
  "    h_VDCC'      = (hinf_VDCC-h_VDCC)/htau_VDCC\n"
  "    : Postsynaptic Ca2+ dynamics\n"
  "    cai_CR'      = - (1e-9)*(ica_NMDA + ica_VDCC)*gamma_ca_CR/((1e-15)*volume_CR*2*FARADAY)\n"
  "                   - (cai_CR - min_ca_CR)/tau_ca_CR\n"
  "    : Long-term synaptic plasticity\n"
  "    effcai_GB'   = - effcai_GB/tau_effca_GB + (cai_CR - min_ca_CR)\n"
  "    rho_GB'      = ( - rho_GB*(1 - rho_GB)*(rho_star_GB - rho_GB)\n"
  "                     + pot_GB*gamma_p_GB*(1 - rho_GB)\n"
  "                     - dep_GB*gamma_d_GB*rho_GB ) / ((1e3)*tau_ind_GB)\n"
  "}\n"
  "\n"
  "\n"
  "NET_RECEIVE (weight, u, tsyn (ms), recovered, unrecovered, nc_type) {\n"
  "    : nc_type: 0=presynaptic netcon, 1=spontmini, 2=replay\n"
  "    LOCAL p_rec, released, tp, factor, rec\n"
  "    INITIAL {\n"
  "        weight = 1\n"
  "        u = 0\n"
  "        tsyn = 0 (ms)\n"
  "        if (init_depleted){\n"
  "            recovered = 0\n"
  "            unrecovered = Nrrp\n"
  "        } else {\n"
  "            recovered = Nrrp\n"
  "            unrecovered = 0\n"
  "        }\n"
  "        if (nc_type == 0) {   : pre-synaptic netcon\n"
  "    VERBATIM\n"
  "            // setup self events for delayed connections to change weights\n"
  "            IvocVect *vv_delay_times = *((IvocVect**)(&_p_delay_times));\n"
  "            IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));\n"
  "            if (vv_delay_times && vector_capacity(vv_delay_times)>=1) {\n"
  "                double* deltm_el = vector_vec(vv_delay_times);\n"
  "                int delay_times_idx;\n"
  "                next_delay = 0;\n"
  "                for(delay_times_idx = 0; delay_times_idx < vector_capacity(vv_delay_times); ++delay_times_idx) {\n"
  "                    double next_delay_t = deltm_el[delay_times_idx];\n"
  "    ENDVERBATIM\n"
  "                    net_send(next_delay_t, 10)  : use flag 10 to avoid interfering with GluSynapse logic\n"
  "    VERBATIM\n"
  "                }\n"
  "            }\n"
  "    ENDVERBATIM\n"
  "        }\n"
  "    }\n"
  "\n"
  "\n"
  "    if(verbose > 0){ UNITSOFF printf(\"Time = %g ms, incoming spike at synapse %g\\n\", t, synapseID) UNITSON }\n"
  "    if(flag == 0) {\n"
  "        if(weight <= 0){\n"
  "            : Do not perform any calculations if the synapse (netcon) is deactivated.\n"
  "            : This avoids drawing from the random stream\n"
  "            : WARNING In this model *weight* is only used to activate/deactivate the\n"
  "            :         synapse. The conductance is stored in gmax_AMPA and gmax_NMDA.\n"
  "            if(verbose > 0){ printf(\"Inactive synapse, weight = %g\\n\", weight) }\n"
  "        } else {\n"
  "            : Flag 0: Regular spike\n"
  "            if(verbose > 0){ printf(\"Flag 0, Regular spike\\n\") }\n"
  "            : Update facilitation variable as Eq. 2 in Fuhrmann et al. 2002\n"
  "            u = Use_GB + u*(1 - Use_GB)*exp(-(t - tsyn)/Fac)\n"
  "            if ( verbose > 0 ) { printf(\"\\tVesicle release probability = %g\\n\", u) }\n"
  "            : Recovery\n"
  "            p_rec = 1 - exp(-(t - tsyn)/Dep)\n"
  "            if ( verbose > 0 ) { printf(\"\\tVesicle recovery probability = %g\\n\", p_rec) }\n"
  "            if ( verbose > 0 ) { printf(\"\\tVesicle available before recovery = %g\\n\", recovered) }\n"
  "            recovered = recovered + brand(unrecovered, p_rec)\n"
  "            if ( verbose > 0 ) { printf(\"\\tVesicles available after recovery = %g\\n\", recovered) }\n"
  "            : Release\n"
  "            rec = recovered  : Make a copy so we can change it for single vesicle minis w/o messing with recovered\n"
  "            : Consider only a single recovered vesicle for minis (if minis_single_vesicle flag is set to 1)\n"
  "            if (rec > 1 && minis_single_vesicle && nc_type == 1) { rec = 1 }\n"
  "            released = brand(rec, u)\n"
  "            if ( verbose > 0 ) { printf(\"\\tReleased %g vesicles out of %g\\n\", released, recovered) }\n"
  "            : Update vesicle pool\n"
  "            recovered = recovered - released\n"
  "            unrecovered = Nrrp - recovered\n"
  "            if ( verbose > 0 ) { printf(\"\\tFinal vesicle count, Recovered = %g, Unrecovered = %g, Nrrp = %g\\n\", recovered, unrecovered, Nrrp) }\n"
  "            : Update AMPA variables\n"
  "            tp = (tau_r_AMPA*tau_d_AMPA)/(tau_d_AMPA-tau_r_AMPA)*log(tau_d_AMPA/tau_r_AMPA)  : Time to peak\n"
  "            factor = 1 / (-exp(-tp/tau_r_AMPA)+exp(-tp/tau_d_AMPA))  : Normalization factor\n"
  "            A_AMPA = A_AMPA + released/Nrrp*factor\n"
  "            B_AMPA = B_AMPA + released/Nrrp*factor\n"
  "            : Update NMDA variables\n"
  "            tp = (tau_r_NMDA*tau_d_NMDA)/(tau_d_NMDA-tau_r_NMDA)*log(tau_d_NMDA/tau_r_NMDA)  : Time to peak\n"
  "            factor = 1 / (-exp(-tp/tau_r_NMDA)+exp(-tp/tau_d_NMDA))  : Normalization factor\n"
  "            A_NMDA = A_NMDA + released/Nrrp*factor\n"
  "            B_NMDA = B_NMDA + released/Nrrp*factor\n"
  "            : Update tsyn\n"
  "            : tsyn knows about all spikes, not only those that released\n"
  "            : i.e. each spike can increase the u, regardless of recovered state\n"
  "            :      and each spike trigger an evaluation of recovery\n"
  "            tsyn = t\n"
  "        }\n"
  "    } else if(flag == 1) {\n"
  "        : Flag 1, Initialize watchers\n"
  "        if(verbose > 0){ printf(\"Flag 1, Initialize watchers\\n\") }\n"
  "        WATCH (effcai_GB > theta_d_GB) 2\n"
  "        WATCH (effcai_GB < theta_d_GB) 3\n"
  "        WATCH (effcai_GB > theta_p_GB) 4\n"
  "        WATCH (effcai_GB < theta_p_GB) 5\n"
  "    } else if(flag == 2) {\n"
  "        : Flag 2, Activate depression mechanisms\n"
  "        if(verbose > 0){ printf(\"Flag 2, Activate depression mechanisms\\n\") }\n"
  "        dep_GB = 1\n"
  "    } else if(flag == 3) {\n"
  "        : Flag 3, Deactivate depression mechanisms\n"
  "        if(verbose > 0){ printf(\"Flag 3, Deactivate depression mechanisms\\n\") }\n"
  "        dep_GB = 0\n"
  "    } else if(flag == 4) {\n"
  "        : Flag 4, Activate potentiation mechanisms\n"
  "        if(verbose > 0){ printf(\"Flag 4, Activate potentiation mechanisms\\n\") }\n"
  "        pot_GB = 1\n"
  "    } else if(flag == 5) {\n"
  "        : Flag 5, Deactivate potentiation mechanisms\n"
  "        if(verbose > 0){ printf(\"Flag 5, Deactivate potentiation mechanisms\\n\") }\n"
  "        pot_GB = 0\n"
  "    } else if(flag == 10) {\n"
  "        : Flag 10, Handle delayed connection weight changes\n"
  "    VERBATIM\n"
  "        IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));\n"
  "        if (vv_delay_weights && vector_capacity(vv_delay_weights)>=next_delay) {\n"
  "            double* weights_v = vector_vec(vv_delay_weights);\n"
  "            double next_delay_weight = weights_v[(int)next_delay];\n"
  "    ENDVERBATIM\n"
  "            weight = conductance * next_delay_weight\n"
  "            next_delay = next_delay + 1\n"
  "    VERBATIM\n"
  "        }\n"
  "    ENDVERBATIM\n"
  "    }\n"
  "}\n"
  "\n"
  "FUNCTION nernst(ci(mM), co(mM), z) (mV) {\n"
  "    nernst = (1000) * R * (celsius + 273.15) / (z*FARADAY) * log(co/ci)\n"
  "    if(verbose > 1) { UNITSOFF printf(\"nernst:%g R:%g temperature (c):%g \\n\", nernst, R, celsius) UNITSON }\n"
  "}\n"
  "\n"
  "PROCEDURE setRNG() {\n"
  "    VERBATIM\n"
  "    #ifndef CORENEURON_BUILD\n"
  "    // For compatibility, allow for either MCellRan4 or Random123\n"
  "    // Distinguish by the arg types\n"
  "    // Object => MCellRan4, seeds (double) => Random123\n"
  "    usingR123 = 0;\n"
  "    if( ifarg(1) && hoc_is_double_arg(1) ) {\n"
  "        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);\n"
  "        uint32_t a2 = 0;\n"
  "        uint32_t a3 = 0;\n"
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
  "    } else if( ifarg(1) ) {   // not a double, so assume hoc object type\n"
  "        void** pv = (void**)(&_p_rng);\n"
  "        *pv = nrn_random_arg(1);\n"
  "    } else {  // no arg, so clear pointer\n"
  "        void** pv = (void**)(&_p_rng);\n"
  "        *pv = (void*)0;\n"
  "    }\n"
  "    #endif\n"
  "    ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "PROCEDURE clearRNG() {\n"
  "VERBATIM\n"
  "    #ifndef CORENEURON_BUILD\n"
  "    if (usingR123) {\n"
  "        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);\n"
  "        if (*pv) {\n"
  "            nrnran123_deletestream(*pv);\n"
  "            *pv = (nrnran123_State*)0;\n"
  "        }\n"
  "    } else {\n"
  "        void** pv = (void**)(&_p_rng);\n"
  "        if (*pv) {\n"
  "            *pv = (void*)0;\n"
  "        }\n"
  "    }\n"
  "    #endif\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION urand() {\n"
  "    VERBATIM\n"
  "    double value;\n"
  "    if ( usingR123 ) {\n"
  "        value = nrnran123_dblpick((nrnran123_State*)_p_rng);\n"
  "    } else if (_p_rng) {\n"
  "        #ifndef CORENEURON_BUILD\n"
  "        value = nrn_random_pick(RANDCAST _p_rng);\n"
  "        #endif\n"
  "    } else {\n"
  "        value = 0.0;\n"
  "    }\n"
  "    _lurand = value;\n"
  "    ENDVERBATIM\n"
  "}\n"
  "\n"
  "FUNCTION brand(n, p) {\n"
  "    LOCAL result, count, success\n"
  "    success = 0\n"
  "    FROM count = 0 TO (n - 1) {\n"
  "        result = urand()\n"
  "        if(result <= p) {\n"
  "            success = success + 1\n"
  "        }\n"
  "    }\n"
  "    brand = success\n"
  "}\n"
  "\n"
  "\n"
  "FUNCTION bbsavestate() {\n"
  "    bbsavestate = 0\n"
  "    VERBATIM\n"
  "    #ifndef CORENEURON_BUILD\n"
  "        /* first arg is direction (0 save, 1 restore), second is array*/\n"
  "        /* if first arg is -1, fill xdir with the size of the array */\n"
  "        double *xdir, *xval;\n"
  "        #ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "        double *hoc_pgetarg();\n"
  "        long nrn_get_random_sequence(void* r);\n"
  "        void nrn_set_random_sequence(void* r, int val);\n"
  "        #endif\n"
  "        xdir = hoc_pgetarg(1);\n"
  "        xval = hoc_pgetarg(2);\n"
  "        if (_p_rng) {\n"
  "            // tell how many items need saving\n"
  "            if (*xdir == -1) {  // count items\n"
  "                if( usingR123 ) {\n"
  "                    *xdir = 2.0;\n"
  "                } else {\n"
  "                    *xdir = 1.0;\n"
  "                }\n"
  "                return 0.0;\n"
  "            } else if(*xdir ==0 ) {  // save\n"
  "                if( usingR123 ) {\n"
  "                    uint32_t seq;\n"
  "                    char which;\n"
  "                    nrnran123_getseq( (nrnran123_State*)_p_rng, &seq, &which );\n"
  "                    xval[0] = (double) seq;\n"
  "                    xval[1] = (double) which;\n"
  "                } else {\n"
  "                    xval[0] = (double)nrn_get_random_sequence(RANDCAST _p_rng);\n"
  "                }\n"
  "            } else {  // restore\n"
  "                if( usingR123 ) {\n"
  "                    nrnran123_setseq( (nrnran123_State*)_p_rng, (uint32_t)xval[0], (char)xval[1] );\n"
  "                } else {\n"
  "                    nrn_set_random_sequence(RANDCAST _p_rng, (long)(xval[0]));\n"
  "                }\n"
  "            }\n"
  "        }\n"
  "    #endif\n"
  "    ENDVERBATIM\n"
  "}\n"
  "\n"
  "\n"
  "VERBATIM\n"
  "static void bbcore_write(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {\n"
  "    IvocVect *vv_delay_times = *((IvocVect**)(&_p_delay_times));\n"
  "    IvocVect *vv_delay_weights = *((IvocVect**)(&_p_delay_weights));\n"
  "\n"
  "    // make sure offset array non-null\n"
  "    if (iArray) {\n"
  "        // get handle to random123 instance\n"
  "        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);\n"
  "        // get location for storing ids\n"
  "        uint32_t* ia = ((uint32_t*)iArray) + *ioffset;\n"
  "        // retrieve/store identifier seeds\n"
  "        nrnran123_getids3(*pv, ia, ia+1, ia+2);\n"
  "        // retrieve/store stream sequence\n"
  "        char which;\n"
  "        nrnran123_getseq(*pv, ia+3, &which);\n"
  "        ia[4] = (int)which;\n"
  "    }\n"
  "\n"
  "    // increment integer offset (2 identifier), no double data\n"
  "    *ioffset += 5;\n"
  "    *doffset += 0;\n"
  "\n"
  "    // serialize connection delay vectors\n"
  "    if (vv_delay_times && vv_delay_weights &&\n"
  "       (vector_capacity(vv_delay_times) >= 1) && (vector_capacity(vv_delay_weights) >= 1)) {\n"
  "        if (iArray) {\n"
  "            uint32_t* di = ((uint32_t*)iArray) + *ioffset;\n"
  "            // store vector sizes for deserialization\n"
  "            di[0] = vector_capacity(vv_delay_times);\n"
  "            di[1] = vector_capacity(vv_delay_weights);\n"
  "        }\n"
  "        if (dArray) {\n"
  "            double* delay_times_el = vector_vec(vv_delay_times);\n"
  "            double* delay_weights_el = vector_vec(vv_delay_weights);\n"
  "            double* x_i = dArray + *doffset;\n"
  "            int delay_vecs_idx;\n"
  "            int x_idx = 0;\n"
  "            for(delay_vecs_idx = 0; delay_vecs_idx < vector_capacity(vv_delay_times); ++delay_vecs_idx) {\n"
  "                 x_i[x_idx++] = delay_times_el[delay_vecs_idx];\n"
  "                 x_i[x_idx++] = delay_weights_el[delay_vecs_idx];\n"
  "            }\n"
  "        }\n"
  "        // reserve space for connection delay data on serialization buffer\n"
  "        *doffset += vector_capacity(vv_delay_times) + vector_capacity(vv_delay_weights);\n"
  "    } else {\n"
  "        if (iArray) {\n"
  "            uint32_t* di = ((uint32_t*)iArray) + *ioffset;\n"
  "            di[0] = 0;\n"
  "            di[1] = 0;\n"
  "        }\n"
  "    }\n"
  "    // reserve space for delay vectors (may be 0)\n"
  "    *ioffset += 2;\n"
  "}\n"
  "\n"
  "static void bbcore_read(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {\n"
  "    // make sure it's not previously set\n"
  "    assert(!_p_rng);\n"
  "    assert(!_p_delay_times && !_p_delay_weights);\n"
  "\n"
  "    uint32_t* ia = ((uint32_t*)iArray) + *ioffset;\n"
  "    // make sure non-zero identifier seeds\n"
  "    if (ia[0] != 0 || ia[1] != 0 || ia[2] != 0) {\n"
  "        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);\n"
  "        // get new stream\n"
  "        *pv = nrnran123_newstream3(ia[0], ia[1], ia[2]);\n"
  "        // restore sequence\n"
  "        nrnran123_setseq(*pv, ia[3], (char)ia[4]);\n"
  "    }\n"
  "    // increment intger offset (2 identifiers), no double data\n"
  "    *ioffset += 5;\n"
  "\n"
  "    int delay_times_sz = iArray[5];\n"
  "    int delay_weights_sz = iArray[6];\n"
  "    *ioffset += 2;\n"
  "\n"
  "    if ((delay_times_sz > 0) && (delay_weights_sz > 0)) {\n"
  "        double* x_i = dArray + *doffset;\n"
  "\n"
  "        // allocate vectors\n"
  "        _p_delay_times = (double*)vector_new1(delay_times_sz);\n"
  "        _p_delay_weights = (double*)vector_new1(delay_weights_sz);\n"
  "\n"
  "        double* delay_times_el = vector_vec((IvocVect*)_p_delay_times);\n"
  "        double* delay_weights_el = vector_vec((IvocVect*)_p_delay_weights);\n"
  "\n"
  "        // copy data\n"
  "        int x_idx;\n"
  "        int vec_idx = 0;\n"
  "        for(x_idx = 0; x_idx < delay_times_sz + delay_weights_sz; x_idx += 2) {\n"
  "            delay_times_el[vec_idx] = x_i[x_idx];\n"
  "            delay_weights_el[vec_idx++] = x_i[x_idx+1];\n"
  "        }\n"
  "        *doffset += delay_times_sz + delay_weights_sz;\n"
  "    }\n"
  "}\n"
  "ENDVERBATIM\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
