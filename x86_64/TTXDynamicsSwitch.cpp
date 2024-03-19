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
static constexpr auto number_of_floating_point_variables = 5;
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
 
#define nrn_init _nrn_init__TTXDynamicsSwitch
#define _nrn_initial _nrn_initial__TTXDynamicsSwitch
#define nrn_cur _nrn_cur__TTXDynamicsSwitch
#define _nrn_current _nrn_current__TTXDynamicsSwitch
#define nrn_jacob _nrn_jacob__TTXDynamicsSwitch
#define nrn_state _nrn_state__TTXDynamicsSwitch
#define _net_receive _net_receive__TTXDynamicsSwitch 
 
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
#define ttxo_level _ml->template fpfield<0>(_iml)
#define ttxo_level_columnindex 0
#define ttxo _ml->template fpfield<1>(_iml)
#define ttxo_columnindex 1
#define ttxi _ml->template fpfield<2>(_iml)
#define ttxi_columnindex 2
#define v _ml->template fpfield<3>(_iml)
#define v_columnindex 3
#define _g _ml->template fpfield<4>(_iml)
#define _g_columnindex 4
#define _ion_ttxo *(_ml->dptr_field<0>(_iml))
#define _p_ion_ttxo static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ttxi *(_ml->dptr_field<1>(_iml))
#define _p_ion_ttxi static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_ttx_erev *_ml->dptr_field<2>(_iml)
#define _style_ttx	*_ppvar[3].get<int*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
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
 {"setdata_TTXDynamicsSwitch", _hoc_setdata},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {0, 0}
};
 /* declare global and static user variables */
#define ttxi_sentinel ttxi_sentinel_TTXDynamicsSwitch
 double ttxi_sentinel = 0.015625;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"ttxi_sentinel_TTXDynamicsSwitch", "mM"},
 {"ttxo_level_TTXDynamicsSwitch", "mM"},
 {0, 0}
};
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {"ttxi_sentinel_TTXDynamicsSwitch", &ttxi_sentinel_TTXDynamicsSwitch},
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
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"TTXDynamicsSwitch",
 "ttxo_level_TTXDynamicsSwitch",
 0,
 0,
 0,
 0};
 static Symbol* _ttx_sym;
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 5);
 	/*initialize range parameters*/
 	ttxo_level = 1e-12;
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 5);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ttx_sym);
 nrn_check_conc_write(_prop, prop_ion, 0);
 nrn_check_conc_write(_prop, prop_ion, 1);
 nrn_promote(prop_ion, 3, 0);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 2); /* ttxo */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 1); /* ttxi */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 0); // erev ttx
 	_ppvar[3] = {neuron::container::do_not_search, &(_nrn_mechanism_access_dparam(prop_ion)[0].literal_value<int>())}; /* iontype for ttx */
 
}
 static void _initlists();
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _TTXDynamicsSwitch_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ttx", 1.0);
 	_ttx_sym = hoc_lookup("ttx_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"ttxo_level"} /* 0 */,
                                       _nrn_mechanism_field<double>{"ttxo"} /* 1 */,
                                       _nrn_mechanism_field<double>{"ttxi"} /* 2 */,
                                       _nrn_mechanism_field<double>{"v"} /* 3 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 4 */,
                                       _nrn_mechanism_field<double*>{"_ion_ttxo", "ttx_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ttxi", "ttx_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_ttx_erev", "ttx_ion"} /* 2 */,
                                       _nrn_mechanism_field<int*>{"_style_ttx", "#ttx_ion"} /* 3 */);
  hoc_register_prop_size(_mechtype, 5, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "ttx_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ttx_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "ttx_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "#ttx_ion");
 	nrn_writes_conc(_mechtype, 0);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 TTXDynamicsSwitch TTXDynamicsSwitch.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
   ttxo = ttxo_level ;
   ttxi = ttxi_sentinel ;
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
  ttxo = _ion_ttxo;
  ttxi = _ion_ttxi;
 initmodel(_threadargs_);
  _ion_ttxo = ttxo;
  _ion_ttxi = ttxi;
  nrn_wrote_conc(_ttx_sym, _ion_ttx_erev, _ion_ttxi, _ion_ttxo, _style_ttx);
}
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{
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
  ttxo = _ion_ttxo;
  ttxi = _ion_ttxi;
 {
   ttxo = ttxo_level ;
   }
  _ion_ttxo = ttxo;
  _ion_ttxi = ttxi;
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
    const char* nmodl_filename = "TTXDynamicsSwitch.mod";
    const char* nmodl_file_text = 
  ": Simple switching TTX dynamics\n"
  ": Outside TTX concentration level stays fixed at ttxo_level, which can be\n"
  ": set as a range variable\n"
  ": Implemented by Werner Van Geit @ BlueBrain Project, Jan 2015\n"
  "\n"
  "NEURON	{\n"
  "	SUFFIX TTXDynamicsSwitch\n"
  "	USEION ttx WRITE ttxo, ttxi VALENCE 1\n"
  "    RANGE ttxo_level\n"
  "}\n"
  "\n"
  "PARAMETER	{\n"
  "    : 1e-12 represent 'no TTX', using 0.0 could generate problems during the \n"
  "    : possible calculation of ettx \n"
  "    ttxo_level = 1e-12 (mM)\n"
  "\n"
  "    : Check at the end of the mod file for the reasoning behind this sentinel\n"
  "    : value\n"
  "    ttxi_sentinel = 0.015625 (mM) : 1.0/64.0\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    ttxo (mM)\n"
  "    ttxi (mM)\n"
  "}\n"
  "\n"
  "BREAKPOINT	{\n"
  "    ttxo = ttxo_level\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    ttxo = ttxo_level\n"
  "    ttxi = ttxi_sentinel\n"
  "}\n"
  "\n"
  ": WVG @ BBP, Jan 2015\n"
  ": The internal ttx concentration is a sentinel value, this value should\n"
  ": not be used for any calculation\n"
  ": The reason it is here is that Neuron's default value for a concentration\n"
  ": is 1.0. In case a sodium channel uses the TTX concentration to \n"
  ": enable/disable itself, it shouldn't use the default value. It should only\n"
  ": use the ttxo value when it has been initialised by this TTXDynamicsSwitch\n"
  ": mechanism.\n"
  ": (or any other value that manages the outside ttx concentration)\n"
  ": The channel should read the ttxi value, and check for this sentinel value\n"
  ": If it matches, it means this mechanism is control the ttxo concentration\n"
  ": otherwise ttxo should be ignored\n"
  ": Chose 1.0/64.0 as sentinel because it can be exactly represented in binary\n"
  ": floating-point representation\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
