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
static constexpr auto number_of_floating_point_variables = 14;
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
 
#define nrn_init _nrn_init__conductanceSource
#define _nrn_initial _nrn_initial__conductanceSource
#define nrn_cur _nrn_cur__conductanceSource
#define _nrn_current _nrn_current__conductanceSource
#define nrn_jacob _nrn_jacob__conductanceSource
#define nrn_state _nrn_state__conductanceSource
#define _net_receive _net_receive__conductanceSource 
#define icur icur__conductanceSource 
#define vstim vstim__conductanceSource 
 
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
#define rs _ml->template fpfield<0>(_iml)
#define rs_columnindex 0
#define dur1 _ml->template fpfield<1>(_iml)
#define dur1_columnindex 1
#define amp1 _ml->template fpfield<2>(_iml)
#define amp1_columnindex 2
#define dur2 _ml->template fpfield<3>(_iml)
#define dur2_columnindex 3
#define amp2 _ml->template fpfield<4>(_iml)
#define amp2_columnindex 4
#define dur3 _ml->template fpfield<5>(_iml)
#define dur3_columnindex 5
#define amp3 _ml->template fpfield<6>(_iml)
#define amp3_columnindex 6
#define i _ml->template fpfield<7>(_iml)
#define i_columnindex 7
#define vc _ml->template fpfield<8>(_iml)
#define vc_columnindex 8
#define tc2 _ml->template fpfield<9>(_iml)
#define tc2_columnindex 9
#define tc3 _ml->template fpfield<10>(_iml)
#define tc3_columnindex 10
#define on _ml->template fpfield<11>(_iml)
#define on_columnindex 11
#define v _ml->template fpfield<12>(_iml)
#define v_columnindex 12
#define _g _ml->template fpfield<13>(_iml)
#define _g_columnindex 13
#define _nd_area *_ml->dptr_field<0>(_iml)
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_icur(void*);
 static double _hoc_vstim(void*);
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
 {"icur", _hoc_icur},
 {"vstim", _hoc_vstim},
 {0, 0}
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {"dur3", 0, 1e+09},
 {"dur2", 0, 1e+09},
 {"rs", 1e-09, 1e+09},
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"rs", "megohm"},
 {"dur1", "ms"},
 {"amp1", "mV"},
 {"dur2", "ms"},
 {"amp2", "mV"},
 {"dur3", "ms"},
 {"amp3", "mV"},
 {"i", "nA"},
 {"vc", "mV"},
 {0, 0}
};
 static double delta_t = 0.01;
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
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"conductanceSource",
 "rs",
 "dur1",
 "amp1",
 "dur2",
 "amp2",
 "dur3",
 "amp3",
 0,
 "i",
 "vc",
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
   _ppvar = nrn_prop_datum_alloc(_mechtype, 2, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 14);
 	/*initialize range parameters*/
 	rs = 1;
 	dur1 = 0;
 	amp1 = 0;
 	dur2 = 0;
 	amp2 = 0;
 	dur3 = 0;
 	amp3 = 0;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 14);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _conductanceSource_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"rs"} /* 0 */,
                                       _nrn_mechanism_field<double>{"dur1"} /* 1 */,
                                       _nrn_mechanism_field<double>{"amp1"} /* 2 */,
                                       _nrn_mechanism_field<double>{"dur2"} /* 3 */,
                                       _nrn_mechanism_field<double>{"amp2"} /* 4 */,
                                       _nrn_mechanism_field<double>{"dur3"} /* 5 */,
                                       _nrn_mechanism_field<double>{"amp3"} /* 6 */,
                                       _nrn_mechanism_field<double>{"i"} /* 7 */,
                                       _nrn_mechanism_field<double>{"vc"} /* 8 */,
                                       _nrn_mechanism_field<double>{"tc2"} /* 9 */,
                                       _nrn_mechanism_field<double>{"tc3"} /* 10 */,
                                       _nrn_mechanism_field<double>{"on"} /* 11 */,
                                       _nrn_mechanism_field<double>{"v"} /* 12 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 13 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */);
  hoc_register_prop_size(_mechtype, 14, 2);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 conductanceSource conductanceSource.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "conductanceSource.mod";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int icur(_internalthreadargsproto_);
static int vstim(_internalthreadargsproto_);
 
static int  icur ( _internalthreadargsproto_ ) {
   if ( on ) {
     i = ( v - vc ) / rs ;
     }
   else {
     i = 0.0 ;
     }
    return 0; }
 
static double _hoc_icur(void* _vptr) {
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
 icur ( _threadargs_ );
 return(_r);
}
 
static int  vstim ( _internalthreadargsproto_ ) {
   on = 1.0 ;
   if ( dur1 ) {
     at_time ( _nt, dur1 ) ;
     }
   if ( dur2 ) {
     at_time ( _nt, tc2 ) ;
     }
   if ( dur3 ) {
     at_time ( _nt, tc3 ) ;
     }
   if ( t < dur1 ) {
     vc = amp1 ;
     }
   else if ( t < tc2 ) {
     vc = amp2 ;
     }
   else if ( t < tc3 ) {
     vc = amp3 ;
     }
   else {
     vc = 0.0 ;
     on = 0.0 ;
     }
   icur ( _threadargs_ ) ;
    return 0; }
 
static double _hoc_vstim(void* _vptr) {
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
 vstim ( _threadargs_ );
 return(_r);
}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
   tc2 = dur1 + dur2 ;
   tc3 = tc2 + dur3 ;
   on = 0.0 ;
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
 initmodel(_threadargs_);
}
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   vstim ( _threadargs_ ) ;
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
 {  { icur(_threadargs_); }
  }}}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "conductanceSource.mod";
    const char* nmodl_file_text = 
  "TITLE conductanceSource.mod\n"
  "COMMENT\n"
  "Single electrode Voltage clamp with three levels.\n"
  "Clamp is on at time 0, and off at time\n"
  "dur1+dur2+dur3. When clamp is off the injected current is 0.\n"
  "The clamp levels are amp1, amp2, amp3.\n"
  "i is the injected current, vc measures the control voltage)\n"
  "Do not insert several instances of this model at the same location in order to\n"
  "make level changes. That is equivalent to independent clamps and they will\n"
  "have incompatible internal state values.\n"
  "The electrical circuit for the clamp is exceedingly simple:\n"
  "vc ---'\\/\\/`--- cell\n"
  "        rs\n"
  "\n"
  "Note that since this is an electrode current model v refers to the\n"
  "internal potential which is equivalent to the membrane potential v when\n"
  "there is no extracellular membrane mechanism present but is v+vext when\n"
  "one is present.\n"
  "Unlike SEClamp, since i is not an electrode current,\n"
  "positive values of i are outward and thus hyperpolarize the cell\n"
  "ENDCOMMENT\n"
  "\n"
  "INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}\n"
  "\n"
  "DEFINE NSTEP 3\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS conductanceSource\n"
  "	NONSPECIFIC_CURRENT i\n"
  "	RANGE dur1, amp1, dur2, amp2, dur3, amp3, rs, vc, i\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(nA) = (nanoamp)\n"
  "	(mV) = (millivolt)\n"
  "	(uS) = (microsiemens)\n"
  "}\n"
  "\n"
  "\n"
  "PARAMETER {\n"
  "	rs = 1 (megohm) <1e-9, 1e9>\n"
  "	dur1 (ms) 	  amp1 (mV)\n"
  "	dur2 (ms) <0,1e9> amp2 (mV)\n"
  "	dur3 (ms) <0,1e9> amp3 (mV)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v (mV)	: automatically v + vext when extracellular is present\n"
  "	i (nA)\n"
  "	vc (mV)\n"
  "	tc2 (ms)\n"
  "	tc3 (ms)\n"
  "	on\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	tc2 = dur1 + dur2\n"
  "	tc3 = tc2 + dur3\n"
  "	on = 0\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE icur METHOD after_cvode\n"
  "	vstim()\n"
  "}\n"
  "\n"
  "PROCEDURE icur() {\n"
  "	if (on) {\n"
  "		i = (v - vc)/rs\n"
  "	}else{\n"
  "		i = 0\n"
  "	}\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "The SOLVE of icur() in the BREAKPOINT block is necessary to compute\n"
  "i=(vc - v(t))/rs instead of i=(vc - v(t-dt))/rs\n"
  "This is important for time varying vc because the actual i used in\n"
  "the implicit method is equivalent to (vc - v(t)/rs due to the\n"
  "calculation of di/dv from the BREAKPOINT block.\n"
  "The reason this works is because the SOLVE statement in the BREAKPOINT block\n"
  "is executed after the membrane potential is advanced.\n"
  "\n"
  "It is a shame that vstim has to be called twice but putting the call\n"
  "in a SOLVE block would cause playing a Vector into vc to be off by one\n"
  "time step.\n"
  "ENDCOMMENT\n"
  "\n"
  "PROCEDURE vstim() {\n"
  "	on = 1\n"
  "	if (dur1) {at_time(dur1)}\n"
  "	if (dur2) {at_time(tc2)}\n"
  "	if (dur3) {at_time(tc3)}\n"
  "	if (t < dur1) {\n"
  "		vc = amp1\n"
  "	}else if (t < tc2) {\n"
  "		vc = amp2\n"
  "	}else if (t < tc3) {\n"
  "		vc = amp3\n"
  "	}else {\n"
  "		vc = 0\n"
  "		on = 0\n"
  "	}\n"
  "	icur()\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
