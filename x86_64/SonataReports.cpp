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
 
#define nrn_init _nrn_init__SonataReport
#define _nrn_initial _nrn_initial__SonataReport
#define nrn_cur _nrn_cur__SonataReport
#define _nrn_current _nrn_current__SonataReport
#define nrn_jacob _nrn_jacob__SonataReport
#define nrn_state _nrn_state__SonataReport
#define _net_receive _net_receive__SonataReport 
 
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
#define tstart _ml->template fpfield<1>(_iml)
#define tstart_columnindex 1
#define tstop _ml->template fpfield<2>(_iml)
#define tstop_columnindex 2
#define v _ml->template fpfield<3>(_iml)
#define v_columnindex 3
#define _nd_area *_ml->dptr_field<0>(_iml)
#define ptr	*_ppvar[2].get<double*>()
#define _p_ptr _ppvar[2].literal_value<void*>()
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  2;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_AddVar(void*);
 static double _hoc_AddNode(void*);
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
 {"AddVar", _hoc_AddVar},
 {"AddNode", _hoc_AddNode},
 {0, 0}
};
#define AddVar AddVar_SonataReport
#define AddNode AddNode_SonataReport
 extern double AddVar( _internalthreadargsproto_ );
 extern double AddNode( _internalthreadargsproto_ );
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"Dt", "ms"},
 {"tstart", "ms"},
 {"tstop", "ms"},
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
 static void _constructor(Prop*);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"SonataReport",
 "Dt",
 "tstart",
 "tstop",
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
   _ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	/*initialize range parameters*/
 	Dt = 0.1;
 	tstart = 0;
 	tstop = 0;
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 4);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 if (!nrn_point_prop_) {_constructor(_prop);}
 
}
 static void _initlists();
 static void bbcore_write(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_write(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 static void bbcore_read(double*, int*, int*, int*, _threadargsproto_);
 extern void hoc_reg_bbcore_read(int, void(*)(double*, int*, int*, int*, _threadargsproto_));
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _SonataReports_reg() {
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
                                       _nrn_mechanism_field<double>{"Dt"} /* 0 */,
                                       _nrn_mechanism_field<double>{"tstart"} /* 1 */,
                                       _nrn_mechanism_field<double>{"tstop"} /* 2 */,
                                       _nrn_mechanism_field<double>{"v"} /* 3 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"ptr", "bbcorepointer"} /* 2 */);
  hoc_register_prop_size(_mechtype, 4, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "bbcorepointer");
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 SonataReport SonataReports.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
/*VERBATIM*/
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
#include <stdint.h>
#include <bbp/sonata/reports.h>
#ifndef NRN_VERSION_GTEQ_8_2_0
extern double* hoc_pgetarg(int iarg);
extern double* getarg(int iarg);
extern char* gargstr(int iarg);
extern int hoc_is_str_arg(int iarg);
extern int ifarg(int iarg);
extern double chkarg(int iarg, double low, double high);
#endif

typedef struct {
    char neuronName_[256];
    char rptName_[512];
} Data;

#endif
#endif
 
double AddNode ( _internalthreadargsproto_ ) {
   double _lAddNode;
 
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    Data** tempdata = (Data**)(&(_p_ptr));
    Data* data = *tempdata;
    if(ifarg(1))
    {
        char population_name[256] = "All";
        unsigned long population_offset = 0;
        if (ifarg(2)) {
            sprintf(population_name,"%s", gargstr(2));
        }
        if (ifarg(3)) {
            population_offset = (unsigned long) *getarg(3);
        }
        unsigned long node_id = (unsigned long) *getarg(1);
        sonata_add_node(data->rptName_, population_name, population_offset, node_id);
    }
#endif
#endif
}
 
return _lAddNode;
 }
 
static double _hoc_AddNode(void* _vptr) {
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
 _r =  AddNode ( _threadargs_ );
 return(_r);
}
 
double AddVar ( _internalthreadargsproto_ ) {
   double _lAddVar;
 
/*VERBATIM*/
{
#ifndef CORENEURON_BUILD
#ifndef DISABLE_REPORTINGLIB
    Data** tempdata = (Data**)(&(_p_ptr));
    Data* data = *tempdata;
    if(ifarg(1))
    {
        int element_id = (int)*getarg(2);
        int node_id = (int) *getarg(3);
        char population_name[256] = "All";
        if (ifarg(4)) {
            sprintf(population_name,"%s", gargstr(4));
        }
#ifdef NRN_MECHANISM_DATA_IS_SOA
        sonata_add_element_handle(data->rptName_, population_name, node_id, element_id, [x=hoc_hgetarg<double>(1)]() { return *x; });
#else
        sonata_add_element(data->rptName_, population_name, node_id, element_id, hoc_pgetarg(1));
#endif
    }
#endif
#endif
}
 
return _lAddVar;
 }
 
static double _hoc_AddVar(void* _vptr) {
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
 _r =  AddVar ( _threadargs_ );
 return(_r);
}
 
/*VERBATIM*/
/** not executed in coreneuron and hence need empty stubs */
static void bbcore_write(double* x, int* d, int* xx, int* offset, _threadargsproto_) { }
static void bbcore_read(double* x, int* d, int* xx, int* offset, _threadargsproto_) { }
 
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
#ifndef DISABLE_REPORTINGLIB
    Data** tempdata = (Data**)(&(_p_ptr));
    Data* data = 0;
    data = (Data*)hoc_Emalloc(sizeof(Data));
    hoc_malchk();
    if(ifarg(2) && hoc_is_str_arg(2) &&
        ifarg(3) && hoc_is_str_arg(3) &&
        ifarg(4) && ifarg(5) && ifarg(6) &&
        ifarg(7) && hoc_is_str_arg(7)
    )
    {
        sprintf(data->rptName_,"%s/%s",gargstr(3),gargstr(2));
        tstart = *getarg(4);
        tstop = *getarg(5);
        Dt = *getarg(6);

        sonata_create_report(data->rptName_, tstart, tstop, Dt, gargstr(7), gargstr(8));

        *tempdata = data; //makes to data available to other procedures through ptr
    }
    else
    {
        int i = 1;
        while(ifarg(i))
        {
            if(i==1)
                printf("There is an error creating report\n");
            printf("It has arg %d: ", i);
            if(hoc_is_str_arg(i))
                printf("%s\n",gargstr(i));
            else
                printf("%d\n",(int)*getarg(i));
            i++;
        }

    }
#else
        static int warning_shown = 0;
        if (ifarg(2) && hoc_is_str_arg(2))
        {
            if (warning_shown == 0)
            {
                printf("WARNING: BinReports Constructor(): Trying to create and write report %s while the NEURODAMUS_DISABLE_REPORTINGLIB is set to ON, ignoring... \n", gargstr(2));
                warning_shown++;
            }
        }
#endif
#endif
}
 }
 
}
}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
 {
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
    const char* nmodl_filename = "SonataReports.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "Modified mod file\n"
  "\n"
  "Pointer is an object with Name of the cell, and the report\n"
  "\n"
  "How to use it:\n"
  "Create the object with this information:\n"
  "\n"
  "receive: location,reportname,cellname,path,tstart,tend,dt,sizemapping,kind,sizeatributesmapping\n"
  "one object per cell to report.\n"
  "\n"
  "\n"
  "AddVar(pointer to the variable (as in the older), mapping information)  so much mapping information as in siz\n"
  "\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "        THREADSAFE\n"
  "        POINT_PROCESS SonataReport\n"
  "        BBCOREPOINTER ptr : an object with two strings\n"
  "        RANGE Dt\n"
  "	RANGE tstart\n"
  "	RANGE tstop\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	Dt = .1 (ms)\n"
  "	tstart = 0 (ms)\n"
  "	tstop  = 0 (ms)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	ptr\n"
  "}\n"
  "\n"
  "\n"
  "INITIAL {\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "#include <stdint.h>\n"
  "#include <bbp/sonata/reports.h>\n"
  "#ifndef NRN_VERSION_GTEQ_8_2_0\n"
  "extern double* hoc_pgetarg(int iarg);\n"
  "extern double* getarg(int iarg);\n"
  "extern char* gargstr(int iarg);\n"
  "extern int hoc_is_str_arg(int iarg);\n"
  "extern int ifarg(int iarg);\n"
  "extern double chkarg(int iarg, double low, double high);\n"
  "#endif\n"
  "\n"
  "typedef struct {\n"
  "    char neuronName_[256];\n"
  "    char rptName_[512];\n"
  "} Data;\n"
  "\n"
  "#endif\n"
  "#endif\n"
  "ENDVERBATIM\n"
  "\n"
  "COMMENT\n"
  "receive: location,reportname,path,tstart,tend,dt,kind\n"
  "ENDCOMMENT\n"
  "CONSTRUCTOR {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    Data** tempdata = (Data**)(&(_p_ptr));\n"
  "    Data* data = 0;\n"
  "    data = (Data*)hoc_Emalloc(sizeof(Data));\n"
  "    hoc_malchk();\n"
  "    if(ifarg(2) && hoc_is_str_arg(2) &&\n"
  "        ifarg(3) && hoc_is_str_arg(3) &&\n"
  "        ifarg(4) && ifarg(5) && ifarg(6) &&\n"
  "        ifarg(7) && hoc_is_str_arg(7)\n"
  "    )\n"
  "    {\n"
  "        sprintf(data->rptName_,\"%s/%s\",gargstr(3),gargstr(2));\n"
  "        tstart = *getarg(4);\n"
  "        tstop = *getarg(5);\n"
  "        Dt = *getarg(6);\n"
  "\n"
  "        sonata_create_report(data->rptName_, tstart, tstop, Dt, gargstr(7), gargstr(8));\n"
  "\n"
  "        *tempdata = data; //makes to data available to other procedures through ptr\n"
  "    }\n"
  "    else\n"
  "    {\n"
  "        int i = 1;\n"
  "        while(ifarg(i))\n"
  "        {\n"
  "            if(i==1)\n"
  "                printf(\"There is an error creating report\\n\");\n"
  "            printf(\"It has arg %d: \", i);\n"
  "            if(hoc_is_str_arg(i))\n"
  "                printf(\"%s\\n\",gargstr(i));\n"
  "            else\n"
  "                printf(\"%d\\n\",(int)*getarg(i));\n"
  "            i++;\n"
  "        }\n"
  "\n"
  "    }\n"
  "#else\n"
  "        static int warning_shown = 0;\n"
  "        if (ifarg(2) && hoc_is_str_arg(2))\n"
  "        {\n"
  "            if (warning_shown == 0)\n"
  "            {\n"
  "                printf(\"WARNING: BinReports Constructor(): Trying to create and write report %s while the NEURODAMUS_DISABLE_REPORTINGLIB is set to ON, ignoring... \\n\", gargstr(2));\n"
  "                warning_shown++;\n"
  "            }\n"
  "        }\n"
  "#endif\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "FUNCTION AddNode() {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    Data** tempdata = (Data**)(&(_p_ptr));\n"
  "    Data* data = *tempdata;\n"
  "    if(ifarg(1))\n"
  "    {\n"
  "        char population_name[256] = \"All\";\n"
  "        unsigned long population_offset = 0;\n"
  "        if (ifarg(2)) {\n"
  "            sprintf(population_name,\"%s\", gargstr(2));\n"
  "        }\n"
  "        if (ifarg(3)) {\n"
  "            population_offset = (unsigned long) *getarg(3);\n"
  "        }\n"
  "        unsigned long node_id = (unsigned long) *getarg(1);\n"
  "        sonata_add_node(data->rptName_, population_name, population_offset, node_id);\n"
  "    }\n"
  "#endif\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "/*\n"
  "	AddVariable  with the next data\n"
  "		variable  A pointer to the value of the variable\n"
  "		information about mapping\n"
  "*/\n"
  "ENDCOMMENT\n"
  "FUNCTION AddVar() {\n"
  "VERBATIM {\n"
  "#ifndef CORENEURON_BUILD\n"
  "#ifndef DISABLE_REPORTINGLIB\n"
  "    Data** tempdata = (Data**)(&(_p_ptr));\n"
  "    Data* data = *tempdata;\n"
  "    if(ifarg(1))\n"
  "    {\n"
  "        int element_id = (int)*getarg(2);\n"
  "        int node_id = (int) *getarg(3);\n"
  "        char population_name[256] = \"All\";\n"
  "        if (ifarg(4)) {\n"
  "            sprintf(population_name,\"%s\", gargstr(4));\n"
  "        }\n"
  "#ifdef NRN_MECHANISM_DATA_IS_SOA\n"
  "        sonata_add_element_handle(data->rptName_, population_name, node_id, element_id, [x=hoc_hgetarg<double>(1)]() { return *x; });\n"
  "#else\n"
  "        sonata_add_element(data->rptName_, population_name, node_id, element_id, hoc_pgetarg(1));\n"
  "#endif\n"
  "    }\n"
  "#endif\n"
  "#endif\n"
  "}\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "VERBATIM\n"
  "/** not executed in coreneuron and hence need empty stubs */\n"
  "static void bbcore_write(double* x, int* d, int* xx, int* offset, _threadargsproto_) { }\n"
  "static void bbcore_read(double* x, int* d, int* xx, int* offset, _threadargsproto_) { }\n"
  "ENDVERBATIM\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
