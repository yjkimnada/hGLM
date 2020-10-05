/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__NMDA5d2nc
#define _nrn_initial _nrn_initial__NMDA5d2nc
#define nrn_cur _nrn_cur__NMDA5d2nc
#define _nrn_current _nrn_current__NMDA5d2nc
#define nrn_jacob _nrn_jacob__NMDA5d2nc
#define nrn_state _nrn_state__NMDA5d2nc
#define _net_receive _net_receive__NMDA5d2nc 
#define _f_rates _f_rates__NMDA5d2nc 
#define kstates kstates__NMDA5d2nc 
#define rates rates__NMDA5d2nc 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define Erev _p[0]
#define gmax _p[1]
#define mg _p[2]
#define vmin _p[3]
#define vmax _p[4]
#define Cmax _p[5]
#define Cmin _p[6]
#define Cdur _p[7]
#define Rb _p[8]
#define Ru _p[9]
#define Ro _p[10]
#define Rc _p[11]
#define Rd1 _p[12]
#define Rr1 _p[13]
#define Rd2 _p[14]
#define Rr2 _p[15]
#define i _p[16]
#define g _p[17]
#define Cglut _p[18]
#define rb _p[19]
#define U _p[20]
#define Cl _p[21]
#define D1 _p[22]
#define D2 _p[23]
#define O _p[24]
#define B _p[25]
#define DU _p[26]
#define DCl _p[27]
#define DD1 _p[28]
#define DD2 _p[29]
#define DO _p[30]
#define DB _p[31]
#define v _p[32]
#define _g _p[33]
#define _tsav _p[34]
#define _nd_area  *_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_rates();
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "rates", _hoc_rates,
 0, 0
};
 
static void _check_rates(double*, Datum*, Datum*, _NrnThread*); 
static void _check_table_thread(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, int _type) {
   _check_rates(_p, _ppvar, _thread, _nt);
 }
 /* declare global and static user variables */
#define usetable usetable_NMDA5d2nc
 double usetable = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "usetable_NMDA5d2nc", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Erev", "mV",
 "gmax", "pS",
 "mg", "mM",
 "vmin", "mV",
 "vmax", "mV",
 "Cmax", "mM",
 "Cmin", "mM",
 "Cdur", "ms",
 "Rb", "/uM",
 "Ru", "/ms",
 "Ro", "/ms",
 "Rc", "/ms",
 "Rd1", "/ms",
 "Rr1", "/ms",
 "Rd2", "/ms",
 "Rr2", "/ms",
 "i", "nA",
 "g", "pS",
 "Cglut", "mM",
 "rb", "/ms",
 0,0
};
 static double B0 = 0;
 static double Cl0 = 0;
 static double D20 = 0;
 static double D10 = 0;
 static double O0 = 0;
 static double U0 = 0;
 static double delta_t = 1;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "usetable_NMDA5d2nc", &usetable_NMDA5d2nc,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[3]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"NMDA5d2nc",
 "Erev",
 "gmax",
 "mg",
 "vmin",
 "vmax",
 "Cmax",
 "Cmin",
 "Cdur",
 "Rb",
 "Ru",
 "Ro",
 "Rc",
 "Rd1",
 "Rr1",
 "Rd2",
 "Rr2",
 0,
 "i",
 "g",
 "Cglut",
 "rb",
 0,
 "U",
 "Cl",
 "D1",
 "D2",
 "O",
 "B",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 35, _prop);
 	/*initialize range parameters*/
 	Erev = 0;
 	gmax = 500;
 	mg = 1;
 	vmin = -120;
 	vmax = 100;
 	Cmax = 1;
 	Cmin = 0;
 	Cdur = 1;
 	Rb = 0.01;
 	Ru = 0.06156;
 	Ro = 0.0465;
 	Rc = 0.0916;
 	Rd1 = 0.02166;
 	Rr1 = 0.004002;
 	Rd2 = 0.00267;
 	Rr2 = 0.00193;
  }
 	_prop->param = _p;
 	_prop->param_size = 35;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[2]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _NMDA5d2nc_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 3,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
  _extcall_thread = (Datum*)ecalloc(2, sizeof(Datum));
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
     _nrn_thread_table_reg(_mechtype, _check_table_thread);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 35, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "netsend");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 2;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 NMDA5d2nc /Users/yjkimnada/hGLM/Neuron/x86_64/NMDA5d2nc.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double *_t_B;
static int _reset;
static char *modelname = "detailed model of glutamate NMDA receptors";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int _f_rates(_threadargsprotocomma_ double);
static int rates(_threadargsprotocomma_ double);
 extern double *_nrn_thread_getelm();
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(_so, _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  
#define _linmat1  1
 static int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static void _n_rates(_threadargsprotocomma_ double _lv);
 static int _slist1[5], _dlist1[5]; static double *_temp1;
 static int kstates();
 
static int kstates (void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt)
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=1;_i<5;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rb = Rb * ( 1e3 ) * Cglut ;
   /* ~ U <-> Cl ( rb , Ru )*/
 f_flux =  rb * U ;
 b_flux =  Ru * Cl ;
 _RHS1( 4) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  rb ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 1 ,4)  -= _term;
 _term =  Ru ;
 _MATELM1( 4 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ Cl <-> D1 ( Rd1 , Rr1 )*/
 f_flux =  Rd1 * Cl ;
 b_flux =  Rr1 * D1 ;
 _RHS1( 1) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  Rd1 ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 3 ,1)  -= _term;
 _term =  Rr1 ;
 _MATELM1( 1 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ D1 <-> D2 ( Rd2 , Rr2 )*/
 f_flux =  Rd2 * D1 ;
 b_flux =  Rr2 * D2 ;
 _RHS1( 3) -= (f_flux - b_flux);
 _RHS1( 2) += (f_flux - b_flux);
 
 _term =  Rd2 ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  Rr2 ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ Cl <-> O ( Ro , Rc )*/
 f_flux =  Ro * Cl ;
 b_flux =  Rc * O ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  Ro ;
 _MATELM1( 1 ,1)  += _term;
 _term =  Rc ;
 _MATELM1( 1 ,0)  -= _term;
 /*REACTION*/
   /* U + Cl + D1 + D2 + O = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= O ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= D2 ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= D1 ;
 _MATELM1(0, 1) = 1;
 _RHS1(0) -= Cl ;
 _MATELM1(0, 4) = 1;
 _RHS1(0) -= U ;
 /*CONSERVATION*/
   } return _reset;
 }
 static double _mfac_rates, _tmin_rates;
  static void _check_rates(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  static double _sav_mg;
  if (!usetable) {return;}
  if (_sav_mg != mg) { _maktable = 1;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_rates =  vmin ;
   _tmax =  vmax ;
   _dx = (_tmax - _tmin_rates)/200.; _mfac_rates = 1./_dx;
   for (_i=0, _x=_tmin_rates; _i < 201; _x += _dx, _i++) {
    _f_rates(_p, _ppvar, _thread, _nt, _x);
    _t_B[_i] = B;
   }
   _sav_mg = mg;
  }
 }

 static int rates(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv) { 
#if 0
_check_rates(_p, _ppvar, _thread, _nt);
#endif
 _n_rates(_p, _ppvar, _thread, _nt, _lv);
 return 0;
 }

 static void _n_rates(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 _f_rates(_p, _ppvar, _thread, _nt, _lv); return; 
}
 _xi = _mfac_rates * (_lv - _tmin_rates);
 if (isnan(_xi)) {
  B = _xi;
  return;
 }
 if (_xi <= 0.) {
 B = _t_B[0];
 return; }
 if (_xi >= 200.) {
 B = _t_B[200];
 return; }
 _i = (int) _xi;
 _theta = _xi - (double)_i;
 B = _t_B[_i] + _theta*(_t_B[_i+1] - _t_B[_i]);
 }

 
static int  _f_rates ( _threadargsprotocomma_ double _lv ) {
   B = 1.0 / ( 1.0 + exp ( 0.062 * - _lv ) * ( mg / 3.57 ) ) ;
    return 0; }
 
static double _hoc_rates(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 
#if 1
 _check_rates(_p, _ppvar, _thread, _nt);
#endif
 _r = 1.;
 rates ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{  double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _thread = (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   if ( _lflag  == 0.0 ) {
     if (  ! _args[1] ) {
       Cglut = Cmax * _args[0] ;
       _args[1] = 1.0 ;
       net_send ( _tqitem, _args, _pnt, t +  Cdur , 1.0 ) ;
       }
     else {
       net_move ( _tqitem, _pnt, t + Cdur ) ;
       }
     }
   if ( _lflag  == 1.0 ) {
     Cglut = Cmin * _args[0] ;
     _args[1] = 0.0 ;
     }
   } }
 
/*CVODE ode begin*/
 static int _ode_spec1(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<5;_i++) _p[_dlist1[_i]] = 0.0;}
 rb = Rb * ( 1e3 ) * Cglut ;
 /* ~ U <-> Cl ( rb , Ru )*/
 f_flux =  rb * U ;
 b_flux =  Ru * Cl ;
 DU -= (f_flux - b_flux);
 DCl += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ Cl <-> D1 ( Rd1 , Rr1 )*/
 f_flux =  Rd1 * Cl ;
 b_flux =  Rr1 * D1 ;
 DCl -= (f_flux - b_flux);
 DD1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ D1 <-> D2 ( Rd2 , Rr2 )*/
 f_flux =  Rd2 * D1 ;
 b_flux =  Rr2 * D2 ;
 DD1 -= (f_flux - b_flux);
 DD2 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ Cl <-> O ( Ro , Rc )*/
 f_flux =  Ro * Cl ;
 b_flux =  Rc * O ;
 DCl -= (f_flux - b_flux);
 DO += (f_flux - b_flux);
 
 /*REACTION*/
   /* U + Cl + D1 + D2 + O = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<5;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 rb = Rb * ( 1e3 ) * Cglut ;
 /* ~ U <-> Cl ( rb , Ru )*/
 _term =  rb ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 1 ,4)  -= _term;
 _term =  Ru ;
 _MATELM1( 4 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ Cl <-> D1 ( Rd1 , Rr1 )*/
 _term =  Rd1 ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 3 ,1)  -= _term;
 _term =  Rr1 ;
 _MATELM1( 1 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ D1 <-> D2 ( Rd2 , Rr2 )*/
 _term =  Rd2 ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 2 ,3)  -= _term;
 _term =  Rr2 ;
 _MATELM1( 3 ,2)  -= _term;
 _MATELM1( 2 ,2)  += _term;
 /*REACTION*/
  /* ~ Cl <-> O ( Ro , Rc )*/
 _term =  Ro ;
 _MATELM1( 1 ,1)  += _term;
 _MATELM1( 0 ,1)  -= _term;
 _term =  Rc ;
 _MATELM1( 1 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
   /* U + Cl + D1 + D2 + O = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 5;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 5; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _cvode_sparse_thread(&_thread[_cvspth1]._pvoid, 5, _dlist1, _p, _ode_matsol1, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(_thread[_cvspth1]._pvoid);
   _nrn_destroy_sparseobj_thread(_thread[_spth1]._pvoid);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  B = B0;
  Cl = Cl0;
  D2 = D20;
  D1 = D10;
  O = O0;
  U = U0;
 {
   rates ( _threadargscomma_ v ) ;
   U = 1.0 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];

#if 0
 _check_rates(_p, _ppvar, _thread, _nt);
#endif
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   rates ( _threadargscomma_ v ) ;
   g = gmax * O * B ;
   i = ( 1e-6 ) * g * ( v - Erev ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {  sparse_thread(&_thread[_spth1]._pvoid, 5, _slist1, _dlist1, _p, &t, dt, kstates, _linmat1, _ppvar, _thread, _nt);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 5; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 }}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
   _t_B = makevector(201*sizeof(double));
 _slist1[0] = &(O) - _p;  _dlist1[0] = &(DO) - _p;
 _slist1[1] = &(Cl) - _p;  _dlist1[1] = &(DCl) - _p;
 _slist1[2] = &(D2) - _p;  _dlist1[2] = &(DD2) - _p;
 _slist1[3] = &(D1) - _p;  _dlist1[3] = &(DD1) - _p;
 _slist1[4] = &(U) - _p;  _dlist1[4] = &(DU) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/Users/yjkimnada/hGLM/Neuron/NMDA5d2nc.mod";
static const char* nmodl_file_text = 
  "\n"
  "TITLE detailed model of glutamate NMDA receptors\n"
  "\n"
  "COMMENT\n"
  "-----------------------------------------------------------------------------\n"
  "\n"
  "Kinetic model of NMDA receptors\n"
  "===============================\n"
  "Based on the simplification of the \n"
  "	\n"
  "	10-state gating model:\n"
  "	Kampa et al. (2004) J Physiol\n"
  "	\n"
  "	by eliminating the dynamics of the Mg-block and hence simplifying the \n"
  "	dynamics to 5-state. \n"
  "	\n"
  "	U -- Cl -- O\n"
  "	     |\n"
  "      	 D1\n"
  "	     |\n"
  "	     D2\n"
  "		   \n"
  "Voltage dependence of Mg2+ block:\n"
  "	Jahr & Stevens 1990. J Neurosci 10: 1830.\n"
  "	Jahr & Stevens 1990. J Neurosci 10: 3178.\n"
  "	\n"
  "-----------------------------------------------------------------------------\n"
  "	\n"
  "  The original model is based on voltage-clamp recordings of NMDA receptor-mediated  \n"
  "  currents in nucleated patches of  rat neocortical layer 5 pyramidal neurons (Kampa 2004), \n"
  "  this model was fit with AxoGraph directly to experimental recordings in \n"
  "  order to obtain the optimal values for the parameters.\n"
  "  \n"
  "  -----------------------------------------------------------------------------\n"
  "  \n"
  "  Rates modified for near physiological temperatures with Q10 values from\n"
  "  O.Cais et al 2008, Mg unbinding from Vargas-Caballero 2003, opening and\n"
  "  closing from Lester and Jahr 1992.\n"
  "\n"
  "  Tiago Branco 2010\n"
  "  \n"
  "  -----------------------------------------------------------------------------\n"
  "  \n"
  "  Modified by Balazs B Ujfalussy in 2016 to \n"
  "  \n"
  "  - include a simple mechnism of transmitter release based on the exampl 10.6. \n"
  "  of the Neuron book. The synapse can be driven by NetCon object. (Note, that there \n"
  "  is some redundancy about the synaptic weight, since it is controlled by the \n"
  "  parameter gmax, Cmax and the weight of the NetCon object)\n"
  "  \n"
  "  - Make the dynamics of the voltage-dependent Mg binding and unbinding instantaneous \n"
  "  (based on the arguments in   \n"
  "  Destexhe, A., Mainen, Z.F. and Sejnowski, T.J.  Kinetic models of \n"
  "  synaptic transmission.  In: Methods in Neuronal Modeling (2nd edition; \n"
  "  edited by Koch, C. and Segev, I.), MIT press, Cambridge, 1998, pp 1-25.)\n"
  "  \n"
  "  -----------------------------------------------------------------------------\n"
  "  \n"
  "ENDCOMMENT\n"
  "\n"
  "INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS NMDA5d2nc\n"
  "	RANGE Cglut, Cmax, Cmin, Cdur\n"
  "	RANGE U, Cl, D1, D2, O, B\n"
  "	RANGE g, gmax, rb\n"
  ":	GLOBAL Erev, mg, Rb, Ru, Rd, Rr, Ro, Rc\n"
  ":	GLOBAL vmin, vmax\n"
  "	RANGE Erev, mg, Rb, Ru, Rd1, Rd2, Rr1, Rr2, Ro, Rc\n"
  "	RANGE vmin, vmax\n"
  "	NONSPECIFIC_CURRENT i\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(nA) = (nanoamp)\n"
  "	(mV) = (millivolt)\n"
  "	(pS) = (picosiemens)\n"
  "	(umho) = (micromho)\n"
  "	(mM) = (milli/liter)\n"
  "	(uM) = (micro/liter)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "\n"
  "	Erev	= 0    (mV)	: reversal potential\n"
  "	gmax	= 500  (pS)	: maximal conductance\n"
  "	mg	= 1    (mM)	: external magnesium concentration\n"
  "	vmin    = -120	(mV)\n"
  "	vmax    = 100	(mV)\n"
  "	Cmax	= 1   (mM)     : maximal transmitter concentration at the synapse\n"
  "	Cmin	= 0   (mM)     : residual transmitter concentration at the synapse\n"
  "	Cdur	= 1   (ms)     : duration of transmitter release \n"
  "	\n"
  ": Rates\n"
  "\n"
  "	: : Destexhe, Mainen & Sejnowski, 1996\n"
  "	: Rb	= 5e-3    (/uM /ms)	: 0.005  binding 		\n"
  "	: Ru	= 12.9e-3  (/ms)	: 0.0129 unbinding		\n"
  "	: Rd	= 8.4e-3   (/ms)	: 0.0084 desensitization\n"
  "	: Rr	= 6.8e-3   (/ms)	: 0.0068 resensitization \n"
  "	: Ro	= 46.5e-3   (/ms)	: 0.046  opening\n"
  "	: Rc	= 73.8e-3   (/ms)	: 0.0738 closing\n"
  "    \n"
  "    : : Kampa et al., 2004 - rates without Mg\n"
  "    : Rb	= 10e-3    (/uM /ms)	: binding 		\n"
  "	: Ru	= 20.16e-3  (/ms)	: unbinding\n"
  "	: Ro	= 46.5e-3   (/ms)	: opening\n"
  "	: Rc	= 91.6e-3   (/ms)	: closing\n"
  "	\n"
  "	: Rd1	= 22.66e-3   (/ms)	: fast desensitization\n"
  "	: Rr1	= 7.36e-3   (/ms)	: fast resensitization \n"
  "	: Rd2	= 4.43e-3   (/ms)	: slow desensitization\n"
  "	: Rr2	= 2.3e-3   (/ms)	: slow resensitization 	\n"
  "	\n"
  "    : Kampa et al., 2004 - rates with Mg\n"
  "    Rb	= 10e-3    (/uM /ms)	: binding 		\n"
  "	Ru	= 61.56e-3  (/ms)	: unbinding\n"
  "	Ro	= 46.5e-3   (/ms)	: opening\n"
  "	Rc	= 91.6e-3   (/ms)	: closing\n"
  "	\n"
  "	Rd1	= 21.66e-3   (/ms)	: fast desensitization\n"
  "	Rr1	= 4.002e-3   (/ms)	: fast resensitization \n"
  "	Rd2	= 2.67e-3   (/ms)	: slow desensitization\n"
  "	Rr2	= 1.93e-3   (/ms)	: slow resensitization 	\n"
  "}\n"
  "\n"
  "\n"
  "ASSIGNED {\n"
  "	v		(mV)		: postsynaptic voltage\n"
  "	i 		(nA)		: current = g*(v - Erev)\n"
  "	g 		(pS)		: conductance\n"
  "	Cglut 		(mM)		: pointer to glutamate concentration\n"
  "\n"
  "	rb		(/ms)    : binding\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	: Channel states (all fractions)\n"
  "	U		: unbound\n"
  "	Cl		: closed - single, corresponds to the old, double bound\n"
  "	D1		: desensitized 1\n"
  "	D2		: desensitized 2\n"
  "	O		: open\n"
  "\n"
  "	B		: fraction free of Mg2+ block\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	rates(v)\n"
  "	U = 1\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	rates(v)\n"
  "	SOLVE kstates METHOD sparse\n"
  "\n"
  "	g = gmax * O * B\n"
  "	i = (1e-6) * g * (v - Erev)\n"
  "}\n"
  "\n"
  "KINETIC kstates {\n"
  "	\n"
  "	rb = Rb * (1e3) * Cglut \n"
  "\n"
  "	~ U <-> Cl	(rb,Ru)\n"
  "	~ Cl <-> D1	(Rd1,Rr1)\n"
  "	~ D1 <-> D2	(Rd2,Rr2)\n"
  "	~ Cl <-> O	(Ro,Rc)\n"
  "\n"
  "	CONSERVE U+Cl+D1+D2+O = 1\n"
  "}\n"
  "\n"
  "PROCEDURE rates(v(mV)) {\n"
  "	TABLE B\n"
  "	DEPEND mg\n"
  "	FROM vmin TO vmax WITH 200\n"
  "\n"
  "	: from Jahr & Stevens\n"
  "\n"
  "	B = 1 / (1 + exp(0.062 (/mV) * -v) * (mg / 3.57 (mM)))\n"
  "}\n"
  "    \n"
  "    \n"
  "    \n"
  "NET_RECEIVE(weight, on) {\n"
  "      : on == 1 if transmitter is present (\"onset\" state), otherwise 0\n"
  "      : flag is an implicit argument of NET_RECEIVE, normally 0\n"
  "      if (flag == 0) {\n"
  "        : a spike happened, so increase the transmitter concentration to Cmax\n"
  "        if (!on) {\n"
  "          Cglut = Cmax * weight\n"
  "	  on = 1\n"
  "          net_send(Cdur, 1)\n"
  "        } else {\n"
  "          : already in onset state, so move offset time\n"
  "          net_move(t+Cdur)\n"
  "        }\n"
  "      }\n"
  "      if (flag == 1) {\n"
  "        : \"turn off transmitter\"\n"
  "        Cglut = Cmin * weight\n"
  "        on = 0\n"
  "      }\n"
  "}\n"
  ;
#endif
