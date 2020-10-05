#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _CaT_reg(void);
extern void _NMDA5d2nc_reg(void);
extern void _ca_reg(void);
extern void _cad_reg(void);
extern void _exp2synNMDA_reg(void);
extern void _kca_reg(void);
extern void _km_reg(void);
extern void _kv_reg(void);
extern void _kvv_reg(void);
extern void _na_reg(void);
extern void _nap_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," CaT.mod");
    fprintf(stderr," NMDA5d2nc.mod");
    fprintf(stderr," ca.mod");
    fprintf(stderr," cad.mod");
    fprintf(stderr," exp2synNMDA.mod");
    fprintf(stderr," kca.mod");
    fprintf(stderr," km.mod");
    fprintf(stderr," kv.mod");
    fprintf(stderr," kvv.mod");
    fprintf(stderr," na.mod");
    fprintf(stderr," nap.mod");
    fprintf(stderr, "\n");
  }
  _CaT_reg();
  _NMDA5d2nc_reg();
  _ca_reg();
  _cad_reg();
  _exp2synNMDA_reg();
  _kca_reg();
  _km_reg();
  _kv_reg();
  _kvv_reg();
  _na_reg();
  _nap_reg();
}
