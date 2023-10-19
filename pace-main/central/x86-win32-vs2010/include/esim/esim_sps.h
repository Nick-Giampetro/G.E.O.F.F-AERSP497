#ifndef F__work_esim_esim_sps_h_PROTECT
#define F__work_esim_esim_sps_h_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
/* Created by Esim dbp version $Revision: 2.0 $, Mon Nov 19 08:38:51 2018 */

#include "F:/work/esim/esim_ref.h"


#ifndef SUL_FUNCTIONS_whenexit_ref
#define SUL_FUNCTIONS_whenexit_ref 1
#define SIZE_whenexit_ref (0 + 4)
#define EQUATE_whenexit_ref(DEST,SRC) {\
    (DEST).nfuncs = (SRC).nfuncs;}
void sulPck_whenexit_ref(void *o, char *buf, int *idx);
void sulUpk_whenexit_ref(void *o, char *buf, int *idx);
void sulPckSwap_whenexit_ref(void *o, char *buf, int *idx);
void sulUpkSwap_whenexit_ref(void *o, char *buf, int *idx);
void *sulAlloc_whenexit_ref(int n);
void sulDealloc_whenexit_ref(void *o);

#endif /*SUL_FUNCTIONS_whenexit_ref */


#ifndef SUL_FUNCTIONS_esim_ref
#define SUL_FUNCTIONS_esim_ref 1
#define SIZE_esim_ref (0 + 4)
#define EQUATE_esim_ref(DEST,SRC) {\
    (DEST).numberOfDirs = (SRC).numberOfDirs;}
void sulPck_esim_ref(void *o, char *buf, int *idx);
void sulUpk_esim_ref(void *o, char *buf, int *idx);
void sulPckSwap_esim_ref(void *o, char *buf, int *idx);
void sulUpkSwap_esim_ref(void *o, char *buf, int *idx);
void *sulAlloc_esim_ref(int n);
void sulDealloc_esim_ref(void *o);

#endif /*SUL_FUNCTIONS_esim_ref */

#if defined(__cplusplus)
}
#endif


#endif

