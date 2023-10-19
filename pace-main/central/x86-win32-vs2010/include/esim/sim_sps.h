#ifndef F__work_esim_sim_sps_h_PROTECT
#define F__work_esim_sim_sps_h_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
/* Created by Esim dbp version $Revision: 2.0 $, Mon Nov 19 08:38:51 2018 */

#include "F:/work/esim/sim_ref.h"


#ifndef SUL_FUNCTIONS_sim_ref
#define SUL_FUNCTIONS_sim_ref 1
#define SIZE_sim_ref (0 + 4 + 4 + 8 + 8 + 8 + 8 + 4 + 8 + 8 + 8 + 4 + 8 + 8 + 4 + 4 + 4 + 8 + 4 + 4 + 4 + 8 + 4 + 4)
#define EQUATE_sim_ref(DEST,SRC) {\
    (DEST).mode = (SRC).mode;\
    (DEST).timeMode = (SRC).timeMode;\
    (DEST).time = (SRC).time;\
    (DEST).wallTime = (SRC).wallTime;\
    (DEST).dt = (SRC).dt;\
    (DEST).wallDt = (SRC).wallDt;\
    (DEST).framesPerSec = (SRC).framesPerSec;\
    (DEST).stepDt = (SRC).stepDt;\
    (DEST).maxDt = (SRC).maxDt;\
    (DEST).updateDt = (SRC).updateDt;\
    (DEST).initSteps = (SRC).initSteps;\
    (DEST).tFinal = (SRC).tFinal;\
    (DEST).scaleTime = (SRC).scaleTime;\
    (DEST).nrepeats = (SRC).nrepeats;\
    (DEST).itime = (SRC).itime;\
    (DEST).frameCount = (SRC).frameCount;\
    (DEST).waitTime = (SRC).waitTime;\
    (DEST).skipTime = (SRC).skipTime;\
    (DEST).useSkipTime = (SRC).useSkipTime;\
    (DEST).externalClock = (SRC).externalClock;\
    (DEST).externalTime = (SRC).externalTime;\
    (DEST).inits = (SRC).inits;\
    (DEST).autoInit = (SRC).autoInit;}
void sulPck_sim_ref(void *o, char *buf, int *idx);
void sulUpk_sim_ref(void *o, char *buf, int *idx);
void sulPckSwap_sim_ref(void *o, char *buf, int *idx);
void sulUpkSwap_sim_ref(void *o, char *buf, int *idx);
void *sulAlloc_sim_ref(int n);
void sulDealloc_sim_ref(void *o);

#endif /*SUL_FUNCTIONS_sim_ref */

#if defined(__cplusplus)
}
#endif


#endif

