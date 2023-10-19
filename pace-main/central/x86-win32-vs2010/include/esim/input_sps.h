#ifndef F__work_esim_input_sps_h_PROTECT
#define F__work_esim_input_sps_h_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
/* Created by Esim dbp version $Revision: 2.0 $, Mon Nov 19 08:38:51 2018 */

#include "F:/work/esim/input_ref.h"


#ifndef SUL_FUNCTIONS_joyInput_ref
#define SUL_FUNCTIONS_joyInput_ref 1
#define SIZE_joyInput_ref (0 + 4 + (8*6) + (4*32) + (4*4) + 4 + 4 + 4 + 4)
#define EQUATE_joyInput_ref(DEST,SRC) { int ii = 0;\
    (DEST).joy = (SRC).joy;\
    for( ii = 0; ii < 6; ii++ ) {\
        (DEST).joyAxis[ii] = (SRC).joyAxis[ii];\
    }\
    for( ii = 0; ii < 32; ii++ ) {\
        (DEST).button[ii] = (SRC).button[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).hatswitch[ii] = (SRC).hatswitch[ii];\
    }\
    (DEST).init = (SRC).init;\
    (DEST).fd = (SRC).fd;\
    (DEST).status = (SRC).status;\
    (DEST).inputRange = (SRC).inputRange;}
void sulPck_joyInput_ref(void *o, char *buf, int *idx);
void sulUpk_joyInput_ref(void *o, char *buf, int *idx);
void sulPckSwap_joyInput_ref(void *o, char *buf, int *idx);
void sulUpkSwap_joyInput_ref(void *o, char *buf, int *idx);
void *sulAlloc_joyInput_ref(int n);
void sulDealloc_joyInput_ref(void *o);

#endif /*SUL_FUNCTIONS_joyInput_ref */


#ifndef SUL_FUNCTIONS_input_ref
#define SUL_FUNCTIONS_input_ref 1
#define SIZE_input_ref (0 + 4)
#define EQUATE_input_ref(DEST,SRC) {\
    (DEST).numJoysticks = (SRC).numJoysticks;}
void sulPck_input_ref(void *o, char *buf, int *idx);
void sulUpk_input_ref(void *o, char *buf, int *idx);
void sulPckSwap_input_ref(void *o, char *buf, int *idx);
void sulUpkSwap_input_ref(void *o, char *buf, int *idx);
void *sulAlloc_input_ref(int n);
void sulDealloc_input_ref(void *o);

#endif /*SUL_FUNCTIONS_input_ref */

#if defined(__cplusplus)
}
#endif


#endif

