#ifndef F__work_esim_cnsl_sps_h_PROTECT
#define F__work_esim_cnsl_sps_h_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
/* Created by Esim dbp version $Revision: 2.0 $, Mon Nov 19 08:38:50 2018 */

#include "F:/work/esim/cnsl_ref.h"


#ifndef SUL_FUNCTIONS_cnslButton_ref
#define SUL_FUNCTIONS_cnslButton_ref 1
#define SIZE_cnslButton_ref (0 + 4 + 4)
#define EQUATE_cnslButton_ref(DEST,SRC) {\
    (DEST).textSwitch = (SRC).textSwitch;\
    (DEST).graphic = (SRC).graphic;}
void sulPck_cnslButton_ref(void *o, char *buf, int *idx);
void sulUpk_cnslButton_ref(void *o, char *buf, int *idx);
void sulPckSwap_cnslButton_ref(void *o, char *buf, int *idx);
void sulUpkSwap_cnslButton_ref(void *o, char *buf, int *idx);
void *sulAlloc_cnslButton_ref(int n);
void sulDealloc_cnslButton_ref(void *o);

#endif /*SUL_FUNCTIONS_cnslButton_ref */


#ifndef SUL_FUNCTIONS_cnslFileNamePopup_ref
#define SUL_FUNCTIONS_cnslFileNamePopup_ref 1
#define SIZE_cnslFileNamePopup_ref (0 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4)
#define EQUATE_cnslFileNamePopup_ref(DEST,SRC) {\
    (DEST).win = (SRC).win;\
    (DEST).open = (SRC).open;\
    (DEST).redraw = (SRC).redraw;\
    (DEST).winw = (SRC).winw;\
    (DEST).winh = (SRC).winh;\
    (DEST).oldw = (SRC).oldw;\
    (DEST).oldh = (SRC).oldh;\
    (DEST).x = (SRC).x;\
    (DEST).y = (SRC).y;\
    (DEST).gap = (SRC).gap;\
    (DEST).cursorPos = (SRC).cursorPos;\
    (DEST).nrows = (SRC).nrows;\
    (DEST).startColumn = (SRC).startColumn;\
    (DEST).endColumn = (SRC).endColumn;\
    (DEST).numberOfColumns = (SRC).numberOfColumns;\
    (DEST).numberOfFiles = (SRC).numberOfFiles;\
    (DEST).fullScreen = (SRC).fullScreen;\
    (DEST).mx = (SRC).mx;\
    (DEST).my = (SRC).my;\
    (DEST).fileMouseOver = (SRC).fileMouseOver;\
    (DEST).mouseOverFolder = (SRC).mouseOverFolder;\
    (DEST).mode = (SRC).mode;}
void sulPck_cnslFileNamePopup_ref(void *o, char *buf, int *idx);
void sulUpk_cnslFileNamePopup_ref(void *o, char *buf, int *idx);
void sulPckSwap_cnslFileNamePopup_ref(void *o, char *buf, int *idx);
void sulUpkSwap_cnslFileNamePopup_ref(void *o, char *buf, int *idx);
void *sulAlloc_cnslFileNamePopup_ref(int n);
void sulDealloc_cnslFileNamePopup_ref(void *o);

#endif /*SUL_FUNCTIONS_cnslFileNamePopup_ref */


#ifndef SUL_FUNCTIONS_cnslConfirmPopup_ref
#define SUL_FUNCTIONS_cnslConfirmPopup_ref 1
#define SIZE_cnslConfirmPopup_ref (0 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4)
#define EQUATE_cnslConfirmPopup_ref(DEST,SRC) {\
    (DEST).win = (SRC).win;\
    (DEST).open = (SRC).open;\
    (DEST).extrawidth = (SRC).extrawidth;\
    (DEST).winw = (SRC).winw;\
    (DEST).winh = (SRC).winh;\
    (DEST).oldw = (SRC).oldw;\
    (DEST).oldh = (SRC).oldh;\
    (DEST).buttonw = (SRC).buttonw;\
    (DEST).buttonh = (SRC).buttonh;\
    (DEST).x = (SRC).x;\
    (DEST).y = (SRC).y;\
    (DEST).mx = (SRC).mx;\
    (DEST).my = (SRC).my;}
void sulPck_cnslConfirmPopup_ref(void *o, char *buf, int *idx);
void sulUpk_cnslConfirmPopup_ref(void *o, char *buf, int *idx);
void sulPckSwap_cnslConfirmPopup_ref(void *o, char *buf, int *idx);
void sulUpkSwap_cnslConfirmPopup_ref(void *o, char *buf, int *idx);
void *sulAlloc_cnslConfirmPopup_ref(int n);
void sulDealloc_cnslConfirmPopup_ref(void *o);

#endif /*SUL_FUNCTIONS_cnslConfirmPopup_ref */


#ifndef SUL_FUNCTIONS_cnsl_ref
#define SUL_FUNCTIONS_cnsl_ref 1
#define SIZE_cnsl_ref (0 + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4)
#define EQUATE_cnsl_ref(DEST,SRC) { int ii = 0;\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).bkgdColor[ii] = (SRC).bkgdColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).fontColor[ii] = (SRC).fontColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).fontColorOld[ii] = (SRC).fontColorOld[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).fontColorFolder[ii] = (SRC).fontColorFolder[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).boxColor[ii] = (SRC).boxColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).shadowDarkColor[ii] = (SRC).shadowDarkColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).shadowLightColor[ii] = (SRC).shadowLightColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).textIconColor[ii] = (SRC).textIconColor[ii];\
    }\
    (DEST).redraw = (SRC).redraw;\
    (DEST).on = (SRC).on;\
    (DEST).verbose = (SRC).verbose;\
    (DEST).pressed = (SRC).pressed;\
    (DEST).key = (SRC).key;\
    (DEST).numLines = (SRC).numLines;\
    (DEST).scrollUp = (SRC).scrollUp;\
    (DEST).xedit = (SRC).xedit;\
    (DEST).yedit = (SRC).yedit;\
    (DEST).wper = (SRC).wper;\
    (DEST).hper = (SRC).hper;\
    (DEST).lxoff = (SRC).lxoff;\
    (DEST).lyoff = (SRC).lyoff;\
    (DEST).wcut = (SRC).wcut;\
    (DEST).winw = (SRC).winw;\
    (DEST).winh = (SRC).winh;\
    (DEST).x = (SRC).x;\
    (DEST).y = (SRC).y;\
    (DEST).nbuttons = (SRC).nbuttons;\
    (DEST).rows = (SRC).rows;\
    (DEST).bpr = (SRC).bpr;\
    (DEST).win = (SRC).win;\
    (DEST).stack = (SRC).stack;\
    (DEST).stackLength = (SRC).stackLength;\
    (DEST).doExtraPrint = (SRC).doExtraPrint;\
    (DEST).saveToLog = (SRC).saveToLog;\
    (DEST).saveFileOpen = (SRC).saveFileOpen;\
    (DEST).titleRealTime = (SRC).titleRealTime;\
    (DEST).cursorPos = (SRC).cursorPos;}
void sulPck_cnsl_ref(void *o, char *buf, int *idx);
void sulUpk_cnsl_ref(void *o, char *buf, int *idx);
void sulPckSwap_cnsl_ref(void *o, char *buf, int *idx);
void sulUpkSwap_cnsl_ref(void *o, char *buf, int *idx);
void *sulAlloc_cnsl_ref(int n);
void sulDealloc_cnsl_ref(void *o);

#endif /*SUL_FUNCTIONS_cnsl_ref */

#if defined(__cplusplus)
}
#endif


#endif

