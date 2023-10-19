#ifndef F__work_esim_brwin_sps_h_PROTECT
#define F__work_esim_brwin_sps_h_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
/* Created by Esim dbp version $Revision: 2.0 $, Mon Nov 19 08:38:51 2018 */

#include "F:/work/esim/brwin_ref.h"


#ifndef SUL_FUNCTIONS_brwin_ref
#define SUL_FUNCTIONS_brwin_ref 1
#define SIZE_brwin_ref (0 + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + 4 + 4 + 4 + 4 + 4 + 4 + 1 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + (4*3) + 4 + 4)
#define EQUATE_brwin_ref(DEST,SRC) { int ii = 0;\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).bkgdColor[ii] = (SRC).bkgdColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).fontColor[ii] = (SRC).fontColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).fontDirColor[ii] = (SRC).fontDirColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).selectedColor[ii] = (SRC).selectedColor[ii];\
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
    (DEST).visibility = (SRC).visibility;\
    (DEST).open = (SRC).open;\
    (DEST).edit = (SRC).edit;\
    (DEST).search = (SRC).search;\
    (DEST).redraw = (SRC).redraw;\
    (DEST).bar = (SRC).bar;\
    (DEST).key = (SRC).key;\
    (DEST).specialKey = (SRC).specialKey;\
    (DEST).selectedVar = (SRC).selectedVar;\
    (DEST).resizeColumns = (SRC).resizeColumns;\
    (DEST).dragx = (SRC).dragx;\
    (DEST).xupDir = (SRC).xupDir;\
    (DEST).yupDir = (SRC).yupDir;\
    (DEST).xbar = (SRC).xbar;\
    (DEST).ybar = (SRC).ybar;\
    (DEST).ystart = (SRC).ystart;\
    (DEST).yedit = (SRC).yedit;\
    (DEST).hper = (SRC).hper;\
    (DEST).xtype = (SRC).xtype;\
    (DEST).xname = (SRC).xname;\
    (DEST).xvalue = (SRC).xvalue;\
    (DEST).xcomment = (SRC).xcomment;\
    (DEST).xend = (SRC).xend;\
    (DEST).wtype = (SRC).wtype;\
    (DEST).wname = (SRC).wname;\
    (DEST).wvalue = (SRC).wvalue;\
    (DEST).wcomment = (SRC).wcomment;\
    (DEST).firstVar = (SRC).firstVar;\
    (DEST).numVar = (SRC).numVar;\
    (DEST).lxoff = (SRC).lxoff;\
    (DEST).lyoff = (SRC).lyoff;\
    (DEST).wcut = (SRC).wcut;\
    (DEST).winw = (SRC).winw;\
    (DEST).winh = (SRC).winh;\
    (DEST).x = (SRC).x;\
    (DEST).y = (SRC).y;\
    (DEST).win = (SRC).win;\
    (DEST).cursorPos = (SRC).cursorPos;\
    for( ii = 0; ii < 3; ii++ ) {\
        (DEST).plotMenu[ii] = (SRC).plotMenu[ii];\
    }\
    (DEST).plotsMenu = (SRC).plotsMenu;\
    (DEST).number = (SRC).number;}
void sulPck_brwin_ref(void *o, char *buf, int *idx);
void sulUpk_brwin_ref(void *o, char *buf, int *idx);
void sulPckSwap_brwin_ref(void *o, char *buf, int *idx);
void sulUpkSwap_brwin_ref(void *o, char *buf, int *idx);
void *sulAlloc_brwin_ref(int n);
void sulDealloc_brwin_ref(void *o);

#endif /*SUL_FUNCTIONS_brwin_ref */


#ifndef SUL_FUNCTIONS_brwins_ref
#define SUL_FUNCTIONS_brwins_ref 1
#define SIZE_brwins_ref (0 + 4)
#define EQUATE_brwins_ref(DEST,SRC) {\
    (DEST).currentWin = (SRC).currentWin;}
void sulPck_brwins_ref(void *o, char *buf, int *idx);
void sulUpk_brwins_ref(void *o, char *buf, int *idx);
void sulPckSwap_brwins_ref(void *o, char *buf, int *idx);
void sulUpkSwap_brwins_ref(void *o, char *buf, int *idx);
void *sulAlloc_brwins_ref(int n);
void sulDealloc_brwins_ref(void *o);

#endif /*SUL_FUNCTIONS_brwins_ref */

#if defined(__cplusplus)
}
#endif


#endif

