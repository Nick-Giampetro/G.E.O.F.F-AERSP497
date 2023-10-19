#ifndef F__work_esim_plwin_sps_h_PROTECT
#define F__work_esim_plwin_sps_h_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
/* Created by Esim dbp version $Revision: 2.0 $, Mon Nov 19 08:38:51 2018 */

#include "F:/work/esim/plwin_ref.h"


#ifndef SUL_FUNCTIONS_plwin_ref
#define SUL_FUNCTIONS_plwin_ref 1
#define SIZE_plwin_ref (0 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 8 + 8 + 8 + 8 + 8 + (8*10) + (8*10) + (8*10) + (8*10) + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + (4*4) + (4*10*4) + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 8 + 8 + 4 + 8 + 8 + 4 + 4 + 4 + 4 + 4 + 4 + (4*10) + 1 + 4 + 4 + (4*2) + (4*2) + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 8 + (4*2*10) + 4 + 4 + 1)
#define EQUATE_plwin_ref(DEST,SRC) { int ii = 0; int jj = 0;\
    (DEST).numberOfPlots = (SRC).numberOfPlots;\
    (DEST).nstore = (SRC).nstore;\
    (DEST).antialias = (SRC).antialias;\
    (DEST).lineWidth = (SRC).lineWidth;\
    (DEST).pointSizeData = (SRC).pointSizeData;\
    (DEST).seekValueLineSize = (SRC).seekValueLineSize;\
    (DEST).lineWidthNeedle = (SRC).lineWidthNeedle;\
    (DEST).lineWidthValue = (SRC).lineWidthValue;\
    (DEST).pointSize = (SRC).pointSize;\
    (DEST).showGrid = (SRC).showGrid;\
    (DEST).autoTick = (SRC).autoTick;\
    (DEST).allOne = (SRC).allOne;\
    (DEST).mode = (SRC).mode;\
    (DEST).autoScaleX = (SRC).autoScaleX;\
    (DEST).autoScaleY = (SRC).autoScaleY;\
    (DEST).autoScaleJump = (SRC).autoScaleJump;\
    (DEST).eps = (SRC).eps;\
    (DEST).xmin = (SRC).xmin;\
    (DEST).xmax = (SRC).xmax;\
    (DEST).xwin = (SRC).xwin;\
    (DEST).xtick = (SRC).xtick;\
    for( ii = 0; ii < 10; ii++ ) {\
        (DEST).ymin[ii] = (SRC).ymin[ii];\
    }\
    for( ii = 0; ii < 10; ii++ ) {\
        (DEST).ymax[ii] = (SRC).ymax[ii];\
    }\
    for( ii = 0; ii < 10; ii++ ) {\
        (DEST).ywin[ii] = (SRC).ywin[ii];\
    }\
    for( ii = 0; ii < 10; ii++ ) {\
        (DEST).ytick[ii] = (SRC).ytick[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).bkgdColor[ii] = (SRC).bkgdColor[ii];\
    }\
    for( ii = 0; ii < 4; ii++ ) {\
        (DEST).labelColor[ii] = (SRC).labelColor[ii];\
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
        (DEST).seekValueColor[ii] = (SRC).seekValueColor[ii];\
    }\
    for( ii = 0; ii < 10; ii++ ) {\
        for( jj = 0; jj < 4; jj++ ) {\
            (DEST).dataColor[ii][jj] = (SRC).dataColor[ii][jj];\
        }\
    }\
    (DEST).hper = (SRC).hper;\
    (DEST).xoffset = (SRC).xoffset;\
    (DEST).yoffset = (SRC).yoffset;\
    (DEST).tickSize = (SRC).tickSize;\
    (DEST).xlabelx = (SRC).xlabelx;\
    (DEST).xlabelyUp = (SRC).xlabelyUp;\
    (DEST).xlabelyDown = (SRC).xlabelyDown;\
    (DEST).ylabelxRight = (SRC).ylabelxRight;\
    (DEST).ylabelxLeft = (SRC).ylabelxLeft;\
    (DEST).ylabely = (SRC).ylabely;\
    (DEST).dlabelr = (SRC).dlabelr;\
    (DEST).dlabelx = (SRC).dlabelx;\
    (DEST).dlabely = (SRC).dlabely;\
    (DEST).cirTol = (SRC).cirTol;\
    (DEST).boxo = (SRC).boxo;\
    (DEST).boxa = (SRC).boxa;\
    (DEST).dialStartAngle = (SRC).dialStartAngle;\
    (DEST).dialEndAngle = (SRC).dialEndAngle;\
    (DEST).dialRows = (SRC).dialRows;\
    (DEST).dialCols = (SRC).dialCols;\
    (DEST).visibility = (SRC).visibility;\
    (DEST).open = (SRC).open;\
    (DEST).redrawAll = (SRC).redrawAll;\
    (DEST).bruteForce = (SRC).bruteForce;\
    for( ii = 0; ii < 10; ii++ ) {\
        (DEST).redraw[ii] = (SRC).redraw[ii];\
    }\
    (DEST).key = (SRC).key;\
    (DEST).specialKey = (SRC).specialKey;\
    (DEST).setScaleMouse = (SRC).setScaleMouse;\
    for( ii = 0; ii < 2; ii++ ) {\
        (DEST).setScaleStart[ii] = (SRC).setScaleStart[ii];\
    }\
    for( ii = 0; ii < 2; ii++ ) {\
        (DEST).setScaleFinish[ii] = (SRC).setScaleFinish[ii];\
    }\
    (DEST).setScaleMin = (SRC).setScaleMin;\
    (DEST).seekValues = (SRC).seekValues;\
    (DEST).number = (SRC).number;\
    (DEST).winw = (SRC).winw;\
    (DEST).winh = (SRC).winh;\
    (DEST).x = (SRC).x;\
    (DEST).y = (SRC).y;\
    (DEST).win = (SRC).win;\
    (DEST).menuMain = (SRC).menuMain;\
    (DEST).mx = (SRC).mx;\
    (DEST).my = (SRC).my;\
    (DEST).bestTick = (SRC).bestTick;\
    (DEST).bestVar = (SRC).bestVar;\
    (DEST).bestDist2 = (SRC).bestDist2;\
    for( ii = 0; ii < 2; ii++ ) {\
        for( jj = 0; jj < 10; jj++ ) {\
            (DEST).done[ii][jj] = (SRC).done[ii][jj];\
        }\
    }\
    (DEST).frame = (SRC).frame;\
    (DEST).init = (SRC).init;\
    (DEST).buffer = (SRC).buffer;}
void sulPck_plwin_ref(void *o, char *buf, int *idx);
void sulUpk_plwin_ref(void *o, char *buf, int *idx);
void sulPckSwap_plwin_ref(void *o, char *buf, int *idx);
void sulUpkSwap_plwin_ref(void *o, char *buf, int *idx);
void *sulAlloc_plwin_ref(int n);
void sulDealloc_plwin_ref(void *o);

#endif /*SUL_FUNCTIONS_plwin_ref */


#ifndef SUL_FUNCTIONS_plwins_ref
#define SUL_FUNCTIONS_plwins_ref 1
#define SIZE_plwins_ref (0)
#define EQUATE_plwins_ref(DEST,SRC) {}
void sulPck_plwins_ref(void *o, char *buf, int *idx);
void sulUpk_plwins_ref(void *o, char *buf, int *idx);
void sulPckSwap_plwins_ref(void *o, char *buf, int *idx);
void sulUpkSwap_plwins_ref(void *o, char *buf, int *idx);
void *sulAlloc_plwins_ref(int n);
void sulDealloc_plwins_ref(void *o);

#endif /*SUL_FUNCTIONS_plwins_ref */

#if defined(__cplusplus)
}
#endif


#endif

