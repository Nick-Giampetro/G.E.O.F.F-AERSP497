/* Created by ESim dbp version $Revision: 2.1$, Sat Jul 22 12:53:17 2023 */

#define REF_FILE 1
#ifndef C__work_esim_plwin_ref_PROTECT
#define C__work_esim_plwin_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif

#ifndef plotMode_enum_PROTECT
#define plotMode_enum_PROTECT 1
enum plotMode {
 MODE_PLOTS=0,
 MODE_DIALS=1
};
#endif

extern struct enum_ref C__work_esim_plwin_enum;

#ifndef scaleMode_enum_PROTECT
#define scaleMode_enum_PROTECT 1
enum scaleMode {
 SCALE_MANUAL=0,
 SCALE_AUTO=1,
 SCALE_WINDOW=2
};
#endif

extern struct enum_ref C__work_esim_plwin_enum;
#define MAXPLOTS 10
#define ZOOM_FACTOR 0.15

#ifndef onoff_enum_PROTECT
#define onoff_enum_PROTECT 1
enum onoff {
 OFF=0,
 ON=1
};
#endif

extern struct enum_ref C__work_esim_plwin_enum;

#ifndef yesno_enum_PROTECT
#define yesno_enum_PROTECT 1
enum yesno {
 NO=0,
 YES=1
};
#endif

extern struct enum_ref C__work_esim_plwin_enum;


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct plwin_ref {
  int numberOfPlots; /* number of plots to show on page (max 10) */
  char xvar[80]; /* (varptr) x axis var pointer */
  char yvar0[80]; /* (varptr) y axis var pointer 0 */
  char yvar1[80]; /* (varptr) y axis var pointer 1 */
  char yvar2[80]; /* (varptr) y axis var pointer 2 */
  char yvar3[80]; /* (varptr) y axis var pointer 3 */
  char yvar4[80]; /* (varptr) y axis var pointer 4 */
  char yvar5[80]; /* (varptr) y axis var pointer 5 */
  char yvar6[80]; /* (varptr) y axis var pointer 6 */
  char yvar7[80]; /* (varptr) y axis var pointer 7 */
  char yvar8[80]; /* (varptr) y axis var pointer 8 */
  char yvar9[80]; /* (varptr) y axis var pointer 9 */
  int nstore; /* number of time points to plot (maximum) */
  int antialias; /* (yesno) antialias data lines */
  float lineWidth; /* line width of actual data */
  float pointSizeData; /* point size at actual data */
  float seekValueLineSize; /* seek value point size */
  char valueFmt[32]; /*  */
  char saveFmt[32]; /*  */
  float lineWidthNeedle; /* line width of needle */
  int lineWidthValue; /* line width of value text (max) */
  float pointSize; /* grid point size */
  int showGrid; /* (yesno) flag to show grid on plots */
  int autoTick; /* (yesno) flag to engage automatic tick space calculation */
  int allOne; /* (yesno) plot together as a single plot */
  int mode; /* (plotMode) 0=plots, 1=dials */
  int autoScaleX; /* (scaleMode) flag to engage automatic scaling of plot, 2 = window mode */
  int autoScaleY; /* (scaleMode) flag to engage automatic scaling of plot, 2 = window mode */
  int autoScaleJump; /* (yesno) jump autoscale by tick level (otherwise, continuous) */
  double eps; /* scale used in autoscaling when only a single point */
  double xmin; /* minimum x axis value */
  double xmax; /* maximum x axis value */
  double xwin; /* window x */
  double xtick; /* change in x per tick mark and label */
  double ymin[10]; /* min y */
  double ymax[10]; /* max y */
  double ywin[10]; /* window y */
  double ytick[10]; /* tick y */
  float labelColor[4]; /* label color */
  float seekValueColor[4]; /* seek value */
  float dataColor[10][4]; /*  */
  int hper; /* pixel height per plot */
  int xoffset; /* offset of plot from edge of box */
  int yoffset; /* offset of plot from edge of box */
  int tickSize; /* size of tick marks */
  int xlabelx; /* x labels x position */
  int xlabelyUp; /* x labels y position, labels up */
  int xlabelyDown; /* x labels y position, labels down */
  int ylabelxRight; /* y labels x position, labels right */
  int ylabelxLeft; /* y labels x position, labels left */
  int ylabely; /* y labels y poisiton */
  int dlabelr; /* radius of label for dials */
  int dlabelx; /* dial label x position */
  int dlabely; /* dial label y position */
  double cirTol; /* tolerance of arc/circle drawing */
  double boxo; /* location of value box, ratio of rad */
  float boxa; /* aspect ratio of value box */
  double dialStartAngle; /* (deg) */
  double dialEndAngle; /* (deg) */
  int dialRows; /*  */
  int dialCols; /*  */
  int visibility; /* window is visible */
  int open; /* window is open */
  int redrawAll; /* redraw everything */
  int bruteForce; /* redraw everything no matter what */
  int redraw[10]; /* redraw flag per plot */
  char key; /* most recent key hit */
  int specialKey; /* most recent special key hit */
  int setScaleMouse; /* set scale with mouse switch */
  int setScaleStart[2]; /*  */
  int setScaleFinish[2]; /*  */
  int setScaleMin; /* number of pixels to move before bother to rescale */
  int seekValues; /* use mouse to find values */
  char title[13]; /* title of window (number added) */
  int number; /* window number */
  int winw; /* width of window */
  int winh; /* height of window */
  int x; /* initial location of window */
  int y; /* initial locaiton of window */
  int win; /* window id # */
  int menuMain; /* main menu id */
  void *xvarp; /* point to xvar Var */
  void *yvarp[10]; /* point to yvar Vars */
  void *xvarpd; /* point to xvar Var */
  void *yvarpd[10]; /* point to yvar Vars->data */
  void *xdata; /* stored x data */
  void *ydata[10]; /* stored y data */
  int mx; /* location of mouse */
  int my; /* location of mouse */
  int bestTick; /* seek value tick mark */
  int bestVar; /* seek value y variable */
  double bestDist2; /* distance between mouse and tick (squared) */
  long done[2][10]; /* indicies plotted */
  long frame; /* frame number */
  int init; /* manual initialize */
  unsigned char buffer; /* buffer drawing into */
  char xvarAlternateName[37]; /* x axis altnerate name */
  char yvarAlternateName0[37]; /* y axis alternate name */
  char yvarAlternateName1[37]; /* y axis alternate name */
  char yvarAlternateName2[37]; /* y axis alternate name */
  char yvarAlternateName3[37]; /* y axis alternate name */
  char yvarAlternateName4[37]; /* y axis alternate name */
  char yvarAlternateName5[37]; /* y axis alternate name */
  char yvarAlternateName6[37]; /* y axis alternate name */
  char yvarAlternateName7[37]; /* y axis alternate name */
  char yvarAlternateName8[37]; /* y axis alternate name */
  char yvarAlternateName9[37]; /* y axis alternate name */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref plwin0_dir;
extern struct dir_ref plwin1_dir;
extern struct dir_ref plwin2_dir;
extern struct plwin_ref plwin[3];




#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct plwins_ref {
  struct plwin_ref *plwin[3]; /* plot window */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref plwins_dir;
extern struct plwins_ref plwins;




extern struct dir_ref plwin0_dir;
extern struct dir_ref plwin1_dir;
extern struct dir_ref plwin2_dir;

#if defined(__cplusplus)
}
#endif

#endif
