/* Created by ESim dbp version $Revision: 2.1$, Sat Jul 22 12:53:17 2023 */

#define REF_FILE 1
#ifndef C__work_esim_sim_ref_PROTECT
#define C__work_esim_sim_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif

#ifndef simMode_enum_PROTECT
#define simMode_enum_PROTECT 1
enum simMode {
 SIM_MODE_PAUSE=0,
 SIM_MODE_INIT=1,
 SIM_MODE_RUN=2
};
#endif

extern struct enum_ref C__work_esim_sim_enum;

#ifndef timeMode_enum_PROTECT
#define timeMode_enum_PROTECT 1
enum timeMode {
 SIM_TIME_REAL=0,
 SIM_TIME_BATCH=1
};
#endif

extern struct enum_ref C__work_esim_sim_enum;


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct sim_ref {
  int mode; /* (simMode) simulation mode */
  int timeMode; /* (timeMode) real vs. batch */
  double time; /* (s) sim time passage */
  double wallTime; /* (s) real time passage */
  double dt; /* (s) time step for model calling */
  double wallDt; /* (s) actual time step per screen update */
  int framesPerSec; /* calculated frames per second */
  double stepDt; /* (s) time step for a single step, max for sim */
  double maxDt; /* (s) max step for display update (go sub realtime to meet) */
  double updateDt; /* (s) update frequency of frame rate calculation */
  int initSteps; /* if not zero, switch to run after this many init steps */
  double tFinal; /* (s) time to pause (if <0 go forever) */
  double scaleTime; /* scale wall time (>1 faster, never zero, do not change once running) */
  int nrepeats; /* number of repeats per visual update */
  long itime; /* integer elapsed real time */
  int frameCount; /* number of frames since update of counter */
  double waitTime; /* time for next frame rate calculation */
  int skipTime; /* flag to allow stoppage of time (for pop-up menus) */
  int useSkipTime; /* use above flag */
  int externalClock; /* switch to use external clock */
  double externalTime; /* (s) external clock time */
  int inits; /* number of inits steps completed (this time) */
  int autoInit; /* automatically do this many init steps if run without init */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref sim_dir;
extern struct sim_ref sim;






#if defined(__cplusplus)
}
#endif

#endif
