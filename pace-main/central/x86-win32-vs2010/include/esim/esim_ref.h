/* Created by ESim dbp version $Revision: 2.1$, Sat Jul 22 12:53:17 2023 */

#define REF_FILE 1
#ifndef C__work_esim_esim_ref_PROTECT
#define C__work_esim_esim_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
#define MAXWHENEXIT 32


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct whenexit_ref {
  int nfuncs; /* nfunctions to execute on exit  */
  void (*funcs[32])(void); /* functions to be called on exit  */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref whenexit_dir;
extern struct whenexit_ref whenexit;




#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct esim_ref {
  struct sim_ref *sim; /* simulation control */
  struct cnsl_ref *cnsl; /* simulation console window */
  struct brwins_ref *brwins; /* browse windows */
  struct plwins_ref *plwins; /* plot windows */
  struct input_ref *input; /* input devices */
  struct whenexit_ref *whenexit; /* callbacks on exit */
  int numberOfDirs; /* total size of database */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref esim_dir;
extern struct esim_ref esim;

extern struct sim_ref sim;
extern struct cnsl_ref cnsl;
extern struct brwins_ref brwins;
extern struct plwins_ref plwins;
extern struct input_ref esimInput;
extern struct whenexit_ref whenexit;



extern struct dir_ref sim_dir;
extern struct dir_ref cnsl_dir;
extern struct dir_ref brwins_dir;
extern struct dir_ref plwins_dir;
extern struct dir_ref esimInput_dir;
extern struct dir_ref whenexit_dir;

#if defined(__cplusplus)
}
#endif

#endif
