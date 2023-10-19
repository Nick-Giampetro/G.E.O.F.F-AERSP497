/* Created by ESim dbp version $Revision: 2.1$, Sat Jul 22 12:53:17 2023 */

#define REF_FILE 1
#ifndef C__work_esim_cnsl_ref_PROTECT
#define C__work_esim_cnsl_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif
#define MAXSTACK 50
#define MAXEDIT 256

#define MAXBUTTONS 30
#define BSIZE 32


#ifndef verboseMode_enum_PROTECT
#define verboseMode_enum_PROTECT 1
enum verboseMode {
 VERBOSE_OFF=0,
 VERBOSE_NORMAL=1,
 VERBOSE_DEBUG=2
};
#endif

extern struct enum_ref C__work_esim_cnsl_enum;

#ifndef textMode_enum_PROTECT
#define textMode_enum_PROTECT 1
enum textMode {
 TEXT_NONE=0,
 TEXT_LABEL=1,
 TEXT_COMMAND=2
};
#endif

extern struct enum_ref C__work_esim_cnsl_enum;


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct cnslButton_ref {
  int textSwitch; /* (textMode) */
  int graphic; /*  */
  char textIcon[12]; /*  */
  char label[22]; /*  */
  char command[82]; /*  */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref cnslButton0_dir;
extern struct dir_ref cnslButton1_dir;
extern struct dir_ref cnslButton2_dir;
extern struct dir_ref cnslButton3_dir;
extern struct dir_ref cnslButton4_dir;
extern struct dir_ref cnslButton5_dir;
extern struct dir_ref cnslButton6_dir;
extern struct dir_ref cnslButton7_dir;
extern struct dir_ref cnslButton8_dir;
extern struct dir_ref cnslButton9_dir;
extern struct dir_ref cnslButton10_dir;
extern struct dir_ref cnslButton11_dir;
extern struct dir_ref cnslButton12_dir;
extern struct dir_ref cnslButton13_dir;
extern struct dir_ref cnslButton14_dir;
extern struct dir_ref cnslButton15_dir;
extern struct dir_ref cnslButton16_dir;
extern struct dir_ref cnslButton17_dir;
extern struct dir_ref cnslButton18_dir;
extern struct dir_ref cnslButton19_dir;
extern struct dir_ref cnslButton20_dir;
extern struct dir_ref cnslButton21_dir;
extern struct dir_ref cnslButton22_dir;
extern struct dir_ref cnslButton23_dir;
extern struct dir_ref cnslButton24_dir;
extern struct dir_ref cnslButton25_dir;
extern struct dir_ref cnslButton26_dir;
extern struct dir_ref cnslButton27_dir;
extern struct dir_ref cnslButton28_dir;
extern struct dir_ref cnslButton29_dir;
extern struct cnslButton_ref cnslButton[30];


#define CNSLSEARCHBUFFERLENGTH 80


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct cnslFileNamePopup_ref {
  char var[80]; /* (varptr) var pointer for file name */
  char search[85]; /*  */
  char file[85]; /*  */
  char callback[85]; /*  */
  int win; /* window id # */
  int open; /* open flag */
  int redraw; /* redraw level */
  int winw; /* width of window */
  int winh; /* height of window */
  int oldw; /* old width of window */
  int oldh; /* old height of window */
  int x; /* initial location of window */
  int y; /* initial locaiton of window */
  int gap; /* pixels between search box and files */
  int cursorPos; /* cursor location in search string */
  int nrows; /* number of rows to display */
  int startColumn; /* shift start column */
  int endColumn; /* shift end column */
  int numberOfColumns; /* number of columns */
  int numberOfFiles; /* number of file names read */
  int fullScreen; /*  */
  int mx; /* mouse cursor position */
  int my; /* mouse cursor position */
  int fileMouseOver; /* mouse over file number */
  int mouseOverFolder; /* 0=no,1=yes */
  int mouseOverCancel; /*  */
  int mode; /* 0=var, 1=callback */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref cnslFileNamePopup_dir;
extern struct cnslFileNamePopup_ref cnslFileNamePopup;



#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct cnslConfirmPopup_ref {
  char comment[90]; /*  */
  char command[90]; /*  */
  int win; /* window id # */
  int open; /* open flag */
  int extrawidth; /* extra width of window */
  int winw; /* width of window */
  int winh; /* height of window */
  int oldw; /* old width of window */
  int oldh; /* old height of window */
  int buttonw; /* width of buttons */
  int buttonh; /* height of buttons */
  int x; /* initial location of window */
  int y; /* initial locaiton of window */
  int mx; /* mouse cursor position */
  int my; /* mouse cursor position */
  int mouseOverButton[2]; /*  */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref cnslConfirmPopup_dir;
extern struct cnslConfirmPopup_ref cnslConfirmPopup;



#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct cnsl_ref {
  struct cnslButton_ref *button[30]; /* buttons */
  struct cnslFileNamePopup_ref *fileNamePopup; /* popup dialog for file name selection */
  struct cnslConfirmPopup_ref *confirmPopup; /* popup dialog for confirming */
  double buttonBrightness; /* how bright they appear */
  double buttonSkew; /* shading style of buttons */
  float bkgdColor[4]; /* background color */
  float shadowDarkColor[4]; /* dark shadow color */
  float shadowLightColor[4]; /* light shadow color */
  float boxColor[4]; /* text box color */
  float fontColor[4]; /* font color */
  float fontColorOld[4]; /* font color, old stuff */
  float fontColorFolder[4]; /* font color, directories */
  float textIconColor[4]; /* text icon color */
  float buttonOverTint[3]; /* tint when mouse over a button */
  int redraw; /* redraw level */
  int on; /* console input enable */
  char title[14]; /* window title */
  int verbose; /* (verboseMode) */
  int over; /* button mouse over */
  int lastOver; /* button mouse over previous */
  int pressed; /* button most recently pressed */
  int key; /* key most recently pressed */
  int numLines; /* number of console lines visible */
  int scrollUp; /* number of lines scrolled up */
  int xedit; /* position of beginning of text */
  int yedit; /* vertical location of edit area */
  int wper; /* width per character */
  int hper; /* height of each row */
  int lxoff; /* font offset from border box */
  int lyoff; /* font offset from border box */
  int wcut; /* cutoff width for last character */
  int winw; /* width of window */
  int winh; /* height of window */
  int x; /* initial location of window */
  int y; /* initial locaiton of window */
  int nbuttons; /* number of buttons */
  int rows; /* number of rows of buttons */
  int bpr; /* number of buttons per row */
  int win; /* window id # */
  char inputFileExt[6]; /* input file extension to use when loading */
  char path[124]; /* input file path */
  char welcome[124]; /* initial greeting */
  int stack; /* stick position */
  int stackLength; /* number of commands in stack */
  struct dir_ref *pwd; /* present working directory */
  int doExtraPrint; /* a flag to do extra printing, so we can switch off if needed */
  int saveToLog; /* save to a log file */
  int saveFileOpen; /* log file open */
  void *saveFileFd; /* log file descriptor */
  void (*extraPrintFunc)(const char *); /* callback for extra printf */
  void (*extraCmdFunc)(const char *); /* callback to execute after commands */
  int titleRealTime; /* show real time as title */
  int cursorPos; /* cursor position in edit buffer */
  int cursorScroll; /* scrolling window to keep cursor visible */
  char repeatCommand[258]; /*  */
  int repeatEnable; /* on/off repeat command */
  double repeatDt; /* (sec) */
  double lastRepeatTime; /* (sec) last update of repeat command */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref cnsl_dir;
extern struct cnsl_ref cnsl;

extern struct cnslFileNamePopup_ref cnslFileNamePopup;
extern struct cnslConfirmPopup_ref cnslConfirmPopup;
extern struct dir_ref root_dir;



extern struct dir_ref cnslButton0_dir;
extern struct dir_ref cnslButton1_dir;
extern struct dir_ref cnslButton2_dir;
extern struct dir_ref cnslButton3_dir;
extern struct dir_ref cnslButton4_dir;
extern struct dir_ref cnslButton5_dir;
extern struct dir_ref cnslButton6_dir;
extern struct dir_ref cnslButton7_dir;
extern struct dir_ref cnslButton8_dir;
extern struct dir_ref cnslButton9_dir;
extern struct dir_ref cnslButton10_dir;
extern struct dir_ref cnslButton11_dir;
extern struct dir_ref cnslButton12_dir;
extern struct dir_ref cnslButton13_dir;
extern struct dir_ref cnslButton14_dir;
extern struct dir_ref cnslButton15_dir;
extern struct dir_ref cnslButton16_dir;
extern struct dir_ref cnslButton17_dir;
extern struct dir_ref cnslButton18_dir;
extern struct dir_ref cnslButton19_dir;
extern struct dir_ref cnslButton20_dir;
extern struct dir_ref cnslButton21_dir;
extern struct dir_ref cnslButton22_dir;
extern struct dir_ref cnslButton23_dir;
extern struct dir_ref cnslButton24_dir;
extern struct dir_ref cnslButton25_dir;
extern struct dir_ref cnslButton26_dir;
extern struct dir_ref cnslButton27_dir;
extern struct dir_ref cnslButton28_dir;
extern struct dir_ref cnslButton29_dir;
extern struct dir_ref cnslFileNamePopup_dir;
extern struct dir_ref cnslConfirmPopup_dir;

#if defined(__cplusplus)
}
#endif

#endif
