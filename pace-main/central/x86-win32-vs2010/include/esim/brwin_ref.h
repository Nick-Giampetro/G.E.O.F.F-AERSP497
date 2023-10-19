/* Created by ESim dbp version $Revision: 2.1$, Sat Jul 22 12:53:17 2023 */

#define REF_FILE 1
#ifndef C__work_esim_brwin_ref_PROTECT
#define C__work_esim_brwin_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif

#ifndef editMode_enum_PROTECT
#define editMode_enum_PROTECT 1
enum editMode {
 EDIT_OFF=0,
 EDIT_NORMAL=1,
 EDIT_ENUM=2
};
#endif

extern struct enum_ref C__work_esim_brwin_enum;

#ifndef glutFont_enum_PROTECT
#define glutFont_enum_PROTECT 1
enum glutFont {
 HELVETICA_18=8,
 HELVETICA_12=7,
 HELVETICA_10=6,
 TIMES_ROMAN_24=5,
 TIMES_ROMAN_10=4,
 BITMAP_8_BY_13=3,
 BITMAP_9_BY_15=2
};
#endif

extern struct enum_ref C__work_esim_brwin_enum;


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct brwin_ref {
  struct dir_ref *currentDir; /* current dir displayed */
  float fontColor[4]; /* font color */
  float fontDirColor[4]; /* font color */
  float fontLabelColor[4]; /* font color */
  float selectedColor[4]; /* selected var color */
  float scrollBarColor[4]; /* moving scroll color */
  float scrollBarMouseOverColor[4]; /* moving scroll color */
  float scrollBarMouseMoveColor[4]; /* moving scroll color */
  float enumEditColor[4]; /* bkgd when editting */
  int visibility; /* window is visible */
  int open; /* window is open */
  int edit; /* (editMode) editing a var */
  int search; /* 1 = search for a var, 2 = search for a dir */
  int xupDir; /* position of up dir button */
  int yupDir; /* position of up dir button */
  int xbar; /* position of scroll bar */
  int ybar; /* position of scroll bar */
  char font; /* (glutFont) */
  char doubleFmt[12]; /* format for double value */
  char floatFmt[12]; /* format for float value */
  char longFmt[12]; /* format for long value */
  char ulongFmt[12]; /* format for long value */
  char intFmt[12]; /* format for int value */
  char uintFmt[12]; /* format for uint value */
  char charFmt[12]; /* format for char value */
  char ucharFmt[12]; /* format for uchar value */
  char shortFmt[12]; /* format for int value */
  char ushortFmt[12]; /* format for uint value */
  char ptrFmt[12]; /* format for pointer values */
  char fptrFmt[12]; /* format for function pointer values */
  int redraw; /* redraw level */
  int bar; /* scroll bar visible */
  float barStart; /* location of scroll bar */
  float barEnd; /* location of scroll bar */
  char key; /* most recent key hit */
  int specialKey; /* most recent special key hit */
  int modifiers; /* shift and control keys */
  int selectedVar; /* currently selected Var */
  int mouseOverUpArrow; /*  */
  int mouseOverDownArrow; /*  */
  int mouseOverScroll; /*  */
  int mouseOverUpDir; /*  */
  int resizeColumns; /* resize columns mode */
  int dragx; /* resize column start position */
  int moveVertical; /* move vertical with mouse */
  double dragy; /* move vertical with mouse start position */
  int ystart; /* location of first column */
  int yedit; /* vertical location of edit area */
  int hper; /* height of each row */
  int xtype; /* position of type column */
  int xname; /* position of name column */
  int xvalue; /* position of value column */
  int xcomment; /* position of comment column */
  int xend; /* position of end of last column compared to window (normally<=0) */
  int wtype; /* width of type column */
  int wname; /* width of name column */
  int wvalue; /* width of value column */
  int wcomment; /* width of comment column */
  int firstVar; /* first variable viewed */
  int numVar; /* number of variables viewed */
  int lxoff; /* font offset from border box */
  int lyoff; /* font offset from border box */
  int wcut; /* cutoff width for last character */
  int winw; /* width of window */
  int winh; /* height of window */
  int x; /* initial location of window */
  int y; /* initial locaiton of window */
  int win; /* window id # */
  int cursorPos; /* cursor position */
  int cursorScroll; /* scroll edit window */
  int plotMenu[3]; /* plot menu ids */
  int plotsMenu; /* plots menu id */
  int number; /* window number */
  int editBufferOption; /*  */
  char commandF1[42]; /*  */
  char commandF2[42]; /*  */
  char editBuffer[258]; /* edit buffer */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref brwin0_dir;
extern struct dir_ref brwin1_dir;
extern struct dir_ref brwin2_dir;
extern struct brwin_ref brwin[3];


extern struct dir_ref root_dir;
extern struct dir_ref root_dir;
extern struct dir_ref root_dir;
#define EDITBUFFERLENGTH 256



#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct brwins_ref {
  struct brwin_ref *brwin[3]; /* browse window */
  int currentWin; /* current browser window */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref brwins_dir;
extern struct brwins_ref brwins;




extern struct dir_ref brwin0_dir;
extern struct dir_ref brwin1_dir;
extern struct dir_ref brwin2_dir;

#if defined(__cplusplus)
}
#endif

#endif
