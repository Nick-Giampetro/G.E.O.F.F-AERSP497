/* Created by ESim dbp version $Revision: 2.1$, Sat Jul 22 12:53:17 2023 */

#define REF_FILE 1
#ifndef C__work_esim_input_ref_PROTECT
#define C__work_esim_input_ref_PROTECT 1

#if defined(__cplusplus)
extern "C"
{
#endif


#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct joyInput_ref {
  int joy; /* enable joystick read */
  double joyAxis[6]; /* joystick x, y, z, r, u ,v */
  int button[32]; /* button flags */
  int hatswitch[4]; /* hatswitch */
  int init; /* initialize linux joystick */
  char devicePath[28]; /* device path in linux */
  char devicePathBase[27]; /* base of the device path in linux - (0 1 etc will be appended) */
  int fd; /* file descriptor in Linux */
  int status; /* return from select command in Linux */
  char joystickType[101]; /* found joystick type (linux) */
  int inputRange; /* The range of values of the input axis when not using GLUT callbacks */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref joyInput0_dir;
extern struct dir_ref joyInput1_dir;
extern struct dir_ref joyInput2_dir;
extern struct dir_ref joyInput3_dir;
extern struct dir_ref joyInput4_dir;
extern struct dir_ref joyInput5_dir;
extern struct dir_ref joyInput6_dir;
extern struct dir_ref joyInput7_dir;
extern struct dir_ref joyInput8_dir;
extern struct dir_ref joyInput9_dir;
extern struct dir_ref joyInput10_dir;
extern struct dir_ref joyInput11_dir;
extern struct dir_ref joyInput12_dir;
extern struct dir_ref joyInput13_dir;
extern struct dir_ref joyInput14_dir;
extern struct dir_ref joyInput15_dir;
extern struct joyInput_ref joyInput[16];




#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif
struct input_ref {
  int numJoysticks; /* The number of joysticks to read */
  struct joyInput_ref *joyInput[16]; /* joystick data */
}
#if defined(__GNUC__)  && !defined(__FreeBSD__) && !defined(__arm__)
__attribute__((packed))
#endif
;
#if defined(_MSC_VER) && !defined(__GCC__)
#pragma pack(pop)
#endif


extern struct dir_ref esimInput_dir;
extern struct input_ref esimInput;




extern struct dir_ref joyInput0_dir;
extern struct dir_ref joyInput1_dir;
extern struct dir_ref joyInput2_dir;
extern struct dir_ref joyInput3_dir;
extern struct dir_ref joyInput4_dir;
extern struct dir_ref joyInput5_dir;
extern struct dir_ref joyInput6_dir;
extern struct dir_ref joyInput7_dir;
extern struct dir_ref joyInput8_dir;
extern struct dir_ref joyInput9_dir;
extern struct dir_ref joyInput10_dir;
extern struct dir_ref joyInput11_dir;
extern struct dir_ref joyInput12_dir;
extern struct dir_ref joyInput13_dir;
extern struct dir_ref joyInput14_dir;
extern struct dir_ref joyInput15_dir;

#if defined(__cplusplus)
}
#endif

#endif
