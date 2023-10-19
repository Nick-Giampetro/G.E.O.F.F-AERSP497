#ifndef rmax_joyinputs_h
#define rmax_joyinputs_h

#if defined(__cplusplus)
extern "C"
{
#endif

float doStickBlend( float first, float second );
void updateJoyInputs(  struct controlInput_ref* ci, double secTime  );
void updateDigitalFunction( struct digitalFunction_ref* digiFunc, struct input_ref* joy, double secTime, struct controlInput_ref* mc );
void updateAnalogFunction( struct analogFunction_ref* analogFunc, struct input_ref* joy, struct controlInput_ref* mc );

#if defined(__cplusplus)
}
#endif

#endif

