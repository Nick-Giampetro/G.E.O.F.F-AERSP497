#ifndef rmax_rotorsMultirotor_h
#define rmax_rotorsMultirotor_h

#if defined(__cplusplus)
extern "C"
{
#endif

void eachRotorMulti( struct vehicleMotion_ref *mo, struct state_ref *s, int rn);
void rotorsMultirotor( struct vehicleMotion_ref *mo, struct state_ref *s);


#if defined(__cplusplus)
}
#endif
#endif


