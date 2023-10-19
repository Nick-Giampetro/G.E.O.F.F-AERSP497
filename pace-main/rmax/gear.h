#ifndef rmax_gear_h
#define rmax_gear_h
#if defined(__cplusplus)
extern "C"
{
#endif

/* in gear.c */
void gear( double terrainAlt, 
		   struct vehicleMotion_ref *mo, 
	       struct state_ref *s, 
	       struct state_ref *sdot,
           double dt);

#if defined(__cplusplus)
}
#endif
#endif

