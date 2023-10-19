#ifndef rmax_xforms_h
#define rmax_xforms_h

#if defined(__cplusplus)
extern "C"
{
#endif

void initXforms( struct vehicleSet_ref *set,
		 struct vehicleMotion_ref *mo );
void updateXforms( struct vehicleSet_ref *set,
		   struct state_ref *s,
		   struct vehicleMotion_ref *mo );


#if defined(__cplusplus)
}
#endif

#endif
