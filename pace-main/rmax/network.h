#ifndef rmax_network_h
#define rmax_network_h

#if defined(__cplusplus)
extern "C"
{
#endif


#if defined(__MSVC__) && !defined(__GCC__)
#pragma pack(push)
#pragma pack(1)
#endif

void network_init( struct onboardControl_ref *con );
void network_f( struct onboardControl_ref *con, struct navout_ref *nav, struct navsmooth_ref *sm );

#if defined(__cplusplus)
}
#endif



#endif

