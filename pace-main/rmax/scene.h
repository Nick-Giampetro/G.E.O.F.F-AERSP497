#ifndef rmax_scene_h
#define rmax_scene_h
#if defined(__cplusplus)
extern "C"
{
#endif

/* Copyright (c) Eric N. Johnson, 1998.  */

int initScenes( void );
void updateScenes( void );
void sceneMessageAll( char * );
void pickPlanter( double n, double e );
// Used by GCS.c as well for camera control inputs
struct scene_ref *whichScene( void );
struct gcsScene_ref *whichGcsScene( struct scene_ref *sc, struct gcsInstance_ref *gi );
int sceneGrabberStart( int win );
int sceneGrabberShutdown( int win );
int sceneGrabberFrame( int win, int *winw, int *winh, void **data );

void drawCylinder( float radius, float height );

#if defined(__cplusplus)
}
#endif
#endif
