#ifndef rmax_wdb_scene_h
#define rmax_wdb_scene_h

#if defined(__cplusplus)
extern "C"
{
#endif

void initWdbScene( struct sceneGlobal_ref *sg, struct scene_ref *sc );
void drawWdbScene(  struct scene_ref *sc, struct sceneGlobal_ref *sg, struct vehicleOutputs_ref *o, float zoom ) ;

#if defined(__cplusplus)
}
#endif



#endif
