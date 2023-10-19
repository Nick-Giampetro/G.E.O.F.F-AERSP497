 /*** BeginCopyright
 * Copyright 2002, Georgia Institute of Technology, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * UAV Laboratory
 * School of Aerospace Engineering
 * Georgia Institute of Technology
 * Atlanta, GA 30332
 * http://controls.ae.gatech.edu
 *
 * Contact Information:
//  * Prof. Eric N. Johnson
 * http://www.ae.gatech.edu/~ejohnson
 * Tel : 404 385 2519
 * EndCopyright
 ***/
/* Copyright (c) Eric N. Johnson, 1998.  */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> /* thread for loading maps */
#include <math.h>
#include <GL/glut.h>
#define GL_BGR         0x80E0
#include <time.h>    /* for clock function */
#include "esim/command.h"
#include "esim/util.h"
#include "esim/esim.h"
#include "esim/cnsl.h"
#include "esim/quat.h"
#include "esim/sim_ref.h"
#include "rmax/texture.h"
#include "rmax/motion_ref.h"
#include "rmax/onboard_ref.h"

#include "rmax/controller_ref.h"
#include "rmax/sensors_ref.h"
#include "rmax/dted.h"
#include "rmax/navigation_ref.h"
#include "rmax/gcs.h"
#include "rmax/joyinputs_ref.h"
#include "rmax/panel.h"
#include "rmax/logger.h"
#include "rmax/matrix.h"

#include "esim/quat.h"
#include "rmax/scene_ref.h"
#include "rmax/generic_ref.h"
#include "rmax/realScene_ref.h"
#include "rmax/wdb_scene.h"
#include "rmax/wdb_ref.h"
#include "rmax/si_ref.h"
#include "esim/rand.h"

#include <float.h>
#define ISFINITE _finite

#include "rmax/generic.h"
#include "rmax/scene_multirotor.h"
#include "rmax/scene.h"
#include "rmax/serial.h" //AAP for the camera control from the GCS

void sceneMouseMotion( int x, int y );

#define glError() { \
        GLenum err = glGetError(); \
        while (err != GL_NO_ERROR) { \
                fprintf(stderr, "glError: %s caught at %s:%u\n", (char *)gluErrorString(err), __FILE__, __LINE__); \
                err = glGetError(); \
        } \
}

void sceneKeyboard( unsigned char key, int x, int y );
void sceneSpecialKeys( int key, int x, int y );

#define BUFFER_SIZE 256

pthread_t       scene_threads;
pthread_mutex_t scene_mutex;
pthread_attr_t  scene_attr;

char scene_url[1024];
char scene_fileName[120];

#if defined(HAVE_GCVLIBPNG)
int sceneSavePng(struct scene_ref* sc, FILE* filep, unsigned char* pixels){

    png_structp png_ptr;
    png_infop info_ptr;
    int irow;
    int index;

    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, png_voidp_NULL, NULL, NULL);
    if (!png_ptr) {
        return -1;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL );
        return -1;
    }

    if (setjmp(png_ptr->jmpbuf)) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return -1;
    }

    png_init_io(png_ptr, filep);

    png_set_compression_level(png_ptr, Z_DEFAULT_COMPRESSION);

    png_set_IHDR(png_ptr, info_ptr, sc->winw, sc->winh,
                8, PNG_COLOR_TYPE_RGB,  PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, info_ptr);

    // openGL row zero is the bottom row, and libpng row zero is top.
    for( irow=sc->winh-1; irow>=0; irow-- ) {
        index = irow*sc->winw*3;
        png_write_row(png_ptr, &pixels[index] );
    }

    png_write_end(png_ptr, NULL);

    return 0;

}
#endif

void *sceneLoadMap( void *igm ) {

	struct internetOverlay_ref *io = (struct internetOverlay_ref *)igm;

	io->loaderThreadRunning = 0;

	pthread_detach(scene_threads);
	pthread_exit(NULL);

	return igm;
}

/* trajectory points */
#define MAXTPOINTS 100000

static struct tpoint {
    double lat;
    double lon;
    double alt;
} tpoints[MAXTPOINTS], tpoints_nav[GCS_MAX_INSTANCES][MAXTPOINTS], tpoints_gps[GCS_MAX_INSTANCES][MAXTPOINTS];
static double tpointTime = 0, tpointTime_nav[GCS_MAX_INSTANCES] = {0}, tpointTime_gps[GCS_MAX_INSTANCES] = {0};

static int trajNPoint[GCS_MAX_INSTANCES][2] = {{0,0}};
static double trajTime[GCS_MAX_INSTANCES][2] = {{-1,-1}};
static double trajT[GCS_MAX_INSTANCES][2][MAXTPOINTS];
static double trajPos[GCS_MAX_INSTANCES][2][MAXTPOINTS][3];
static double trajAcc[GCS_MAX_INSTANCES][2][MAXTPOINTS][3];
static double trajPsi[GCS_MAX_INSTANCES][2][MAXTPOINTS];

static float circle[HORIZONPTS+1][2] = {{0}};
//static struct imgAnalysis_ref *ia = &imgAnalysis;

#define REARTH 20900000.0

#define NSTARS 173

float star_data[NSTARS][4] = {
/*NorthPole	right	down	sqrt(flux)*/
  {-0.26387305f, 0.945941521f, 0.18858858f, 1.958844674f},
  {-0.78061225f, 0.621648487f, 0.064789456f, 1.393156803f},
  {0.328866647f, -0.526721302f, 0.783843989f, 1.018591388f},
  {-0.858810913f, -0.328438451f, 0.393156456f, 1.00461579f},
  {0.626150305f, -0.769672492f, -0.124659742f, 0.986279486f},
  {0.719137702f, 0.682331283f, -0.131396294f, 0.963829024f},
  {-0.13542737f, 0.971495725f, -0.19456486f, 0.946237161f},
  {0.091211947f, 0.90399341f, 0.417703598f, 0.839459987f},
  {-0.836286156f, 0.22628447f, -0.499420468f, 0.809095899f},
  {0.128795597f, 0.991435172f, -0.021633192f, 0.794328235f},
  {-0.862954939f, -0.259482579f, 0.433563796f, 0.758577575f},
  {0.15384817f, -0.875052899f, -0.458926099f, 0.701455298f},
  {-0.97324575f, 0.025412573f, -0.22835698f, 0.685488226f},
  {0.284015345f, 0.894684786f, -0.344781697f, 0.676082975f},
  {-0.431561011f, -0.832358437f, 0.347756418f, 0.660693448f},
  {-0.188524131f, -0.355939002f, 0.915295624f, 0.630957344f},
  {0.469985161f, 0.791305547f, 0.39108756f, 0.591561634f},
  {-0.47485639f, -0.237036296f, -0.84754068f, 0.586138165f},
  {-0.851116673f, -0.108252368f, 0.513694301f, 0.575439937f},
  {0.710390135f, -0.536573479f, -0.45545006f, 0.562341325f},
  {-0.890477679f, -0.052299498f, 0.452011355f, 0.54200089f},
  {-0.858661857f, -0.328598599f, 0.393348161f, 0.54200089f},
  {0.207911691f, 0.458458628f, 0.864053479f, 0.537031796f},
  {-0.45450879f, 0.861979446f, 0.224528828f, 0.501187234f},
  {-0.600420225f, -0.794304449f, 0.092606672f, 0.478630092f},
  {-0.837877536f, -0.073609494f, 0.54087233f, 0.472063041f},
  {0.110312556f, 0.982329487f, -0.151194969f, 0.469894109f},
  {0.478691858f, 0.868452009f, -0.129016328f, 0.467735141f},
  {-0.929240087f, 0.245786962f, 0.275865238f, 0.461317575f},
  {-0.01396218f, 0.994470463f, -0.104084364f, 0.45708819f},
  {-0.890477679f, -0.052299498f, 0.452011355f, 0.450816705f},
  {-0.731353702f, -0.320178835f, -0.602168811f, 0.44874539f},
  {0.764171411f, 0.501446487f, -0.405701213f, 0.438530698f},
  {0.881028384f, 0.115245347f, 0.458810959f, 0.438530698f},
  {-0.72757323f, 0.579554872f, 0.367087652f, 0.436515832f},
  {0.829037573f, -0.130540991f, 0.543742359f, 0.436515832f},
  {-0.432085749f, 0.86219631f, 0.264422822f, 0.42854852f},
  {-0.553633813f, -0.828198283f, -0.087047148f, 0.426579519f},
  {-0.852640164f, 0.42484398f, 0.304158416f, 0.424619564f},
  {0.758703111f, -0.293718114f, 0.581463034f, 0.424619564f},
  {-0.68199836f, -0.727705811f, 0.072955395f, 0.422668614f},
  {0.706489445f, 0.707720278f, -0.002161615f, 0.416869383f},
  {-0.933371779f, -0.341441325f, 0.110611684f, 0.413047502f},
  {0.282620498f, 0.946419663f, 0.156254519f, 0.411149721f},
  {0.591309648f, 0.738995779f, 0.322859317f, 0.40926066f},
  {-0.821646938f, -0.45908186f, -0.337846349f, 0.40926066f},
  {-0.801949453f, 0.449829786f, 0.393103344f, 0.405508535f},
  {-0.127930151f, 0.612306288f, 0.780201824f, 0.401790811f},
  {0.397948631f, 0.482752705f, -0.780119678f, 0.398107171f},
  {-0.052045464f, 0.569940089f, -0.820036319f, 0.398107171f},
  {0.999914328f, 0.007872953f, -0.010457252f, 0.398107171f},
  {-0.27647611f, 0.956352021f, 0.094613806f, 0.398107171f},
  {0.437325056f, -0.777442668f, 0.452028421f, 0.398107171f},
  {-0.433396954f, -0.875282966f, -0.214585205f, 0.394457302f},
  {-0.30874033f, 0.179449898f, -0.934064849f, 0.390840896f},
  {-0.000872665f, 0.996419078f, -0.084547383f, 0.389045145f},
  {0.58212297f, 0.242811725f, -0.775999558f, 0.387257645f},
  {-0.144931859f, 0.988017012f, -0.05307673f, 0.387257645f},
  {-0.582832313f, -0.426390808f, 0.691735046f, 0.387257645f},
  {0.96213875f, -0.184751871f, 0.200389052f, 0.383707245f},
  {0.217575439f, -0.970195778f, 0.106682144f, 0.383707245f},
  {0.485826958f, 0.031266361f, -0.87349561f, 0.380189396f},
  {0.655180441f, 0.552517557f, -0.515230957f, 0.380189396f},
  {-0.708339838f, -0.23417257f, -0.6658963f, 0.380189396f},
  {0.252069358f, 0.046850627f, 0.966574393f, 0.373250158f},
  {-0.85476093f, 0.33879662f, 0.393192833f, 0.363078055f},
  {0.449838665f, -0.719320794f, 0.529360719f, 0.363078055f},
  {0.645901937f, -0.621512384f, -0.443320476f, 0.363078055f},
  {-0.67687597f, 0.502486612f, 0.537909032f, 0.361409863f},
  {0.834046339f, 0.096511839f, -0.543187049f, 0.358096437f},
  {0.005235964f, 0.992479277f, -0.122300735f, 0.358096437f},
  {0.782427042f, -0.622669644f, 0.00950992f, 0.358096437f},
  {-0.615890672f, 0.676363483f, 0.403993958f, 0.354813389f},
  {0.673227635f, 0.380007335f, -0.634317725f, 0.35318317f},
  {0.818818213f, -0.205020462f, 0.536193384f, 0.351560441f},
  {-0.55508641f, -0.793294787f, 0.250124885f, 0.348337315f},
  {0.858363527f, 0.214790014f, -0.465915557f, 0.34673685f},
  {-0.824620158f, -0.238398278f, 0.512998886f, 0.34673685f},
  {-0.667182767f, -0.467006712f, 0.580320503f, 0.34673685f},
  {-0.726774506f, -0.445406285f, 0.52288819f, 0.34673685f},
  {-0.364605936f, -0.806815862f, 0.464877054f, 0.343557948f},
  {0.832921241f, 0.139259374f, 0.535582891f, 0.335737614f},
  {0.455285936f, -0.586461052f, 0.669909062f, 0.335737614f},
  {-0.664795866f, 0.085213318f, -0.742149006f, 0.332659553f},
  {-0.628868159f, -0.775194332f, 0.059988209f, 0.331131121f},
  {0.171069365f, -0.550949833f, -0.81681672f, 0.331131121f},
  {0.470498601f, -0.214592626f, -0.8559095f, 0.331131121f},
  {-0.246717127f, -0.946298976f, 0.20892321f, 0.326587832f},
  {0.806100458f, 0.016265318f, 0.591555146f, 0.325087297f},
  {0.887547502f, -0.298599349f, -0.350853048f, 0.325087297f},
  {-0.480223498f, 0.819023569f, 0.313983736f, 0.323593657f},
  {0.558469219f, -0.621277677f, -0.549660058f, 0.322106879f},
  {0.261908456f, -0.231024707f, -0.937033375f, 0.317687407f},
  {0.872069272f, 0.119635066f, -0.474534124f, 0.316227766f},
  {-0.819152044f, 0.364839479f, 0.442585679f, 0.316227766f},
  {0.070917145f, 0.711759626f, -0.698834024f, 0.311888958f},
  {-0.728171627f, -0.330715491f, 0.600327698f, 0.309029543f},
  {0.351024649f, 0.186681399f, 0.917568391f, 0.307609681f},
  {-0.163899899f, -0.922489353f, 0.349499951f, 0.307609681f},
  {-0.278432383f, 0.953649804f, -0.114137901f, 0.304789499f},
  {-0.283736423f, -0.065637445f, 0.956653212f, 0.303389118f},
  {-0.686664691f, 0.690008441f, 0.228866673f, 0.301995172f},
  {-0.758134336f, -0.023326614f, 0.651681055f, 0.301995172f},
  {-0.471268462f, -0.849499576f, -0.237184544f, 0.301995172f},
  {-0.150110553f, -0.74841771f, 0.646016837f, 0.30060763f},
  {0.604830791f, 0.796352118f, -0.001737373f, 0.299226464f},
  {-0.312334919f, -0.833264341f, 0.456198901f, 0.299226464f},
  {0.354835018f, 0.44790099f, -0.820656331f, 0.296483139f},
  {-0.557986526f, 0.82653242f, -0.074129583f, 0.296483139f},
  {0.339832455f, 0.398210237f, 0.852022599f, 0.296483139f},
  {-0.38483236f, -0.137222648f, 0.912728875f, 0.295120923f},
  {0.112047064f, -0.82405942f, 0.555312099f, 0.295120923f},
  {0.31592504f, -0.45490202f, 0.832619674f, 0.291071712f},
  {-0.680507752f, -0.51426855f, 0.521955034f, 0.291071712f},
  {0.547076321f, 0.805456066f, -0.227921093f, 0.289734359f},
  {-0.75011107f, 0.207920875f, 0.627775671f, 0.289734359f},
  {-0.932848777f, -0.057910431f, 0.355583383f, 0.289734359f},
  {-0.597858349f, -0.795015151f, 0.102549035f, 0.289734359f},
  {0.867909981f, 0.18124215f, -0.462475457f, 0.28840315f},
  {-0.600420225f, 0.754973506f, 0.263648551f, 0.28840315f},
  {-0.472037963f, -0.877949997f, -0.079899709f, 0.28840315f},
  {0.183951351f, -0.879471167f, -0.438967387f, 0.285759054f},
  {-0.040422445f, -0.894591635f, 0.445052617f, 0.2831392f},
  {0.878955875f, -0.435587984f, 0.194164046f, 0.2831392f},
  {-0.578095005f, -0.280415817f, 0.766272233f, 0.281838293f},
  {-0.275357725f, -0.651634929f, 0.706788542f, 0.281838293f},
  {-0.895970287f, 0.146786069f, 0.419155216f, 0.280543364f},
  {-0.009890038f, -0.180081157f, 0.983602035f, 0.280543364f},
  {-0.071207298f, 0.99167426f, -0.107292514f, 0.279254384f},
  {0.366501227f, -0.859904138f, 0.355305115f, 0.279254384f},
  {0.079619008f, -0.994180131f, 0.072571906f, 0.279254384f},
  {-0.654080958f, -0.609819276f, 0.447547261f, 0.277971327f},
  {-0.085416923f, 0.970514815f, -0.225399518f, 0.276694165f},
  {0.791401386f, -0.606171053f, 0.078996843f, 0.276694165f},
  {0.26162771f, 0.055138565f, -0.96359259f, 0.27542287f},
  {-0.402214115f, 0.777483348f, 0.483470215f, 0.27542287f},
  {-0.841353597f, -0.035349416f, 0.539327863f, 0.27542287f},
  {0.523985906f, -0.801750468f, 0.287462967f, 0.274157417f},
  {-0.41575175f, -0.902795406f, -0.110049708f, 0.274157417f},
  {-0.466129306f, -0.825538374f, 0.318134974f, 0.272897778f},
  {0.190523444f, -0.262756013f, 0.945864734f, 0.271643927f},
  {-0.329416007f, 0.935110458f, -0.13058915f, 0.270395836f},
  {0.527944334f, 0.724129366f, -0.443747046f, 0.26915348f},
  {-0.887681481f, -0.393442421f, 0.239216742f, 0.26915348f},
  {-0.813946564f, -0.574178487f, 0.088374521f, 0.26915348f},
  {-0.86949493f, 0.244540546f, -0.429160213f, 0.267916832f},
  {-0.863542275f, -0.216500629f, -0.455436293f, 0.267916832f},
  {0.40806491f, 0.764144471f, -0.499566069f, 0.266685866f},
  {-0.732146748f, -0.122082986f, 0.670117068f, 0.266685866f},
  {0.708545138f, -0.63316427f, -0.311555446f, 0.266685866f},
  {0.64278761f, 0.659366647f, -0.389948348f, 0.264240876f},
  {-0.922874505f, -0.293486732f, 0.249335491f, 0.264240876f},
  {-0.436801788f, -0.776674184f, 0.453851748f, 0.264240876f},
  {-0.358096367f, -0.89095953f, -0.279209793f, 0.264240876f},
  {0.382952162f, 0.919200668f, 0.091748424f, 0.263026799f},
  {0.144356201f, 0.919081006f, 0.366676141f, 0.263026799f},
  {0.620235491f, -0.189435208f, 0.761197896f, 0.263026799f},
  {-0.41336932f, -0.826921986f, 0.381216257f, 0.263026799f},
  {-0.273119837f, -0.527798105f, -0.804260353f, 0.263026799f},
  {-0.077009056f, -0.602110424f, -0.794690281f, 0.261818301f},
  {0.528438335f, 0.777817636f, 0.340224413f, 0.260615355f},
  {0.8036838f, 0.428948686f, -0.412426205f, 0.259417936f},
  {-0.759271307f, 0.635470946f, 0.140298821f, 0.259417936f},
  {0.503019947f, -0.285655261f, -0.815703381f, 0.258226019f},
  {-0.216155612f, 0.841043021f, -0.495906632f, 0.257039578f},
  {-0.267518674f, -0.124518258f, 0.955473162f, 0.257039578f},
  {-0.74469995f, -0.662281759f, 0.082491553f, 0.257039578f},
  {-0.732146748f, -0.122082986f, 0.670117068f, 0.255858589f},
  {0.005817731f, -0.478683756f, -0.877968118f, 0.255858589f},
  {0.424989519f, 0.888717617f, 0.171944486f, 0.253512863f},
  {0.403279128f, 0.506063272f, 0.762407968f, 0.253512863f},
  {-0.49368892f, -0.869378979f, -0.021247165f, 0.252348077f},
  {0.239663262f, -0.931713816f, -0.272893543f, 0.252348077f}
};

//for displaying frontier guidance
#define SCAN_PER_SECTOR   31
#define ANGLE_PER_SCAN    0.3515625
#define HALF_SCOPE        120

static void sceneClose( struct scene_ref *sc, struct gcsInstance_ref *gi ) {

	int i;

	/* the textures get deleted when we close, need to erase handle */

	sc->gtexture  = 0;
	sc->ttexture  = 0;
	sc->rooftexture  = 0;
    sc->walltexture  = 0;
	sc->treelinetexture = 0;
	sc->watertexture = 0;
	sc->etexture  = 0;
	sc->ctexture  = 0;
	sc->btexture  = 0;
	sc->ftexture  = 0;
	sc->tptexture = 0;
	sc->potexture = 0;
	sc->gdtexture = 0;
	sc->gptexture = 0;
	sc->grtexture = 0;
	for( i=0; i<9; i++ ) sc->gmtexture[i] = 0;
	for( i=0; i<SCENE_NUMBEROFOVERLAYS; i++ ) sc->otexture[i] = 0;
	sc->slamtexture = 0;
    sc->ahstexture  = 0;
    sc->ahstexture2 = 0;
    sc->ahstexture3 = 0;
	sc->vtexture    = 0;
	sc->vtextureFrame = 0;
	sc->wvtexture  = 0;
	sc->wv2texture = 0;
	sc->trucktexture = 0;

	for( i=0; i<9; i++ ) scenePIP.gmtexture[i] = 0;

	sc->videoModeDerived = 0;

	gi->cntrlInput->leftVirtualJoystickInUse  = 0;
	gi->cntrlInput->rightVirtualJoystickInUse = 0;

	sc->open = 0;
	glutDestroyWindow( sc->win );

}


static void sceneChecklistClear( struct sceneGlobal_ref *sg ) {

	int i;

	for( i=0; i<SCENE_CHECKLIST_MAXITEMS; i++ ) {
		sprintf( sg->checklist->item[i]->text, "" );
		sprintf( sg->checklist->item[i]->command, "" );
	}

	sg->checklist->activeItem = 0;

}


static void sceneAddMessage( struct sceneGlobal_ref *sg, struct scene_ref *sc, char *text ) {

	char newone[SCENE_MESSAGEMAXLENGTH+1];

	/* store new one */
	if( strlen( text ) < SCENE_MESSAGEMAXLENGTH ) {
		strcpy( newone, text );
	} else {
		memcpy( newone, text, SCENE_MESSAGEMAXLENGTH-3 );
		newone[SCENE_MESSAGEMAXLENGTH-3] = '\0';
		strcat( newone, "..." );
	}

	if( strcmp( newone, sc->messageText0 ) ) { /* new one */

		/* buffer old ones */
		if( sim.time < sc->messageTime0 ) {
			if( sim.time < sc->messageTime1 ) {
				if( sim.time < sc->messageTime2 ) {
					strcpy( sc->messageText3, sc->messageText2 );
					sc->messageTime3 = sc->messageTime2;
				}
				strcpy( sc->messageText2, sc->messageText1 );
				sc->messageTime2 = sc->messageTime1;
			}
			strcpy( sc->messageText1, sc->messageText0 );
			sc->messageTime1 = sc->messageTime0;
		}

		strcpy( sc->messageText0, newone );
	}

	/* set up timer */
	sc->messageTime0 = sim.time + sg->messageHangTime + sg->messageFadeTime;

}

void sceneMessageAll( char *text ) {

	struct sceneGlobal_ref *sg = &sceneGlobal;

	if( scene0.open ) sceneAddMessage( sg, &scene0, text );
	if( scene1.open ) sceneAddMessage( sg, &scene1, text );
	if( scene2.open ) sceneAddMessage( sg, &scene2, text );

}

static void createSceneCaptureDirectory( struct sceneCapture_ref *cap ) {

    char buf1[200];
    char buf2[200];
    time_t current_time;
    struct tm *local_time;
	#if defined( SUL_OS_WIN32 )
 	int i;
	#endif

    current_time = time ( NULL );
    local_time = localtime( &current_time );

    sprintf( buf2, "images_%02d%02d%02d_%02d%02d%02d",
             (1900 + local_time->tm_year)%100,
             local_time->tm_mon + 1,
             local_time->tm_mday,
             local_time->tm_hour,
             local_time->tm_min,
             local_time->tm_sec);
    sprintf( buf1, "mkdir %s/%s", cap->dir, buf2 );
    #if defined( SUL_OS_WIN32 )
    for( i=0;i<(int)strlen(buf1);i++ ) {
        if ( buf1[i] == 0x2f ) {
            buf1[i] = 0x5c;
        }
    }
    #endif
    system( buf1 );
    sprintf( cap->fullPath, "%s/%s/", cap->dir, buf2 );

}


static void captureImage( struct sceneGlobal_ref *sg,
                          struct scene_ref        *sc,
						  struct sceneCapture_ref *cap,
						  int mode ) {

	if( mode == cap->mode || ( cap->singleShot && mode == cap->singleShotMode ) ) {

		if( ( sc == &scene0 && cap->window == 0 ) ||
			( sc == &scene1 && cap->window == 1 ) ||
			( sc == &scene2 && cap->window == 2 ) ) {

			FILE *filep;

			if( cap->init ) {
				char buf[200];
				createSceneCaptureDirectory( cap );
				cap->imageNo = 0;
				sprintf( buf, "%s%s", cap->fullPath, cap->dataname );
				if( !(filep = fopen( buf, "w" )) ) {
					logError( "scene: Error opening image capture data file" );
					sceneAddMessage( sg, sc, "Error opening image capture file" );
				} else {
					fprintf( filep, "image# time phi theta psi x y z\n" );
					fclose( filep );
				}
				cap->init = 0;
			}

			if( sim.mode == SIM_MODE_INIT ) {

				cap->lastUpdate = -cap->updateDt;

			} else {

				if( sim.time >= cap->lastUpdate + cap->updateDt || cap->singleShot ) {

					char buf[200];
					char fileName[100];

					if( cap->singleShot == 0 ) {
						if( cap->lastUpdate < sim.time - 5 ) {
							cap->lastUpdate = sim.time; /* reset time if just turned on */
						} else {
							cap->lastUpdate += cap->updateDt;  // sloppy
						}
					}

					sprintf( fileName, cap->filename, cap->imageNo );
					sprintf( buf, "%s%s", cap->fullPath, fileName );
					if( !(filep = fopen(buf,"wb")) ) {
						logError( "scene: Error opening image capture file" );
						sceneAddMessage( sg, sc, "Error opening image capture file" );
						cap->singleShot = 0;
						cap->mode = SCENECAPTURE_OFF;
					} else {

						unsigned char *pixels = ( unsigned char* ) malloc( sc->winw * sc->winh * 3 );;
						int i, index;
						char filler = 0;
						int fileSize = 0;

						if( strstr( cap->filename, ".bmp" ) != NULL ) {

							glFinish();
                            glReadPixels( 0, 0, sc->winw-1, sc->winh-1, GL_BGR, GL_UNSIGNED_BYTE, (GLvoid *)pixels );

							/* set some basic file information*/
							scbmpinfoheader.bfType = 19778;
							scbmpinfoheader.bfReserved = 0;
							scbmpinfoheader.bfOffBits = sizeof( struct scbmpinfoheader_ref );
							scbmpinfoheader.biSizeImage = sc->winh*sc->winw*3;
							scbmpinfoheader.bfSize = scbmpinfoheader.bfOffBits + (unsigned long)(scbmpinfoheader.biSizeImage);
							scbmpinfoheader.biHeight = sc->winh;
							scbmpinfoheader.biWidth  = sc->winw;
							scbmpinfoheader.biSize   = 40;
							scbmpinfoheader.biPlanes = 1;
							scbmpinfoheader.biBitCount = 8*3;
							scbmpinfoheader.biCompression = 0;
							scbmpinfoheader.biXPelsPerMeter = 0;
							scbmpinfoheader.biYPelsPerMeter = 0;
							scbmpinfoheader.biClrUsed = 0;
							scbmpinfoheader.biClrImportant = 0;
							fileSize = scbmpinfoheader.bfSize;

							fwrite( &scbmpinfoheader, sizeof (struct scbmpinfoheader_ref), 1, filep );

							/* write out pixel data */
							for( i=0; i<sc->winh;i++ ) {
								index = i*sc->winw*3;
								fwrite( &pixels[index], sizeof(unsigned char), sc->winw*scbmpinfoheader.biBitCount/8, filep );
								// pack till it is 4 bytes aligned.
								// (don't think this should be here)
								//for (k=0; k<((sc->winw*3)%4);k++)
								//	fwrite( &filler, sizeof(unsigned char), 1, filep );
							}

						} else if( strstr( cap->filename, ".jpg" ) != NULL ) {
#if ALLOWJPEG
							struct jpeg_compress_struct cinfo;
							struct jpeg_error_mgr jerr;
							cjpeg_source_ptr src_mgr;
							JDIMENSION num_scanlines;

							glFinish();
                            glReadPixels( 0, 0, sc->winw-1, sc->winh-1, GL_BGR, GL_UNSIGNED_BYTE, (GLvoid *)pixels );

							bih0 = &bih;
							bfh0 = &bfh;
							pixels0 = pixels;

							/* Write output file header */
							bfh0->bfType = 19778;
							bfh0->bfReserved = 0;
							//bih0->biBitCount = 8*3;
							//scbmpinfoheader.biCompression = 0;
							//scbmpinfoheader.biXPelsPerMeter = 0;
							//scbmpinfoheader.biYPelsPerMeter = 0;
							//scbmpinfoheader.biClrUsed = 0;
							//scbmpinfoheader.biClrImportant = 0;
							bih0->biHeight=sc->winh;
							bih0->biWidth=sc->winw;
							bih0->biSizeImage=sc->winw*sc->winh*3;
							bih0->biSize = 40;
							bih0->biPlanes=1;
							bih0->biBitCount=24;

							bfh0->bfOffBits=sizeof(struct bmpfileheader_ref) + sizeof(struct bmpinfoheader_ref);
							bfh0->bfSize = bfh0->bfOffBits+(unsigned long)(bih0->biSizeImage);
							fileSize = bfh0->bfSize;

							/* Initialize the JPEG compression object with default error handling. */

							cinfo.err = jpeg_std_error(&jerr);

							jpeg_create_compress(&cinfo);
							/* Add some application-specific error messages (from cderror.h) */
							jerr.addon_message_table = cdjpeg_message_table;
							jerr.first_addon_message = JMSG_FIRSTADDONCODE;
							jerr.last_addon_message = JMSG_LASTADDONCODE;

							/* Initialize JPEG parameters.
							* Much of this may be overridden later.
							* In particular, we don't yet know the input file's color space,
							* but we need to provide some value for jpeg_set_defaults() to work.
							*/
							cinfo.in_color_space = JCS_RGB; /* arbitrary guess */
							jpeg_set_defaults(&cinfo);

							src_mgr = jinit_read_mem(&cinfo );
							/* Read the input file header to obtain file size & colorspace. */
							(*src_mgr->start_input) (&cinfo, src_mgr);

							/* Now that we know input colorspace, fix colorspace-dependent defaults */
							jpeg_default_colorspace(&cinfo);

							saveSwitches(&cinfo);

							/* Specify data destination for compression */
							jpeg_stdio_dest(&cinfo, filep);

							/* Start compressor */
							jpeg_start_compress(&cinfo, TRUE);

							/* Process data */
							while (cinfo.next_scanline < cinfo.image_height) {
								num_scanlines = (*src_mgr->get_pixel_rows) (&cinfo, src_mgr);
								(void) jpeg_write_scanlines(&cinfo, src_mgr->buffer, num_scanlines);
							}

							/* Finish compression and release memory */
							(*src_mgr->finish_input) (&cinfo, src_mgr);
							jpeg_finish_compress(&cinfo);
							jpeg_destroy_compress(&cinfo);
#endif
						} else if( strstr( cap->filename, ".png" ) != NULL ) {
#if defined(HAVE_GCVLIBPNG)
							glFinish();
                            glReadPixels( 0, 0, sc->winw-1, sc->winh-1, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid *)pixels );

                            if( -1 == sceneSavePng(sc, filep, pixels) ){
                                logError( "scene: Error saving as png" );
                            } else {
                                //logInfo( "scene: Sucess saving as png" );
                            }
#endif
                        }
						fclose( filep );
						free( pixels );

						/*if( cap->sendToOtherGCS ) {
							gcsSendFile( fileName, fileSize );
						}*/

						if( cap->singleShot == 0 ) {
							struct gcs_ref *g = &gcs;
							struct gcsInstance_ref *gi = gcsActiveInstance( g ); /* maybe save all instances? */
							/* save state data */
							char buf[200];
							sprintf( buf, "%s%s", cap->fullPath, cap->dataname );
							if( filep = fopen( buf, "a" ) ) {
								if( sc->showTruth ) {
									fprintf( filep, "%ld %.3f %.2f %.2f %.2f %.2f %.2f %.2f\n",
										cap->imageNo,
										sim.time,
										vehicleOutputs.phi, vehicleOutputs.theta, vehicleOutputs.psi,
										state.p_b_e_L[0], state.p_b_e_L[1], state.p_b_e_L[2] );
								} else {
									fprintf( filep, "%ld %.3f %.2f %.2f %.2f %.2f %.2f %.2f\n",
										cap->imageNo,
										navout.time,
										gi->outputs->phi, gi->outputs->theta, gi->outputs->psi,
										navout.p_b_e_L[0], navout.p_b_e_L[1], navout.p_b_e_L[2] );
								}
								fclose( filep );
							}
						}

						cap->singleShot = 0;
					}

					cap->imageNo++;
				}

			}
		}
	}

}


static void getWindowInfo( struct winInfo_ref *win ) {

    glGetDoublev(  GL_MODELVIEW_MATRIX,  &(win->mvm[0])      );
    glGetDoublev(  GL_PROJECTION_MATRIX, &(win->mpm[0])      );
    glGetIntegerv( GL_VIEWPORT,          &(win->viewport[0]) );

}


static void drawThreats( struct sceneGlobal_ref *sg,
				         struct scene_ref *sc,
				         struct vehicleOutputs_ref *o ) {

	struct scThreats_ref *th = &scThreats;

    float color[4]  = { 1.0f, 0.0f, 0.0f, 1.0f };

	int i;


	glColor4fv( color );

	glPushMatrix();

	glTranslatef( (float)(th->startx + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
		(float)(th->starty + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
		(float)(-o->datumAlt + sc->eyeAlt) );


	for( i=0; i< th->number; i++ ) {

		glPushMatrix();
		glTranslatef( th->xy[i][0] ,
			th->xy[i][1] ,0);

		glRotatef(180.0,1,0,0);
		glutSolidCone((GLdouble)th->rad[i], (GLdouble)th->height,(GLint)100, (GLint)100);
		glPopMatrix();

	}

	glPopMatrix();

}

struct scene_ref *whichScene( void ) {

	/* get pointer to structure of window correspoding to handle */

	int win = glutGetWindow();
	if( win == scene0.win ) return &scene0;
	if( win == scene1.win ) return &scene1;
	return &scene2;

}

struct gcsScene_ref *whichGcsScene( struct scene_ref *sc, struct gcsInstance_ref *gi ) {

	if(      sc == &scene0 ) return gi->set->scene0;
	else if( sc == &scene1 ) return gi->set->scene1;
	else if( sc == &scene2 ) return gi->set->scene2;
	else                     return gi->set->scenePIP;

}


static struct view_ref *whichView( struct gcsSet_ref *set, int viewMode ) {

	/* get pointer to structure of view corresponding to actual */

	switch( viewMode ) {
	default:
	case VIEW_COCKPIT:
		return set->cockpit;
	case VIEW_NAV:
		return set->nav;
	case VIEW_CHASE:
		return set->chase;
	case VIEW_GROUND:
		return set->ground;
	case VIEW_HOVER:
		return set->hover;
	case VIEW_CAMERA:
		return set->camera;
	case VIEW_CAMERA2:
		return set->camera2;
	case VIEW_CAMERA3:
		return set->camera3;
	case VIEW_CAMERA4:
		return set->camera4;
	}
	return set->cockpit;

}


struct sceneFontChar_ref {
  const int width;
  const int height;
  const float xorig;
  const float yorig;
  const float advance;
  unsigned char *bitmap;
} ;


static void sceneInitCharacter( unsigned char *in, struct sceneFontChar_ref *ch ) {

	int i, j;

	/* this only works up to a width of 16 */
	if( ch->width <= 8 ) {
		for( i=0; i<ch->height; i++ ) {
			ch->bitmap[ch->height-1-i] = 0;
			for( j=0; j<ch->width; j++ ) {
				if( in[i*ch->width+j] != ' ' ) {
					ch->bitmap[ch->height-1-i] += 1<<(7-j);
				}
			}
		}
	} else {
		for( i=0; i<ch->height; i++ ) {
			ch->bitmap[(ch->height-1-i)*2] = 0;
			ch->bitmap[(ch->height-1-i)*2+1] = 0;
			for( j=0; j<8; j++ ) {
				if( in[i*ch->width+j] != ' ' ) {
					ch->bitmap[(ch->height-1-i)*2] += 1<<(7-j);
				}
			}
			for( j=8; j<ch->width; j++ ) {
				if( in[i*ch->width+j] != ' ' ) {
					ch->bitmap[(ch->height-1-i)*2+1] += 1<<(15-j);
				}
			}
		}
	}

}


#define SCENEFONT_LEFTARROW 129
#define SCENEFONT_RIGHTARROW 130
#define SCENEFONT_CW 131
#define SCENEFONT_CCW 132
#define SCENEFONT_DEGREES 133
#define SCENEFONT_BATT 134
#define SCENEFONT_PIP 135
#define SCENEFONT_GRID 136
#define SCENEFONT_TRIANGLE 137
#define SCENEFONT_TEMP 138
#define SCENEFONT_UNCHECKED 139
#define SCENEFONT_CHECKED 140
#define SCENEFONT_INFINITY 141
#define SCENEFONT_UPARROW 142
#define SCENEFONT_DOWNARROW 143

unsigned char bitmapla12[7];
static struct sceneFontChar_ref sceneFontLeftArrow12 = {8, 7, -1, 0, 9, bitmapla12};

unsigned char bitmapra12[7];
static struct sceneFontChar_ref sceneFontRightArrow12 = {8, 7, -1, 0, 9, bitmapra12};

unsigned char bitmapua12[7];
static struct sceneFontChar_ref sceneFontUpArrow12 = { 8, 7, -1, 0, 9, bitmapua12 };

unsigned char bitmapda12[7];
static struct sceneFontChar_ref sceneFontDownArrow12 = { 8, 7, -1, 0, 9, bitmapda12 };

unsigned char bitmapcw12[9*2];
static struct sceneFontChar_ref sceneFontCW12 = {9, 9, -1, 0, 12, bitmapcw12};

unsigned char bitmapccw12[9*2];
static struct sceneFontChar_ref sceneFontCCW12 = {9, 9, -1, 0, 12, bitmapccw12};

unsigned char bitmapla18[10*2];
static struct sceneFontChar_ref sceneFontLeftArrow18 = {10, 10, -1, 0, 12, bitmapla18};

unsigned char bitmapra18[10*2];
static struct sceneFontChar_ref sceneFontRightArrow18 = {10, 10, -1, 0, 12, bitmapra18};

unsigned char bitmapua18[10 * 2];
static struct sceneFontChar_ref sceneFontUpArrow18 = { 10, 10, -1, 0, 12, bitmapua18 };

unsigned char bitmapda18[10 * 2];
static struct sceneFontChar_ref sceneFontDownArrow18 = { 10, 10, -1, 0, 12, bitmapda18 };

unsigned char bitmapcw18[14*2];
static struct sceneFontChar_ref sceneFontCW18 = {14, 14, -1, 0, 18, bitmapcw18};

unsigned char bitmapccw18[14*2];
static struct sceneFontChar_ref sceneFontCCW18 = {14, 14, -1, 0, 18, bitmapccw18};

unsigned char bitmapdeg12[5];
static struct sceneFontChar_ref sceneFontDegrees12 = {5,5,-1,-5,7, bitmapdeg12};

unsigned char bitmapdeg18[7*2];
static struct sceneFontChar_ref sceneFontDegrees18 = {7,7,-1,-8,11, bitmapdeg18};

unsigned char bitmapbatt12[9];
static struct sceneFontChar_ref sceneFontBatt12 = {5,9,-1,0,7, bitmapbatt12};

unsigned char bitmapbatt18[14*2];
static struct sceneFontChar_ref sceneFontBatt18 = {9,14,-1,1,11, bitmapbatt18};

unsigned char bitmappip12[7*2];
static struct sceneFontChar_ref sceneFontPIP12 = {9,7,-1,0,9, bitmappip12};

unsigned char bitmappip18[11*2];
static struct sceneFontChar_ref sceneFontPIP18 = {14,11,-1,0,15, bitmappip18};

unsigned char bitmapgrid12[9*2];
static struct sceneFontChar_ref sceneFontGrid12 = {9,9,-1,0,9, bitmapgrid12};

unsigned char bitmapgrid18[14*2];
static struct sceneFontChar_ref sceneFontGrid18 = {14,14,-1,0,15, bitmapgrid18};

unsigned char bitmaptriangle12[9*2];
static struct sceneFontChar_ref sceneFontTriangle12 = {9,9,-1,0,9, bitmaptriangle12};

unsigned char bitmaptriangle18[14*2];
static struct sceneFontChar_ref sceneFontTriangle18 = {14,14,-1,0,15, bitmaptriangle18};

unsigned char bitmaptemp12[9];
static struct sceneFontChar_ref sceneFontTemp12 = {5,9,-1,0,7, bitmaptemp12};

unsigned char bitmaptemp18[14*2];
static struct sceneFontChar_ref sceneFontTemp18 = {9,14,-1,1,11, bitmaptemp18};

unsigned char bitmapunchecked12[9*2];
static struct sceneFontChar_ref sceneFontUnchecked12 = {9,9,-1,0,9, bitmapunchecked12};

unsigned char bitmapunchecked18[14*2];
static struct sceneFontChar_ref sceneFontUnchecked18 = {14,14,-1,0,15, bitmapunchecked18};

unsigned char bitmapchecked12[9*2];
static struct sceneFontChar_ref sceneFontChecked12 = {9,9,-1,0,9, bitmapchecked12};

unsigned char bitmapchecked18[14*2];
static struct sceneFontChar_ref sceneFontChecked18 = {14,14,-1,0,15, bitmapchecked18};

unsigned char bitmapinfinity12[9*2];
static struct sceneFontChar_ref sceneFontInfinity12 = {9,5,-1,-1,10, bitmapinfinity12};

unsigned char bitmapinfinity18[14*2];
static struct sceneFontChar_ref sceneFontInfinity18 = {15,8,-1,-1,15, bitmapinfinity18};

struct sceneFont_ref {
	struct sceneFontChar_ref *leftArrow;
	struct sceneFontChar_ref *rightArrow;
	struct sceneFontChar_ref *upArrow;
	struct sceneFontChar_ref *downArrow;
	struct sceneFontChar_ref *cw;
	struct sceneFontChar_ref *ccw;
	struct sceneFontChar_ref *degrees;
	struct sceneFontChar_ref *batt;
	struct sceneFontChar_ref *pip;
	struct sceneFontChar_ref *grid;
	struct sceneFontChar_ref *triangle;
	struct sceneFontChar_ref *temp;
	struct sceneFontChar_ref *unchecked;
	struct sceneFontChar_ref *checked;
	struct sceneFontChar_ref *infinity;
};
static struct sceneFont_ref sceneFont12 = {
	&sceneFontLeftArrow12,
	&sceneFontRightArrow12,
	&sceneFontUpArrow12,
	&sceneFontDownArrow12,
	&sceneFontCW12,
	&sceneFontCCW12,
	&sceneFontDegrees12,
	&sceneFontBatt12,
	&sceneFontPIP12,
	&sceneFontGrid12,
	&sceneFontTriangle12,
	&sceneFontTemp12,
	&sceneFontUnchecked12,
	&sceneFontChecked12,
	&sceneFontInfinity12,
};
static struct sceneFont_ref sceneFont18 = {
	&sceneFontLeftArrow18,
	&sceneFontRightArrow18,
	&sceneFontUpArrow18,
	&sceneFontDownArrow18,
	&sceneFontCW18,
	&sceneFontCCW18,
	&sceneFontDegrees18,
	&sceneFontBatt18,
	&sceneFontPIP18,
	&sceneFontGrid18,
	&sceneFontTriangle18,
	&sceneFontTemp18,
	&sceneFontUnchecked18,
	&sceneFontChecked18,
	&sceneFontInfinity18,
};

static void sceneInitFonts( void ) {

	unsigned char leftArrow12[7][8] = {
		"   x    ",
		"  xx    ",
		" xxx    ",
		"xxxxxxxx",
		" xxx    ",
		"  xx    ",
		"   x    ",
	};
	unsigned char rightArrow12[7][8] = {
		"    x   ",
		"    xx  ",
		"    xxx ",
		"xxxxxxxx",
		"    xxx ",
		"    xx  ",
		"    x   ",
	};
	unsigned char upArrow12[8][7] = {
		"   x   ",
		"  xxx  ",
		" xxxxx ",
		"xxxxxxx",
		"   x   ",
		"   x   ",
		"   x   ",
		"   x   ",
	};
	unsigned char downArrow12[8][7] = {
		"   x   ",
		"   x   ",
		"   x   ",
		"   x   ",
		"xxxxxxx",
		" xxxxx ",
		"  xxx  ",
		"   x   ",
	};
	unsigned char cw12[9][9] = {
		"   xxx   ",
		"  x   x  ",
		" x     x ",
		"x       x",
		"x       x",
		"x       x",
		"x     x x",
		" x    xx ",
		"      xxx",
	};
	unsigned char ccw12[9][9] = {
		"   xxx   ",
		"  x   x  ",
		" x     x ",
		"x       x",
		"x       x",
		"x       x",
		"x x     x",
		" xx    x ",
		"xxx      ",
	};
	unsigned char degrees12[5][5] = {
		" xxx ",
		"x   x",
		"x   x",
		"x   x",
		" xxx ",
	};
	unsigned char batt12[9][5] = {
		" xxx ",
		"xxxxx",
		"xx xx",
		"x   x",
		"xx xx",
		"xxxxx",
		"xxxxx",
		"x   x",
		"xxxxx",
	};
	unsigned char pip12[7][9] = {
		"xxxxxxxxx",
		"x       x",
		"x       x",
		"x    xx x",
		"x    xx x",
		"x       x",
		"xxxxxxxxx",
	};
	unsigned char grid12[9][9] = {
		"xxxxxxxxx",
		"x   x   x",
		"x   x   x",
		"x   x   x",
		"xxxxxxxxx",
		"x   x   x",
		"x   x   x",
		"x   x   x",
		"xxxxxxxxx",
	};
	unsigned char triangle12[9][9] = {
		"xx       ",
		"xxxx     ",
		"xxxxxx   ",
		"xxxxxxxx ",
		"xxxxxxxxx",
		"xxxxxxxx ",
		"xxxxxx   ",
		"xxxx     ",
		"xx       ",
	};
	unsigned char temp12[9][5] = {
		"  x  ",
		" x xx",
		" x x ",
		" x xx",
		" xxx ",
		" xxx ",
		"xxxxx",
		"xxxxx",
		" xxx ",
	};
	unsigned char unchecked12[9][9] = {
		"xxxxxxxxx",
		"x       x",
		"x       x",
		"x       x",
		"x       x",
		"x       x",
		"x       x",
		"x       x",
		"xxxxxxxxx",
	};
	unsigned char checked12[9][9] = {
		"xxxxxxxx ",
		"x       x",
		"x      x ",
		"x     x x",
		"x    x  x",
		"x x x   x",
		"x  x    x",
		"x       x",
		"xxxxxxxxx",
	};
	unsigned char infinity12[5][9] = {
		" xx   xx ",
		"x  x x  x",
		"x   x   x",
		"x  x x  x",
		" xx   xx ",
	};

	unsigned char leftArrow18[10][10] = {
		"    x     ",
		"   xx     ",
		"  xxx     ",
		" xxxx     ",
		"xxxxxxxxxx",
		"xxxxxxxxxx",
		" xxxx     ",
		"  xxx     ",
		"   xx     ",
		"    x     ",
	};
	unsigned char rightArrow18[10][10] = {
		"     x    ",
		"     xx   ",
		"     xxx  ",
		"     xxxx ",
		"xxxxxxxxxx",
		"xxxxxxxxxx",
		"     xxxx ",
		"     xxx  ",
		"     xx   ",
		"     x    ",
	};
	unsigned char upArrow18[10][10] = {
		"    xx    ",
		"   xxxx   ",
		"  xxxxxx  ",
		" xxxxxxxx ",
		"xxxxxxxxxx",
		"    xx    ",
		"    xx    ",
		"    xx    ",
		"    xx    ",
		"    xx    ",
	};
	unsigned char downArrow18[10][10] = {
		"    xx    ",
		"    xx    ",
		"    xx    ",
		"    xx    ",
		"    xx    ",
		"xxxxxxxxxx",	
		" xxxxxxxx ",
		"  xxxxxx  ",
		"   xxxx   ",
		"    xx    ",		
	};
	unsigned char cw18[14][14] = {
		"     xxxx     ",
		"   xxxxxxxx   ",
		" xxx      xxx ",
		" xx        xx ",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx      x   xx",
		" xx     xx xx ",
		" xxx    xxxxx ",
		"  xx    xxxxx ",
		"        xxxxxx",
	};
	unsigned char ccw18[14][14] = {
		"     xxxx     ",
		"   xxxxxxxx   ",
		" xxx      xxx ",
		" xx        xx ",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx   x      xx",
		" xx xx     xx ",
		" xxxxx    xxx ",
		" xxxxx    xx  ",
		"xxxxxx        ",
	};
	unsigned char degrees18[7][7] = {
		"  xxx  ",
		" xxxxx ",
		"xx   xx",
		"xx   xx",
		"xx   xx",
		" xxxxx ",
		"  xxx  ",
	};
	unsigned char batt18[14][9] = {
		"   xxx   ",
		"xxxxxxxxx",
		"xxxx xxxx",
		"xxxx xxxx",
		"xx     xx",
		"xxxx xxxx",
		"xxxx xxxx",
		"xxxxxxxxx",
		"xxxxxxxxx",
		"xxxxxxxxx",
		"xxxxxxxxx",
		"xx     xx",
		"xxxxxxxxx",
		"xxxxxxxxx",
	};
	unsigned char pip18[11][14] = {
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx     xxxx xx",
		"xx     xxxx xx",
		"xx     xxxx xx",
		"xx          xx",
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
	};
	unsigned char grid18[14][14] = {
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
		"xx    xx    xx",
		"xx    xx    xx",
		"xx    xx    xx",
		"xx    xx    xx",
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
		"xx    xx    xx",
		"xx    xx    xx",
		"xx    xx    xx",
		"xx    xx    xx",
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
	};
	unsigned char triangle18[14][14] = {
		"xx            ",
		"xxxx          ",
		"xxxxxx        ",
		"xxxxxxxx      ",
		"xxxxxxxxxx    ",
		"xxxxxxxxxxxx  ",
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxx  ",
		"xxxxxxxxxx    ",
		"xxxxxxxx      ",
		"xxxxxx        ",
		"xxxx          ",
		"xx            ",
	};
	unsigned char temp18[14][9] = {
		"  xxx    ",
		" xxxxx xx",
		" xx xx xx",
		" xx xx   ",
		" xx xx xx",
		" xx xx xx",
		" xx xx   ",
		" xxxxx xx",
		" xxxxx xx",
		" xxxxx   ",
		"xxxxxxx  ",
		"xxxxxxx  ",
		" xxxxx   ",
		"  xxx    ",
	};
	unsigned char unchecked18[14][14] = {
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xx          xx",
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
	};
	unsigned char   checked18[14][14] = {
		"xxxxxxxxxxxx  ",
		"xxxxxxxxxxx  x",
		"xx          xx",
		"xx         xxx",
		"xx        xxx ",
		"xx x     xxx  ",
		"xx xx   xxx  x",
		"xx xxx xxx  xx",
		"xx  xxxxx   xx",
		"xx   xxx    xx",
		"xx    x     xx",
		"xx          xx",
		"xxxxxxxxxxxxxx",
		"xxxxxxxxxxxxxx",
	};
	unsigned char   infinity18[8][15] = {
		"  xxxx   xxxx  ",
		" xxxxxx xxxxxx ",
		"xxx  xxxxx  xxx",
		"xx    xxx    xx",
		"xx    xxx    xx",
		"xxx  xxxxx  xxx",
		" xxxxxx xxxxxx ",
		"  xxxx   xxxx  ",
	};

	sceneInitCharacter( (unsigned char *)leftArrow12,  &sceneFontLeftArrow12 );
	sceneInitCharacter( (unsigned char *)rightArrow12, &sceneFontRightArrow12 );
	sceneInitCharacter( (unsigned char *)leftArrow18,  &sceneFontLeftArrow18 );
	sceneInitCharacter( (unsigned char *)rightArrow18, &sceneFontRightArrow18 );
	sceneInitCharacter( (unsigned char *)upArrow12,		 &sceneFontUpArrow12);
	sceneInitCharacter( (unsigned char *)downArrow12,	 &sceneFontDownArrow12);
	sceneInitCharacter( (unsigned char *)upArrow18,		 &sceneFontUpArrow18);
	sceneInitCharacter( (unsigned char *)downArrow18,	 &sceneFontDownArrow18);
	sceneInitCharacter( (unsigned char *)cw12,         &sceneFontCW12 );
	sceneInitCharacter( (unsigned char *)ccw12,        &sceneFontCCW12 );
	sceneInitCharacter( (unsigned char *)cw18,         &sceneFontCW18 );
	sceneInitCharacter( (unsigned char *)ccw18,        &sceneFontCCW18 );
	sceneInitCharacter( (unsigned char *)degrees12,    &sceneFontDegrees12 );
	sceneInitCharacter( (unsigned char *)degrees18,    &sceneFontDegrees18 );
	sceneInitCharacter( (unsigned char *)batt12,       &sceneFontBatt12 );
	sceneInitCharacter( (unsigned char *)batt18,       &sceneFontBatt18 );
	sceneInitCharacter( (unsigned char *)pip12,        &sceneFontPIP12 );
	sceneInitCharacter( (unsigned char *)pip18,        &sceneFontPIP18 );
	sceneInitCharacter( (unsigned char *)grid12,       &sceneFontGrid12 );
	sceneInitCharacter( (unsigned char *)grid18,       &sceneFontGrid18 );
	sceneInitCharacter( (unsigned char *)triangle12,   &sceneFontTriangle12 );
	sceneInitCharacter( (unsigned char *)triangle18,   &sceneFontTriangle18 );
	sceneInitCharacter( (unsigned char *)temp12,       &sceneFontTemp12 );
	sceneInitCharacter( (unsigned char *)temp18,       &sceneFontTemp18 );
	sceneInitCharacter( (unsigned char *)unchecked12,  &sceneFontUnchecked12 );
	sceneInitCharacter( (unsigned char *)unchecked18,  &sceneFontUnchecked18 );
	sceneInitCharacter( (unsigned char *)checked12,    &sceneFontChecked12 );
	sceneInitCharacter( (unsigned char *)checked18,    &sceneFontChecked18 );
	sceneInitCharacter( (unsigned char *)infinity12,   &sceneFontInfinity12 );
	sceneInitCharacter( (unsigned char *)infinity18,   &sceneFontInfinity18 );

}


static void drawText( unsigned char *message ) {

	/* draw each character in a stroke font */

	while( *message ) {
		glutStrokeCharacter( GLUT_STROKE_MONO_ROMAN, *message );
		message++;
	}

}


static void showMessage( GLfloat x, GLfloat y, unsigned char *message, float scale ) {

	/* generic string drawing routine, 2-D */

	glPushMatrix();
	glTranslatef( x, y, 0.0 );
	glScalef( 0.0001f, 0.0001f, 0.0001f );
	glScalef( scale, scale, scale );
	drawText( message );
	glPopMatrix();

}


static int getBitmapLength( char font, unsigned char *message ) {

	struct sceneFont_ref *sf;
	int length = 0;
	void *fp;

	switch( font ) {
		default:
		case 8:
			fp = GLUT_BITMAP_HELVETICA_18;
			sf = &sceneFont18;
			break;
		case 7:
			fp = GLUT_BITMAP_HELVETICA_12;
			sf = &sceneFont12;
			break;
		case 6:
			fp = GLUT_BITMAP_HELVETICA_10;
			sf = &sceneFont12;
			break;
		case 5:
			fp = GLUT_BITMAP_TIMES_ROMAN_24;
			sf = &sceneFont18;
			break;
		case 4:
			fp = GLUT_BITMAP_TIMES_ROMAN_10;
			sf = &sceneFont12;
			break;
		case 3:
			fp = GLUT_BITMAP_8_BY_13;
			sf = &sceneFont12;
			break;
		case 2:
			fp = GLUT_BITMAP_9_BY_15;
			sf = &sceneFont12;
			break;
	}

	while( *message ) {
		switch( *message ) {
		default:
			length += glutBitmapWidth( fp, *message );
			break;
		case SCENEFONT_LEFTARROW:
			length += (int)sf->leftArrow->advance;
			break;
		case SCENEFONT_RIGHTARROW:
			length += (int)sf->rightArrow->advance;
			break;
		case SCENEFONT_UPARROW:
			length += (int)sf->upArrow->advance;
			break;
		case SCENEFONT_DOWNARROW:
			length += (int)sf->downArrow->advance;
			break;
		case SCENEFONT_CW:
			length += (int)sf->cw->advance;
			break;
		case SCENEFONT_CCW:
			length += (int)sf->ccw->advance;
			break;
		case SCENEFONT_DEGREES:
			length += (int)sf->degrees->advance;
			break;
		case SCENEFONT_BATT:
			length += (int)sf->batt->advance;
			break;
		case SCENEFONT_PIP:
			length += (int)sf->pip->advance;
			break;
		case SCENEFONT_GRID:
			length += (int)sf->grid->advance;
			break;
		case SCENEFONT_TRIANGLE:
			length += (int)sf->triangle->advance;
			break;
		case SCENEFONT_TEMP:
			length += (int)sf->temp->advance;
			break;
		case SCENEFONT_UNCHECKED:
			length += (int)sf->temp->advance;
			break;
		case SCENEFONT_CHECKED:
			length += (int)sf->temp->advance;
			break;
		case SCENEFONT_INFINITY:
			length += (int)sf->temp->advance;
			break;
		}
		message++;
	}

	return length;

}


static void sceneFontBitmapCharcter( struct sceneFontChar_ref *ch ) {

  GLint swapbytes, lsbfirst, rowlength;
  GLint skiprows, skippixels, alignment;

    glGetIntegerv(GL_UNPACK_SWAP_BYTES, &swapbytes);
    glGetIntegerv(GL_UNPACK_LSB_FIRST, &lsbfirst);
    glGetIntegerv(GL_UNPACK_ROW_LENGTH, &rowlength);
    glGetIntegerv(GL_UNPACK_SKIP_ROWS, &skiprows);
    glGetIntegerv(GL_UNPACK_SKIP_PIXELS, &skippixels);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &alignment);
    /* Little endian machines (DEC Alpha for example) could
       benefit from setting GL_UNPACK_LSB_FIRST to GL_TRUE
       instead of GL_FALSE, but this would require changing the
       generated bitmaps too. */
    glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
    glPixelStorei(GL_UNPACK_LSB_FIRST, GL_FALSE);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glBitmap(ch->width, ch->height, ch->xorig, ch->yorig,
      ch->advance, 0, ch->bitmap);
    /* Restore saved modes. */
    glPixelStorei(GL_UNPACK_SWAP_BYTES, swapbytes);
    glPixelStorei(GL_UNPACK_LSB_FIRST, lsbfirst);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, rowlength);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, skiprows);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, skippixels);
    glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);

}

static void drawBitmapText( unsigned char *message, char font ) {

	struct sceneFont_ref *sf;
	void *fp;

	switch( font ) {
	default:
	case 8:
		fp = GLUT_BITMAP_HELVETICA_18;
		sf = &sceneFont18;
		break;
	case 7:
		fp = GLUT_BITMAP_HELVETICA_12;
		sf = &sceneFont12;
		break;
	case 6:
		fp = GLUT_BITMAP_HELVETICA_10;
		sf = &sceneFont12;
		break;
	case 5:
		fp = GLUT_BITMAP_TIMES_ROMAN_24;
		sf = &sceneFont18;
		break;
	case 4:
		fp = GLUT_BITMAP_TIMES_ROMAN_10;
		sf = &sceneFont12;
		break;
	case 3:
		fp = GLUT_BITMAP_8_BY_13;
		sf = &sceneFont12;
		break;
	case 2:
		fp = GLUT_BITMAP_9_BY_15;
		sf = &sceneFont12;
		break;
	}

	while( *message ) {
		switch( *message ) {
		default:
			glutBitmapCharacter( fp, *message );
			break;
		case SCENEFONT_LEFTARROW:
			sceneFontBitmapCharcter( sf->leftArrow );
			break;
		case SCENEFONT_RIGHTARROW:
			sceneFontBitmapCharcter( sf->rightArrow );
			break;
		case SCENEFONT_UPARROW:
			sceneFontBitmapCharcter(sf->upArrow);
			break;
		case SCENEFONT_DOWNARROW:
			sceneFontBitmapCharcter(sf->downArrow);
			break;
		case SCENEFONT_CW:
			sceneFontBitmapCharcter( sf->cw );
			break;
		case SCENEFONT_CCW:
			sceneFontBitmapCharcter( sf->ccw );
			break;
		case SCENEFONT_DEGREES:
			sceneFontBitmapCharcter( sf->degrees );
			break;
		case SCENEFONT_BATT:
			sceneFontBitmapCharcter( sf->batt );
			break;
		case SCENEFONT_PIP:
			sceneFontBitmapCharcter( sf->pip );
			break;
		case SCENEFONT_GRID:
			sceneFontBitmapCharcter( sf->grid );
			break;
		case SCENEFONT_TRIANGLE:
			sceneFontBitmapCharcter( sf->triangle );
			break;
		case SCENEFONT_TEMP:
			sceneFontBitmapCharcter( sf->temp );
			break;
		case SCENEFONT_UNCHECKED:
			sceneFontBitmapCharcter( sf->unchecked );
			break;
		case SCENEFONT_CHECKED:
			sceneFontBitmapCharcter( sf->checked );
			break;
		case SCENEFONT_INFINITY:
			sceneFontBitmapCharcter( sf->infinity );
			break;
		}
		message++;
	}

}


static void showBitmapMessage( GLfloat x, GLfloat y, GLfloat z, unsigned char *message, char font ) {

	/* generic string drawing routine, 2-D */

	glRasterPos3f( x, y, z );
	drawBitmapText( message, LIMIT( font, 2, 8 ) );

}


static int messageHeight( struct sceneGlobal_ref *sg, char font ) {

	return sg->fontHeight[LIMIT( font, 2, 8 )];

}


static void drawMenuItem( struct sceneGlobal_ref *sg, struct scene_ref *sc,
						 float x, float y, unsigned char *message,
						 int flag, int inverse, int active ) {

	int width, height, heightL;

	height = messageHeight( sg, sg->menuFont );
	heightL = height + (int)sg->menuH;

	glPushMatrix();
	glTranslatef( x, y, 0 );
	if( sg->instBackColor[3] > 0 && active ) {
		if( inverse )  glColor4f( sg->menuTextColor[0], sg->menuTextColor[1], sg->menuTextColor[2], sg->instBackColor[3] );
		else           glColor4fv( sg->instBackColor );
		glBegin( GL_POLYGON );
		glVertex3f( (float)(1.0/sc->winw), (float)(-1.0/sc->winw), 0 );
		glVertex3f( (float)(1.0/sc->winw), (float)(-(float)heightL/sc->winw + 1.0/sc->winw), 0 );
		glVertex3f( (float)((sg->menuW-1.0)/sc->winw), (float)(-(float)heightL/sc->winw + 1.0/sc->winw), 0 );
		glVertex3f( (float)((sg->menuW-1.0)/sc->winw), (float)(-1.0/sc->winw), 0 );
		glEnd();
	}

	if( active == 0 )  glColor4fv( sg->instBackColor );
	else if( inverse ) glColor4f( sg->instBackColor[0], sg->instBackColor[1], sg->instBackColor[2], sg->menuTextColor[3] );
	else               glColor4fv( sg->menuTextColor );

    width = getBitmapLength( sg->menuFont, message );
	showBitmapMessage((float)((sg->menuW - width)*0.5/sc->winw),
		(float)(-((float)heightL-(float)sg->menuH/2)/sc->winw), 0, message, sg->menuFont );

	if( flag && active ) {
		glColor4fv( sg->menuTextColor );
		/*glBegin( GL_LINE_LOOP );  asdf 
		glVertex3f( sg->menuW/sc->winw, 0, 0 );
		glVertex3f( sg->menuW/sc->winw, -(float)heightL/sc->winw, 0 );
		glVertex3f( 0, -(float)heightL/sc->winw, 0 );
		glVertex3f( 0, 0, 0 );
		glEnd();*/

		glBegin( GL_QUAD_STRIP );
		glVertex3f( sg->menuW/sc->winw, 0, 0 );
		glVertex3f( (sg->menuW-sg->menuTextLW)/sc->winw, -sg->menuTextLW/sc->winw, 0 );
		glVertex3f( sg->menuW/sc->winw, -(float)heightL/sc->winw, 0 );
		glVertex3f( (sg->menuW-sg->menuTextLW)/sc->winw, -(float)(heightL-sg->menuTextLW)/sc->winw, 0 );
		glVertex3f( 0, -(float)heightL/sc->winw, 0 );
		glVertex3f( sg->menuTextLW/sc->winw, -(float)(heightL-sg->menuTextLW)/sc->winw, 0 );
		glVertex3f( 0, 0, 0 );
		glVertex3f( sg->menuTextLW/sc->winw, -sg->menuTextLW/sc->winw, 0 );
		glVertex3f( sg->menuW/sc->winw, 0, 0 );
		glVertex3f( (sg->menuW-sg->menuTextLW)/sc->winw, -sg->menuTextLW/sc->winw, 0 );
		glEnd();
	}
	glPopMatrix();

}


static void initHUDDL( struct sceneGlobal_ref *sg ) {

	/* intialize HUD display lists */

	float hh_horizon_line[5][3] = { { 1.0, 0.015f, 0.0 },
	{ 0.0, 1.0, 0.0 }, { -1.0, 0.0, 0.0 },{ 0.0, -1.0, 0.0 },
	{ 1.0, -0.015f, 0.0}};
	unsigned char labels[36][3] = { "N\0", "1\0", "2\0", "3\0", "4\0", "5\0",
		"6\0", "7\0", "8\0", "E\0", "10\0", "11\0",
		"12\0", "13\0", "14\0", "15\0", "16\0", "17\0",
		"S\0", "19\0", "20\0", "21\0", "22\0", "23\0",
		"24\0", "25\0", "26\0", "W\0", "28\0", "29\0",
		"30\0", "31\0", "32\0", "33\0", "34\0", "35\0" };
	unsigned char *buffer;
	float lab_x[36] = { -50, -50, -50, -50, -50, -50,
		-50, -50, -50, -50, -110, -110,
		-110, -110, -110, -110, -110, -110,
		-50, -110, -110, -110, -110, -110,
		-110, -110, -110, -50, -110, -110,
		-110, -110, -110, -110, -110, -110 };

	float xl, zl;
	unsigned char scratch[10];
	float hl_zenith[8][3] = { { 0.027f, 0.0, -1.0 },{ 0.003f, 0.003f, -1.0 },
	{ 0.0, 0.009f, -1.0 },{ -0.003f, 0.003f, -1.0 },
	{ -0.009f, 0.0, -1.0 },{ -0.003f, -0.003f, -1.0 },
	{ 0.0, -0.009f, -1.0 },{ 0.003f, -0.003f, -1.0 } };

	float hr_aircraft[7][3] = { { 1.0, -0.015f, 0.0 },{ 1.0, -0.005f, 0.0 },
	{ 1.0, -0.0025f, 0.004f },{ 1.0, 0.0, 0.0 },
	{ 1.0, 0.0025f, 0.004f },{ 1.0, 0.005f, 0.0 },
	{ 1.0, 0.015f, 0.0 } };

	float hfp[6][3] = {{1.0, 0.005f, 0.0 },{1.0, 0.015f, 0.0 },
	{1.0, -0.005f, 0.0 },{1.0, -0.015f, 0.0 },
	{1.0, 0.0, -0.005f },{1.0, 0.0, -0.01f }};

	float bankTickAngle[15] = {-135.0,-90.0,-60.0,-45.0,-30.0,-20.0,-10.0,
		10.0,20.0,30.0,45.0,60.0,90.0,135.0,0.0};
	float bankTickL[15] = {0.005f,0.005f,0.005f,0.003f,0.005f,0.003f,0.003f,
		0.003f,0.003f,0.005f,0.003f,0.005f,0.005f,0.005f,0.007f};
	float bankMarker[3][3] = {{1.0,0.005f,0.082f},{1.0,-0.005f,0.082f},{1.0,0.0,0.07f}};

	float hfd[6][3] =  {{1.0, -0.015f, 0.0 },{1.0, -0.005f, 0.0 },
	{1.0, 0.015f, 0.0 },{1.0, 0.005f, 0.0 },
	{1.0, 0.0, -0.01f },{1.0, 0.0, -0.003f }};
	float hsb[8][3] =  {{1.0, -0.015f, 0.0 },{1.0, -0.1f, 0.0 },
	{1.0, -0.015f, 0.0 },{1.0, -0.015f, 0.015f },
	{1.0, 0.015f, 0.0 },{1.0, 0.015f, 0.015f },
	{1.0, 0.015f, 0.0 },{1.0, 0.1f, 0.0 }};

	int i;
	//float angle;

	/* hud horizon */

	if( !glIsList( sg->hud_horizon_dl ) )
		sg->hud_horizon_dl = glGenLists( 1 );
	glNewList( sg->hud_horizon_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
	/* ticks and labels */
	for( i=0; i<360; i+=5 ) {
		glBegin( GL_LINES );
		glVertex3f( 1.0, 0.0, 0.0 );
		glVertex3f( 1.0, 0.0, -0.005f );
		glEnd();
		if( !(i%10) ) {
			glPushMatrix();
			glTranslatef( 1.0, 0.0, -0.0075f );
			glRotatef( -90.0, 0.0, 1.0, 0.0 );
			glRotatef(  90.0, 0.0, 0.0, 1.0 );
			glScalef( 0.0001f, 0.0001f, 0.0001f );
			glTranslatef( lab_x[i/10], 0.0, 0.0 );
			buffer = labels[i/10];
			while( *buffer ) {
				glutStrokeCharacter( GLUT_STROKE_MONO_ROMAN, *buffer );
				buffer++;
			}
			glPopMatrix();
		}
		glRotatef( 5.0, 0.0, 0.0, 1.0 );
	}
	glPopMatrix();

	glEndList();

	/* hud pitch ladder */

	if( !glIsList( sg->hud_ladder_dl ) )
		sg->hud_ladder_dl = glGenLists( 1 );
	glNewList( sg->hud_ladder_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );

	/* horizon line */

	glBegin( GL_LINE_STRIP );
    for( i=0; i<5; i++ )
		glVertex3fv( hh_horizon_line[i] );
	glEnd();

	/* climb bars */

	for( i=5; i<85; i+=5 ) {
		glPushMatrix();
		glRotatef( (GLfloat)i, 0.0, 1.0, 0.0 );
		glBegin( GL_LINE_STRIP );
		glVertex3f( 1.0, -0.016f, 0.0 );
		glVertex3f( 1.0, -0.046f, 0.0 );
		glVertex3f( 1.0, -0.046f, 0.005f );
		glEnd();
		glBegin( GL_LINE_STRIP );
		glVertex3f( 1.0, 0.016f, 0.0 );
		glVertex3f( 1.0, 0.046f, 0.0 );
		glVertex3f( 1.0, 0.046f, 0.005f );
		glEnd();
		glPushMatrix();
		glTranslatef( 1.0, -0.045f, 0.013f );
		glRotatef( -90.0, 0.0, 1.0, 0.0 );
		glRotatef(  90.0, 0.0, 0.0, 1.0 );
		glScalef( 0.0001f, 0.0001f, 0.0001f );
		sprintf( scratch, "%d", i );
		drawText( scratch );
		glPopMatrix();
		glPopMatrix();
	}

	/* dive bars */

	for( i=-80; i<0; i+=5 ) {
		glPushMatrix();
		glRotatef( (GLfloat)i, 0.0, 1.0, 0.0 );
		xl = (float)(0.016 + 0.03*cos(  0.5*i*C_DEG2RAD ));
		zl = (float)(        0.03*sin( -0.5*i*C_DEG2RAD ));
		glBegin( GL_LINE_STRIP );
		glVertex3f( 1.0, -xl, zl );
		glVertex3f( 1.0, -0.016f, 0.0 );
		glVertex3f( 1.0, -0.016f, -0.005f );
		glEnd();
		glBegin( GL_LINE_STRIP );
		glVertex3f( 1.0, xl, zl );
		glVertex3f( 1.0, 0.016f, 0.0 );
		glVertex3f( 1.0, 0.016f, -0.005f );
		glEnd();
		glPushMatrix();
		glTranslatef( 1.0, -xl-0.015f, zl-0.007f );
		glRotatef( -90.0, 0.0, 1.0, 0.0 );
		glRotatef(  90.0, 0.0, 0.0, 1.0 );
		glScalef( 0.0001f, 0.0001f, 0.0001f );
		sprintf( scratch, "%d", i );
		drawText( scratch );
		glPopMatrix();
		glPopMatrix();
	}

	/* nadir */

	glBegin( GL_LINE_STRIP );
    glVertex3f(  0.0225f, 0.0, 1.0 );
    glVertex3f(  0.0075f, 0.0, 1.0 );
    glVertex3f( 0.0,  0.0075f, 1.0 );
    glVertex3f( -0.0075f, 0.0, 1.0 );
    glVertex3f( 0.0, -0.0075f, 1.0 );
    glVertex3f(  0.0075f, 0.0, 1.0 );
	glEnd();

	/* zenith */

	glBegin( GL_LINE_STRIP );
    for( i=0; i<=8; i++ )
		glVertex3fv( hl_zenith[i%8] );
	glEnd();

	glPopMatrix();

	glEndList();

	/* hud alternate */

	if( !glIsList( sg->hud_alternate_dl ) )
		sg->hud_alternate_dl = glGenLists( 1 );
	glNewList( sg->hud_alternate_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear, sg->znear, sg->znear );

	/* arcs */
    for( i=0; i<4; i++ ) {
		glBegin( GL_LINE_STRIP );
		if( i==0 ) {
			glVertex3f( (float)cos( 0.1 ),        0.1f, 0 );
			glVertex3f( (float)(0.8*cos( 0.1 )),  0.1f, 0 );
			glVertex3f( (float)(cos( 0.1 )),     -0.1f, 0 );
			glVertex3f( (float)(0.8*cos( 0.1 )), -0.1f, 0 );
			/*for( angle = -10.0; angle <= 10.0; angle += 5.0 ) {
				glVertex3f( cos( angle*C_DEG2RAD ), sin( angle*C_DEG2RAD ), 0 );
			}*/
		} else {
			glVertex3f( 1.0,  0, 0 );
			glVertex3f( 0.8f, 0, 0 );
		}
		glEnd();
		glRotatef( 90.0, 0, 0, 1.0 );
	}

	glPopMatrix();

	glEndList();

	/* hud flight path */

	if( !glIsList( sg->hud_fp_dl ) )
		sg->hud_fp_dl = glGenLists( 1 );
	glNewList( sg->hud_fp_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
	glBegin( GL_LINES );
    for( i=0; i<6; i++ )
		glVertex3fv( hfp[i] );
	glEnd();
	glBegin( GL_LINE_LOOP );
    for( i=0; i<360; i+= 10 )
		glVertex3f( 1, (float)( 0.005*sin( C_DEG2RAD*i )),
		               (float)( 0.005*cos( C_DEG2RAD*i )) );
	glEnd();
	glPopMatrix();

	glEndList();

	/* bank scale */

	if( !glIsList( sg->hud_bankscale_dl ) )
		sg->hud_bankscale_dl = glGenLists( 1 );
	glNewList( sg->hud_bankscale_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
	glBegin( GL_LINES );        /* bank */
    for( i=0; i<15; i++ ) {
		glVertex3f( 1,
			(float)(0.070*sin( C_DEG2RAD*bankTickAngle[i] )),
			(float)(0.070*cos( C_DEG2RAD*bankTickAngle[i] )) );
		glVertex3f( 1,
			(float)(( 0.070-bankTickL[i] )*sin( C_DEG2RAD*bankTickAngle[i] )),
			(float)(( 0.070-bankTickL[i] )*cos( C_DEG2RAD*bankTickAngle[i] )) );
    }
	glEnd();
	glPopMatrix();

	glEndList();

	/* flight director */

	if( !glIsList( sg->hud_fd_dl ) )
		sg->hud_fd_dl = glGenLists( 1 );
	glNewList( sg->hud_fd_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
	glBegin( GL_LINES );
    for( i=0; i<6; i++ )
		glVertex3fv( hfd[i] );
	glEnd();
	glPopMatrix();

	glEndList();

	if( !glIsList( sg->hud_reference_dl ) )
		sg->hud_reference_dl = glGenLists( 1 );
	glNewList( sg->hud_reference_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
	glBegin( GL_LINE_STRIP );
    for( i=0; i<7; i++ )
		glVertex3fv( hr_aircraft[i] );
	glEnd();
	glPopMatrix();

	glEndList();

	if( !glIsList( sg->hud_sb_dl ) )
		sg->hud_sb_dl = glGenLists( 1 );
	glNewList( sg->hud_sb_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
	glBegin( GL_LINES );
    for( i=0; i<8; i++ )
		glVertex3fv( hsb[i] );
	glEnd();
	glPopMatrix();

	glEndList();

	if( !glIsList( sg->hud_scales_dl ) )
		sg->hud_scales_dl = glGenLists( 1 );
	glNewList( sg->hud_scales_dl, GL_COMPILE );

	glBegin( GL_POINTS );       /* airspeed */
    for( i=0; i<360; i+= 36 )
		glVertex3f( (float)(-sg->hrpx + 0.018*sin( C_DEG2RAD*i )),
		            (float)( sg->hrpy + 0.018*cos( C_DEG2RAD*i )), 0.0 );
	glEnd();
	glBegin( GL_POINTS );       /* altitude */
    for( i=0; i<360; i+= 36 )
		glVertex3f( (float)(sg->hrpx + 0.025*sin( C_DEG2RAD*i )),
		            (float)(sg->hrpy + 0.025*cos( C_DEG2RAD*i )), 0.0 );
	glEnd();

	glEndList();

	if( !glIsList( sg->hud_bank_dl ) )
		sg->hud_bank_dl = glGenLists( 1 );
	glNewList( sg->hud_bank_dl, GL_COMPILE );

	glPushMatrix();
	glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
	glBegin( GL_LINE_STRIP );
    for( i=0; i<4; i++ ) {
		glVertex3fv( bankMarker[i%3] );
    }
	glEnd();
	glPopMatrix();

	glEndList();
}


static void initStars( struct sceneGlobal_ref *sg ) {

  int i;

  if( !glIsList( sg->stars_dl ) )
    sg->stars_dl = glGenLists( 1 );
  glNewList( sg->stars_dl, GL_COMPILE );

  glColor4f( 1.0, 1.0, 1.0, 1.0 );
  if( sg->antialias )
    glEnable( GL_POINT_SMOOTH );

  glPushMatrix();
    glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );
    for( i=0; i<NSTARS; i++ ) {
      glPointSize( (float)(star_data[i][3]*sg->starSize) );
      glBegin( GL_POINTS );
        glVertex3fv( star_data[i] );
      glEnd();
    }
  glPopMatrix();

  if( sg->antialias )
    glDisable( GL_POINT_SMOOTH );

  glPointSize( 1.0 );

  glEndList();

}


static void drawStars( struct scene_ref *sc, struct sceneGlobal_ref *sg ) {

	glDisable( GL_LIGHTING );
	glShadeModel( GL_FLAT );
	glPushMatrix();
	glRotatef( (float)(sc->eyeLat), 0.0, 1.0, 0.0 );
	glRotatef( (float)(-sc->eyeLon + sg->starsAngle), 1.0, 0.0, 0.0 );
	glCallList( sg->stars_dl );
	glPopMatrix();
	glEnable( GL_LIGHTING );

}


static void initHorizon( struct sceneGlobal_ref *sg ) {

	/* store away points correspoinding to a circle */

	int i;

	for( i=0; i<=HORIZONPTS; i++ ) {
		circle[i][0] = (float)cos( C_DEG2RAD*360.0/HORIZONPTS*i );
		circle[i][1] = (float)sin( C_DEG2RAD*360.0/HORIZONPTS*i );
	}

}


static void drawEarth( char showTex, unsigned int etexture, struct scene_ref *sc, struct sceneGlobal_ref *sg, double terrainAlt ) {

	double alt;
	double horizon, ground, d;
	float sground, cground;
	int i;
	float phi, theta, psi, left, as;
    double latitude, longitude;
    float white[4] = { 1.0, 1.0, 1.0, 1.0 };

	alt = MAX( 0.01, sc->eyeAlt - terrainAlt );

	horizon = -acos( REARTH/( REARTH + alt ) );
	left    = (float)(C_PI*0.5 + horizon);
	d       = sqrt( 2*REARTH*alt + alt*alt ); /*distance to limb */

	glPushMatrix();

    /* draw on a circle of znear*10 radius: */
    glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );

    /* Earth */

    glShadeModel( GL_SMOOTH );

	if( showTex && etexture ) {
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );
	    glEnable( GL_TEXTURE_2D );
        glBindTexture( GL_TEXTURE_2D, etexture );
	} else {
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->groundColor );
	}

	/* draw ground polygons as quad strips */

	for( i=0; i<HORIZONPTS; i++ ) {
		glBegin( GL_QUAD_STRIP );
		as = (float)MIN( alt/sg->vis, sg->angStep*left );
		for( ground=horizon; ground>-C_PI*0.5; ground -= as ) {
			if( ground != horizon ) {
				/* determine step to go a given angle over Earth */
				as = (float)(sg->earthStep*REARTH/d*cos( phi )/cos( theta ));
				if( ground + C_PI*0.5 < as )
					ground = -C_PI*0.5;
			}
			sground  = (float)sin( ground );
			cground  = (float)cos( ground );

			/* theta is around earth this point is in front of us */
			phi = (float)(C_PI*0.5 + ground);
			if( ground == horizon )
				theta = (float)(-horizon);
			else
				theta = (float)(asin( ( REARTH + alt )/REARTH*sin( phi ) ) - phi);
			d = ( REARTH*( 1 - cos( theta ) ) + alt )/
				MAX( -sground, 0.001 );

			if( showTex && etexture ) {
				psi = (float)(C_DEG2RAD*360.0*(i+1)/HORIZONPTS);
				greatCircle( sc->eyeLat*C_DEG2RAD, sc->eyeLon*C_DEG2RAD, theta, psi, &latitude, &longitude );
				glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
				glTexCoord2f( (float)(0.5 + 0.5*longitude/C_PI), (float)(0.5 + latitude/C_PI) );
				glVertex3f( cground*circle[i+1][0], cground*circle[i+1][1], -sground );

				psi = (float)(C_DEG2RAD*360.0*i/HORIZONPTS);
				greatCircle( sc->eyeLat*C_DEG2RAD, sc->eyeLon*C_DEG2RAD, theta, psi, &latitude, &longitude );
				glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
				glTexCoord2f( (float)(0.5 + 0.5*longitude/C_PI), (float)(0.5 + latitude/C_PI) );
				glVertex3f( cground*circle[i][0], cground*circle[i][1], -sground );
			} else {
				psi = (float)(C_DEG2RAD*360.0*(i+1)/HORIZONPTS);
				glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
				glVertex3f( cground*circle[i+1][0], cground*circle[i+1][1], -sground );

				psi = (float)(C_DEG2RAD*360.0*i/HORIZONPTS);
				glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
				glVertex3f( cground*circle[i][0],   cground*circle[i][1],   -sground );
			}
		}
		glEnd();
	}

 	if( showTex && etexture ) {
		glDisable( GL_TEXTURE_2D );
	}
	glPopMatrix();

}


static void drawAtmosphere( struct scene_ref *sc,
						   struct sceneGlobal_ref *sg, double terrainAlt ) {

	double alt, vis;
	double horizon, air, ground, d, d2;
	float shorizon, chorizon, sair, cair, sground, cground;
	float oldsground, oldcground, alpha, oldalpha, oldtheta;
	int i, j;
	float dark, left;
	float phi, theta, psi, as;

	alt = MAX( 0.01, sc->eyeAlt - terrainAlt );
	vis = MAX( sg->vis, alt - sg->air ); /* this is a little hack so the Earth map is visible when we are very high */

	horizon = -acos( REARTH/( REARTH + alt ) );
	left    = (float)(C_PI*0.5 + horizon);
	d       = sqrt( 2*REARTH*alt + alt*alt ); /*distance to limb */
	air     = horizon + atan( sg->air/MAX( d, vis ) );
	air     = MIN( C_PI*2.0, air );

	shorizon = (float)sin( horizon );
	chorizon = (float)cos( horizon );
	sair     = (float)sin( air );
	cair     = (float)cos( air );

	glPushMatrix();
    glScalef( sg->znear*10, sg->znear*10, sg->znear*10 );

    glShadeModel( GL_SMOOTH );

	as = (float)MIN( alt/vis, sg->angStep*left );
	for( ground=horizon; ground>-C_PI*0.5; ground -= as ) {
		if( ground != horizon ) {
			/* determine step to go a given angle over Earth */
			as = (float)(sg->earthStep*REARTH/d*cos( phi )/cos( theta ));
			if( ground + C_PI*0.5 < as )
				ground = -C_PI*0.5;
		}
		sground = (float)sin( ground );
		cground = (float)cos( ground );

		phi = (float)(C_PI*0.5 + ground);
		if( ground != horizon ) {
			theta = (float)(asin( ( REARTH + alt )/REARTH*sin( phi ) ) - phi);
			psi = (float)(C_PI*0.5 - theta - phi);
			d = ( REARTH*( 1 - cos( theta ) ) + alt )/
				MAX( -sground, 0.001 );
			d2 = MIN( d, sg->air/MAX( sin( psi ), 0.001 ) );
			alpha = (float)LIMIT( d2/vis, 0.01, 1.0 );
		} else {
			theta = (float)(-horizon);
			d = ( REARTH*( 1 - cos( theta ) ) + alt )/
				MAX( -sground, 0.001 );
			alpha = 1.0; /* ready for next time */
		}

		if( ground != horizon ) {
			/* quad_strip doesn't draw this right on some graphics cards */
			glBegin( GL_QUADS );
			glNormal3f( 0.0, 0.0, -1.0 );
			for( j=0; j<HORIZONPTS + sg->extraQuad; j++ ) {
				i = j%HORIZONPTS;  /* draw extra quad for some graphics cards */
				sg->horizonColor[3] = oldalpha;
				psi = (float)(C_DEG2RAD*360.0*(i+1)/HORIZONPTS);
				glNormal3f( (float)(cos( psi )*sin( oldtheta )), (float)(sin( psi )*sin( oldtheta )), (float)(-cos( oldtheta )) );
				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->horizonColor );
				glVertex3f( oldcground*circle[i+1][0], oldcground*circle[i+1][1], -oldsground );
				psi = (float)(C_DEG2RAD*360.0*i/HORIZONPTS);
				glNormal3f( (float)(cos( psi )*sin( oldtheta )), (float)(sin( psi )*sin( oldtheta )), (float)(-cos( oldtheta )) );
				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->horizonColor );
				glVertex3f( oldcground*circle[i][0], oldcground*circle[i][1], -oldsground );
				sg->horizonColor[3] = alpha;
				psi = (float)(C_DEG2RAD*360.0*i/HORIZONPTS);
				glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->horizonColor );
				glVertex3f( cground*circle[i][0], cground*circle[i][1], -sground );
				psi = (float)(C_DEG2RAD*360.0*(i+1)/HORIZONPTS);
				glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->horizonColor );
				glVertex3f( cground*circle[i+1][0], cground*circle[i+1][1], -sground );
			}
			glEnd();
		}
		oldcground = cground;
		oldsground = sground;
		oldalpha = alpha;
		oldtheta = theta;
    }

    sg->horizonColor[3] = 1.0; /* ready for next time */

    dark = (float)(( sg->air*1.5 - alt )/( sg->air*1.5 )); /* ? */
    dark = MAX( 0.01f, dark );
    sg->skyColor[3] = dark;

    /* atmosphere element of limb */
	/* quad_strip doesn't draw this right on some graphics cards */
    glBegin( GL_QUADS );
	theta = (float)(-horizon);
	for( j=0; j<HORIZONPTS + sg->extraQuad; j++ ) {
		i = j%HORIZONPTS;  /* draw extra quad for some graphics cards */
		psi = (float)(C_DEG2RAD*360.0*(i+1)/HORIZONPTS);
		glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->skyColor );
		glVertex3f( cair*circle[i+1][0], cair*circle[i+1][1], -sair );
		psi = (float)(C_DEG2RAD*360.0*i/HORIZONPTS);
		glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->skyColor );
		glVertex3f( cair*circle[i][0], cair*circle[i][1], -sair );
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->horizonColor );
		glVertex3f( chorizon*circle[i][0], chorizon*circle[i][1], -shorizon );
		psi = (float)(C_DEG2RAD*360.0*(i+1)/HORIZONPTS);
		glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->horizonColor );
		glVertex3f( chorizon*circle[i+1][0], chorizon*circle[i+1][1], -shorizon );
	}
    glEnd();

    /* upper cone */
    glMaterialfv( GL_FRONT, GL_AMBIENT, sg->skyColor );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->skyColor );
    glBegin( GL_TRIANGLE_FAN );
	glNormal3f( 0.0, 0.0, -1.0 );
	glVertex3f( 0.0, 0.0, -1.0 );
	for( i=0; i<=HORIZONPTS; i++ ) {
		psi = (float)(C_DEG2RAD*360.0*i/HORIZONPTS);
		glNormal3f( (float)(cos( psi )*sin( theta )), (float)(sin( psi )*sin( theta )), (float)(-cos( theta )) );
		glVertex3f( cair*circle[i][0], cair*circle[i][1], -sair );
	}
    glEnd();

	glPopMatrix();

}

static char nextChar( FILE *filep ) {

    char cn;

	if( fread( &cn, sizeof( char ), 1, filep ) == 0 ) {
        cn = EOF;
	}

	return cn;
}

static char kmlReadString( FILE *filep, char *variable, char *value ) {

    char foundSomething = 0, cn = ' ';
    char variableCheck[100];
    int i;


    /* find < */
	while( cn != '<' && cn != EOF ) {
		cn = nextChar( filep );
	}
    cn = nextChar( filep ); /* skip < */

    while( cn != '/' && cn != EOF ) {

        /* find >, storing variable name */
        i = 0;
        while( cn != '>' && cn != EOF && i < 100 ) {
            variable[i++] = cn;
            cn = nextChar( filep );
        }
        variable[i] = '\0';
        /*printf( "variable = %s\n", variable );*/
        cn = nextChar( filep ); /* skip > */

        /* find <, storing value */
        i = 0;
        while( cn != '<' && cn != EOF && i < 100 ) {
            value[i++] = cn;
            cn = nextChar( filep );
        }
        value[i] = '\0';
        /*printf( " value = %s\n", value );*/
        cn = nextChar( filep ); /* skip < */

    }

    /* skip /, find >, storing variable name again */
    cn = nextChar( filep ); /* skip / */
    i = 0;
    while( cn != '>' && cn != EOF && i < 100 ) {
        variableCheck[i++] = cn;
        cn = nextChar( filep );
    }
    variableCheck[i] = '\0';
    /*printf( "  variableCheck = %s\n", variableCheck );*/
    cn = nextChar( filep ); /* skip > */

    /* verify variable name */
    if( !strcmp( variableCheck, variable ) && strlen( variableCheck ) == strlen( variable ) &&
        strlen( variable ) > 1 ) {
        foundSomething = 1;
        /*printf( "   got one!\n" );*/
    } else {
        foundSomething = 0;
        /*printf( "   this one no good2\n" );*/
    }

    if( cn == EOF ) foundSomething = EOF;

	return foundSomething;

}

static void generateOverlay( struct sceneGlobal_ref *sg,
							struct scene_ref *sc,
                            struct sceneOverlay_ref *m,
							int number ) {

	float white[4] = { 1.0, 1.0, 1.0, 1.0 };
    char buffer[BUFFER_SIZE], fileName[50];
    int gridn, i, j;
    FILE *filep;

    if( strlen( m->file ) ) {

        m->successFlag = 1;

        strcpy( buffer, m->file );
        i = strlen( buffer );
        if( i > 4 ) {
            if( !strcmp( &buffer[i-4], ".kml" ) ) {
                /*logInfo( "scene: Found .kml file." );*/
                filep = fopen( m->file, "r");
                if( filep != NULL ) {
                    char variable[1024], value[1024], returnValue, *myname;
                    double north = 0, south = 0, east = 0, west = 0;
                    m->rotation = 0.0; /* in case rotation not specified */
                    strcpy( fileName, "" ); /* in case file name is missing */
                    while( returnValue = kmlReadString( filep, variable, value ) != EOF ) {
                        if( returnValue ) {
                            if( !strcmp( variable, "href" ) ) {
                                /* just get file name, ignore path */
                                myname = strrchr( value, '/' );
                                if( myname != NULL ) {
                                    myname++;
                                } else {
                                    myname = value;
                                }
                                /*printf( "fileName = %s\n", myname );*/
                                /* strip off directory */
                                strcpy( fileName, myname );
                            } else if( !strcmp( variable, "north" ) ) {
                                north = atof( value );
                            } else if( !strcmp( variable, "south" ) ) {
                                south = atof( value );
                            } else if( !strcmp( variable, "east" ) ) {
                                east = atof( value );
                            } else if( !strcmp( variable, "west" ) ) {
                                west = atof( value );
                            } else if( !strcmp( variable, "rotation" ) ) {
                                m->rotation = -atof( value );
                            }
                        }
                    }
                    if( north != 0.0 && south != 0.0 && east != 0.0 && west != 0.0 ) {
                        m->lat    = ( north + south )*0.5;
                        m->lon    = ( east  + west  )*0.5;
                        m->length = ( north - south )*60.0*C_NM2FT;
                        m->width  = hmodDeg( east  - west  )*60.0*C_NM2FT*cos( m->lat*C_DEG2RAD );
                        m->offset[0] = 0.0;
                        m->offset[1] = 0.0;
                    }
                    fclose( filep );
                } else {
                    sprintf( buffer, "scene: Could not open .kml file %s.", m->file );
                    logError( buffer );
                    m->successFlag = 0;
                }
            } else {
                strcpy( fileName, m->file );
            }
        } else {
            strcpy( fileName, m->file );
        }

        /* read texture and generate display list */
        if( m->successFlag ) {

            //loadOpenGL2DTextureBMP( fileName, &(sc->otexture[number]), GL_RGB );

            if( !glIsList( m->dl ) ) m->dl = glGenLists( 1 );
            glNewList( m->dl, GL_COMPILE );
            glShadeModel( GL_SMOOTH );

            if( sc->otexture[number] ) {

				gridn = (int)sg->overlayGridn;

				glEnable( GL_TEXTURE_2D );

				glBindTexture( GL_TEXTURE_2D, sc->otexture[number] );
				/*glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );*/
				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, m->rgba );
    			glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

				glPushMatrix();
				glRotatef( (float)(90 + m->rotation), 0, 0, 1 );
				glBegin( GL_QUADS );
				glNormal3f( 0.0, 0.0, -1.0 );
				for( i=-gridn; i<gridn; i++ ) {
					for( j=-gridn; j<gridn; j++ ) {
						glTexCoord2f( 0.5f*i/gridn + 0.5f, 0.5f*j/gridn + 0.5f );
						glVertex3f( (float)(0.5f*m->width*i/gridn), (float)(-0.5f*m->length*j/gridn), 0.0 );

						glTexCoord2f( 0.5f*(i+1)/gridn + 0.5f, 0.5f*j/gridn + 0.5f );
						glVertex3f( (float)(0.5f*m->width*(i+1)/gridn), (float)(-0.5f*m->length*j/gridn), 0.0 );

						glTexCoord2f( 0.5f*(i+1)/gridn + 0.5f, 0.5f*(j+1)/gridn + 0.5f );
						glVertex3f( (float)(0.5f*m->width*(i+1)/gridn), (float)(-0.5f*m->length*(j+1)/gridn), 0.0 );

						glTexCoord2f( 0.5f*i/gridn + 0.5f, 0.5f*(j+1)/gridn + 0.5f );
						glVertex3f( (float)(0.5f*m->width*i/gridn), (float)(-0.5f*m->length*(j+1)/gridn), 0.0 );
					}
				}
				glEnd();
				glPopMatrix();

				glDisable( GL_TEXTURE_2D );

            }

            glEndList();

        }

    } else {
        m->successFlag = 0;
        if( !glIsList( m->dl ) ) m->dl = glGenLists( 1 );
        glNewList( m->dl, GL_COMPILE );
        glEndList();
    }

}

static void initDisplayLists( struct sceneGlobal_ref *sg, struct scene_ref *sc, struct realScene_ref *rs ) {

	struct building_ref *bldg;
	struct pole_ref     *pole;
	int i, j, k, gridn;
	GLubyte *image;
	float white[4] = { 1.0, 1.0, 1.0, 1.0 };
	float light[4] = { 0.7f, 0.7f, 0.7f, 1.0 };
	float grey[4]  = { 0.5f, 0.5f, 0.5f, 1.0 };
	/*float sand[4] = {0.8f, 0.8f, 0.0, 1.0};*/
	/*float red[4] = {1.0, 0.0 ,0.0 ,.90};*/
	float roofColor[4] = {0,0,0,1};
	float angle;

	/* pre-computes */

	initStars( sg );
	initHorizon( sg );

	/* object display lists */

	//initGeneric();
	//initGTMax();
	//initSlungload();
	//initHelispy();
	//initR22();
	//initAirguard();
	//initAirscout();
	//initVan(sc);
	//initShip();
	//initWamv( sc );
	//initPerson();
	//initLogo();
	//initEdge();
	//initMtr();
	//initComplexScene( sg );
	//initYellowJacket();
	//initQuadrotor();
	initMultirotor();
	//initAlia();
 //   initPsp();
	//initHelicycle();
	//initFreewing();

	for( i=0; i<9; i++ ) {
		glDeleteTextures(1,&sc->gmtexture[i]);
		sc->gmtexture[i] = 0;
	}
	sc->map_higherZoom = 0;
	sc->map_tryHigher = 0;

	/* grass on ground */

	/* read texture */

	/* read texture does the malloc */
	/* converts all textures to rgba, regardless of components return arg */
	glDeleteTextures(1,&sc->gtexture);
	glGenTextures(1,&sc->gtexture);
	if( ( image = (GLubyte *)read_texture( sg->gfile,
		&sg->gwidth, &sg->gheight,
		&sg->gcomponents) ) == 0 ) {
		glDeleteTextures(1,&sc->gtexture);
		sc->gtexture = 0;
    } else {
        glBindTexture( GL_TEXTURE_2D, sc->gtexture );
		glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, sg->gwidth, sg->gheight, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, image );
		free(image);
  	}

	gridn = (int)(sg->vis/sg->gridx);
	gridn = MIN( gridn, sg->gridmax ); /* just too many polygons... */

	if( !glIsList( sg->grass_dl ) )
		sg->grass_dl = glGenLists( 1 );
	glNewList( sg->grass_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

	if( sc->gtexture ) {

        glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->gtexture );

		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light );

		glBegin( GL_QUADS );
		glNormal3f( 0.0, 0.0, -1.0 );

		for( i=-gridn; i<gridn; i++ ) {
			for( j=-gridn; j<gridn; j++ ) {
				glTexCoord2f( 0, 0 );
				glVertex3f( sg->gridx*i,     sg->gridx*j,     0.0 );
				glTexCoord2f( 1, 0 );
				glVertex3f( sg->gridx*i,     sg->gridx*(j+1), 0.0 );
				glTexCoord2f( 1, 1 );
				glVertex3f( sg->gridx*(i+1), sg->gridx*(j+1), 0.0 );
				glTexCoord2f( 0, 1 );
				glVertex3f( sg->gridx*(i+1), sg->gridx*j,     0.0 );
			}
		}
		glEnd();

		glDisable( GL_TEXTURE_2D );

	}

	glEndList();


	/* grid on ground */

	if( !glIsList( sg->grid_dl ) )
		sg->grid_dl = glGenLists( 1 );
	glNewList( sg->grid_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

	gridn = (int)(sg->vis/sg->gridx);
	gridn = MIN( gridn, sg->gridmax ); /* just too many polygons... */

	glMaterialfv( GL_FRONT, GL_AMBIENT, sg->gridColor );
	glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->gridColor );
	glBegin( GL_LINES );
	glNormal3f( 0.0, 0.0, -1.0 );
	/* BIG grid */
	/*for( i=-gridn; i<=gridn; i++ ) {
		glVertex3f(  sg->gridx*i*gridn,    -sg->gridx*gridn*gridn, 0.0 );
		glVertex3f(  sg->gridx*i*gridn,     sg->gridx*gridn*gridn, 0.0 );
		glVertex3f( -sg->gridx*gridn*gridn, sg->gridx*i*gridn,     0.0 );
		glVertex3f(  sg->gridx*gridn*gridn, sg->gridx*i*gridn,     0.0 );
	}*/
	/* small grid */
	for( i=-gridn; i<=gridn; i++ ) {
		glVertex3f(  sg->gridx*i,    -sg->gridx*gridn, 0.0 );
		glVertex3f(  sg->gridx*i,     sg->gridx*gridn, 0.0 );
		glVertex3f( -sg->gridx*gridn, sg->gridx*i,     0.0 );
		glVertex3f(  sg->gridx*gridn, sg->gridx*i,     0.0 );
	}
	glEnd();

	glEndList();


	/* read Earth texture */
#if 1
    //loadOpenGL2DTextureBMP( sg->efile, &sc->etexture, GL_RGB );
#else
	/* read texture does the malloc */
	/* converts all textures to rgba, regardless of components return arg */
	glDeleteTextures(1,&sc->etexture);
	glGenTextures(1,&sc->etexture);
	if( ( image = (GLubyte *)read_texture( sg->efile,
		&sg->ewidth, &sg->eheight,
		&sg->ecomponents) ) == 0 ) {
		glDeleteTextures(1,&sc->etexture);
		sc->etexture = 0;
    } else {
        glBindTexture( GL_TEXTURE_2D, sc->etexture );
		glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
	    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, sg->ewidth, sg->eheight, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, image );
		free(image);
    }
#endif

	/* north on map */

	if( !glIsList( sg->northOnMap_dl ) )
		sg->northOnMap_dl = glGenLists( 1 );
	glNewList( sg->northOnMap_dl, GL_COMPILE );

    glBegin( GL_LINE_STRIP );
    glVertex3f( (float)(    cos( 0.3 )),  0.1f, 0 );
    glVertex3f( (float)(0.6*cos( 0.3 )),  0.1f, 0 );
    glVertex3f( (float)(    cos( 0.3 )), -0.1f, 0 );
    glVertex3f( (float)(0.6*cos( 0.3 )), -0.1f, 0 );
    glEnd();

    glBegin( GL_LINES );
    glVertex3f( -0.2f, 0,    0 );
    glVertex3f(  0.5f, 0,    0 );
    glVertex3f(  0,   -0.2f, 0 );
    glVertex3f(  0,    0.2f, 0 );
    glEnd();

	glEndList();


	/* ground overlay textures */

    generateOverlay( sg, sc, sg->overlays->m0, 0 );
    generateOverlay( sg, sc, sg->overlays->m1, 1 );
    generateOverlay( sg, sc, sg->overlays->m2, 2 );
    generateOverlay( sg, sc, sg->overlays->m3, 3 );
    generateOverlay( sg, sc, sg->overlays->m4, 4 );
    generateOverlay( sg, sc, sg->overlays->m5, 5 );
    generateOverlay( sg, sc, sg->overlays->m6, 6 );
    generateOverlay( sg, sc, sg->overlays->m7, 7 );
    generateOverlay( sg, sc, sg->overlays->m8, 8 );
    generateOverlay( sg, sc, sg->overlays->m9, 9 );

	/* tree */

	/* read texture */

	/* read texture does the malloc */
	/* converts all textures to rgba, regardless of components return arg */
	glDeleteTextures(1,&sc->ttexture);
	glGenTextures(1,&sc->ttexture);
	if( ( image = (GLubyte *)read_texture( sg->tfile,
		&sg->twidth, &sg->theight,
		&sg->tcomponents) ) == 0 ) {
		glDeleteTextures(1,&sc->ttexture);
		sc->ttexture = 0;
    } else {
        glBindTexture( GL_TEXTURE_2D, sc->ttexture );
		glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, sg->twidth, sg->theight, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, image );
		free(image);
    }

	if( !glIsList( sg->tree_dl ) )
		sg->tree_dl = glGenLists( 1 );
	glNewList( sg->tree_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

	if( sc->ttexture ) {

		glEnable( GL_TEXTURE_2D );
		glEnable( GL_ALPHA_TEST );

        glBindTexture( GL_TEXTURE_2D, sc->ttexture );

		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

		glBegin( GL_POLYGON );
		glNormal3f( -0.3f, 0, -0.9f );
		glTexCoord2f( 0, 0 );
		glVertex3f( 0, -0.5f, 0 );
		glTexCoord2f( 1, 0 );
		glVertex3f( 0,  0.5f, 0 );
		glTexCoord2f( 1, 1 );
		glVertex3f( 0,  0.5f, -1 );
		glTexCoord2f( 0, 1 );
		glVertex3f( 0, -0.5f, -1 );
		glEnd();

		glDisable( GL_ALPHA_TEST );
		glDisable( GL_TEXTURE_2D );

	}

	glEndList();

	if( !glIsList( sg->treeNoTex_dl ) )
		sg->treeNoTex_dl = glGenLists( 1 );
	glNewList( sg->treeNoTex_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

    glMaterialfv( GL_FRONT, GL_AMBIENT, sg->treeColor );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->treeColor );
    glBegin( GL_POLYGON );
    glNormal3f( -0.99f, 0, -0.1f );
    glVertex3f( 0, 0, -1 );
    glNormal3f( 0, -0.99f, -0.1f );
    glVertex3f( 0, -0.2f, 0 );
    glNormal3f( -0.99f, 0, -0.1f );
    glVertex3f( -0.2f, 0, 0 );
    glNormal3f( 0, 0.99f, -0.1f );
    glVertex3f( 0, 0.2f, 0 );
    glEnd();

	glEndList();

	/* generic building */

	if( !glIsList( sg->building_dl ) )
		sg->building_dl = glGenLists( 1 );
	glNewList( sg->building_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

	glMaterialfv( GL_FRONT, GL_AMBIENT, sg->roofColor );
	glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->roofColor );
	glBegin( GL_POLYGON );
	glNormal3f( 0.0, 0.0, -1.0 );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingW, -sg->buildingH );
	glVertex3f( -0.5f*sg->buildingL, +0.5f*sg->buildingW, -sg->buildingH );
	glVertex3f( +0.5f*sg->buildingL, +0.5f*sg->buildingW, -sg->buildingH );
	glVertex3f( +0.5f*sg->buildingL, -0.5f*sg->buildingW, -sg->buildingH );
	glEnd();

	glMaterialfv( GL_FRONT, GL_AMBIENT, sg->buildingColor[0] );
	glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->buildingColor[0] );
	glBegin( GL_QUAD_STRIP );
	glNormal3f( -1.0, 0.0, 0.0 );
	glVertex3f( -0.5f*sg->buildingL, +0.5f*sg->buildingW, 0 );
	glVertex3f( -0.5f*sg->buildingL, +0.5f*
		sg->buildingW, -sg->buildingH );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingW, 0 );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingW, -sg->buildingH );
	glNormal3f( 0.0, -1.0, 0.0 );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingW, 0 );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingW, -sg->buildingH );
	glVertex3f( +0.5f*sg->buildingL, -0.5f*sg->buildingW, 0 );
	glVertex3f( +0.5f*sg->buildingL, -0.5f*sg->buildingW, -sg->buildingH );
	glNormal3f( +1.0, 0.0, 0.0 );
	glVertex3f( +0.5f*sg->buildingL, -0.5f*sg->buildingW, 0 );
	glVertex3f( +0.5f*sg->buildingL, -0.5f*sg->buildingW, -sg->buildingH );
	glVertex3f( +0.5f*sg->buildingL, +0.5f*sg->buildingW, 0 );
	glVertex3f( +0.5f*sg->buildingL, +0.5f*sg->buildingW, -sg->buildingH );
	glNormal3f( 0.0, +1.0, 0.0 );
	glVertex3f( +0.5f*sg->buildingL, +0.5f*sg->buildingW, 0 );
	glVertex3f( +0.5f*sg->buildingL, +0.5f*sg->buildingW, -sg->buildingH );
	glVertex3f( -0.5f*sg->buildingL, +0.5f*sg->buildingW, 0 );
	glVertex3f( -0.5f*sg->buildingL, +0.5f*sg->buildingW, -sg->buildingH );
	glEnd();

	glDisable( GL_DEPTH_TEST );

	glMaterialfv( GL_FRONT, GL_AMBIENT, sg->blackColor );
	glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->blackColor );
	glBegin( GL_POLYGON );
	glNormal3f( 0.0, 0.0, -1.0 );
	glVertex3f( 0.5f*sg->buildingL, +0.5f*sg->buildingWinWidth, -sg->buildingWinH-0.5f*sg->buildingWinHeight );
	glVertex3f( 0.5f*sg->buildingL, +0.5f*sg->buildingWinWidth, -sg->buildingWinH+0.5f*sg->buildingWinHeight );
	glVertex3f( 0.5f*sg->buildingL, -0.5f*sg->buildingWinWidth, -sg->buildingWinH+0.5f*sg->buildingWinHeight );
	glVertex3f( 0.5f*sg->buildingL, -0.5f*sg->buildingWinWidth, -sg->buildingWinH-0.5f*sg->buildingWinHeight );
	glVertex3f( 0.5f*sg->buildingL, +0.5f*sg->buildingWinWidth, -sg->buildingWinH-0.5f*sg->buildingWinHeight );
	glEnd();
	glBegin( GL_POLYGON );
	glNormal3f( 0.0, 1.0, 0.0 );
	glVertex3f( -0.5f*sg->buildingWinWidth, +0.5f*sg->buildingW, -sg->buildingWinH+0.5f*sg->buildingWinHeight );
	glVertex3f( +0.5f*sg->buildingWinWidth, +0.5f*sg->buildingW, -sg->buildingWinH+0.5f*sg->buildingWinHeight );
	glVertex3f( +0.5f*sg->buildingWinWidth, +0.5f*sg->buildingW, -sg->buildingWinH-0.5f*sg->buildingWinHeight );
	glVertex3f( -0.5f*sg->buildingWinWidth, +0.5f*sg->buildingW, -sg->buildingWinH-0.5f*sg->buildingWinHeight );
	glVertex3f( -0.5f*sg->buildingWinWidth, +0.5f*sg->buildingW, -sg->buildingWinH+0.5f*sg->buildingWinHeight );
	glEnd();
	/* door */
	glBegin( GL_POLYGON );
	glNormal3f( 0.0, 0.0, -1.0 );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingWinWidth, 0 );
	glVertex3f( -0.5f*sg->buildingL, +0.5f*sg->buildingWinWidth, 0 );
	glVertex3f( -0.5f*sg->buildingL, +0.5f*sg->buildingWinWidth, -2.0f*sg->buildingWinHeight );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingWinWidth, -2.0f*sg->buildingWinHeight );
	glVertex3f( -0.5f*sg->buildingL, -0.5f*sg->buildingWinWidth, 0 );
	glEnd();

	glEnable( GL_DEPTH_TEST );

	glEndList();

    /* building symbol */

    //loadOpenGL2DTextureBMP( sg->bfile, &sc->btexture, GL_RGB );

	/* symbol */

	if( !glIsList( sg->buildingSym_dl ) )
		sg->buildingSym_dl = glGenLists( 1 );
	glNewList( sg->buildingSym_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

	glDisable( GL_DEPTH_TEST );

	if( sc->btexture ) {

		glEnable( GL_TEXTURE_2D );

		glBindTexture( GL_TEXTURE_2D, sc->btexture );

		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

/*		glBegin( GL_POLYGON );
		glTexCoord2f( 0, 0 );
		glVertex3f( -0.5f*sg->buildingL, -0.5f*CF_M2FT, -2.0f*CF_M2FT );
		glTexCoord2f( 1, 0 );
		glVertex3f( -0.5f*sg->buildingL, +0.5f*CF_M2FT, -2.0f*CF_M2FT );
		glTexCoord2f( 1, 1 );
		glVertex3f( -0.5f*sg->buildingL, +0.5f*CF_M2FT, -3.0f*CF_M2FT );
		glTexCoord2f( 0, 1 );
		glVertex3f( -0.5f*sg->buildingL, -0.5f*CF_M2FT, -3.0f*CF_M2FT );
		glVertex3f( -0.5f*sg->buildingL, -0.5f*CF_M2FT, -2.0f*CF_M2FT );
		glEnd();
*/

		glBegin( GL_POLYGON );
		glTexCoord2f( 0, 0 );
		glVertex3f( -0.5f*sg->buildingL, +0.75f*CF_M2FT, -1.0*CF_M2FT );
		glTexCoord2f( 1, 0 );
		glVertex3f( -0.5f*sg->buildingL, +1.75f*CF_M2FT, -1.0*CF_M2FT );
		glTexCoord2f( 1, 1 );
		glVertex3f( -0.5f*sg->buildingL, +1.75f*CF_M2FT, -2.0f*CF_M2FT );
		glTexCoord2f( 0, 1 );

		glVertex3f( -0.5f*sg->buildingL, +0.75f*CF_M2FT, -2.0f*CF_M2FT );
		glVertex3f( -0.5f*sg->buildingL, +0.75f*CF_M2FT, -1.0*CF_M2FT );
		glEnd();

		glDisable( GL_TEXTURE_2D );

	} else {

		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, grey );

		glBegin( GL_POLYGON );
		glNormal3f( 0.0, 0.0, -1.0 );
		glVertex3f( -0.5f*sg->buildingL, -0.5f*CF_M2FT, -2.5f*CF_M2FT );
		glVertex3f( -0.5f*sg->buildingL, +0.5f*CF_M2FT, -2.5f*CF_M2FT );
		glVertex3f( -0.5f*sg->buildingL, +0.5f*CF_M2FT, -3.5f*CF_M2FT );
		glVertex3f( -0.5f*sg->buildingL, -0.5f*CF_M2FT, -3.5f*CF_M2FT );
		glVertex3f( -0.5f*sg->buildingL, -0.5f*CF_M2FT, -2.5f*CF_M2FT );
		glEnd();

	}

	glEnable( GL_DEPTH_TEST );

	glEndList();


	/* wdb scene */
	//initWdbScene( sg, sc );

	/* real scene */

	/* now load up the fake symbol */

	//loadOpenGL2DTextureBMP( sg->ffile, &sc->ftexture, GL_RGB );

	if( !glIsList( sg->realScene_dl ) )
		sg->realScene_dl = glGenLists( 1 );
	glNewList( sg->realScene_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

	/* real scene buildings */

	for( i=0; i<rs->set->numBuildings; i++ ) {

		bldg = rs->bldg[i];

		glPushMatrix();
		glTranslatef( (float)(( bldg->latitude - rs->set->datumLat )*C_NM2FT*60.0),
			(float)(hmodDeg( bldg->longitude - rs->set->datumLon )*C_NM2FT*60.0*cos( rs->set->datumLat*C_DEG2RAD ) ), 0.0 );
		glRotatef( (float)bldg->angle, 0.0, 0.0, 1.0 );

		for( j=0; j<3; j++ ) /* so all roofs are slightly different colors */
			roofColor[j] = sg->roofColor[j] + 0.01f*i;

		glMaterialfv( GL_FRONT, GL_AMBIENT, roofColor );
		glMaterialfv( GL_FRONT, GL_DIFFUSE, roofColor );
		glBegin( GL_POLYGON );
		glNormal3f( 0.0, 0.0, -1.0 );
		glVertex3f(        0.0,                 0.0,         -(float)bldg->height );
		glVertex3f(        0.0,          (float)bldg->width, -(float)bldg->height );
		glVertex3f( (float)bldg->length, (float)bldg->width, -(float)bldg->height );
		glVertex3f( (float)bldg->length,        0.0,         -(float)bldg->height );
		glEnd();

		glMaterialfv( GL_FRONT, GL_AMBIENT, sg->buildingColor[bldg->color] );
		glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->buildingColor[bldg->color] );
		glBegin( GL_QUAD_STRIP );
		glNormal3f( -1.0, 0.0, 0.0 );
		glVertex3f( 0.0,          (float)bldg->width,         0.0          );
		glVertex3f( 0.0,          (float)bldg->width, -(float)bldg->height );
		glVertex3f( 0.0,                 0.0,                 0.0          );
		glVertex3f( 0.0,                 0.0,         -(float)bldg->height );
		glNormal3f( 0.0, -1.0, 0.0 );
		glVertex3f(        0.0,          0.0,                0.0           );
		glVertex3f(        0.0,          0.0,         -(float)bldg->height );
		glVertex3f( (float)bldg->length, 0.0,                0.0           );
		glVertex3f( (float)bldg->length, 0.0,         -(float)bldg->height );
		glNormal3f( 1.0, 0.0, 0.0 );
		glVertex3f( (float)bldg->length,        0.0,                0.0           );
		glVertex3f( (float)bldg->length,        0.0,         -(float)bldg->height );
		glVertex3f( (float)bldg->length, (float)bldg->width,        0.0           );
		glVertex3f( (float)bldg->length, (float)bldg->width, -(float)bldg->height );
		glNormal3f( 0.0, 1.0, 0.0 );
		glVertex3f( (float)bldg->length, (float)bldg->width,        0.0           );
		glVertex3f( (float)bldg->length, (float)bldg->width, -(float)bldg->height );
		glVertex3f(        0.0,          (float)bldg->width,        0.0           );
		glVertex3f(        0.0,          (float)bldg->width, -(float)bldg->height );
		glEnd();

		glMaterialfv( GL_FRONT, GL_AMBIENT, sg->blackColor );
		glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->blackColor );
		glBegin( GL_QUADS );
		glNormal3f( 0.0, 0.0, 1.0 );
		for( k=1; k<(int)(bldg->height*CF_FT2M)-2; k+=3 ) {
			for( j=1; j<(int)(bldg->width*CF_FT2M)-1; j+=3 ) {
				glVertex3f( -0.5f, CF_M2FT*j,     -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( -0.5f, CF_M2FT*(j+1), -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( -0.5f, CF_M2FT*(j+1), -CF_M2FT*(k+2) );
				glVertex3f( -0.5f, CF_M2FT*j,     -CF_M2FT*(k+2) );

				glVertex3f( (float)bldg->length+0.5f, CF_M2FT*(j+1), -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( (float)bldg->length+0.5f, CF_M2FT*j,     -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( (float)bldg->length+0.5f, CF_M2FT*j,     -CF_M2FT*(k+2) );
				glVertex3f( (float)bldg->length+0.5f, CF_M2FT*(j+1), -CF_M2FT*(k+2) );
			}
			for( j=1; j<(int)(bldg->length*CF_FT2M)-1; j+=3 ) {
				glVertex3f( CF_M2FT*(j+1), -0.5f, -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( CF_M2FT*j,     -0.5f, -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( CF_M2FT*j,     -0.5f, -CF_M2FT*(k+2) );
				glVertex3f( CF_M2FT*(j+1), -0.5f, -CF_M2FT*(k+2) );

				glVertex3f( CF_M2FT*j,     (float)bldg->width+0.5f, -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( CF_M2FT*(j+1), (float)bldg->width+0.5f, -CF_M2FT*(k+1-(k==1&&(!((j-4)%9))?1:0)) );
				glVertex3f( CF_M2FT*(j+1), (float)bldg->width+0.5f, -CF_M2FT*(k+2) );
				glVertex3f( CF_M2FT*j,     (float)bldg->width+0.5f, -CF_M2FT*(k+2) );
			}
		}
		glEnd();

		/* symbol (if shown) */

		if( i == sg->symbolBuilding ) {

			/* symbol */

			if( sc->btexture ) {

				glEnable( GL_TEXTURE_2D );

				glBindTexture( GL_TEXTURE_2D, sc->btexture );

				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

                glPushMatrix();
                    switch( sg->symbolFace ) {
                    default:
                    case 0:
                        break;
                    case 1:
                        glTranslatef( (float)bldg->length, 0, 0 );
                        glRotatef( 90.0, 0, 0, 1 );
                        break;
                    case 2:
                        glTranslatef( (float)bldg->length, (float)bldg->width, 0 );
                        glRotatef( 180.0, 0, 0, 1 );
                        break;
                    case 3:
                        glTranslatef( 0, (float)bldg->width, 0 );
                        glRotatef( 270.0, 0, 0, 1 );
                        break;
                    }
					glBegin( GL_POLYGON );
					glNormal3f( -1.0, 0.0, 0.0 );
					glTexCoord2f( 0, 0 );
					glVertex3f( -0.5f, sg->symbolX-0.5f*CF_M2FT, -sg->symbolH+0.5f*CF_M2FT );
					glTexCoord2f( 1, 0 );
					glVertex3f( -0.5f, sg->symbolX+0.5f*CF_M2FT, -sg->symbolH+0.5f*CF_M2FT );
					glTexCoord2f( 1, 1 );
					glVertex3f( -0.5f, sg->symbolX+0.5f*CF_M2FT, -sg->symbolH-0.5f*CF_M2FT );
					glTexCoord2f( 0, 1 );
					glVertex3f( -0.5f, sg->symbolX-0.5f*CF_M2FT, -sg->symbolH-0.5f*CF_M2FT );
					glEnd();
				glPopMatrix();

				glDisable( GL_TEXTURE_2D );

			}

		}

		if(i == sg->fakeSymbolBuilding) {

			/* fake symbol */

			if( sc->ftexture ) {

				glEnable( GL_TEXTURE_2D );

				glBindTexture( GL_TEXTURE_2D, sc->ftexture );

				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

                glBegin( GL_POLYGON );
                glNormal3f( -1.0, 0.0, 0.0 );
                glTexCoord2f( 0, 0 );
                glVertex3f( -0.5f, sg->fsymbolX-0.5f*CF_M2FT, -sg->fsymbolH+0.5f*CF_M2FT );
                glTexCoord2f( 1, 0 );
                glVertex3f( -0.5f, sg->fsymbolX+0.5f*CF_M2FT, -sg->fsymbolH+0.5f*CF_M2FT );
                glTexCoord2f( 1, 1 );
                glVertex3f( -0.5f, sg->fsymbolX+0.5f*CF_M2FT, -sg->fsymbolH-0.5f*CF_M2FT );
                glTexCoord2f( 0, 1 );
                glVertex3f( -0.5f, sg->fsymbolX-0.5f*CF_M2FT, -sg->fsymbolH-0.5f*CF_M2FT );
                glEnd();

				glDisable( GL_TEXTURE_2D );

			}
		}

		glPopMatrix();

	}

	/* real scene poles */

	glMaterialfv( GL_FRONT, GL_AMBIENT, sg->poleColor );
	glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->poleColor );

	for( i=0; i<rs->set->numPoles; i++ ) {

		pole = rs->pole[i];

		glPushMatrix();
		glTranslatef( (float)(( pole->latitude - rs->set->datumLat )*C_NM2FT*60.0),
			(float)(hmodDeg( pole->longitude - rs->set->datumLon )*C_NM2FT*60.0*cos( rs->set->datumLat*C_DEG2RAD )), 0.0 );

		glBegin( GL_LINES );
		glNormal3f( 0.0, 0.0, -1.0 );
		glVertex3f( 0.0, 0.0, -(float)pole->height );
		glVertex3f( 0.0, 0.0, 0.0 );
		glEnd();

		glPopMatrix();

	}

	/* real scene trees */

	glEndList();

	/* cloud */

	/* read texture */

	/* read texture does the malloc */
	/* converts all textures to rgba, regardless of components return arg */
	glDeleteTextures(1,&sc->ctexture);
	glGenTextures(1,&sc->ctexture);
	if( ( image = (GLubyte *)read_texture( sg->cfile,
		&sg->cwidth, &sg->cheight,
		&sg->ccomponents) ) == 0 ) {
		glDeleteTextures(1,&sc->ctexture);
		sc->ctexture = 0;
    } else {
        glBindTexture( GL_TEXTURE_2D, sc->ctexture );
		glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, sg->cwidth, sg->cheight, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, image );
		free(image);
	}

	if( !glIsList( sg->cloud_dl ) )
		sg->cloud_dl = glGenLists( 1 );
	glNewList( sg->cloud_dl, GL_COMPILE );
	glShadeModel( GL_SMOOTH );

	if( sc->ctexture ) {

        glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->ctexture );

		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

		glBegin( GL_POLYGON );
		glNormal3f( 0.0, 0.0, -1.0 );
		glTexCoord2f( 0, 0 );
		glVertex3f( 1.0, -1.0, 0.0 );
		glTexCoord2f( 1, 0 );
		glVertex3f( 1.0,  1.0, 0.0 );
		glTexCoord2f( 1, 1 );
		glVertex3f( -1.0,  1.0, 0.0 );
		glTexCoord2f( 0, 1 );
		glVertex3f( -1.0, -1.0, 0.0 );
		glEnd();
		glBegin( GL_POLYGON );
		glNormal3f( 0.0, 0.0, -1.0 );
		glTexCoord2f( 0, 0 );
		glVertex3f( 1.0, -1.0, 0.0 );
		glTexCoord2f( 0, 1 );
		glVertex3f( -1.0, -1.0, 0.0 );
		glTexCoord2f( 1, 1 );
		glVertex3f( -1.0,  1.0, 0.0 );
		glTexCoord2f( 1, 0 );
		glVertex3f( 1.0,  1.0, 0.0 );
		glEnd();

		glDisable( GL_TEXTURE_2D );

	}

	glEndList();


    /* runway */

    if( !glIsList( sg->runway_dl ) )
        sg->runway_dl = glGenLists( 1 );
    glNewList( sg->runway_dl, GL_COMPILE );
    glShadeModel( GL_FLAT );

    /* asphalt */
    glMaterialfv( GL_FRONT, GL_AMBIENT, sg->runwayColor );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->runwayColor );
    glBegin( GL_POLYGON );
    glNormal3f( 0.0, 0.0, -1.0 );
    glVertex3f( 0.0, -sg->runwayW*0.5f, 0.0 );
    glVertex3f( 0.0,  sg->runwayW*0.5f, 0.0 );
    glVertex3f( sg->runwayL,  sg->runwayW*0.5f, 0.0 );
    glVertex3f( sg->runwayL, -sg->runwayW*0.5f, 0.0 );
    glEnd();

    glMaterialfv( GL_FRONT, GL_AMBIENT, sg->runwayMarkingsColor );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->runwayMarkingsColor );
    /* center line */
    for( i=1; i<sg->runwayL/sg->stripeL; i+=2 ) {
        glBegin( GL_POLYGON );
        glNormal3f( 0.0, 0.0, -1.0 );
        glVertex3f(               sg->stripeL*i, -sg->stripeW*0.5f, 0.0 );
        glVertex3f(               sg->stripeL*i,  sg->stripeW*0.5f, 0.0 );
        glVertex3f( sg->stripeL + i*sg->stripeL,  sg->stripeW*0.5f, 0.0 );
        glVertex3f( sg->stripeL + i*sg->stripeL, -sg->stripeW*0.5f, 0.0 );
        glEnd();
    }

    glEndList();

    /* Corner Markings for runway */
    if( !glIsList( sg->cornerMarker_dl ) )
        sg->cornerMarker_dl = glGenLists( 1 );
    glNewList( sg->cornerMarker_dl, GL_COMPILE );
    glShadeModel( GL_FLAT );
    glMaterialfv( GL_FRONT, GL_AMBIENT, sg->runwayCornerMarkColor );
    glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->runwayCornerMarkColor );
    glBegin( GL_TRIANGLE_FAN );
    glNormal3f( 0.0,0.0, -1.0 );
    glVertex3f( 0,sg->runwayCornerMarkRadius,-sg->runwayCornerMarkHeight );
    for( i=1; i<20; i++ ) {
        glVertex3f( sg->runwayCornerMarkRadius*(float)sin(C_PI/10.0*i),
            sg->runwayCornerMarkRadius*(float)cos(C_PI/10.0*i),
            -sg->runwayCornerMarkHeight);
    }
    glEnd();

    glBegin( GL_QUAD_STRIP );
    glNormal3f( 0.0,1.0, 0.0 );
    glVertex3f( 0.0, sg->runwayCornerMarkRadius, 0.0 );
    glVertex3f( 0.0, sg->runwayCornerMarkRadius,-sg->runwayCornerMarkHeight );
    for( i=1; i<20; i++ ) {
        glNormal3f( -(float)sin(C_PI/10.0*i),-(float)cos(C_PI/10.0*i),0.0 );
        glVertex3f( sg->runwayCornerMarkRadius*(float)sin(C_PI/10.0*i),
            sg->runwayCornerMarkRadius*(float)cos(C_PI/10.0*i),
            0.0 );
        glVertex3f( sg->runwayCornerMarkRadius*(float)sin(C_PI/10.0*i),
            sg->runwayCornerMarkRadius*(float)cos(C_PI/10.0*i),
            -sg->runwayCornerMarkHeight);
    }
    glNormal3f( 0.0,-1.0, 0.0 );
    glVertex3f( 0.0, sg->runwayCornerMarkRadius, 0.0 );
    glVertex3f( 0.0, sg->runwayCornerMarkRadius,-sg->runwayCornerMarkHeight );
    glEnd();

    glBegin( GL_TRIANGLE_FAN );
    glNormal3f( 0.0,0.0, 1.0 );
    glVertex3f( 0.0, sg->runwayCornerMarkRadius,0.0 );
    for( i=1; i<20; i++ ) {
        glVertex3f( sg->runwayCornerMarkRadius*(float)sin(C_PI/10.0*i),
            sg->runwayCornerMarkRadius*(float)cos(C_PI/10.0*i),
            0.0);
    }
    glEnd();

    glEndList();

	/* GCS symbology */

	/* sphere */

    if( !glIsList( sg->sphereObstacle_dl ) )
        sg->sphereObstacle_dl = glGenLists( 1 );
    glNewList( sg->sphereObstacle_dl, GL_COMPILE );
	glutSolidSphere( sg->sphereObstacleRadius, sg->sphereObstacleSlices, sg->sphereObstacleStacks );
    glEndList();


    /* checkerboard banner light */
    if( !glIsList( sg->bannerObstacle->light_dl) ) {
        sg->bannerObstacle->light_dl = glGenLists( 1 );
    }

    glNewList( sg->bannerObstacle->light_dl, GL_COMPILE );
    glBegin( GL_QUADS );
        // draw every other square for checkboard pattern
        // draw each square clockwise and counter-clockwise so its visible from both sides
        for( j=0; j<(sg->bannerObstacle->height/sg->bannerObstacle->checkerH); j+=1 ) {
            for( i=0; i<(sg->bannerObstacle->width/sg->bannerObstacle->checkerW-(j%2)); i+=2 ) {
                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH );                              // lower left
                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH );                              // lower right
                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper right
                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper left

                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH );                              // lower left
                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper left
                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper right
                glVertex3f( 0.0f, (j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH );                              // lower right
            }
        }
    glEnd();
    glEndList();

    /* checkerboard banner dark */
    if( !glIsList( sg->bannerObstacle->dark_dl) ) {
        sg->bannerObstacle->dark_dl = glGenLists( 1 );
    }

    glNewList( sg->bannerObstacle->dark_dl, GL_COMPILE );
    glBegin( GL_QUADS );
        for( j=0; j<(sg->bannerObstacle->height/sg->bannerObstacle->checkerH); j+=1 ) {
            for( i=0; i<(sg->bannerObstacle->width/sg->bannerObstacle->checkerW-(1-j%2)); i+=2 ) {
                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH );                              // lower left
                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH );                              // lower right
                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper right
                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper left

                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH );                              // lower left
                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW,                              -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper left
                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH-sg->bannerObstacle->checkerH ); // upper right
                glVertex3f( 0.0f, (1-j%2+i)*sg->bannerObstacle->checkerW+sg->bannerObstacle->checkerW, -j*sg->bannerObstacle->checkerH );                              // lower right
            }
        }
    glEnd();
    glEndList();
    
    
    //GTAR AHS
     /* checkerboard banner dark */
    if( !glIsList( sg->gtarAHS->dark_dl) ) {
        sg->gtarAHS->dark_dl = glGenLists( 1 );
    }

    glNewList( sg->gtarAHS->dark_dl, GL_COMPILE );
    glBegin( GL_QUADS );
                  //Lower line
                  glVertex3f(0.0f, 0.0f,0.0f ); //lower left
		  glVertex3f(sg->gtarAHS->width,0.0f,0.0f); //lower right
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->thickness,0.0f); //upper riight
		  glVertex3f(0.0f, sg->gtarAHS->thickness,0.0f); //upper left
		  //lower line drawn in opposite direction to be able to see from both sides
		  glVertex3f(0.0f,0.0f,0.0f); //lowerleft
		  glVertex3f(0.0f,sg->gtarAHS->thickness,0.0f);//upperleft
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->thickness,0.0f); //upper right
		  glVertex3f(sg->gtarAHS->width,0.0f,0.0f); //lower right
		  
		 //left line going up
                  glVertex3f(0.0f, 0.0f,0.0f ); //lower left
		  glVertex3f(sg->gtarAHS->thickness ,0.0f,0.0f); //lower right
		  glVertex3f(sg->gtarAHS->thickness,sg->gtarAHS->height,0.0f); //upper riight
		  glVertex3f(0.0f,sg->gtarAHS->height,0.0f); //upper left
		  //left line drawn in opposite direction to be able to see from both sides
		  glVertex3f(0.0f,0.0f,0.0f); //lowerleft
		  glVertex3f(0.0f,sg->gtarAHS->height,0.0f);//upperleft
		  glVertex3f(sg->gtarAHS->thickness,sg->gtarAHS->height,0.0f); //upper right
		  glVertex3f(sg->gtarAHS->thickness,0.0f,0.0f); //lower right
		  
		 //right line going up
          glVertex3f(sg->gtarAHS->width, 0.0f,0.0f ); //lower right
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->height,0.0f); //upper riight
		  glVertex3f(sg->gtarAHS->width-sg->gtarAHS->thickness,sg->gtarAHS->height,0.0f); //upper left
		  glVertex3f(sg->gtarAHS->width-sg->gtarAHS->thickness,0.0f,0.0f); //lower left
		  
		  //right line drawn in opposite direction to be able to see from both sides
		  glVertex3f(sg->gtarAHS->width, 0.0f,0.0f ); //lower right
		  glVertex3f(sg->gtarAHS->width-sg->gtarAHS->thickness,0.0f,0.0f); //lower left
		  glVertex3f(sg->gtarAHS->width-sg->gtarAHS->thickness,sg->gtarAHS->height,0.0f); //upper left
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->height,0.0f); //upper riight
		  
		  //upper line
		  glVertex3f(0.0f,sg->gtarAHS->height,0.0f); //upper left
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->height,0.0f ); //upper right
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->height-sg->gtarAHS->thickness,0.0f ); //lower right
		  glVertex3f(0.0f,sg->gtarAHS->height-sg->gtarAHS->thickness,0.0f ); //lower left
		  //upper line in reverse
		  glVertex3f(0.0f,sg->gtarAHS->height,0.0f); //upper left
		  glVertex3f(0.0f,sg->gtarAHS->height-sg->gtarAHS->thickness,0.0f ); //lower left
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->height-sg->gtarAHS->thickness,0.0f ); //lower right
		  glVertex3f(sg->gtarAHS->width,sg->gtarAHS->height,0.0f ); //upper right
		  
		  
          
    glEnd();
    glEndList();
    
    

    //loadOpenGL2DTextureBMP( sg->pictureObstacle->filename, &(sc->potexture), GL_RGB );

    /* picture obstacle */
    if( !glIsList( sg->pictureObstacle->obstacle_dl ) ) {
        sg->pictureObstacle->obstacle_dl = glGenLists( 1 );
    }
    glNewList( sg->pictureObstacle->obstacle_dl, GL_COMPILE );

    glShadeModel( GL_SMOOTH );

    glDisable( GL_DEPTH_TEST );

	if( sc->potexture ) {
        glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->potexture );
        glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

        glBegin( GL_QUADS );
            //glNormal3f( 1.0, 0.0, 0.0 );
            glTexCoord2f( 1.0, 0.0 );
            glVertex3f( 0.0, 0.0, 0.0 );
            glTexCoord2f( 1.0, 1.0 );
            glVertex3f( 0.0, 0.0, -sg->pictureObstacle->height );
            glTexCoord2f( 0.0, 1.0 );
            glVertex3f( 0.0, sg->pictureObstacle->width, -sg->pictureObstacle->height );
            glTexCoord2f( 0.0, 0.0 );
            glVertex3f( 0.0, sg->pictureObstacle->width, 0.0 );

            glTexCoord2f( 1.0, 0.0 );
            glVertex3f( 0.0, 0.0, 0.0 );
            glTexCoord2f( 0.0, 0.0 );
            glVertex3f( 0.0, sg->pictureObstacle->width, 0.0 );
            glTexCoord2f( 0.0, 1.0 );
            glVertex3f( 0.0, sg->pictureObstacle->width, -sg->pictureObstacle->height );
            glTexCoord2f( 1.0, 1.0 );
            glVertex3f( 0.0, 0.0, -sg->pictureObstacle->height );
        glEnd();

        glDisable( GL_TEXTURE_2D );
    }

	glEnable( GL_DEPTH_TEST );

    glEndList();

    /* gtar desk */
    //loadOpenGL2DTextureBMP( sg->gtarDesk->filename, &(sc->gdtexture), GL_RGB );

    if( !glIsList( sg->gtarDesk->dl ) ) {
        sg->gtarDesk->dl = glGenLists( 1 );
    }
    glNewList( sg->gtarDesk->dl, GL_COMPILE );

    glShadeModel( GL_SMOOTH );

	/* desk itself */
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->gtarDesk->deskColor );
	glBegin( GL_QUADS );
	glNormal3f( 0, -1, 0 );
	glVertex3f( 0, 0, 0 );
	glVertex3f( 0, 0, -sg->gtarDesk->deskHeight );
	glVertex3f( sg->gtarDesk->deskDepth, 0, -sg->gtarDesk->deskHeight );
	glVertex3f( sg->gtarDesk->deskDepth, 0, 0 );
	glNormal3f( 0, 1, 0 );
	glVertex3f( 0, sg->gtarDesk->deskLength, -sg->gtarDesk->deskHeight );
	glVertex3f( 0, sg->gtarDesk->deskLength, 0 );
	glVertex3f( sg->gtarDesk->deskDepth, sg->gtarDesk->deskLength, 0 );
	glVertex3f( sg->gtarDesk->deskDepth, sg->gtarDesk->deskLength, -sg->gtarDesk->deskHeight );
	glNormal3f( 1, 0, 0 );
	glVertex3f( sg->gtarDesk->deskDepth, 0, 0 );
	glVertex3f( sg->gtarDesk->deskDepth, 0, -sg->gtarDesk->deskHeight );
	glVertex3f( sg->gtarDesk->deskDepth, sg->gtarDesk->deskLength, -sg->gtarDesk->deskHeight );
	glVertex3f( sg->gtarDesk->deskDepth, sg->gtarDesk->deskLength, 0 );
	glNormal3f( -1, 0, 0 );
	glVertex3f( 0, 0, -sg->gtarDesk->deskHeight );
	glVertex3f( 0, 0, 0 );
	glVertex3f( 0, sg->gtarDesk->deskLength, 0 );
	glVertex3f( 0, sg->gtarDesk->deskLength, -sg->gtarDesk->deskHeight );
	glNormal3f( 0, 0, -1 );
	glVertex3f( 0, 0, -sg->gtarDesk->deskHeight );
	glVertex3f( 0, sg->gtarDesk->deskLength, -sg->gtarDesk->deskHeight );
	glVertex3f( sg->gtarDesk->deskDepth, sg->gtarDesk->deskLength, -sg->gtarDesk->deskHeight );
	glVertex3f( sg->gtarDesk->deskDepth, 0, -sg->gtarDesk->deskHeight );
	glEnd();

	/* box on desk */
	glPushMatrix();

	glTranslatef( sg->gtarDesk->boxLocation[0], sg->gtarDesk->boxLocation[1], -sg->gtarDesk->deskHeight );
	glRotatef( sg->gtarDesk->boxAngle, 0, 0, 1 );

    glDisable( GL_DEPTH_TEST );

	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->gtarDesk->boxColor );
	glBegin( GL_QUADS );
	glNormal3f( 0, 0, -1 );
	glVertex3f( 0, 0, 0 );
	glVertex3f( 0, sg->gtarDesk->boxWidth, 0 );
	glVertex3f( sg->gtarDesk->boxLength, sg->gtarDesk->boxWidth, 0 );
	glVertex3f( sg->gtarDesk->boxLength, 0, 0 );
	glEnd();

	/* memory stick */
    if( sc->gdtexture ) {
		glPushMatrix();
		glTranslatef( sg->gtarDesk->stickLocation[0], sg->gtarDesk->stickLocation[1], 0 );
		glRotatef( sg->gtarDesk->stickAngle, 0, 0, 1 );
        glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->gdtexture );
        glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

        glBegin( GL_QUADS );
			glNormal3f( 0, 0, -1 );
            glTexCoord2f( 1, 1 );
            glVertex3f( sg->gtarDesk->stickSize, 0, 0 );
            glTexCoord2f( 1, 0 );
            glVertex3f( 0, 0, 0 );
            glTexCoord2f( 0, 0 );
            glVertex3f( 0, sg->gtarDesk->stickSize, 0 );
            glTexCoord2f( 0, 1 );
            glVertex3f( sg->gtarDesk->stickSize, sg->gtarDesk->stickSize, 0 );
        glEnd();

        glDisable( GL_TEXTURE_2D );
		glPopMatrix();
    }

	glEnable( GL_DEPTH_TEST );

	glPopMatrix();

    glEndList();


	/* gtar switch post */
	//loadOpenGL2DTextureBMP( sg->gtarswitchpost->filename, &(sc->gptexture), GL_RGB );

    if( !glIsList( sg->gtarswitchpost->dl ) ) {
        sg->gtarswitchpost->dl = glGenLists( 1 );
    }
    glNewList( sg->gtarswitchpost->dl, GL_COMPILE );

    glShadeModel( GL_SMOOTH );

	/* post itself */
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->gtarswitchpost->postColor );
	glBegin( GL_QUADS );
	glNormal3f( 0, -1, 0 );
	glVertex3f( 0, 0, 0 );
	glVertex3f( 0, 0, -sg->gtarswitchpost->postHeight );
	glVertex3f( sg->gtarswitchpost->postDepth, 0, -sg->gtarswitchpost->postHeight );
	glVertex3f( sg->gtarswitchpost->postDepth, 0, 0 );
	glNormal3f( 0, 1, 0 );
	glVertex3f( 0, sg->gtarswitchpost->postLength, -sg->gtarswitchpost->postHeight );
	glVertex3f( 0, sg->gtarswitchpost->postLength, 0 );
	glVertex3f( sg->gtarswitchpost->postDepth, sg->gtarswitchpost->postLength, 0 );
	glVertex3f( sg->gtarswitchpost->postDepth, sg->gtarswitchpost->postLength, -sg->gtarswitchpost->postHeight );
	glNormal3f( 1, 0, 0 );
	glVertex3f( sg->gtarswitchpost->postDepth, 0, 0 );
	glVertex3f( sg->gtarswitchpost->postDepth, 0, -sg->gtarswitchpost->postHeight );
	glVertex3f( sg->gtarswitchpost->postDepth, sg->gtarswitchpost->postLength, -sg->gtarswitchpost->postHeight );
	glVertex3f( sg->gtarswitchpost->postDepth, sg->gtarswitchpost->postLength, 0 );
	glNormal3f( -1, 0, 0 );
	glVertex3f( 0, 0, -sg->gtarswitchpost->postHeight );
	glVertex3f( 0, 0, 0 );
	glVertex3f( 0, sg->gtarswitchpost->postLength, 0 );
	glVertex3f( 0, sg->gtarswitchpost->postLength, -sg->gtarswitchpost->postHeight );
	glNormal3f( 0, 0, -1 );
	glVertex3f( 0, 0, -sg->gtarswitchpost->postHeight );
	glVertex3f( 0, sg->gtarswitchpost->postLength, -sg->gtarswitchpost->postHeight );
	glVertex3f( sg->gtarswitchpost->postDepth, sg->gtarswitchpost->postLength, -sg->gtarswitchpost->postHeight );
	glVertex3f( sg->gtarswitchpost->postDepth, 0, -sg->gtarswitchpost->postHeight );
	glEnd();

	/* box on post */
	glPushMatrix();

	glTranslatef( sg->gtarswitchpost->boxLocation[0], sg->gtarswitchpost->boxLocation[1], -sg->gtarswitchpost->postHeight );
	glRotatef( sg->gtarswitchpost->boxAngle, 0, 0, 1 );

    glDisable( GL_DEPTH_TEST );

	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->gtarswitchpost->boxColor );
	glBegin( GL_QUADS );
	glNormal3f( 0, -1, 0 );
	glVertex3f( 0, 0, 0 );
	glVertex3f( -sg->gtarswitchpost->boxWidth, 0, 0 );
	glVertex3f( -sg->gtarswitchpost->boxWidth, 0, -sg->gtarswitchpost->boxLength );
	glVertex3f( 0, 0, -sg->gtarswitchpost->boxLength );
	glEnd();

	/* memory stick */
    if( sc->gptexture ) {
		glPushMatrix();
		glTranslatef( sg->gtarswitchpost->stickLocation[0], sg->gtarswitchpost->stickLocation[1], 0 );
		glRotatef( sg->gtarswitchpost->stickAngle, 0, 0, 1 );
        glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->gptexture );
        glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

        glBegin( GL_QUADS );
			glNormal3f( 0, 1, 0);
/*            glTexCoord2f( 1, 1 );
            glVertex3f( sg->gtarswitchpost->stickSize, 0, 0 );
            glTexCoord2f( 1, 0 );
            glVertex3f( 0, 0, 0 );
            glTexCoord2f( 0, 0 );
            glVertex3f( 0, sg->gtarswitchpost->stickSize, 0 );
            glTexCoord2f( 0, 1 );
            glVertex3f( sg->gtarswitchpost->stickSize, sg->gtarswitchpost->stickSize, 0 );
        glEnd(); */

		    glTexCoord2f( 1, -1 );
            glVertex3f( -sg->gtarswitchpost->stickSize, 0, sg->gtarswitchpost->stickSize );
            glTexCoord2f( 1, 0 );
            glVertex3f( -sg->gtarswitchpost->stickSize, 0, 0 );
            glTexCoord2f( 0, 0 );
            glVertex3f( 0, 0, 0 );
            glTexCoord2f( 0, -1 );
            glVertex3f( 0, 0, sg->gtarswitchpost->stickSize);
        glEnd();

        glDisable( GL_TEXTURE_2D );
		glPopMatrix();
    }

	glEnable( GL_DEPTH_TEST );
	glPopMatrix();
    glEndList();


    /* gtar rover */
    ////loadOpenGL2DTextureBMP( sg->gtarRover->filename, &(sc->grtexture), GL_RGB );

    if( !glIsList( sg->gtarRover->dl ) ) {
        sg->gtarRover->dl = glGenLists( 1 );
    }
    glNewList( sg->gtarRover->dl, GL_COMPILE );

    glShadeModel( GL_SMOOTH );

    glPushMatrix();
	/*	glTranslatef( sg->gtarRover->boxLocation[0], sg->gtarRover->boxLocation[1], -sg->gtarRover->deskHeight );
	glRotatef( sg->gtarRover->boxAngle, 0, 0, 1 );*/

    glDisable( GL_DEPTH_TEST );

	/* rover part*/
    if( sc->grtexture ) {
		glPushMatrix();
		glTranslatef( sg->gtarRover->roverLocation[0], sg->gtarRover->roverLocation[1], 0 );
		glRotatef( sg->gtarRover->roverAngle, 0, 0, 1 );
		glEnable( GL_TEXTURE_2D );

		glBindTexture( GL_TEXTURE_2D, sc->grtexture );
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

		glBegin( GL_QUADS );
		glNormal3f( 0, 0, -1 );
		glTexCoord2f( 1, 1 );
		glVertex3f( sg->gtarRover->roverSize, 0, 0 );
		glTexCoord2f( 1, 0 );
		glVertex3f( 0, 0, 0 );
		glTexCoord2f( 0, 0 );
		glVertex3f( 0, sg->gtarRover->roverSize, 0 );
		glTexCoord2f( 0, 1 );
		glVertex3f( sg->gtarRover->roverSize, sg->gtarRover->roverSize, 0 );
		glEnd();

		glDisable( GL_TEXTURE_2D );
		glPopMatrix();
    }

	glEnable( GL_DEPTH_TEST );

	glPopMatrix();

	glEndList();

    //AHS 2 //
    ////loadOpenGL2DTextureBMP( sg->gtarAHS2->filename, &(sc->ahstexture), GL_RGB );

    if( !glIsList( sg->gtarAHS2->dl ) ) {
        sg->gtarAHS2->dl = glGenLists( 1 );
    }
    glNewList( sg->gtarAHS2->dl, GL_COMPILE );

    glShadeModel( GL_SMOOTH );

    glPushMatrix();
	//	glTranslatef( sg->gtarRover->boxLocation[0], sg->gtarRover->boxLocation[1], -sg->gtarRover->deskHeight );
	//glRotatef( sg->gtarRover->boxAngle, 0, 0, 1 );//

    glDisable( GL_DEPTH_TEST );

	//AHS2
    if( sc->ahstexture ) {
		glPushMatrix();
        glTranslatef( sg->gtarAHS2->roverLocation[0], sg->gtarAHS2->roverLocation[1], 0 );
        glRotatef( sg->gtarAHS2->roverAngle, 0, 0, 1 );
		glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->ahstexture );
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

		glBegin( GL_QUADS );
		glNormal3f( 0, 0, -1 );
		glTexCoord2f( 1, 1 );
        glVertex3f( sg->gtarAHS2->roverSize, 0, 0 );
		glTexCoord2f( 1, 0 );
		glVertex3f( 0, 0, 0 );
		glTexCoord2f( 0, 0 );
        glVertex3f( 0, sg->gtarAHS2->roverSize, 0 );
		glTexCoord2f( 0, 1 );
        glVertex3f( sg->gtarAHS2->roverSize, sg->gtarAHS2->roverSize, 0 );
		glEnd();

		glDisable( GL_TEXTURE_2D );
		glPopMatrix();
    }

	glEnable( GL_DEPTH_TEST );

	glPopMatrix();

	glEndList();

    // AHS 3 target //
    //AHS 3 //
    ////loadOpenGL2DTextureBMP( sg->gtarAHS3->filename, &(sc->ahstexture2), GL_RGB );

    if( !glIsList( sg->gtarAHS3->dl ) ) {
        sg->gtarAHS3->dl = glGenLists( 1 );
    }
    glNewList( sg->gtarAHS3->dl, GL_COMPILE );

    glShadeModel( GL_SMOOTH );

    glPushMatrix();

    glDisable( GL_DEPTH_TEST );

    //AHS3
    if( sc->ahstexture2 ) {
        glPushMatrix();
        glTranslatef( sg->gtarAHS3->roverLocation[0], sg->gtarAHS3->roverLocation[1], 0 );
        glRotatef( sg->gtarAHS3->roverAngle, 0, 0, 1 );
        glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->ahstexture2 );
        glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

        glBegin( GL_QUADS );
        glNormal3f( 0, 0, -1 );
        glTexCoord2f( 1, 1 );
        glVertex3f( sg->gtarAHS3->roverSize, 0, 0 );
        glTexCoord2f( 1, 0 );
        glVertex3f( 0, 0, 0 );
        glTexCoord2f( 0, 0 );
        glVertex3f( 0, sg->gtarAHS3->roverSize, 0 );
        glTexCoord2f( 0, 1 );
        glVertex3f( sg->gtarAHS3->roverSize, sg->gtarAHS3->roverSize, 0 );
        glEnd();

        glDisable( GL_TEXTURE_2D );
        glPopMatrix();
    }

    glEnable( GL_DEPTH_TEST );

    glPopMatrix();

    glEndList();

	//AHS 4 target //
    ////loadOpenGL2DTextureBMP( sg->gtarAHS4->filename, &(sc->ahstexture3), GL_RGB );

    if( !glIsList( sg->gtarAHS4->dl ) ) {
        sg->gtarAHS4->dl = glGenLists( 1 );
    }
    glNewList( sg->gtarAHS4->dl, GL_COMPILE );

    glShadeModel( GL_SMOOTH );

    glPushMatrix();
    //	glTranslatef( sg->gtarRover->boxLocation[0], sg->gtarRover->boxLocation[1], -sg->gtarRover->deskHeight );
    //glRotatef( sg->gtarRover->boxAngle, 0, 0, 1 );//

    glDisable( GL_DEPTH_TEST );

    //AHS4
    if( sc->ahstexture3 ) {
        glPushMatrix();
        glTranslatef( sg->gtarAHS4->roverLocation[0], sg->gtarAHS4->roverLocation[1], 0 );
        glRotatef( sg->gtarAHS4->roverAngle, 0, 0, 1 );
        glEnable( GL_TEXTURE_2D );

        glBindTexture( GL_TEXTURE_2D, sc->ahstexture3 );
        glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );

        glBegin( GL_QUADS );
        glNormal3f( 0, 0, -1 );
        glTexCoord2f( 1, 1 );
        glVertex3f( sg->gtarAHS4->roverSize, 0, 0 );
        glTexCoord2f( 1, 0 );
        glVertex3f( 0, 0, 0 );
        glTexCoord2f( 0, 0 );
        glVertex3f( 0, sg->gtarAHS4->roverSize, 0 );
        glTexCoord2f( 0, 1 );
        glVertex3f( sg->gtarAHS4->roverSize, sg->gtarAHS4->roverSize, 0 );
        glEnd();

        glDisable( GL_TEXTURE_2D );
        glPopMatrix();
    }

    glEnable( GL_DEPTH_TEST );

    glPopMatrix();

    glEndList();

	/* roomba targets*/
	if( !glIsList( sg->roombaTargets->dl ) ) {
        sg->roombaTargets->dl = glGenLists( 1 );
    }
	
    glNewList( sg->roombaTargets->dl, GL_COMPILE );

	glShadeModel( GL_SMOOTH );
	
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->roombaTargets->color );
	glBegin( GL_POLYGON );
	glNormal3f( 0, 0, -1 );
	for( angle = 0.0; angle < 360.0; angle += 5.0 ) {
		glVertex3f( (float)cos( angle*C_DEG2RAD )*sg->roombaTargets->radius, -(float)sin( angle*C_DEG2RAD )*sg->roombaTargets->radius, -sg->roombaTargets->height );
	}
	glEnd();

	glBegin( GL_QUAD_STRIP );
	for( angle = 0.0; angle < 361.0; angle += 5.0 ) {
		glNormal3f( (float)cos( angle*C_DEG2RAD ), -(float)sin( angle*C_DEG2RAD ), 0 );
		glVertex3f( (float)cos( angle*C_DEG2RAD )*sg->roombaTargets->radius, -(float)sin( angle*C_DEG2RAD )*sg->roombaTargets->radius, -sg->roombaTargets->height );
		glVertex3f( (float)cos( angle*C_DEG2RAD )*sg->roombaTargets->radius, -(float)sin( angle*C_DEG2RAD )*sg->roombaTargets->radius, 0 );
	}
	glEnd();
	glEndList();
	

	/* gtar pylns */

    if( !glIsList( sg->gtarPylns->dl ) ) {
        sg->gtarPylns->dl = glGenLists( 1 );
    }
    glNewList( sg->gtarPylns->dl, GL_COMPILE );

	glShadeModel( GL_SMOOTH );
	glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->gtarPylns->color );

	glBegin( GL_POLYGON );
	glNormal3f( 0, 0, -1 );
	for( angle = 0.0; angle < 360.0; angle += 5.0 ) {
		glVertex3f( (float)cos( angle*C_DEG2RAD )*sg->gtarPylns->radius, -(float)sin( angle*C_DEG2RAD )*sg->gtarPylns->radius, -sg->gtarPylns->height );
	}
	glEnd();

	glBegin( GL_QUAD_STRIP );
	for( angle = 0.0; angle < 361.0; angle += 5.0 ) {
		glNormal3f( (float)cos( angle*C_DEG2RAD ), -(float)sin( angle*C_DEG2RAD ), 0 );
		glVertex3f( (float)cos( angle*C_DEG2RAD )*sg->gtarPylns->radius, -(float)sin( angle*C_DEG2RAD )*sg->gtarPylns->radius, -sg->gtarPylns->height );
		glVertex3f( (float)cos( angle*C_DEG2RAD )*sg->gtarPylns->radius, -(float)sin( angle*C_DEG2RAD )*sg->gtarPylns->radius, 0 );
	}
	glEnd();

	glEndList();

	/* circle */

	if( !glIsList( sg->circle_dl ) )
		sg->circle_dl = glGenLists( 1 );
	glNewList( sg->circle_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	glBegin( GL_LINE_STRIP );
    for( i=0; i<=360; i+= 9 )
		glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
	glEnd();

	glEndList();

	/* generic waypoint */

	if( !glIsList( sg->waypoint_dl ) )
		sg->waypoint_dl = glGenLists( 1 );
	glNewList( sg->waypoint_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	if( sg->waypointThickness > 0 ) {
		glBegin( GL_QUAD_STRIP );
		for( i=0; i<=360; i+= 9 ) {
			glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
			glVertex3f( (float)cos( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), (float)sin( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), 0 );
		}
		glEnd();
		glBegin( GL_QUAD_STRIP );
		for( i=0; i<=360; i+= 9 ) {
			glVertex3f( (float)cos( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), (float)sin( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), 0 );
			glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
		}
		glEnd();
	}

	glBegin( GL_LINE_STRIP );
    for( i=0; i<=360; i+= 9 )
		glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
	glEnd();

	glEndList();

	/* generic waypoint with heading marker */

	if( !glIsList( sg->waypointH_dl ) )
		sg->waypointH_dl = glGenLists( 1 );
	glNewList( sg->waypointH_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	if( sg->waypointThickness > 0 ) {
		glBegin( GL_QUAD_STRIP );
		for( i=0; i<=360; i+= 9 ) {
			glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
			glVertex3f( (float)cos( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), (float)sin( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), 0 );
		}
		glEnd();

		glBegin( GL_QUAD_STRIP );
		for( i=0; i<=360; i+= 9 ) {
			glVertex3f( (float)cos( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), (float)sin( C_DEG2RAD*i )*( 1 - sg->waypointThickness ), 0 );
			glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
		}
		glEnd();
	}

	glBegin( GL_LINE_STRIP );
    for( i=0; i<=360; i+= 9 )
		glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
	glEnd();

	glBegin( GL_TRIANGLES );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -1.5f, +0.25f/.86f, 0 );
	glVertex3f( -1.5f, -0.25f/.86f, 0 );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -1.5f, -0.25f/.86f, 0 );
	glVertex3f( -1.5f, +0.25f/.86f, 0 );
	glEnd();

	glBegin( GL_LINE_LOOP );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -1.5f, +0.25f/.86f, 0 );
	glVertex3f( -1.5f, -0.25f/.86f, 0 );
	glEnd();

	glEndList();

	/* generic waypoint with unfilled heading marker */

	if( !glIsList( sg->waypointH2_dl ) )
		sg->waypointH2_dl = glGenLists( 1 );
	glNewList( sg->waypointH2_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	if( sg->waypointThickness > 0 ) {
		glBegin( GL_QUAD_STRIP );
		for( i=-180+9; i<=180-9; i+= 9 ) {
			glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
			glVertex3f( (float)cos( C_DEG2RAD*i )*( 1 + sg->waypointThickness ), (float)sin( C_DEG2RAD*i )*( 1 + sg->waypointThickness ), 0 );
		}
		glEnd();

		glBegin( GL_QUAD_STRIP );
		for( i=-180+9; i<=180-9; i+= 9 ) {
			glVertex3f( (float)cos( C_DEG2RAD*i )*( 1 + sg->waypointThickness ), (float)sin( C_DEG2RAD*i )*( 1 + sg->waypointThickness ), 0 );
			glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
		}
		glEnd();

		/* I will be the first to admit the tail of this got complicated... */
		glBegin( GL_QUAD_STRIP );
		glVertex3f( (float)cos( C_DEG2RAD*(180-9) ), (float)sin( C_DEG2RAD*(180-9) ), 0 );
		glVertex3f( (float)cos( C_DEG2RAD*(180-9) )*( 1 + sg->waypointThickness ), (float)sin( C_DEG2RAD*(180-9) )*( 1 + sg->waypointThickness ), 0 );
		glVertex3f( -1, 0, 0 );
		glVertex3f( -1 - sg->waypointThickness, sg->waypointThickness*1.5f, 0 );
		glVertex3f( -1.5f, +0.25f/.86f, 0 );
		glVertex3f( -1.5f - sg->waypointThickness, +0.25f/.86f + sg->waypointThickness*1.5f, 0 );
		glVertex3f( -1.5f, -0.25f/.86f, 0 );
		glVertex3f( -1.5f - sg->waypointThickness, -0.25f/.86f - sg->waypointThickness*1.5f, 0 );
		glVertex3f( -1, 0, 0 );
		glVertex3f( -1 - sg->waypointThickness, -sg->waypointThickness*1.5f, 0 );
		glVertex3f( (float)cos( C_DEG2RAD*(-180+9) ), (float)sin( C_DEG2RAD*(-180+9) ), 0 );
		glVertex3f( (float)cos( C_DEG2RAD*(-180+9) )*( 1 + sg->waypointThickness ), (float)sin( C_DEG2RAD*(-180+9) )*( 1 + sg->waypointThickness ), 0 );
		glVertex3f( -1, 0, 0 );
		glVertex3f( -1 - sg->waypointThickness, -sg->waypointThickness*1.5f, 0 );
		glVertex3f( -1.5f, -0.25f/.86f, 0 );
		glVertex3f( -1.5f - sg->waypointThickness, -0.25f/.86f - sg->waypointThickness*1.5f, 0 );
		glVertex3f( -1.5f, +0.25f/.86f, 0 );
		glVertex3f( -1.5f - sg->waypointThickness, +0.25f/.86f + sg->waypointThickness*1.5f, 0 );
		glVertex3f( -1, 0, 0 );
		glVertex3f( -1 - sg->waypointThickness, sg->waypointThickness*1.5f, 0 );
		glVertex3f( (float)cos( C_DEG2RAD*(180-9) ), (float)sin( C_DEG2RAD*(180-9) ), 0 );
		glVertex3f( (float)cos( C_DEG2RAD*(180-9) )*( 1 + sg->waypointThickness ), (float)sin( C_DEG2RAD*(180-9) )*( 1 + sg->waypointThickness ), 0 );
		glEnd();
	}

	glBegin( GL_LINE_STRIP );
    for( i=0; i<=360; i+= 9 )
		glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
	glEnd();

	glBegin( GL_LINE_LOOP );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -1.5f, +0.25f/.86f, 0 );
	glVertex3f( -1.5f, -0.25f/.86f, 0 );
	glEnd();

	glEndList();

	/* circle with "plus" (pointPos) */

	if( !glIsList( sg->waypointP_dl ) )
		sg->waypointP_dl = glGenLists( 1 );
	glNewList( sg->waypointP_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	glBegin( GL_LINE_STRIP );
    for( i=0; i<=360; i+= 9 )
		glVertex3f( (float)cos( C_DEG2RAD*i ), (float)sin( C_DEG2RAD*i ), 0 );
	glEnd();
	glBegin( GL_LINES );
	glVertex3f( +1, 0, 0 );
	glVertex3f( +0.5f, 0, 0 );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -0.5f, 0, 0 );
	glVertex3f( 0, -0.5f, 0 );
	glVertex3f( 0, -1, 0 );
	glVertex3f( 0, +0.5f, 0 );
	glVertex3f( 0, +1, 0 );
	glVertex3f( 0, 0, 0 );
	glVertex3f( 0, 0, 0 );
	glEnd();

	glEndList();

	/* leader marker */

	if( !glIsList( sg->leader_dl ) )
		sg->leader_dl = glGenLists( 1 );
	glNewList( sg->leader_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	glBegin( GL_LINE_LOOP );
	glVertex3f( +1, 0, 0 );
	glVertex3f( -1, +1, 0 );
	glVertex3f( -1, -1, 0 );
	glEnd();
	glBegin( GL_LINE_LOOP );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -1.5f, +0.25f/.86f, 0 );
	glVertex3f( -1.5f, -0.25f/.86f, 0 );
	glEnd();

	glEndList();

	/* datum marker */

	if( !glIsList( sg->datum_dl ) )
		sg->datum_dl = glGenLists( 1 );
	glNewList( sg->datum_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	glBegin( GL_LINES );
	glVertex3f( +1.5f, 0, 0 );
	glVertex3f( +0.5f, 0, 0 );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -0.5f, 0, 0 );
	glVertex3f( 0, -0.5f, 0 );
	glVertex3f( 0, -1, 0 );
	glVertex3f( 0, +0.5f, 0 );
	glVertex3f( 0, +1, 0 );
	glEnd();

	glEndList();

	/* traffic (airplane) */

	if( !glIsList( sg->traffic_dl ) )
		sg->traffic_dl = glGenLists( 1 );
	glNewList( sg->traffic_dl, GL_COMPILE );
	glShadeModel( GL_FLAT );

	glBegin( GL_LINES );
	glVertex3f( 1, 0, 0 );
	glVertex3f( -1, 0, 0 );
	glVertex3f( -0.2f, -0.9f, 0 );
	glVertex3f( 0.2f, 0, 0 );
	glVertex3f( 0.2f, 0, 0 );
	glVertex3f( -0.2f, +0.9f, 0 );
	glVertex3f( -1, -0.3f, 0 );
	glVertex3f( -0.85f, 0, 0 );
	glVertex3f( -0.85f, 0, 0 );
	glVertex3f( -1, +0.3f, 0 );
	glVertex3f( -0.85f, 0, 0 );
	glVertex3f( -1, 0, -0.3f );
	glEnd();

	glEndList();

	/* Random Polygons */
	// Display lists are static, so a dl is not a good choice for something that changes with time...

	if( !glIsList( sg->randPolygons->randPolygons_dl ) ){
		sg->randPolygons->randPolygons_dl = glGenLists( 1 );
	}
	glNewList( sg->randPolygons->randPolygons_dl, GL_COMPILE );
		glShadeModel( GL_SMOOTH );
		glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->randPolygons->color );

		/*for(j=0; j<sg->randPolygons->numPolygons; j++){
			glBegin(GL_TRIANGLES);
			glNormal3f( 0.0, 0.0, -1.0 );
			glVertex3f(sg->randPolygons->randomSeed[0][j],
					   sg->randPolygons->randomSeed[1][j] + sg->randPolygons->randRadius,
					   0);
			glVertex3f((float)(sg->randPolygons->randomSeed[0][j] + sg->randPolygons->randRadius*((sqrt(3.0f)/2.0f))),
					   (float)(sg->randPolygons->randomSeed[1][j] + sg->randPolygons->randRadius*(-1.0f/2.0f)),
					   0);
			glVertex3f((float)(sg->randPolygons->randomSeed[0][j] + sg->randPolygons->randRadius*(-(sqrt(3.0f)/2.0f))),
					   (float)(sg->randPolygons->randomSeed[1][j] + sg->randPolygons->randRadius*(-1.0f/2.0f)),
					   0);
			glEnd();
		} */
			glBegin(GL_TRIANGLES);
			glNormal3f( 0.0, 0.0, -1.0 );
			glVertex3f(0,sg->randPolygons->randRadius, 0);
			glVertex3f((float)(sg->randPolygons->randRadius*((sqrt(3.0f)/2.0f))),(float)(sg->randPolygons->randRadius*(-1.0f/2.0f)),0);
			glVertex3f((float)(sg->randPolygons->randRadius*(-(sqrt(3.0f)/2.0f))),(float)(sg->randPolygons->randRadius*(-1.0f/2.0f)),0);
			glEnd();
	glEndList();



	/* video camgrab */

	glDeleteTextures( 1, &sc->vtexture );
	glGenTextures( 1, &sc->vtexture );

	/* now load up video test pattern */

	//loadOpenGL2DTextureBMP( sg->tpfile, &sc->tptexture, GL_RGB );

	/* do HUD display lists */

	initHUDDL( sg );

}


static void swapChar( char *one, char *two ) {

	char hold;

	hold = *one;
	*one = *two;
	*two = hold;

}


static void swapFloat( float *one, float *two ) {

	float hold;

	hold = *one;
	*one = *two;
	*two = hold;

}


static void finishSwapScenePIP( struct scene_ref *sc ) {

	struct gcsInstance_ref *gi = gcsActiveInstance( &gcs );
	struct gcsScene_ref *gsc = whichGcsScene( sc, gi ), *gscPIP = gi->set->scenePIP;

	sc->animatePIP = 0;

	swapChar( &(gsc->viewMode),    &(gscPIP->viewMode) );
	swapChar( &(sc->lookat),       &(scenePIP.lookat) );
	swapChar( &(gsc->videoMode),   &(gscPIP->videoMode) );
	swapChar( &(sc->showTruth),    &(scenePIP.showTruth) );
	swapChar( &(sc->showGCS),      &(scenePIP.showGCS) );
	swapChar( &(gsc->showTex),     &(gscPIP->showTex) );
	swapChar( &(sc->hudAlternate), &(scenePIP.hudAlternate) );
	swapChar( &(sc->showGrid),     &(scenePIP.showGrid) );
	swapChar( &(sc->showTrack),    &(scenePIP.showTrack) );
	swapChar( &(sc->showMag),      &(scenePIP.showMag) );
	swapChar( &(sc->showBodyAxes), &(scenePIP.showBodyAxes) );
	swapChar( &(sc->showLgAxes),   &(scenePIP.showLgAxes) );
	swapChar( &(sc->showVel),      &(scenePIP.showVel) );
	swapChar( &(sc->showOmega),    &(scenePIP.showOmega) );
	swapChar( &(sc->showAngMom),   &(scenePIP.showAngMom) );
	swapChar( &(sc->showTraj),     &(scenePIP.showTraj) );
	swapChar( &(sc->showLabels),   &(scenePIP.showLabels) );
	swapChar( &(sc->showRawGps),   &(scenePIP.showRawGps) );
	swapChar( &(sc->showGpsTraj),  &(scenePIP.showGpsTraj) );
	swapChar( &(sc->showSVInfo),   &(scenePIP.showSVInfo) );
	swapChar( &(sc->showThreats),  &(scenePIP.showThreats) );
	swapChar( &(sc->showCloud),    &(scenePIP.showCloud) );
	swapChar( &(sc->showNavIP),    &(scenePIP.showNavIP) );
	swapChar( &(sc->showMapFP),    &(scenePIP.showMapFP) );
	swapChar( &(sc->showImpf),     &(scenePIP.showImpf) );
	swapChar( &(sc->showWorldPoints),     &(scenePIP.showWorldPoints) );
	swapChar( &(sc->showVisionFormation), &(scenePIP.showVisionFormation) );
	swapChar( &(gsc->showCameraFOV),  &(gscPIP->showCameraFOV) );
	swapChar( &(sc->showScan),        &(scenePIP.showScan) );
	swapChar( &(sc->showScanPoints),  &(scenePIP.showScanPoints) );
	swapChar( &(sc->showKeyPts),      &(scenePIP.showKeyPts) );
	swapChar( &(sc->showCurrentPts),  &(scenePIP.showCurrentPts) );
	swapChar( &(sc->showMatchedPts),  &(scenePIP.showMatchedPts) );
	swapChar( &(sc->showSlamData),    &(scenePIP.showSlamData) );
	swapChar( &(sc->show2dCov),       &(scenePIP.show2dCov) );
	swapChar( &(sc->showPspFilament), &(scenePIP.showPspFilament) );
	swapChar( &(gsc->show3Dmap),      &(gscPIP->show3Dmap) );
	swapChar( &(gsc->mapUpMode),      &(gscPIP->mapUpMode) );
	swapChar( &(gsc->showGeofence),   &(gscPIP->showGeofence) );
	swapChar( &(sc->showMessages),    &(scenePIP.showMessages) );
	swapChar( &(sc->showMarkOnClick), &(scenePIP.showMarkOnClick) );
	swapChar( &(sc->showSLAM),        &(scenePIP.showSLAM) );
	swapChar( &(sc->showWDB),         &(scenePIP.showWDB) );
	swapChar( &(sc->showGraph),       &(scenePIP.showGraph) );
	swapChar( &(sc->showEvimap),      &(scenePIP.showEvimap) );
	swapChar( &(sc->showChecklist),   &(scenePIP.showChecklist) );
    swapChar( &(sc->showAnnotationFP),  &(scenePIP.showAnnotationFP) );
    swapChar( &(sc->showAnnotationSlung),  &(scenePIP.showAnnotationSlung) );
    swapChar( &(sc->showAnnotationBoxes),  &(scenePIP.showAnnotationBoxes) );
	swapChar( &(gsc->showPlan),       &(gscPIP->showPlan) );
	swapChar( &(sc->showTraffic),     &(scenePIP.showTraffic) );
	swapChar( &(sc->showBenchLOS),    &(scenePIP.showBenchLOS) );
	swapChar( &(gsc->showAutopilotDels), &(gscPIP->showAutopilotDels) );
    swapChar( &(sc->showMBZIRC),    &(scenePIP.showMBZIRC) );

	swapFloat( &(gsc->angle3D),        &(gscPIP->angle3D) );
	swapFloat( &(gsc->mapUpAngle),     &(gscPIP->mapUpAngle) );

}


static void swapScenePIP( struct sceneGlobal_ref *sg, struct scene_ref *sc ) {

	sceneAddMessage( sg, sc, "PIP Swapped" );
	sc->animatePIP = 1.0f;

}


static void selectNone( struct scene_ref *sc ) {

	int i;

	for( i=0; i<MAN_NMANS; i++ )
		sc->waySelected[i] = 0;

}


static int numberSelected( struct scene_ref *sc, int *pick ) {

	return 0;

}


static int allFormationWaypointsHeading( struct scene_ref *sc ) {

	return 0;

}


static int anyFormationWaypointsSelectedAndNotLimited( struct scene_ref *sc ) {

	
	return 0;

	/* 3 = at least one attitude point, 2 = at least one circle point, 1 = any selected formation flight, 0 = none */

}


static void drawVehicle( struct sceneGlobal_ref *sg, struct vehicleOutputs_ref *o, struct scene_ref *sc, float zoom, int mode ) {

	switch( o->model ) {
	//case MODEL_RMAX:
	//
	//	drawGTMax( o->latitude, o->longitude, o->altitudeMSL, o->terrainAlt,
	//		sc->eyeLat, sc->eyeLon, sc->eyeAlt, sc->cosDatumLat, zoom, o->float_dcm_lb,
	//		sg->sunPosition, o->a0, o->a1, o->b1, mode, sc->winh, o->uniqueID );
	//	break;
	
	default:
	case MODEL_MULTIROTOR:
        drawMultirotor( o->latitude, o->longitude, o->altitudeMSL, o->terrainAlt,
			sc->eyeLat, sc->eyeLon, sc->eyeAlt, sc->cosDatumLat, zoom, o->float_dcm_lb,
			sg->sunPosition, mode, sc->winh );
		break;
	}

}

static void draw3DObjects( struct scene_ref *sc,
						   struct sceneGlobal_ref *sg,
						   struct vehicleOutputs_ref *o,
						   struct realScene_ref *rs,
						   float zoom ) {

	struct gcs_ref *g = &gcs;
	struct gcsInstance_ref *gi = gcsActiveInstance( g );
	struct gcsScene_ref *gsc = whichGcsScene( sc, gi );

	struct vehicleOutputs_ref *vo = &vehicleOutputs;
	struct vehicleOutputs_ref *no = gi->outputs;

	struct si_ref				*s = &si;
	struct sceneMbzirc_ref *sm;

	int i, j;
	int ig;
	float targetColor[4] = {0.0f,0.0f,0.0f,1.0f};
	//unsigned char buffer[100];

	if( sc->videoModeDerived == 0 ) {

		/* complex scene (First Responder) */
		//if( sg->drawComplexScene ) {
		//	drawComplexScene( sc, sg, o );
		//}

		/* draw wdb scene */
		//if( sc->showWDB ) {
		//	drawWdbScene(sc,sg,o,zoom);
		//}

		/* real scene */
        if( sg->showRealScene ) {
            glPushMatrix();
            glTranslatef( (float)(( realScene.set->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                (float)(hmodDeg( realScene.set->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                (float)(-o->datumAlt + sc->eyeAlt) );
            glCallList( sg->realScene_dl );
            glPopMatrix();

			/* trees */
			for( i=0; i<rs->set->numTrees; i++ ) {
				glPushMatrix();
				glTranslatef( (float)(( rs->tree[i]->latitude - sc->eyeLat )*C_NM2FT*60.0),
					(float)(hmodDeg( rs->tree[i]->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(-o->datumAlt + sc->eyeAlt) );
				glRotatef( (float)(sc->eyePsi*C_RAD2DEG), 0.0, 0.0, 1.0 ); /* so it points at viewer */
				glScalef( (float)rs->tree[i]->size, (float)rs->tree[i]->size, (float)rs->tree[i]->size );
				if( gsc->showTex ) glCallList( sg->tree_dl );
				else               glCallList( sg->treeNoTex_dl );
				glPopMatrix();
			}
		}

		/* cloud */
		if( gsc->viewMode != VIEW_NAV && gsc->showTex && sc->showCloud ) {
			glPushMatrix();
			glTranslatef( (float)(sg->cloudPos[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(sg->cloudPos[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(-o->datumAlt + sg->cloudPos[2] + sc->eyeAlt) );
			glScalef( sg->cloudSize, sg->cloudSize, sg->cloudSize );
			glCallList( sg->cloud_dl );
			glPopMatrix();
		}

		/* generic buildings */
		if( sg->showBuildings ) {
			for( i=0; i<3; i++ ) {
				glPushMatrix();
				glTranslatef( (float)(sg->buildingPos[i][0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
					(float)(sg->buildingPos[i][1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(-o->datumAlt + sc->eyeAlt - gi->datalink->terrainH ) );
				glRotatef( sg->buildingAngle[i], 0.0, 0.0, 1.0 );
				glCallList( sg->building_dl );

				if( i == 0 ) {
					if( sg->showSymbol )
						glCallList( sg->buildingSym_dl );
				}
				glPopMatrix();
			}
		}

		/* sphere Obstacles */
		for( i=0; i<sg->numberOfObstacles; i++ ) {
			glPushMatrix();
			glTranslatef( (float)(sg->sphereObstacle[i]->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(sg->sphereObstacle[i]->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(sg->sphereObstacle[i]->location[2] - o->datumAlt + sc->eyeAlt ));
			glScalef( sg->sphereObstacle[i]->radius, sg->sphereObstacle[i]->radius, sg->sphereObstacle[i]->radius );
            
			if (sg->sphereObstacle[i]->whichColor == 0){ //use global color
				glMaterialfv( GL_FRONT, GL_AMBIENT, sg->sphereObstacleColor );
				glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->sphereObstacleColor );
			}else{ //use local color
				glMaterialfv( GL_FRONT, GL_AMBIENT, sg->sphereObstacle[i]->color );
				glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->sphereObstacle[i]->color );
			}
			
			glCallList( sg->sphereObstacle_dl );
			glPopMatrix();

			//tether
			if (sg->sphereObstacle[i]->hasTether){
			glMaterialfv( GL_FRONT, GL_DIFFUSE,sg->sphereObstacle[i]->tetherColor);
			glBegin(GL_LINES);
				glVertex3f( (float)(sg->sphereObstacle[i]->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(sg->sphereObstacle[i]->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(0.0f - o->datumAlt + sc->eyeAlt ));

				glVertex3f( (float)(sg->sphereObstacle[i]->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(sg->sphereObstacle[i]->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(sg->sphereObstacle[i]->location[2] - o->datumAlt + sc->eyeAlt ));

			glEnd();
			}

		}

        /* banner obstacle */
        if( sg->bannerObstacle->show ) {
            glPushMatrix();
                glTranslatef( (float)(sg->bannerObstacle->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                    (float)(sg->bannerObstacle->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                    (float)(sg->bannerObstacle->location[2] - o->datumAlt + sc->eyeAlt) );

                glRotatef( sg->bannerObstacle->angle, 0.0, 0.0, 1.0 );

                glMaterialfv( GL_FRONT, GL_AMBIENT, sg->bannerObstacle->color1 );
                glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->bannerObstacle->color1 );
                glCallList( sg->bannerObstacle->light_dl );

                glMaterialfv( GL_FRONT, GL_AMBIENT, sg->bannerObstacle->color2 );
                glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->bannerObstacle->color2 );
                glCallList( sg->bannerObstacle->dark_dl );
            glPopMatrix();
        }
        //AHS GTAR HERE
         if( sg->gtarAHS->show ) {
            glPushMatrix();
                glTranslatef( (float)(sg->gtarAHS->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                    (float)(sg->gtarAHS->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                    (float)(sg->gtarAHS->location[2] - o->datumAlt + sc->eyeAlt) );

                glRotatef( sg->gtarAHS->angle, 0.0, 0.0, 1.0 );

                //glMaterialfv( GL_FRONT, GL_AMBIENT, sg->gtarAHS->color1 );
                //glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->gtarAHS->color1 );
                //glCallList( sg->bannerObstacle->light_dl );

                glMaterialfv( GL_FRONT, GL_AMBIENT, sg->gtarAHS->color2 );
                glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->gtarAHS->color2 );
                glCallList( sg->gtarAHS->dark_dl );
            glPopMatrix();
        }
        
        /* picture obstacle */
        if (sg->pictureObstacle->show ) {
            glPushMatrix();
                glTranslatef( (float)(sg->pictureObstacle->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                              (float)(sg->pictureObstacle->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                              (float)(sg->pictureObstacle->location[2] - o->datumAlt + sc->eyeAlt) );
                glRotatef( sg->pictureObstacle->angle, 0.0, 0.0, 1.0 );
                glCallList( sg->pictureObstacle->obstacle_dl );
            glPopMatrix();
        }
 
		/* gtar desk */
        if( sg->gtarDesk->show ) {
            glPushMatrix();
                glTranslatef( (float)(sg->gtarDesk->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                              (float)(sg->gtarDesk->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                              (float)(sg->gtarDesk->location[2] - o->datumAlt + sc->eyeAlt) );
                glRotatef( sg->gtarDesk->angle, 0.0, 0.0, 1.0 );
                glCallList( sg->gtarDesk->dl );
            glPopMatrix();
        }

		/* gtar switchpost */
        if( sg->gtarswitchpost->show ) {
            glPushMatrix();
                glTranslatef( (float)(sg->gtarswitchpost->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                              (float)(sg->gtarswitchpost->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                              (float)(sg->gtarswitchpost->location[2] - o->datumAlt + sc->eyeAlt) );
                glRotatef( sg->gtarswitchpost->angle, 0.0, 0.0, 1.0 );
                glCallList( sg->gtarswitchpost->dl );
            glPopMatrix();
        }

        /* gtar rover */
        if( sg->gtarRover->show  ) {  //gtar_desk added so that the rover moves wrt to desk location //took out && sg->gtarDesk->show
            glPushMatrix();
                glTranslatef( (float)(sg->gtarRover->location[0]+sg->gtarDesk->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                              (float)(sg->gtarRover->location[1]+sg->gtarDesk->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                              (float)(sg->gtarRover->location[2]+sg->gtarDesk->location[2] - o->datumAlt + sc->eyeAlt) );
                glRotatef( sg->gtarRover->angle, 0.0, 0.0, 1.0 );
		//glScalef( 5.0f,5.0f, 5.0f );
		glScalef( sg->gtarRover->roverScale[0], sg->gtarRover->roverScale[1],sg->gtarRover->roverScale[2]);
                glCallList( sg->gtarRover->dl );
            glPopMatrix();
        }

                // gtar ahs2 //
       if( sg->gtarAHS2->show  ) {
            glPushMatrix();
                glTranslatef( (float)(sg->gtarAHS2->location[0]+sg->gtarDesk->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                              (float)(sg->gtarAHS2->location[1]+sg->gtarDesk->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                              (float)(sg->gtarAHS2->location[2]+sg->gtarDesk->location[2] - o->datumAlt + sc->eyeAlt) );
                glRotatef( sg->gtarAHS2->angle, 0.0, 0.0, 1.0 );
		//glScalef( 5.0f,5.0f, 5.0f );
        glScalef( sg->gtarAHS2->roverScale[0], sg->gtarAHS2->roverScale[1],sg->gtarAHS2->roverScale[2]);
                glCallList( sg->gtarAHS2->dl );
            glPopMatrix();
        }

       // gtar AHS3 //
       if( sg->gtarAHS3->show  ) {
           glPushMatrix();
           glTranslatef( (float)(sg->gtarAHS3->location[0]+sg->gtarDesk->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                   (float)(sg->gtarAHS3->location[1]+sg->gtarDesk->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                   (float)(sg->gtarAHS3->location[2]+sg->gtarDesk->location[2] - o->datumAlt + sc->eyeAlt) );
           glRotatef( sg->gtarAHS3->angle, 0.0, 0.0, 1.0 );
           //glScalef( 5.0f,5.0f, 5.0f );
           glScalef( sg->gtarAHS3->roverScale[0], sg->gtarAHS3->roverScale[1],sg->gtarAHS3->roverScale[2]);
           glCallList( sg->gtarAHS3->dl );
           glPopMatrix();
       }

       // gtar AHS4 //
       if( sg->gtarAHS4->show  ) {
           glPushMatrix();
           glTranslatef( (float)(sg->gtarAHS4->location[0]+sg->gtarDesk->location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                   (float)(sg->gtarAHS4->location[1]+sg->gtarDesk->location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                   (float)(sg->gtarAHS4->location[2]+sg->gtarDesk->location[2] - o->datumAlt + sc->eyeAlt) );
           glRotatef( sg->gtarAHS4->angle, 0.0, 0.0, 1.0 );
           //glScalef( 5.0f,5.0f, 5.0f );
           glScalef( sg->gtarAHS4->roverScale[0], sg->gtarAHS4->roverScale[1],sg->gtarAHS4->roverScale[2]);
           glCallList( sg->gtarAHS4->dl );
           glPopMatrix();
       }
	
	   /*mbzirc targets */
	   
		//if( sc->showTruth == 1 ) {
	   if( 1 ) { //the line above won't work in multivehicle sim -Ep
	   sm = sg->sceneMbzirc;

		 if (sm->show){

			}//end show mbzirc objects	  
		}

		// Random Polygons
		if( sg->randPolygons->randTestInit ){
			sg->randPolygons->randTestNextUpdate = sim.time + sg->randPolygons->randTestdt;
			sg->randPolygons->numPolygons = 0;
			sg->randPolygons->show = 1;
			sg->randPolygons->randTestIsRunning = 1;
			sg->randPolygons->randTestInit = 0;
		}
		if( sg->randPolygons->randTestIsRunning && sim.time > sg->randPolygons->randTestNextUpdate ){
			sg->randPolygons->numPolygons = sg->randPolygons->numPolygons + 1;
			sg->randPolygons->randTestNextUpdate = sim.time + sg->randPolygons->randTestdt;
		}
		if( sg->randPolygons->show ){
                        sg->randPolygons->range = (float)(-(sg->randPolygons->rangeFactor) * ((float)o->altitudeAGL) * tan((sc->fovy/2.0f)));
			sg->randPolygons->simDummyTime = sim.time;
			if(sg->randPolygons->updateTime[0] ==  sg->randPolygons->updateTime[1]){
				sg->randPolygons->updateTime[0] = sim.time - (sg->randPolygons->updateRate / 1000.0);
				sg->randPolygons->updateTime[1] = sim.time + (sg->randPolygons->updateRate / 1000.0);
			}/*
			if(sim.time < sg->randPolygons->updateTime[0]){
				sg->randPolygons->updateTime[0] = sg->randPolygons->updateTime[0] - (sg->randPolygons->updateRate / 1000.0);
				sg->randPolygons->updateTime[1] = sg->randPolygons->updateTime[1] - (sg->randPolygons->updateRate / 1000.0);
				for(i=0; i<2; i++){
					for(j=0; j<sg->randPolygons->numPolygons; j++){
						sg->randPolygons->randomSeed[i][j] = (float)(rand() % (int)ceil(1000*sg->randPolygons->range))/1000.0f - ((float)(sg->randPolygons->range)/2.0f);
					}
				}
			}*/
			else if(sim.time > sg->randPolygons->updateTime[1]){
				sg->randPolygons->updateTime[0] = sg->randPolygons->updateTime[0] + (sg->randPolygons->updateRate / 1000.0);
				sg->randPolygons->updateTime[1] = sg->randPolygons->updateTime[1] + (sg->randPolygons->updateRate / 1000.0);
				//sg->randPolygons->updateTime[0] = sim.time - (sg->randPolygons->updateRate / 1000.0);
				//sg->randPolygons->updateTime[1] = sim.time + (sg->randPolygons->updateRate / 1000.0);
				for(i=0; i<2; i++){
					for(j=0; j<sg->randPolygons->numPolygons; j++){
						sg->randPolygons->randomSeed[i][j] = (float)(rand() % (int)ceil(1000*sg->randPolygons->range))/1000.0f - ((float)(sg->randPolygons->range)/2.0f);
                                                sg->randPolygons->randomRots[j] = (float)(rand() % (360*1000))/1000.0f;
					}
				}
			}

			glPushMatrix();
				//glTranslatef( 0.0f , 0.0f , (float)(-o->datumAlt + sc->eyeAlt) );
				glTranslatef( (float)(sg->randPolygons->location[0] + ( o->latitude - sc->eyeLat )*C_NM2FT*60.0),
					      (float)(sg->randPolygons->location[1] + hmodDeg( o->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
			       	              (float)(sg->randPolygons->location[2] - o->terrainAlt + sc->eyeAlt) ); //o->terainAlt or o->datumAlt?

				//glCallList( sg->randPolygons->randPolygons_dl ); // Display lists are static...

				//glShadeModel( GL_SMOOTH );
				//glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, sg->randPolygons->color );

				for(j=0; j<sg->randPolygons->numPolygons; j++){
					glRotatef(sg->randPolygons->randomRots[j],0,0,1);
					glBegin(GL_TRIANGLES);
						glNormal3f( 0.0, 0.0, -1.0 );
						glVertex3f(sg->randPolygons->randomSeed[0][j],
								   sg->randPolygons->randomSeed[1][j] + sg->randPolygons->randRadius,
								   0);
						glVertex3f((float)(sg->randPolygons->randomSeed[0][j] + sg->randPolygons->randRadius*((sqrt(3.0f)/2.0f))),
								   (float)(sg->randPolygons->randomSeed[1][j] + sg->randPolygons->randRadius*(-1.0f/2.0f)),
									0);
						glVertex3f((float)(sg->randPolygons->randomSeed[0][j] + sg->randPolygons->randRadius*(-(sqrt(3.0f)/2.0f))),
								   (float)(sg->randPolygons->randomSeed[1][j] + sg->randPolygons->randRadius*(-1.0f/2.0f)),
								    0);
						glEnd();
				}

			glPopMatrix();
		}

		/* vehicle (true/simulated one) */

		if( sc->showTruth ==1 ) {
	
			//truth vehicle driven by vehicleOutputs in this instance of GUST
			drawVehicle( sg, vo, sc, zoom, 1 );
						
		}

	}

	if( sc->showGCS ) {

		for( ig=0; ig<GCS_MAX_INSTANCES; ig++ ) {

			struct gcsInstance_ref *gis = gcsGetInstance( g, ig );

			if( gis->run ) {

				/* draw ownship */

				drawVehicle( sg, gis->outputs, sc, zoom, 0 );

				if( ( sg->drawLineToText && gsc->showGCStext ) /*|| sg->showNameText*/ ) {    /* label it or determine where to draw line to aircraft from gcs text box */
					unsigned char valid;
					glPushMatrix();
					//glDisable( GL_DEPTH_TEST );
					//glDisable( GL_LIGHTING );
					//glColor4fv( sg->menuTextColor );
					glTranslatef( (float)(( gis->outputs->latitude - sc->eyeLat )*C_NM2FT*60.0),
						(float)(hmodDeg( gis->outputs->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
						(float)(-gis->outputs->altitudeMSL + sc->eyeAlt) );
					glRasterPos3f( 0, 0, 0 );
					glGetFloatv( GL_CURRENT_RASTER_POSITION, sc->rasterPosVehicle[ig] );
					glGetBooleanv( GL_CURRENT_RASTER_POSITION_VALID, &valid );
					if( !valid ) {
						sc->rasterPosVehicle[ig][0] = -1;
					} else {
						/* make sure not over PIP */
						if( gsc->showPIP )
							if( sc->rasterPosVehicle[ig][0] > sc->winw - sg->pipOffsetX - scenePIP.winw )
								if( sc->rasterPosVehicle[ig][0] < sc->winw - sg->pipOffsetX )
									if( sc->rasterPosVehicle[ig][1] > sg->pipOffsetY )
										if( sc->rasterPosVehicle[ig][1] < sg->pipOffsetY + scenePIP.winh )
											sc->rasterPosVehicle[ig][0] = -1;
					}
					//sprintf( buffer, "" );
					//gcsAddVehicleName( buffer, gis->outputs );
					//drawBitmapText( buffer, LIMIT( sg->wayFont, 2, 8 ) );
					//glEnable( GL_LIGHTING );
					//glEnable( GL_DEPTH_TEST );
					glPopMatrix();
				}
			}
		}

        /* draw mapped feature points */

        //if( sc->showMapFP ) {
        //    int npoints;
        //    struct datalinkMessageMapFP_ref *mapfp = gi->datalink->mapfp;
        //    if( sc->showMapFP == 2 ) {
        //        npoints = mapfpWork.nstore;
        //    } else {
        //        npoints = (int)(mapfp->ptsInMsg);
        //    }
        //    for( i=0;i<npoints;i++ ) {
        //        float color[4], blend, location[3];
        //        int j;
        //        short observations;
        //        float scale = 1.0;
        //        int converged = 0;
        //        if( sc->showMapFP == 2 ) {
        //            
        //            
        //            if( 0==mapfpSet.convergenceTest )
        //                converged = mapfpIDPMat.sigd[i] < mapfpSet.sigdLim;
        //            else
        //                converged = mapfpIDPMat.depthRangeExtents[i] < mapfpSet.distLim;
        //            
        //            
        //            for( j=0; j<3; j++ ) location[j] = (float)mapfpState.fpPos[i][j];
        //            observations = mapfpGenMat.ptObservations[i];
        //        } else {
        //            for( j=0; j<3; j++ ) location[j] = mapfp->fpPos[i][j];
        //            observations = mapfp->ptObservations[i];
        //        }
        //        if(sg->mapfpObstacleMaxScaleAlt > 0)
        //            scale = LIMIT( (float)(no->altitudeAGL/sg->mapfpObstacleMaxScaleAlt), .05f, 1.0f );
        //        else
        //            scale = 1.0f;
        //        glPushMatrix();
        //        glTranslatef( (float)(location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
        //                      (float)(location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
        //                      (float)(location[2] - o->datumAlt + sc->eyeAlt) );
        //        glScalef( sg->mapfpObstacleRadius*scale, sg->mapfpObstacleRadius*scale, sg->mapfpObstacleRadius*scale  );
        //        blend = LIMIT( (float)observations/MAX( sg->mapfpObsThresh, 1 ), 0, 1 );
        //        
        //        if( !converged )
        //            for( j=0; j<4; j++ ) color[j] = sg->sphereObstacleColor[j]*blend + sg->sphereObstacleColorAlt[j]*( 1.0f - blend );
        //        else
        //            for( j=0; j<4; j++ ) color[j] = sg->sphereObstacleColorConverged[j]*blend + sg->sphereObstacleColorConvergedAlt[j]*( 1.0f - blend );
        //        glMaterialfv( GL_FRONT, GL_AMBIENT, color );
        //        glMaterialfv( GL_FRONT, GL_DIFFUSE, color );
        //        glCallList( sg->sphereObstacle_dl );
        //        glPopMatrix();
        //        
        //        
        //        if( sc->show3dCov )
        //        {
        //            struct datalinkMessage3dCov_ref *cov = gi->datalink->cov3d;
        //            
        //            //float dcm_ci[3][3];
        //            //euler2dcmf(dcm_ci, cov->roll, cov->pitch, cov->yaw);
        //            
        //            glPushMatrix();
        //            glTranslatef( (float)(location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
        //                        (float)(location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
        //                        (float)(location[2] - o->datumAlt + sc->eyeAlt) );
        //            glRotatef( cov->yaw*CF_RAD2DEG,   0.0f, 0.0f, 1.0f );
        //            glRotatef( cov->pitch*CF_RAD2DEG, 0.0f, 1.0f, 0.0f );
        //            glRotatef( cov->roll*CF_RAD2DEG,  1.0f, 0.0f, 0.0f );
        //            glScalef( cov->s0/sg->sphereObstacleRadius, cov->s1/sg->sphereObstacleRadius, cov->s2/sg->sphereObstacleRadius );
        //            
        //            
        //            glMaterialfv( GL_FRONT, GL_AMBIENT, sg->sphereCovColor );
        //            glMaterialfv( GL_FRONT, GL_DIFFUSE, sg->sphereCovColor );
        //            glCallList( sg->sphereObstacle_dl );
        //            glPopMatrix();
        //            
        //        }
        //    }
        //}

#ifdef HAVE_LSSM
		// draw mapped and tracked feature points (visualize the result of LSSM (least squares sparse mapping))
		if( sc->showMapLssmFP ) {
		    int npoints;
			struct datalinkMessageLssm_ref *mapfplssm = gi->datalink->Lssm;
		    if( sc->showGCS == 0 ) {
				npoints = mapfpWork.nstore;
		    } else {
		        npoints = (int)(mapfplssm->fpNum);
		    }
		    for( i=0;i<npoints;i++ ) {
		        float color[4], blend, location[3];
		        int j;
		        short observations;
		        float scale = 1.0;
		        if( sc->showGCS == 0 ) {
		            for( j=0; j<3; j++ ) location[j] = (float)mapfplssm->fpPos[i][j];
		            observations = mapfpGenMat.ptObservations[i];
		        } else {
		            //for( j=0; j<3; j++ ) location[j] = mapfp->SVT_fp[i][j];			// for onboard link
		            for( j=0; j<3; j++ ) location[j] = (float)mapfplssm->fpPos[i][j];	// tempolary code for simulation
					//printf("------");
		            //observations = mapfp->ptObservations[i];
		        }
		        if(sg->mapfpObstacleMaxScaleAlt > 0)
		            scale = LIMIT( (float)(no->altitudeAGL/sg->mapfpObstacleMaxScaleAlt), .05f, 1.0f );
		        else
		            scale = 1.0f;
		        glPushMatrix();
		        glTranslatef( (float)(location[0] + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0),
		                      (float)(location[1] + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
		                      (float)(location[2] - o->datumAlt + sc->eyeAlt) );
		        glScalef( sg->mapfpObstacleRadius*scale, sg->mapfpObstacleRadius*scale, sg->mapfpObstacleRadius*scale  );
		        blend = LIMIT( (float)observations/MAX( sg->mapfpObsThresh, 1 ), 0, 1 );
				// if accurate point, then change color
				if(mapfplssm->fpPosAccFlag[i] == 0){	
		            for( j=0; j<4; j++ ) color[j] = sg->sphereObstacleColor[j]*blend + sg->sphereObstacleColorAlt[j]*( 1.0f - blend );
				}
				else{
					color[0]=0.8f;
					color[1]=0.8f;
					color[2]=0.8f;
					color[3]=1.0f;
					// Green
					//color[0]=0.0f;
					//color[1]=1.0f;
					//color[2]=0.0f;
					//color[3]=1.0f;
					// enge
					//color[0]=0.1745f;
					//color[1]=0.01175f;
					//color[2]=0.01175f;
					//color[3]=1.0f;
				}
		        //for (j =0;j<32;j++){
				//	printf("%d\t",(int)mapfplssm->fpPosAccFlag[j]);
				//}
				//printf("\n");

				glMaterialfv( GL_FRONT, GL_AMBIENT, color );
		        glMaterialfv( GL_FRONT, GL_DIFFUSE, color );
		        glCallList( sg->sphereObstacle_dl );
		        glPopMatrix();
		    }
		}
#endif

    }

    /* Note: These should be drawn last and in order from farthest to nearest the camera due to transparency*/
	glDepthMask( 0 );
    if( sc->showDjiGuidance)                
	glDepthMask( 1 );

}


static void determineMapUp( struct gcsSet_ref *set, struct sceneGlobal_ref *sg, struct gcsScene_ref *gsc, struct scene_ref *sc, struct vehicleOutputs_ref *o ) {

	if( set->nav->lookat == VIEW_LOOKAT_NOTHING ) {
		set->nav->neckPsi = gsc->mapUpAngle;
	} else {
		switch( gsc->mapUpMode ) {
		default:
			set->nav->neckPsi = gsc->mapUpAngle;
			break;
		case MAPUP_HEADING:
			if( set->rotate90 ) set->nav->neckPsi = (float)(o->psi90*C_DEG2RAD);
			else                set->nav->neckPsi = (float)(o->psi*C_DEG2RAD);
			break;
		case MAPUP_TRACK:
			set->nav->neckPsi = (float)(o->track*C_DEG2RAD);
			break;
		}
	}

}

static void waypointLocateInverse( struct gcsInstance_ref *gi, int index, double *x ) {

	int i;

	struct maneuver_ref *m = &maneuver[index];
	char formation = 0;

	if( m->type == MAN_FORMATION ) {
		formation = 1;
	} else if( m->type == MAN_LANDING || m->type == MAN_EXT || m->type == MAN_EXT_SEQ) {
		if( index > 0 ) {
			if( maneuver[index-1].type == MAN_FORMATION ) {
				formation = 1;
			}
		}
	}



		for( i=0; i<3; i++ ) {
			m->x[i] = x[i];
		}

		if( m->altMode == ALT_AGL ) {
			m->x[2] += gi->datalink->terrainH;
		}

}


static void waypointLocate( struct gcsInstance_ref *gi, int index, double *x ) {

	struct maneuver_ref *m = &maneuver[index];
	char formation = 0;

	if( m->type == MAN_FORMATION ) {
		formation = 1;
	} else if( m->type == MAN_LANDING || m->type == MAN_EXT || m->type == MAN_EXT_SEQ) {
		if( index > 0 ) {
			if( maneuver[index-1].type == MAN_FORMATION ) {
				formation = 1;
			}
		}
	}

 /* normal */

		x[0] = (float)(m->x[0]);
		x[1] = (float)(m->x[1]);
		x[2] = (float)(m->x[2]);

		if( m->altMode == ALT_AGL ) {
			x[2] -= (float)gi->datalink->terrainH;
		}

	if( m->type == MAN_LANDING ) {
		x[2] = -(float)gi->datalink->terrainH;
	}

}


static float sceneGetTerrainH( struct dted_ref *t, double latitude, double longitude, double defaultH ) {

	float terrainH;

	if( t->error == DTED_ERROR_NONE ) terrainH = (float)srtmGetElevation( t, latitude, longitude );

	if( t->error ) terrainH = (float)defaultH;

	return terrainH;
}


static void addEvimapVertex( struct sceneGlobal_ref *sg, float v0, float v1, float v2 ) {

	int i;
	float scaleit, color[4];

	scaleit = -v2/256; /* based on a char */
	for( i=0; i<4; i++ ) color[i] = sg->evimapColorHigh[i]*scaleit + ( 1 - scaleit )*sg->evimapColorLow[i];
	glEnable(GL_CULL_FACE);
	glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color ); 
	glVertex3f( v0, v1, v2 );

}

static void addOctomapVertex( struct sceneGlobal_ref *sg, float x, float y, float z, float colorOctomap[4])
{
	glDisable(GL_CULL_FACE);
	glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE,colorOctomap);
	glEnable(GL_BLEND);
	glVertex3f( x, y, z);

}

/*	// toshi changed
inline void Normalize3(float v[3])
{
   float len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
   v[0] /= len;
   v[1] /= len;
   v[2] /= len;
}
*/

static void drawOctomapTriangle (struct scene_ref *sc,struct sceneGlobal_ref *sg, struct evimap_ref *evi, float point1[3], float point2[3], float point3[3], float mvm[4][4], float colorOctomap[4]) {
	float u[3], v[3], n[3];		// u,v: surface vector cosists of triangle, n: normal vector, 
	//float n_eye[3];
	//float detMvm, InvMvmT[4][4];
  
	u[0] = point2[0]-point1[0]; u[1]=point2[1]-point1[1]; u[2]=point2[2]-point1[2];
	v[0] = point3[0]-point1[0]; v[1]=point3[1]-point1[1]; v[2]=point3[2]-point1[2];
	n[0] = u[1]*v[2] - u[2]*v[1];
	n[1] = u[2]*v[0] - u[0]*v[2];
	n[2] = u[0]*v[1] - u[1]*v[0];
	
	// toshi changed
	//Normalize3(n);
	//float len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	//v[0] /= len;
	//v[1] /= len;
	//v[2] /= len;

	glNormal3f( n[0],n[1],n[2] );

	addOctomapVertex( sg, point1[0], point1[1], point1[2],colorOctomap );
	addOctomapVertex( sg, point2[0], point2[1], point2[2],colorOctomap );
	addOctomapVertex( sg, point3[0], point3[1], point3[2],colorOctomap );
}


static void redrawSingleScene( struct sceneGlobal_ref *sg,
							   struct scene_ref *sc,
							   struct scene_ref *scMain,
	                           int winx, int winy ) {

    struct gcs_ref              *g   = &gcs;
	struct gcsInstance_ref      *gi  = gcsActiveInstance( g );
	struct gcsScene_ref         *gsc = whichGcsScene( sc, gi );
	struct trajectory_ref       *la  = gi->traj->la;
	struct realScene_ref        *rs  = &realScene;
    struct vehicle_ref          *veh = &vehicle;
	struct vehicleOutputs_ref   *no  = gi->outputs;

	struct vehicleOutputs_ref *o;
	struct view_ref *v;
	struct flightPlan_ref *fp;

	double time;
	unsigned char buffer[BUFFER_SIZE];
	double dx[3], dist;
//	double dcm[3][3];
    double pos[3];
	float angle;
	float mvm[4][4];
	int i,j;
	float dmin;

	float t, p[3];
	int pickWay;
	float white[4] = { 1.0, 1.0, 1.0, 1.0 };

	unsigned char gn;
	unsigned char knowView = 1;
	unsigned char wayBoxValid[MAN_NMANS] = {0};
	float wayBoxHeightL = 0;

	gn = LIMIT( g->active, 0, GCS_MAX_INSTANCES-1 );

	v = whichView( gi->set, gsc->viewMode );
        
    switch( sc->lookat ) {
    default:
    case LOOKAT_TRUTH:
    case LOOKAT_BOTH:
        o  = &vehicleOutputs;
        break;
    case LOOKAT_GCS:
        o  = gi->outputs;
        break;
    }

	if( gsc->viewMode == VIEW_NAV ) {
		float targetAngle;
		if( gsc->show3Dmap /*|| sc->altMenuOpen*/ ) {
			targetAngle = gi->set->angle3D;
		} else {
			targetAngle = 0;
		}
		if( sim.mode == SIM_MODE_PAUSE ) {
			gsc->angle3D = targetAngle;
		} else {
			gsc->angle3D += ( targetAngle - gsc->angle3D )*sg->angle3Dspeed;
		}
		if( gsc->angle3D < 0.1f ) sc->show3D = 0;
		else                      sc->show3D = 1;
	} else {
		sc->show3D = 0;
	}

	/* plop in video */
	sc->videoModeDerived = 0;

	/* for consistent latitude shorteding */
	sc->cosDatumLat = cos( o->datumLat*C_DEG2RAD );

    /* populate camera view parameters */

    /* would like to replace with a real ptz camera model at some point... */
    gi->set->camera->neckPhi    = (float)(o->roll *C_DEG2RAD);
    gi->set->camera->neckTheta  = (float)(o->tilt *C_DEG2RAD);
    gi->set->camera->neckPsi    = (float)(o->pan  *C_DEG2RAD);
    gi->set->camera->zoom       = (float)(o->fovy /MAX( 0.0001, sc->fovy ))*v->fovscale;

    gi->set->camera2->neckPhi   = (float)(o->roll2*C_DEG2RAD);
    gi->set->camera2->neckTheta = (float)(o->tilt2*C_DEG2RAD);
    gi->set->camera2->neckPsi   = (float)(o->pan2 *C_DEG2RAD);
	gi->set->camera2->zoom      = (float)(o->fovy2/MAX( 0.0001, sc->fovy ))*v->fovscale;

    gi->set->camera3->neckPhi   = (float)(o->roll3*C_DEG2RAD);
    gi->set->camera3->neckTheta = (float)(o->tilt3*C_DEG2RAD);
    gi->set->camera3->neckPsi   = (float)(o->pan3 *C_DEG2RAD);
	gi->set->camera3->zoom      = (float)(o->fovy3/MAX( 0.0001, sc->fovy ))*v->fovscale;

    gi->set->camera4->neckPhi   = (float)(o->roll4*C_DEG2RAD);
    gi->set->camera4->neckTheta = (float)(o->tilt4*C_DEG2RAD);
    gi->set->camera4->neckPsi   = (float)(o->pan4 *C_DEG2RAD);
	gi->set->camera4->zoom      = (float)(o->fovy4/MAX( 0.0001, sc->fovy ))*v->fovscale;

	/* figure out map orientation */

	if( gsc->viewMode == VIEW_NAV ) determineMapUp( gi->set, sg, gsc, sc, o );

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	switch( gsc->viewMode ) {
	case VIEW_NAV:
		glOrtho( -v->zoom*sc->winw/sc->winh, v->zoom*sc->winw/sc->winh, -v->zoom, v->zoom, -v->zoom-20000, v->zoom+20000 );
		break;
	default:
		gluPerspective( sc->fovy*v->zoom, (GLfloat)sc->winw/sc->winh, sg->znear, sg->vis );
		break;
	}

	//printf("angle = %.4lf  AR = %.4lf  NearClipping = %.4lf FarClipping = %.4lf\n", sc->fovy*v->zoom,(GLfloat)sc->winw/sc->winh,	sg->znear, sg->vis );
	glFogf( GL_FOG_START, sg->fogNear );
	glFogf( GL_FOG_END, (float)sg->vis );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	if( sc->show3D ) glRotatef( gsc->angle3D, -1, 0, 0 );

	/* 3-d stuff */

	glPushMatrix();

	glRotatef( 90.0, 0.0, 1.0, 0.0 ); /* switch to Etkin Axes */
	glRotatef( 90.0, 1.0, 0.0, 0.0 );

	glPushMatrix();

	/* waypoint picking */

	/*if( sc->pick ) {*/
	if( gsc->viewMode == VIEW_NAV ) {
		pos[0] = ( (float)sc->mx/sc->winw - 0.5 )*2*v->zoom*(GLfloat)sc->winw/sc->winh;
		pos[1] = ( (float)sc->my/sc->winh - 0.5 )*2*v->zoom/cos( gsc->angle3D*C_DEG2RAD*sc->show3D ) -
			( sc->eyeAlt - gi->outputs->datumAlt )*sc->show3D*tan( gsc->angle3D*C_DEG2RAD );
		sc->vec[0] = (float)(( sc->eyeLat  - gi->outputs->datumLat )*C_NM2FT*60.0
		- cos( v->neckPsi )*pos[1] - sin( v->neckPsi )*pos[0]);
		sc->vec[1] = (float)(hmodDeg( sc->eyeLon - gi->outputs->datumLon )*C_NM2FT*60.0*sc->cosDatumLat
		+ cos( v->neckPsi )*pos[0] - sin( v->neckPsi )*pos[1]);
		sc->vec[2] = 0;
	} else {
		glPushMatrix();
		glRotatef( (float)(( (float)sc->mx/sc->winw - 0.5 )*sc->fovy*v->zoom*(GLfloat)sc->winw/sc->winh),
			0, 0, 1 );
		glRotatef( (float)(( (float)sc->my/sc->winh - 0.5 )*sc->fovy*v->zoom),
			0, -1, 0 );

		glGetFloatv( GL_MODELVIEW_MATRIX, &(mvm[0][0]) );
		/*printf( " %.2f %.2f %.2f %.2f\n", mvm[0][0], mvm[0][1], mvm[0][2], mvm[0][3] );
		printf( " %.2f %.2f %.2f %.2f\n", mvm[1][0], mvm[1][1], mvm[1][2], mvm[1][3] );
		printf( " %.2f %.2f %.2f %.2f\n", mvm[2][0], mvm[2][1], mvm[2][2], mvm[2][3] );
		printf( " %.2f %.2f %.2f %.2f\n", mvm[3][0], mvm[3][1], mvm[3][2], mvm[3][3] );*/
		sc->vec[0] = mvm[0][0];
		sc->vec[1] = mvm[0][1];
		sc->vec[2] = mvm[0][2];
		glPopMatrix();
	}
	/*}*/

	if( sc->redraw && sc != &scenePIP ) {

		initDisplayLists( sg, sc, rs );

		glEnable( GL_CULL_FACE );
		glEnable( GL_NORMALIZE );
		glEnable( GL_LIGHTING );

		glLightfv( GL_LIGHT0, GL_AMBIENT,  sg->sunAmbient );
		glLightfv( GL_LIGHT0, GL_DIFFUSE,  sg->sunDiffuse );
		glLightfv( GL_LIGHT0, GL_SPECULAR, sg->sunSpecular );
		glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, 1 );
		glEnable( GL_LIGHT0 );

		glFogi( GL_FOG_MODE, GL_LINEAR );
		sg->horizonColor[3] = 1.0;
		glFogfv( GL_FOG_COLOR, sg->horizonColor );

		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		glEnable( GL_BLEND );
		if( sg->antialias ) {
			glEnable( GL_LINE_SMOOTH );
		} else {
			glDisable( GL_LINE_SMOOTH );
		}
		glDepthFunc( GL_LEQUAL );
		glAlphaFunc( GL_GREATER, (GLclampf)0.5 );

		glPointSize( 1.0 );
		glLineWidth( 1.0 );

		if( gsc->showPanel ) {
			gi->panel->check->init = 1;
		}

		sc->redraw = 0;

	}

	switch( gsc->viewMode ) {
	default:
	case VIEW_COCKPIT:
	case VIEW_CAMERA:
	case VIEW_CAMERA2:
	case VIEW_CAMERA3:
	case VIEW_CAMERA4:
		if( v->zoom <= 0 ) knowView = 0;  /* allow for a case where we do not know where the camera is pointing, supress parts of display */
        glRotatef( -CF_PI/2*v->rotate*CF_RAD2DEG, 1, 0, 0 );
		glRotatef( -v->neckPhi  *CF_RAD2DEG, 1, 0, 0 );
		glRotatef( -v->neckTheta*CF_RAD2DEG, 0, 1, 0 );
		glRotatef( -v->neckPsi  *CF_RAD2DEG, 0, 0, 1 );
		glMultMatrixf( &(o->float_dcm_bl[0][0]) );
		glGetFloatv( GL_MODELVIEW_MATRIX, &(mvm[0][0]) );
		/* this get's my definition of euler angles from current mvm */
		/*dcm2eulerf( mvm(transpose?), sc->eyePhi, sc->eyeTheta, sc->eyePsi );*/
		sc->eyePhi   = (float)atan2( +mvm[2][0], -mvm[2][1] );
		sc->eyeTheta = (float)asin( LIMIT( mvm[2][2], -1.0, 1.0 ) );
		sc->eyePsi   = (float)atan2( -mvm[1][2], -mvm[0][2] );

			sc->eyeLat = o->latitude +
				( o->float_dcm_bl[0][0]*v->seat[0] +
				o->float_dcm_bl[0][1]*v->seat[1] +
				o->float_dcm_bl[0][2]*v->seat[2] )*C_FT2NM/60.0;
			sc->eyeLon = o->longitude +
				( o->float_dcm_bl[1][0]*v->seat[0] +
				o->float_dcm_bl[1][1]*v->seat[1] +
				o->float_dcm_bl[1][2]*v->seat[2] )*C_FT2NM/60.0/sc->cosDatumLat;
			sc->eyeAlt = o->altitudeMSL -
				( o->float_dcm_bl[2][0]*v->seat[0] +
				o->float_dcm_bl[2][1]*v->seat[1] +
				o->float_dcm_bl[2][2]*v->seat[2] );
		break;

	case VIEW_NAV:
        if( sg->autoscaleNav ) {
            double xmin, xmax, ymin, ymax, x, y, cpsi, spsi;
            cpsi = cos( v->neckPsi );
            spsi = sin( v->neckPsi );
            x = ( o->latitude  - o->datumLat )*60*C_NM2FT;
            y = hmodDeg( o->longitude - o->datumLon )*60*C_NM2FT*sc->cosDatumLat;
            ymin = ymax = -x*spsi + y*cpsi;
            if( sc->show3D ) {
                xmin = xmax = ( x*cpsi + y*spsi )*cos( gsc->angle3D*C_DEG2RAD )
                    + sin( gsc->angle3D*C_DEG2RAD )*( o->altitudeMSL - o->datumAlt );
            } else {
                xmin = xmax =  x*cpsi + y*spsi;
            }
            xmin -= sg->gridx;  xmax += sg->gridx;
            ymin -= sg->gridx;  ymax += sg->gridx;
            x = 0.5*( xmin + xmax );  y = 0.5*( ymin + ymax );
            if( sc->show3D ) x = x/cos( gsc->angle3D*C_DEG2RAD ) + v->seat[2]*tan( gsc->angle3D*C_DEG2RAD );
            v->seat[0] = (float)(x*cpsi - y*spsi);
            v->seat[1] = (float)(x*spsi + y*cpsi);
            v->zoom = (float)(MAX( sg->autoscaleNavMinX, ymax - ymin )*0.5*sc->winh/MAX( sc->winw, 1 ));
            v->zoom = (float)(MAX( v->zoom, MAX( sg->autoscaleNavMinX, xmax - xmin )*0.5 ));
        }
		if( v->lookat == VIEW_LOOKAT_VEHICLE && sg->autoscaleNav == 0 ) {
			double cpsi, spsi;
            cpsi = cos( v->neckPsi );
            spsi = sin( v->neckPsi );
			if( numberSelected( sc, &pickWay ) != 1 || v->followWaypoints == 0 ) {
				sc->eyeLat = o->latitude  + ( v->seat[0]*cpsi - v->seat[1]*spsi )*C_FT2NM/60.0;
				sc->eyeLon = o->longitude + ( v->seat[1]*cpsi + v->seat[0]*spsi )*C_FT2NM/60.0/sc->cosDatumLat;
				sc->eyeAlt = o->altitudeMSL - v->seat[2];
			} else {
				double mx[3];
				waypointLocate( gi, pickWay, mx );
				sc->eyeLat = gi->outputs->datumLat +
					( mx[0] + v->seat[0]*cpsi - v->seat[1]*spsi )*C_FT2NM/60.0;
				sc->eyeLon = gi->outputs->datumLon +
					( mx[1] + v->seat[1]*cpsi + v->seat[0]*spsi )*C_FT2NM/60.0/sc->cosDatumLat;
				sc->eyeAlt = o->terrainAlt - mx[2] - v->seat[2];
			}
		} else {
			sc->eyeLat = o->datumLat + v->seat[0]*C_FT2NM/60.0;
			sc->eyeLon = o->datumLon + v->seat[1]*C_FT2NM/60.0/sc->cosDatumLat;
			sc->eyeAlt = o->datumAlt - v->seat[2];
		}
		sc->eyePhi   = 0;
		sc->eyeTheta = -CF_PI*0.5f;
		sc->eyePsi   = v->neckPsi;
		glRotatef( -sc->eyeTheta*CF_RAD2DEG, 0.0, 1.0, 0.0 );
		glRotatef( -sc->eyePsi  *CF_RAD2DEG, 0.0, 0.0, 1.0 );
		break;

	case VIEW_CHASE:
		if( numberSelected( sc, &pickWay ) != 1 || v->followWaypoints == 0  ) {
			sc->eyeLat = o->latitude  + v->seat[0]*C_FT2NM/60.0;
			sc->eyeLon = o->longitude + v->seat[1]*C_FT2NM/60.0/sc->cosDatumLat;
			sc->eyeAlt = o->altitudeMSL - v->seat[2];
		} else {
			double mx[3];
			waypointLocate( gi, pickWay, mx );
			sc->eyeLat = gi->outputs->datumLat +
				( mx[0] + v->seat[0] )*C_FT2NM/60.0;
			sc->eyeLon = gi->outputs->datumLon +
				( mx[1] + v->seat[1] )*C_FT2NM/60.0/sc->cosDatumLat;
			sc->eyeAlt = o->terrainAlt - mx[2] - v->seat[2];
		}
		dx[0] = -v->seat[0];
		dx[1] = -v->seat[1];
		dx[2] = -v->seat[2];
		if( v->lookat == VIEW_LOOKAT_VEHICLE ) {
			sc->eyePhi   = 0.0;
			sc->eyeTheta = (float)atan2( -dx[2], sqrt( dx[0]*dx[0] + dx[1]*dx[1] ) );
			sc->eyePsi   = (float)atan2( dx[1], dx[0] );
		} else {
			sc->eyePhi   = 0.0;
			sc->eyeTheta = 0.0;
			sc->eyePsi   = 0.0;
		}
		sc->eyePhi   += v->neckPhi;
		sc->eyeTheta += v->neckTheta;
		sc->eyePsi   += v->neckPsi;
		glRotatef( -sc->eyePhi  *CF_RAD2DEG, 1.0, 0.0, 0.0 );
		glRotatef( -sc->eyeTheta*CF_RAD2DEG, 0.0, 1.0, 0.0 );
		glRotatef( -sc->eyePsi  *CF_RAD2DEG, 0.0, 0.0, 1.0 );
		break;

	case VIEW_GROUND:
		sc->eyeLat = o->datumLat + v->seat[0]*C_FT2NM/60.0;
		sc->eyeLon = o->datumLon + v->seat[1]*C_FT2NM/60.0/sc->cosDatumLat;
		sc->eyeAlt = o->datumAlt - v->seat[2];
		if( numberSelected( sc, &pickWay ) != 1 || v->followWaypoints == 0 ) {
			dx[0] = ( o->latitude  - sc->eyeLat )*C_NM2FT*60.0;
			dx[1] = hmodDeg( o->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat;
			dx[2] = -o->altitudeMSL + sc->eyeAlt;
		} else {
			double mx[3];
			waypointLocate( gi, pickWay, mx );
			dx[0] = ( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0 + mx[0];
			dx[1] = hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat + mx[1];
			dx[2] = -o->terrainAlt + mx[2] + sc->eyeAlt;
		}
		if( v->lookat == VIEW_LOOKAT_VEHICLE ) {
			sc->eyePhi   = 0.0;
			sc->eyeTheta = (float)atan2( -dx[2], sqrt( dx[0]*dx[0] + dx[1]*dx[1] ) );
			sc->eyePsi   = (float)atan2( dx[1], dx[0] );
		} else {
			sc->eyePhi   = 0.0;
			sc->eyeTheta = 0.0;
			sc->eyePsi   = 0.0;
		}
		sc->eyePhi   += v->neckPhi;
		sc->eyeTheta += v->neckTheta;
		sc->eyePsi   += v->neckPsi;
		glRotatef( -sc->eyePhi  *CF_RAD2DEG, 1.0, 0.0, 0.0 );
		glRotatef( -sc->eyeTheta*CF_RAD2DEG, 0.0, 1.0, 0.0 );
		glRotatef( -sc->eyePsi  *CF_RAD2DEG, 0.0, 0.0, 1.0 );
		/*v->zoom = 3.0*25.0/sqrt( dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2] );
		if( v->zoom < 0.1 ) v->zoom = 0.1;*/
		break;

	case VIEW_HOVER:
		sc->eyeLat = v->seat[0]*C_FT2NM/60.0;
		sc->eyeLon = v->seat[1]*C_FT2NM/60.0/sc->cosDatumLat;
		sc->eyeAlt = - v->seat[2];
		if( numberSelected( sc, &pickWay ) != 1 || v->followWaypoints == 0 ) {
			dx[0] = ( o->latitude  - sc->eyeLat )*C_NM2FT*60.0;
			dx[1] = hmodDeg( o->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat;
			dx[2] = -o->altitudeMSL + sc->eyeAlt;
		} else {
			double mx[3];
			waypointLocate( gi, pickWay, mx );
			dx[0] = ( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0 + mx[0];
			dx[1] = hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat + mx[1];
			dx[2] = -o->terrainAlt + mx[2] + sc->eyeAlt;
		}
		dist = sqrt( dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2] );
		if( sim.mode == SIM_MODE_INIT ) {
			v->seat[0] = (float)(( o->latitude  )*C_NM2FT*60.0                 + gi->set->chase->seat[0]);
			v->seat[1] = (float)(( o->longitude )*C_NM2FT*60.0*sc->cosDatumLat + gi->set->chase->seat[1]);
			v->seat[2] = (float)(-o->altitudeMSL + gi->set->chase->seat[2]);
		} else if ( dist > v->jumpStep ) {
			v->seat[0] += (float)(1.5*dx[0]);
			v->seat[1] += (float)(1.5*dx[1]);
			v->seat[2] += (float)(1.5*dx[2]);
			v->seat[2] = MIN( v->seat[2], -(float)o->terrainAlt );
		}
		sc->eyeLat =  v->seat[0]*C_FT2NM/60.0;
		sc->eyeLon =  v->seat[1]*C_FT2NM/60.0/sc->cosDatumLat;
		sc->eyeAlt = -v->seat[2];
		if( numberSelected( sc, &pickWay ) != 1 || v->followWaypoints == 0 ) {
			dx[0] = ( o->latitude  - sc->eyeLat )*C_NM2FT*60.0;
			dx[1] = hmodDeg( o->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat;
			dx[2] = -o->altitudeMSL + sc->eyeAlt;
		} else {
			double mx[3];
			waypointLocate( gi, pickWay, mx );
			dx[0] = ( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0 + mx[0];
			dx[1] = hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat + mx[1];
			dx[2] = -o->terrainAlt + mx[2] + sc->eyeAlt;
		}

		if( v->lookat == VIEW_LOOKAT_VEHICLE ) {
			sc->eyePhi   = 0.0;
			sc->eyeTheta = (float)atan2( -dx[2], sqrt( dx[0]*dx[0] + dx[1]*dx[1] ) );
			sc->eyePsi   = (float)atan2( dx[1], dx[0] );
		} else {
			sc->eyePhi   = 0.0;
			sc->eyeTheta = 0.0;
			sc->eyePsi   = 0.0;
		}
		/*phi=0-- glRotatef( -sc->eyePhi*CF_RAD2DEG,   1.0, 0.0, 0.0 );*/
		glRotatef( -sc->eyeTheta*CF_RAD2DEG, 0.0, 1.0, 0.0 );
		glRotatef( -sc->eyePsi*CF_RAD2DEG,   0.0, 0.0, 1.0 );
		/*v->zoom = 3.0*25.0/sqrt( dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2] );
		if( v->zoom < 0.1 ) v->zoom = 0.1;*/
		break;
	}

	/* fix quadrants for eyeLon */
	sc->eyeLon = hmodDeg( sc->eyeLon );

	/* stars */

	if( gsc->viewMode != VIEW_NAV && sc->videoModeDerived == 0 && sc->showStars && knowView ) {
		drawStars( sc, sg );
	}

	/* position lighting source */

	sg->sunPosition[0] = (float)(sin( -sc->eyeLat*C_DEG2RAD ));
	sg->sunPosition[1] = (float)( sin( ( -sc->eyeLon + sg->sunAngle )*C_DEG2RAD )*sc->cosDatumLat);
	sg->sunPosition[2] = (float)(-cos( ( -sc->eyeLon + sg->sunAngle )*C_DEG2RAD )*sc->cosDatumLat);

    glLightfv( GL_LIGHT0, GL_POSITION, sg->sunPosition );

	/* Earth */

	if( gsc->viewMode != VIEW_NAV ) {
		if( sc->videoModeDerived == 0 && knowView ) drawEarth( gsc->showTex, scMain->etexture, sc, sg, o->terrainAlt );
	} else {
		if( gsc->showTex && scMain->etexture ) { /* plop down Earth texture */
			float white[4] = {1,1,1,1};
			double longitude, latitude, scalex, scaley;
			double dx, dy, dxB, dyB;

		    glEnable( GL_TEXTURE_2D );
		    glBindTexture( GL_TEXTURE_2D, scMain->etexture );
			glPushMatrix();
			glRotatef( (float)(sc->eyePsi*C_RAD2DEG), 0, 0, 1 );
			glRotatef( gsc->angle3D, 0, 1, 0 );
			glScalef( (float)cos( gsc->angle3D*C_DEG2RAD ), 1, 1 );
			glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );
    		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
		    glShadeModel( GL_SMOOTH );

			scalex = v->zoom;
			scaley = v->zoom*sc->winw/sc->winh;

			glBegin( GL_QUADS );
			glNormal3f( 0.0, 0.0, -1.0 );

			for( i=-5; i<5; i++ ) {
				for( j=-5; j<5; j++ ) {
					dx = (double)i/5*scalex/cos( gsc->angle3D*C_DEG2RAD );
					dy = (double)j/5*scaley;
					dxB = dx*cos( sc->eyePsi ) - dy*sin( sc->eyePsi );
					dyB = dy*cos( sc->eyePsi ) + dx*sin( sc->eyePsi );
					latitude  = sc->eyeLat + dxB*C_FT2NM/60;
					longitude = sc->eyeLon + dyB*C_FT2NM/60/sc->cosDatumLat;
					glTexCoord2f( (float)(0.5 + 0.5*longitude/180), (float)LIMIT(0.5 + latitude/180, 0, 1) );
					glNormal3f( 0, (float)sin( ( longitude - sc->eyeLon )*C_DEG2RAD ), -(float)cos( ( longitude - sc->eyeLon )*C_DEG2RAD ) );
					glVertex3f( (float)dx, (float)dy, 0.0 );

					dx = (double)i/5*scalex/cos( gsc->angle3D*C_DEG2RAD );
					dy = (double)(j+1)/5*scaley;
					dxB = dx*cos( sc->eyePsi ) - dy*sin( sc->eyePsi );
					dyB = dy*cos( sc->eyePsi ) + dx*sin( sc->eyePsi );
					latitude  = sc->eyeLat + dxB*C_FT2NM/60;
					longitude = sc->eyeLon + dyB*C_FT2NM/60/sc->cosDatumLat;
					glTexCoord2f( (float)(0.5 + 0.5*longitude/180), (float)LIMIT(0.5 + latitude/180, 0, 1) );
					glNormal3f( 0, (float)sin( ( longitude - sc->eyeLon )*C_DEG2RAD ), -(float)cos( ( longitude - sc->eyeLon )*C_DEG2RAD ) );
					glVertex3f( (float)dx, (float)dy, 0.0 );

					dx = (double)(i+1)/5*scalex/cos( gsc->angle3D*C_DEG2RAD );
					dy = (double)(j+1)/5*scaley;
					dxB = dx*cos( sc->eyePsi ) - dy*sin( sc->eyePsi );
					dyB = dy*cos( sc->eyePsi ) + dx*sin( sc->eyePsi );
					latitude  = sc->eyeLat + dxB*C_FT2NM/60;
					longitude = sc->eyeLon + dyB*C_FT2NM/60/sc->cosDatumLat;
					glTexCoord2f( (float)(0.5 + 0.5*longitude/180), (float)LIMIT(0.5 + latitude/180, 0, 1) );
					glNormal3f( 0, (float)sin( ( longitude - sc->eyeLon )*C_DEG2RAD ), -(float)cos( ( longitude - sc->eyeLon )*C_DEG2RAD ) );
					glVertex3f( (float)dx, (float)dy, 0.0 );

					dx = (double)(i+1)/5*scalex/cos( gsc->angle3D*C_DEG2RAD );
					dy = (double)j/5*scaley;
					dxB = dx*cos( sc->eyePsi ) - dy*sin( sc->eyePsi );
					dyB = dy*cos( sc->eyePsi ) + dx*sin( sc->eyePsi );
					latitude  = sc->eyeLat + dxB*C_FT2NM/60;
					longitude = sc->eyeLon + dyB*C_FT2NM/60/sc->cosDatumLat;
					glTexCoord2f( (float)(0.5 + 0.5*longitude/180), (float)LIMIT(0.5 + latitude/180, 0, 1) );
					glNormal3f( 0, (float)sin( ( longitude - sc->eyeLon )*C_DEG2RAD ), -(float)cos( ( longitude - sc->eyeLon )*C_DEG2RAD ) );
					glVertex3f( (float)dx, (float)dy, 0.0 );
				}
			}

			glEnd();
			glPopMatrix();

			glDisable( GL_TEXTURE_2D );
		}
	}

	/* atmosphere */

	if( gsc->viewMode != VIEW_NAV && sc->videoModeDerived == 0 && knowView )
		drawAtmosphere( sc, sg, o->terrainAlt );

	/* fog */

	if( sg->fog && gsc->viewMode != VIEW_NAV )
		glEnable( GL_FOG );
	else
		glDisable( GL_FOG );

	/* surface grass */

	if( o->altitudeAGL < sg->vis && gsc->showTex && sc->videoModeDerived == 0 && sc->showGrass && knowView ) {
		glPushMatrix();
		dx[0] = ( sc->eyeLat - o->datumLat )*C_NM2FT*60.0;
		dx[1] = hmodDeg( sc->eyeLon - o->datumLon )*C_NM2FT*sc->cosDatumLat*60.0;
		if( gsc->viewMode != VIEW_NAV ) {
			dx[2] = ABS( ( sc->eyeAlt - o->terrainAlt )/tan( MIN( sc->eyeTheta, -sc->fovy*v->zoom*0.25*C_DEG2RAD ) ) );
		} else {
			dx[2] = 0;
		}
		glTranslatef(
			(float)( sg->gridx*(int)( ( dx[0] + dx[2]*cos( sc->eyePsi ) )/sg->gridx + 0.5 ) - dx[0] ),
			(float)( sg->gridx*(int)( ( dx[1] + dx[2]*sin( sc->eyePsi ) )/sg->gridx + 0.5 ) - dx[1] ),
			(float)( -o->terrainAlt + sc->eyeAlt ));
		glCallList( sg->grass_dl );
		glPopMatrix();
	}

	/* google maps */

	if( gsc->showTex && sc->videoModeDerived == 0 && knowView &&
		( sg->overlaySource == OVERLAY_GOOGLETYPE1 || sg->overlaySource == OVERLAY_GOOGLETYPE2 ||
		  sg->overlaySource == OVERLAY_GOOGLETYPEUNKNOWN || sg->overlaySource == OVERLAY_GOOGLETYPE1_AND_KML ||
		  sg->overlaySource == OVERLAY_BINGAERIAL || sg->overlaySource == OVERLAY_BINGAERIALWITHLABELS ||
		  sg->overlaySource == OVERLAY_BINGUNKNOWN || sg->overlaySource == OVERLAY_BING_AND_KML ) ) {

		struct internetOverlay_ref *io = sg->io;
		double dz;
		unsigned int newTexture[9];
		char tryLoading, bing = 0;

		switch( sg->overlaySource ) {
		case OVERLAY_GOOGLETYPE1:
		case OVERLAY_GOOGLETYPE1_AND_KML:
			io->mapType = 1;
			break;
		case OVERLAY_GOOGLETYPE2:
			io->mapType = 2;
			break;
		case OVERLAY_BINGAERIAL:
			bing = 1;
			io->mapType = 4; /* this use of mapType will force it to re-load if you switch sources */
			break;
		case OVERLAY_BINGAERIALWITHLABELS:
		case OVERLAY_BING_AND_KML:
			bing = 1;
			io->mapType = 5;
			break;
		case OVERLAY_BINGUNKNOWN:
			bing = 1;
			break;
		default: /* leave alone */
			break;
		}

		if( gsc->viewMode != VIEW_NAV ) {
			dz = ABS( ( sc->eyeAlt - o->terrainAlt )/tan( MIN( sc->eyeTheta, -sc->fovy*v->zoom*0.25*C_DEG2RAD ) ) );
		} else {
			dz = 0;
		}

		if( io->autoZoom ) {
			int screenSize;

			screenSize = MAX( sc->winw, sc->winh );
			io->tileScenario = (int)(screenSize/io->size/2+1)*2 + 1;  /* tileScenario x tileScenario tiles */
			io->tileScenario = LIMIT( io->tileScenario, 1, io->maxTileScenario );

			if( gsc->viewMode == VIEW_NAV ) {
				io->requiredWidth = v->zoom*sc->winw/sc->winh;
			} else {
				io->requiredWidth = sc->fovy*v->zoom*C_DEG2RAD*dz*sc->winw/sc->winh*2;
				/* times 2 just because we tend to want bigger maps here */
			}
			io->zoom = (int)( log( C_NM2FT*60*360*sc->cosDatumLat/io->requiredWidth*io->size*io->tileScenario/256 )/log(2) - io->fudge );
			if( sc->map_higherZoom ) {
				sc->map_higherZoom = MIN( sc->map_higherZoom, io->zoom - io->zoomMin );
				io->zoom -= sc->map_higherZoom; /* try higher level */
			}
			io->zoom = LIMIT( io->zoom, io->zoomMin, io->zoomMax );

			/* recompute tile scenario based on actual width of tiles */
			io->width        = C_NM2FT*60*360*cos( io->lat*C_DEG2RAD )*io->size/256/pow( 2, io->zoom );
			io->tileScenario = (int)(io->requiredWidth/io->width/2+1)*2 + 1;
			io->tileScenario = LIMIT( io->tileScenario, 1, io->maxTileScenario );
		}

		/* identify center tile */
		io->latprec = 360/pow( 2, io->zoom )*io->size/256/2;
		io->latprec = floor( io->latprec*1000000 )/1000000; /* 6 digit precision */
		io->lat = round( ( sc->eyeLat + dz*cos( sc->eyePsi )*C_FT2NM/60                 )/io->latprec )*io->latprec;
		io->lon = round( ( sc->eyeLon + dz*sin( sc->eyePsi )*C_FT2NM/60/sc->cosDatumLat )/io->latprec )*io->latprec;

		tryLoading = 0;
		if( io->lat != sc->oldlat || io->lon != sc->oldlon || io->zoom != sc->oldzoom || io->mapType != sc->oldmapType || sc->gmtexture[0] == 0 ) {
			sc->oldzoom    = io->zoom;
			sc->oldlat     = io->lat;
			sc->oldlon     = io->lon;
			sc->oldmapType = io->mapType;
			tryLoading = 1;
			/* first time map moved after a valid load */
			if( sc->map_tryHigher == 0 ) sc->map_higherZoom = MAX( 0, sc->map_higherZoom - 1 );
		} else {
			if( sc->map_tryHigher ) {
				tryLoading = 1;
				sc->map_tryHigher++;  /* retry counter */
				if( io->loaderThreadRunning == 0 ) {
					/* not trying to load a map, but know we are waiting for one - let's try loading it */
				}
				if( (sc->map_tryHigher%MAX(1,io->retryPeriod)) == 0 ) {
					sc->map_higherZoom++;
					tryLoading = 0; /* no point, this is no longer the map we want - get it next time */
				}
			}
		}

		if( tryLoading ) {

			int tile, ix, iy;
			char fileName[120];
			double lat, lon;

			io->successFlag = 1;
			for( tile=0; tile<SQ(io->tileScenario) && io->successFlag; tile++ ) {

				switch( io->tileScenario ) {
				default:
				case 0:
					ix = 0;  iy = 0;
					break;
				case 3:
					switch( tile ) {
					default:
					case 0:  ix = 1;  iy = -1; break;
					case 1:  ix = 1;  iy = 0;  break;
					case 2:  ix = 1;  iy = 1;  break;
					case 3:  ix = 0;  iy = -1; break;
					case 4:  ix = 0;  iy = 0;  break;
					case 5:  ix = 0;  iy = 1;  break;
					case 6:  ix = -1; iy = -1; break;
					case 7:  ix = -1; iy = 0;  break;
					case 8:  ix = -1; iy = 1;  break;
					}
					break;
				}

				lat = ( round( ( sc->eyeLat + dz*cos( sc->eyePsi )*C_FT2NM/60                 )/io->latprec ) + ix   )*io->latprec;
				lon = ( round( ( sc->eyeLon + dz*sin( sc->eyePsi )*C_FT2NM/60/sc->cosDatumLat )/io->latprec ) + iy*2 )*io->latprec;

				sprintf( fileName, "%s/tile_%.6f_%.6f_z%d_t%d.png", io->cachePath, lat, lon, io->zoom, io->mapType );

			}

			if( io->successFlag ) {
				int gridn;
				float x[4][3]; //, a[3], b[3];
				double cosGmLat;

				sc->map_tryHigher = 0;
				sc->gmLat = io->lat;
				sc->gmLon = io->lon;
				cosGmLat = cos( io->lat*C_DEG2RAD );
				sg->dted->error = DTED_ERROR_NONE; /* clear error flag, try again */
				sc->gmZoom = io->zoom;
				io->width = C_NM2FT*60*360/pow( 2, io->zoom )*cos( io->lat*C_DEG2RAD )*io->size/256;

				for( tile=0; tile<SQ(io->tileScenario); tile++ ) {
					glDeleteTextures(1,&(sc->gmtexture[tile]));
					sc->gmtexture[tile] = newTexture[tile];
				}

				if( !glIsList( sc->gm_dl ) ) sc->gm_dl = glGenLists( 1 );
				glNewList( sc->gm_dl, GL_COMPILE );
				glShadeModel( GL_SMOOTH );

				gridn = (int)sg->overlayGridn;

				glEnable( GL_TEXTURE_2D );

				glPushMatrix();
				glRotatef( 90.0f, 0, 0, 1 ); /* rotated 90 degrees! */
				glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white );
				glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

				for( tile=0; tile<SQ(io->tileScenario); tile++ ) {

					switch( io->tileScenario ) {
					default:
					case 0:
						ix = 0;  iy = 0;
						break;
					case 3:
						switch( tile ) {
						default:
						case 0:  ix = 1;  iy = -1; break;
						case 1:  ix = 1;  iy = 0;  break;
						case 2:  ix = 1;  iy = 1;  break;
						case 3:  ix = 0;  iy = -1; break;
						case 4:  ix = 0;  iy = 0;  break;
						case 5:  ix = 0;  iy = 1;  break;
						case 6:  ix = -1; iy = -1; break;
						case 7:  ix = -1; iy = 0;  break;
						case 8:  ix = -1; iy = 1;  break;
						}
						break;
					}

					lat = ( round( ( sc->eyeLat + dz*cos( sc->eyePsi )*C_FT2NM/60                 )/io->latprec ) + ix   )*io->latprec;
					lon = ( round( ( sc->eyeLon + dz*sin( sc->eyePsi )*C_FT2NM/60/sc->cosDatumLat )/io->latprec ) + iy*2 )*io->latprec;

					glBindTexture( GL_TEXTURE_2D, sc->gmtexture[tile] );

					glPushMatrix();
					glTranslatef( (float)(iy*2*io->latprec*C_NM2FT*60*sc->cosDatumLat), (float)(-ix*io->latprec*C_NM2FT*60), 0 );

					for( i=-gridn; i<gridn; i++ ) {
						for( j=-gridn; j<gridn; j++ ) {
							if( 1 == sg->useSRTM ) {

								/* would be good take this great circle */

								/* get all verticies */
								/* this could be sped up by not re-looking up every corner four times... */
								x[0][0] = (float)(+0.5f*io->width* i   /gridn);
								x[1][0] = (float)(+0.5f*io->width*(i+1)/gridn);
								x[2][0] = (float)(+0.5f*io->width*(i+1)/gridn);
								x[3][0] = (float)(+0.5f*io->width* i   /gridn);
								x[0][1] = (float)(-0.5f*io->width* j   /gridn);
								x[1][1] = (float)(-0.5f*io->width* j   /gridn);
								x[2][1] = (float)(-0.5f*io->width*(j+1)/gridn);
								x[3][1] = (float)(-0.5f*io->width*(j+1)/gridn);
								x[0][2] = -sceneGetTerrainH( sg->dted, lat + 0.5*io->width* j   /gridn*C_FT2NM/60, lon + 0.5*io->width* i   /gridn*C_FT2NM/60/cosGmLat, o->terrainAlt );
								x[1][2] = -sceneGetTerrainH( sg->dted, lat + 0.5*io->width* j   /gridn*C_FT2NM/60, lon + 0.5*io->width*(i+1)/gridn*C_FT2NM/60/cosGmLat, o->terrainAlt );
								x[2][2] = -sceneGetTerrainH( sg->dted, lat + 0.5*io->width*(j+1)/gridn*C_FT2NM/60, lon + 0.5*io->width*(i+1)/gridn*C_FT2NM/60/cosGmLat, o->terrainAlt );
								x[3][2] = -sceneGetTerrainH( sg->dted, lat + 0.5*io->width*(j+1)/gridn*C_FT2NM/60, lon + 0.5*io->width* i   /gridn*C_FT2NM/60/cosGmLat, o->terrainAlt );

								/* get normal */
								/* this didn't work very well, needs to be computed per vertext to look smooth - also the google map is already shaded anyway... */
								/*a[0] = x[1][0] - x[0][0];  a[1] = x[1][1] - x[0][1];  a[2] = x[1][2] - x[0][2];
								b[0] = x[2][0] - x[0][0];  b[1] = x[2][1] - x[0][1];  b[2] = x[2][2] - x[0][2];
								glNormal3f( a[1]*b[2] - a[2]*b[1],
											a[2]*b[0] - a[0]*b[2],
											a[0]*b[1] - a[1]*b[0] );*/

								/* draw it */
								glBegin( GL_QUADS );
								glNormal3f( 0.0, 0.0, -1.0 );
								glTexCoord2f( 0.5f* i   /gridn + 0.5f, 0.5f* j   /gridn + 0.5f );  glVertex3fv( x[0] );
								glTexCoord2f( 0.5f*(i+1)/gridn + 0.5f, 0.5f* j   /gridn + 0.5f );  glVertex3fv( x[1] );
								glTexCoord2f( 0.5f*(i+1)/gridn + 0.5f, 0.5f*(j+1)/gridn + 0.5f );  glVertex3fv( x[2] );
								glTexCoord2f( 0.5f* i   /gridn + 0.5f, 0.5f*(j+1)/gridn + 0.5f );  glVertex3fv( x[3] );
								glEnd();

							} else {
								glBegin( GL_QUADS );
								glNormal3f( 0.0, 0.0, -1.0 );

								glTexCoord2f( 0.5f* i   /gridn + 0.5f, 0.5f* j   /gridn + 0.5f );
								glVertex3f( (float)(0.5f*io->width* i   /gridn), (float)(-0.5f*io->width* j   /gridn), 0 );

								glTexCoord2f( 0.5f*(i+1)/gridn + 0.5f, 0.5f* j   /gridn + 0.5f );
								glVertex3f( (float)(0.5f*io->width*(i+1)/gridn), (float)(-0.5f*io->width* j   /gridn), 0 );

								glTexCoord2f( 0.5f*(i+1)/gridn + 0.5f, 0.5f*(j+1)/gridn + 0.5f );
								glVertex3f( (float)(0.5f*io->width*(i+1)/gridn), (float)(-0.5f*io->width*(j+1)/gridn), 0 );

								glTexCoord2f( 0.5f* i   /gridn + 0.5f, 0.5f*(j+1)/gridn + 0.5f );
								glVertex3f( (float)(0.5f*io->width* i   /gridn), (float)(-0.5f*io->width*(j+1)/gridn), 0 );

								glEnd();
							}
						}
					}
					glPopMatrix();
					if( 1 == sg->useSRTM && DTED_ERROR_NONE == sg->dted->error ) sc->usedSRTM = 1;
					else                                                         sc->usedSRTM = 0;

				}

				glDisable( GL_TEXTURE_2D );

				glPopMatrix();
				glEndList();
			}
		}

		/* now actually draw it */
        if( sc->gmtexture ) {
			glPushMatrix();
			if( sc->usedSRTM ) {
				glEnable( GL_DEPTH_TEST );
				glTranslatef( (float)(( sc->gmLat - sc->eyeLat )*C_NM2FT*60.0),
					(float)(hmodDeg( sc->gmLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(sc->eyeAlt) );
				glScalef( 1, (float)(sc->cosDatumLat/cos( sc->gmLat*C_DEG2RAD )), 1 ); /* stretch in case way off of the datum, normally not important */
				glCallList( sc->gm_dl );
				glDisable( GL_DEPTH_TEST );
			} else {
				glTranslatef( (float)(( sc->gmLat - sc->eyeLat )*C_NM2FT*60.0),
					(float)(hmodDeg( sc->gmLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(-o->terrainAlt + sc->eyeAlt) );
				glScalef( 1, (float)(sc->cosDatumLat/cos( sc->gmLat*C_DEG2RAD )), 1 ); /* stretch in case way off of the datum, normally not important */
				glCallList( sc->gm_dl );
			}
			glPopMatrix();
        }
	}

	/* kml ground textures */

	if( gsc->showTex && sc->videoModeDerived == 0 && knowView &&
		( sg->overlaySource == OVERLAY_KML || sg->overlaySource == OVERLAY_GOOGLETYPE1_AND_KML || sg->overlaySource == OVERLAY_BING_AND_KML ) ) {

	    struct sceneOverlay_ref *m;

        for( i=0; i<SCENE_NUMBEROFOVERLAYS; i++ ) {
            switch( i ) {
				case 0: default: m = sg->overlays->m0; break;
                case 1:          m = sg->overlays->m1; break;
                case 2:          m = sg->overlays->m2; break;
                case 3:          m = sg->overlays->m3; break;
                case 4:          m = sg->overlays->m4; break;
                case 5:          m = sg->overlays->m5; break;
                case 6:          m = sg->overlays->m6; break;
                case 7:          m = sg->overlays->m7; break;
                case 8:          m = sg->overlays->m8; break;
                case 9:          m = sg->overlays->m9; break;
            }
            if( m->successFlag ) {
				glPushMatrix();
				if( 0 ) { /* currently commented out, because not rendered using terrain model   sc->usedSRTM ) { */
					glEnable( GL_DEPTH_TEST );
					glTranslatef( (float)(( m->lat - sc->eyeLat )*C_NM2FT*60.0 + m->offset[0]),
						(float)(hmodDeg( m->lon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat + m->offset[1]),
						(float)(sc->eyeAlt) );
					glCallList( m->dl );
					glDisable( GL_DEPTH_TEST );
				} else {
					glTranslatef( (float)(( m->lat - sc->eyeLat )*C_NM2FT*60.0 + m->offset[0]),
						(float)(hmodDeg( m->lon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat + m->offset[1]),
						(float)(-o->terrainAlt + sc->eyeAlt) );
					glCallList( m->dl );
				}
				glPopMatrix();
            }
        }
	}

	/* slam map as an "alternate" ground texture */
	//if( sc->showSLAM && knowView ) drawSlam( sg, sc, gi->outputs );

	/* surface grid */

	if( sc->showGrid && knowView ) {
		if( gsc->viewMode != VIEW_NAV || sg->gridmax*sg->gridx*sc->winh > v->zoom*2 ) {
			float gridExtent, unitConversion;
			char buffer[20], units[20];
			dx[0] = ( sc->eyeLat - o->datumLat )*C_NM2FT*60.0;
			dx[1] = hmodDeg( sc->eyeLon - o->datumLon )*C_NM2FT*sc->cosDatumLat*60.0;
			if( gsc->viewMode != VIEW_NAV ) {
				dx[2] = ABS( ( sc->eyeAlt - o->terrainAlt )/tan( MIN( sc->eyeTheta, -sc->fovy*v->zoom*0.25*C_DEG2RAD ) ) );
				gridExtent = (float)sqrt( SQ( dx[2] ) + SQ( sc->eyeAlt - o->terrainAlt ) )*sc->fovy*v->zoom*CF_DEG2RAD*MAX( sc->winh, sc->winw )/sc->winh;
			} else {
				dx[2] = 0;
				gridExtent = v->zoom*2;
			}
			if( sc->yzoom != -1 ) glLineWidth( sg->gridLW[1] );
			else                  glLineWidth( sg->gridLW[0] );

			if( sg->showGridScale ) {
				switch( sg->distanceUnits ) {
				default:
				case DISTANCE_FT:
				case DISTANCE_NM:
				case DISTANCE_SM:
					unitConversion = 1;
					sprintf( units, "ft" );
					break;
				case DISTANCE_M:
				case DISTANCE_KM:
					unitConversion = CF_FT2M;
					sprintf( units, "m" );
					break;
				}
			}

			if( sg->gridDraw100x && gridExtent > sg->gridx*sg->gridmax*sg->gridmax ) {
				glPushMatrix();
				glTranslatef(
					(float)( sg->gridx*sg->gridmax*sg->gridmax*(int)( ( dx[0] + dx[2]*cos( sc->eyePsi ) )/sg->gridx/sg->gridmax/sg->gridmax + 0.5 ) - dx[0] ),
					(float)( sg->gridx*sg->gridmax*sg->gridmax*(int)( ( dx[1] + dx[2]*sin( sc->eyePsi ) )/sg->gridx/sg->gridmax/sg->gridmax + 0.5 ) - dx[1] ),
					(float)( -o->terrainAlt + sc->eyeAlt ));
				glScalef( (float)sg->gridmax*sg->gridmax, (float)sg->gridmax*sg->gridmax, 1.0 );
				glCallList( sg->grid_dl );
				if( sg->showGridScale ) {
					sprintf( buffer, "%g%s", sg->gridx*sg->gridmax*sg->gridmax*unitConversion, units );
					glRotatef( 180, 1, 1, 0 );
					glTranslatef( 0.5f*sg->gridx - strlen( buffer )*0.0525f*sg->gridx, 1.05f*sg->gridx, 0 ); 
					glScalef( 0.1f*sg->gridx/100, 0.1f*sg->gridx/100, 1 );
					drawText( buffer );
				}
				glPopMatrix();
			}

			if( sg->gridDraw10x && gridExtent > sg->gridx*sg->gridmax ) {
				glPushMatrix();
				glTranslatef(
					(float)( sg->gridx*sg->gridmax*(int)( ( dx[0] + dx[2]*cos( sc->eyePsi ) )/sg->gridx/sg->gridmax + 0.5 ) - dx[0] ),
					(float)( sg->gridx*sg->gridmax*(int)( ( dx[1] + dx[2]*sin( sc->eyePsi ) )/sg->gridx/sg->gridmax + 0.5 ) - dx[1] ),
					(float)( -o->terrainAlt + sc->eyeAlt ));
				glScalef( (float)sg->gridmax, (float)sg->gridmax, 1.0 );
				glCallList( sg->grid_dl );
				if( sg->showGridScale ) {
					sprintf( buffer, "%g%s", sg->gridx*sg->gridmax*unitConversion, units );
					glRotatef( 180, 1, 1, 0 );
					glTranslatef( 0.5f*sg->gridx - strlen( buffer )*0.0525f*sg->gridx, 1.05f*sg->gridx, 0 ); 
					glScalef( 0.1f*sg->gridx/100, 0.1f*sg->gridx/100, 1 );
					drawText( buffer );
				}
				glPopMatrix();
			}

			glPushMatrix();
			glTranslatef(
				(float)( sg->gridx*(int)( ( dx[0] + dx[2]*cos( sc->eyePsi ) )/sg->gridx + 0.5 ) - dx[0] ),
				(float)( sg->gridx*(int)( ( dx[1] + dx[2]*sin( sc->eyePsi ) )/sg->gridx + 0.5 ) - dx[1] ),
				(float)( -o->terrainAlt + sc->eyeAlt ));
			glCallList( sg->grid_dl );
			if( sg->showGridScale ) {
				sprintf( buffer, "%g%s", sg->gridx*unitConversion, units );
				glRotatef( 180, 1, 1, 0 );
				glTranslatef( 0.5f*sg->gridx - strlen( buffer )*0.0525f*sg->gridx, 1.05f*sg->gridx, 0 ); 
				glScalef( 0.1f*sg->gridx/100, 0.1f*sg->gridx/100, 1 );
				drawText( buffer );
			}
			glPopMatrix();

			glLineWidth( 1.0 );

		}
	}

	/* everything else... */

    /*glMaterialfv( GL_FRONT, GL_SPECULAR, sg->sunSpecular );*/

	/* runway */

    if( sg->showRunway && sc->videoModeDerived == 0 && knowView ) {
        glPushMatrix();
        glTranslatef( (float)(( sg->runwayLat - sc->eyeLat )*C_NM2FT*60.0),
            (float)(hmodDeg(sg->runwayLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
            (float)(-sg->runwayAlt + sc->eyeAlt) );
        glRotatef( sg->runwayCrs, 0.0, 0.0, 1.0 );
        glCallList( sg->runway_dl );
        glPopMatrix();
    }

	glEnable( GL_DEPTH_TEST );

	/* 3-D objects */

	/* trajectory */
	if( sc->showTraj && knowView ) {

		char ig;
		struct gcsInstance_ref *gis;

		glDisable( GL_LIGHTING );
		/*glDisable( GL_FOG );*/
		if( sc->showTruth ) {
			glColor4fv( sg->trajColor );
			glBegin( GL_LINE_STRIP );
			for( i=0; i<sg->ntpoints; i++ ) {
				glVertex3f( (float)(( tpoints[i].lat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(hmodDeg( tpoints[i].lon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(-tpoints[i].alt + sc->eyeAlt) );
			}
			glEnd();
		}

		if( sc->showGCS ) {
			for( ig=0; ig<GCS_MAX_INSTANCES; ig++ ) {
				gis = gcsGetInstance( g, ig );
				if( gis->run ) {

					if( ( sc->showTruth == 0 || sg->showBothTraj ) && ( sg->showAllTraj || g->active == ig ) ) {
						if( ( sg->showBothTraj && sc->showTruth == 1 ) || g->active != ig ) glColor4fv( sg->trajColor2 );
						else                                                                glColor4fv( sg->trajColor );
						glBegin( GL_LINE_STRIP );
						for( i=0; i<sg->ntpoints_nav[ig]; i++ )
							glVertex3f( (float)(( tpoints_nav[ig][i].lat - sc->eyeLat )*C_NM2FT*60.0),
							(float)(hmodDeg( tpoints_nav[ig][i].lon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
							(float)(-tpoints_nav[ig][i].alt + sc->eyeAlt) );
						glEnd();
					}

					if( sc->showGpsTraj ) {
						if( g->active != ig ) glColor4fv( sg->trajColorGps2 );
						else                  glColor4fv( sg->trajColorGps  );
						glBegin( GL_LINE_STRIP );
						for( i=0; i<sg->ntpoints_gps[ig]; i++ )
							glVertex3f( (float)(( tpoints_gps[ig][i].lat - sc->eyeLat )*C_NM2FT*60.0),
							(float)(hmodDeg( tpoints_gps[ig][i].lon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
							(float)(-tpoints_gps[ig][i].alt + sc->eyeAlt) );
						glEnd();
					}

				}
			}
		}
		glEnable( GL_LIGHTING );
		/*if( sg->fog && gsc->viewMode != VIEW_NAV )
			glEnable( GL_FOG );*/
	}

#ifdef SIKPOT
	if( sikpot.run && sc->showGCS ) {
		extern float pathinspace[3][STEPLIMIT];
		extern int path_length;
		glDisable( GL_LIGHTING );
		glColor4fv( sg->planColor );
		glPushMatrix();
			glTranslatef( (float)(( no->datumLat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(hmodDeg( no->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(-no->datumAlt + sc->eyeAlt) );
			glBegin( GL_LINE_STRIP );
			for( i=0; i<path_length; i++ ) glVertex3f( (float)pathinspace[0][i], (float)pathinspace[1][i], (float)pathinspace[2][i] );
			glEnd();
		glPopMatrix();
		glEnable( GL_LIGHTING );
	}
#endif
#ifdef SORDS
	if( sords.run && sc->showGCS ) {
		//extern float pathinspace2[3][SORDS_PATH_INCREM];
		glDisable( GL_LIGHTING );
		glPushMatrix();

        glTranslatef( (float)(( no->datumLat - sc->eyeLat )*C_NM2FT*60.0),
            (float)(hmodDeg( no->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
            (float)(-no->datumAlt + sc->eyeAlt) );


        /* draws the sords planned path */
        glColor4fv( sg->planColor );
        glBegin( GL_LINE_STRIP );
        for( i=0; i<SORDS_PATH_INCREM; i++ ) glVertex3f( (float)pathinspace2[0][i], (float)pathinspace2[1][i], (float)pathinspace2[2][i] );
        glEnd();

		glPopMatrix();
		glEnable( GL_LIGHTING );
	}

	// Map and search area
	if( mapAndSearch.run && sc->showGCS ) {
		int masi;
        glDisable( GL_LIGHTING );
		glPushMatrix();

        glTranslatef( (float)(( no->datumLat - sc->eyeLat )*C_NM2FT*60.0),
            (float)(hmodDeg( no->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
            (float)(-no->datumAlt + sc->eyeAlt) );


        /* draws the outline of mapandsearch area */
        glColor4fv( sg->masAreaColor );
        glBegin( GL_LINE_STRIP );

        for( masi = 0; masi < mapAndSearchSearchArea.numVertices; masi++ ) {
            glVertex3f( mapAndSearchSearchArea.vertex[masi][0], mapAndSearchSearchArea.vertex[masi][1], -mapAndSearch.mappingAltitude );
        }
        glVertex3f( mapAndSearchSearchArea.vertex[0][0], mapAndSearchSearchArea.vertex[0][1], -mapAndSearch.mappingAltitude );
//        glVertex3f( mapAndSearchSearchArea.mapOrigin[0], mapAndSearchSearchArea.mapOrigin[1], -mapAndSearch.mappingAltitude );
//        glVertex3f( mapAndSearchSearchArea.mapOrigin[0] + mapAndSearchSearchArea.searchAreaSize[0], mapAndSearchSearchArea.mapOrigin[1], -mapAndSearch.mappingAltitude );
//        glVertex3f( mapAndSearchSearchArea.mapOrigin[0] + mapAndSearchSearchArea.searchAreaSize[0], mapAndSearchSearchArea.mapOrigin[1] + mapAndSearchSearchArea.searchAreaSize[1], -mapAndSearch.mappingAltitude );
//        glVertex3f( mapAndSearchSearchArea.mapOrigin[0], mapAndSearchSearchArea.mapOrigin[1] + mapAndSearchSearchArea.searchAreaSize[1], -mapAndSearch.mappingAltitude );
//        glVertex3f( mapAndSearchSearchArea.mapOrigin[0], mapAndSearchSearchArea.mapOrigin[1], -mapAndSearch.mappingAltitude );
        glEnd();

		glPopMatrix();
		glEnable( GL_LIGHTING );

	}
#endif

	if( knowView ) draw3DObjects( sc, sg, o, rs, ( gsc->viewMode == VIEW_NAV ? -v->zoom*2 : v->zoom ) );

	/* capture images */

	/* this is for the opengl grabber - assume we want "real" scene elements for this purpose */
	if( sc->openglGrabber ) {
		unsigned int requiredSize;
		requiredSize = sc->winw*sc->winh*3;
		if( sc->grabberAllocatedSize < requiredSize ) {
			if( sc->grabberAllocatedSize ) {
				free( sc->grabberData );
			}
			sc->grabberData = malloc( requiredSize );
			sc->grabberAllocatedSize = requiredSize;
		}
	    //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glFinish();
		glReadPixels( 0, 0, sc->winw, sc->winh, GL_BGR, GL_UNSIGNED_BYTE, (GLvoid *)sc->grabberData );
	} else {
		if( sc->grabberAllocatedSize ) {
			free( sc->grabberData );
			sc->grabberAllocatedSize = 0;
		}
	}

	captureImage( sg, sc, &sceneCapture, SCENECAPTURE_REAL );

	/* for the visionNav image processing */
    if( gsc->viewMode == VIEW_CAMERA ) {
        getWindowInfo( &winInfo );
    }

	/* draw GCS information */

	glDisable( GL_LIGHTING );
	glDisable( GL_FOG );

	if( gi->datalink->trajPreview->nofPoints && sc->showGCS && knowView ) {
		glLineWidth( sg->planLineWidth );
		glColor4fv( sg->planColor );
		/* turn off z-buffer check */
		glDisable( GL_DEPTH_TEST );
		glPushMatrix();
			glTranslatef( (float)(( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(-gi->outputs->datumAlt + sc->eyeAlt) );
			glBegin( GL_LINE_STRIP );
			for( i=0; i<gi->datalink->trajPreview->nofPoints; i++ ) {
				if( ISFINITE( gi->datalink->trajPreview->point[i][0] ) &&
					ISFINITE( gi->datalink->trajPreview->point[i][1] ) &&
					ISFINITE( gi->datalink->trajPreview->point[i][2] ) ) {
					glVertex3fv( gi->datalink->trajPreview->point[i] );
				}
			}
			glEnd();
		glPopMatrix();
		/* turn on z-buffer check */
		glEnable( GL_DEPTH_TEST );
		glLineWidth( 1.0);
	}


    //if( sc->showTruth ) drawSlamFeatures( sg, sc, o );

	if( sc->showGCS && knowView ) {

		double zoom, distance;
		float enlarge;
		char step3D;
		int minType = MAN_LANDING;

		glLineWidth( sg->gcsLineWidth );

		if( sg->showAnnotations ) {
			for( j=0; j<SCENE_ANNOTATIONS; j++ ) {
				if( sg->annotate->object[j]->type ) {
					struct sceneAnnotation_ref *obj = sg->annotate->object[j];
					glColor4fv( obj->color );
					glPushMatrix();
					glTranslatef( (float)(( obj->latitude - sc->eyeLat )*C_NM2FT*60.0),
						(float)(hmodDeg( obj->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
						(float)(-gi->outputs->terrainAlt + sc->eyeAlt) );
					switch( obj->type ) {
					case ANNOTATION_CYLINDER:
						glScalef( (float)(obj->radius), (float)(obj->radius), 1 );
						glTranslatef( 0, 0, (float)(-obj->height + gi->outputs->terrainAlt) );
						glCallList( sg->circle_dl );
						glTranslatef( 0, 0, (float)(+obj->height - gi->outputs->terrainAlt) );
						glBegin( GL_QUAD_STRIP );
						for( i=0; i<=360; i+= 9 ) {
							glVertex3f( (float)sin( C_DEG2RAD*i ), (float)cos( C_DEG2RAD*i ), (float)(-obj->height + gi->outputs->terrainAlt) );
							glVertex3f( (float)sin( C_DEG2RAD*i ), (float)cos( C_DEG2RAD*i ), 0 );
						}
						glEnd();
						glColor4fv( sg->shadowColor );
						glBegin( GL_POLYGON );
						for( i=0; i<=360; i+= 9 )
							glVertex3f( (float)sin( C_DEG2RAD*i ), (float)cos( C_DEG2RAD*i ), (float)(-obj->height + gi->outputs->terrainAlt) );
						glEnd();
						break;
					case ANNOTATION_MARKER:
						/* size constraint */
						zoom = ( gsc->viewMode == VIEW_NAV ? -v->zoom*2 : v->zoom );
						if( zoom > 0 ) {
							glGetFloatv( GL_MODELVIEW_MATRIX, &(mvm[0][0]) );
							distance = sqrt( SQ( mvm[3][0] ) + SQ( mvm[3][1] ) + SQ( mvm[3][2] ) );
							enlarge = (float)(MAX( gi->set->waypointR, distance*sg->minWaySize/(2)*zoom/sc->winh ));
						} else {
 							enlarge = (float)(MAX( gi->set->waypointR, -sg->minWaySize/(2)*zoom/sc->winh ));
						}
						glScalef( enlarge, enlarge, enlarge );
						glCallList( sg->waypointP_dl );
						showBitmapMessage( -(float)sin( sc->eyePsi ), (float)cos( sc->eyePsi ), 0, obj->text, LIMIT( sg->wayFont, 2, 8 ) );
						break;
					default:
						break;
					}
					glPopMatrix();
				}
			}
		}

		if( sc->showThreats )
			drawThreats( sg, sc, o );

        //if( sc->showSlamData )
        //    drawSlamData( sg, sc, gi->outputs );

		/* deterimine intersection of pick with ground and print coordinates or move */

		if( sc->pick ) {

			char visible = 1;

			if( gsc->viewMode == VIEW_NAV ) {
				dx[0] = sc->vec[0];
				dx[1] = sc->vec[1];
			} else {
				glPushMatrix();
				glTranslatef( (float)(( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0),
					(float)(hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(-gi->outputs->datumAlt + sc->eyeAlt) );
				glGetFloatv( GL_MODELVIEW_MATRIX, &(mvm[0][0]) );
				glPopMatrix();

				/*
				printf( "%.2f %.2f %.2f %.2f\n", mvm[0][0], mvm[0][1], mvm[0][2], mvm[0][3] );
				printf( "%.2f %.2f %.2f %.2f\n", mvm[1][0], mvm[1][1], mvm[1][2], mvm[1][3] );
				printf( "%.2f %.2f %.2f %.2f\n", mvm[2][0], mvm[2][1], mvm[2][2], mvm[2][3] );
				printf( "%.2f %.2f %.2f %.2f\n", mvm[3][0], mvm[3][1], mvm[3][2], mvm[3][3] );
				*/

				t = - ( mvm[2][0]*mvm[3][0]  + mvm[2][1]*mvm[3][1]  + mvm[2][2]*mvm[3][2]  )
					/(  mvm[2][0]*sc->vec[0] + mvm[2][1]*sc->vec[1] + mvm[2][2]*sc->vec[2] );
				if( t>0 ) visible = 0;
				p[0] = -mvm[3][0] - sc->vec[0]*t; /* intersection of click vector and ground plane */
				p[1] = -mvm[3][1] - sc->vec[1]*t;
				p[2] = -mvm[3][2] - sc->vec[2]*t;

				dx[0] = mvm[0][0]*p[0] + mvm[0][1]*p[1] + mvm[0][2]*p[2]; /* now in local frame */
				dx[1] = mvm[1][0]*p[0] + mvm[1][1]*p[1] + mvm[1][2]*p[2];
			}

			/*
			if( sc->selectedWay != -1 && sc->selectedNum == 1 && sc->pick == 2 ) { (the old click to move waypoint)

				maneuver[sc->selectedWay].x[0] = dx[0]; 
				maneuver[sc->selectedWay].x[1] = dx[1];
				gi->traj->upload = 1;
				sc->pick = 0;

			} else
			*/
			if( visible ) {
				if( sc->pick == 3 ) { // Ctrl+click (pan/tilt/roll position command)

					//gcsSendPointPos( dx[0], dx[1], gi->traj->nav->pointPos[2] );
					gi->traj->nav->pointPos[0] = dx[0];
					gi->traj->nav->pointPos[1] = dx[1];

					/*
					sprintf( buffer, "rc trajectorySet.pointPos[0]=%g", dx[0] );
					commandExecute( buffer );
					sprintf( buffer, "rc trajectorySet.pointPos[1]=%g", dx[1] );
					commandExecute( buffer );
					*/

				} else if( sc->pick == 1 ) { // Default, single click

					if( sc->mouseOverWay == -1 ) {
						if( sg->singleClickGreenCircle ) {
							//gcsSendPointPos( dx[0], dx[1], gi->traj->nav->pointPos[2] );
							gi->traj->nav->pointPos[0] = dx[0];
							gi->traj->nav->pointPos[1] = dx[1];
						}

						/* show coordinates */
						sprintf( buffer, "scene: pick (%.0f,%.0f)%.6f,%.6f",
							dx[0], dx[1],
							gi->outputs->datumLat + C_FT2NM/60*dx[0],
							hmodDeg( gi->outputs->datumLon + C_FT2NM/60*dx[1]/sc->cosDatumLat ) );
						logInfo( buffer );

						if (1 == sc->pickPlant) {
							pickPlanter( dx[0], dx[1] );
						}

						if( sc->showMarkOnClick ) {
							switch( sg->posUnits ) {
							case POS_LATLONG:
								sprintf( buffer, "Mark (%.6f,%.6f)",
									gi->outputs->datumLat + C_FT2NM/60*dx[0],
									hmodDeg( gi->outputs->datumLon + C_FT2NM/60*dx[1]/sc->cosDatumLat ) );
								break;
							case POS_FEET:
							default:
								sprintf( buffer, "Mark (%.0f,%.0f)", dx[0], dx[1] );
								break;
							}
							if( sg->showMarkOnClickWithRange ) {
								double range, dxrb[2];
								char units[10];
								dxrb[0] =        -( o->latitude  - gi->outputs->datumLat )*C_NM2FT*60.0                 + dx[0];
								dxrb[1] = -hmodDeg( o->longitude - gi->outputs->datumLon )*C_NM2FT*60.0*sc->cosDatumLat + dx[1];
								range = sqrt( SQ( dxrb[0] ) + SQ( dxrb[1] ) );
								switch( sg->distanceUnits ) {
								default:
								case DISTANCE_FT:
									sprintf( units, "ft" );
									break;
								case DISTANCE_M:
									range *= C_FT2M;
									sprintf( units, "m" );
									break;
								case DISTANCE_NM:
									range *= C_FT2NM;
									sprintf( units, "nm" );
									break;
								case DISTANCE_KM:
									range *= C_FT2KM;
									sprintf( units, "km" );
									break;
								case DISTANCE_SM:
									range *= C_FT2SM;
									sprintf( units, "sm" );
									break;
								}
								if(      range < 10   ) sprintf( buffer, "%s,  %.2f %s away", buffer, range, units );
								else if( range < 100  ) sprintf( buffer, "%s,  %.1f %s away", buffer, range, units );
								else                    sprintf( buffer, "%s,  %.0f %s away", buffer, range, units );
							}
							sceneAddMessage( sg, sc, buffer );
						}
					}
				} else if( sc->pick == 4 ) {

				}
			}

		}
		sc->pick = 0;

		/* draw waypoints, and waypoint picking */

		glColor4fv( sg->planColor );
		sc->mouseOverWay = -1;
		if( gsc->viewMode == VIEW_NAV )
			dmin = sg->pickTol/sc->winh;
		else
			dmin = (float)(sg->pickTol*sc->fovy*v->zoom/sc->winh*C_DEG2RAD);

		glLineWidth( sg->planLineWidth );

		for( step3D=0; step3D<2; step3D++ ) {

			if( step3D == 0 ) {
				glColor4fv( sg->shadowColor );
				glDepthMask( 0 );
			} else {
				glDepthMask( 1 );
			}			
			
			/* datum marker */

			glLineWidth( sg->gcsLineWidth );

			if( step3D ) {
				glPushMatrix();
				glTranslatef( (float)(( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0),
					(float)(hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(-gi->outputs->datumAlt + sc->eyeAlt) );

				/* size constraint */
				zoom = ( gsc->viewMode == VIEW_NAV ? -v->zoom*2 : v->zoom );
				if( zoom > 0 ) {
					glGetFloatv( GL_MODELVIEW_MATRIX, &(mvm[0][0]) );
					distance = sqrt( SQ( mvm[3][0] ) + SQ( mvm[3][1] ) + SQ( mvm[3][2] ) );
					enlarge = (float)(MAX( gi->set->waypointR, distance*sg->minWaySize/(2)*zoom/sc->winh ));
				} else {
 					enlarge = (float)(MAX( gi->set->waypointR, -sg->minWaySize/(2)*zoom/sc->winh ));
				}
				glScalef( enlarge, enlarge, enlarge );

				glColor4fv( sg->wayDatumColor );
				glCallList( sg->datum_dl );
				glPopMatrix();
			}

			/* point at marker (pointPos) */

			glDisable( GL_DEPTH_TEST );
			glPushMatrix();
			glTranslatef( (float)(( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0),
				(float)(hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
				(float)(-gi->outputs->datumAlt + sc->eyeAlt) );
			glTranslatef( (float)gi->traj->nav->pointPos[0],
				(float)gi->traj->nav->pointPos[1],
				0 );

			if( step3D ) {
				glColor4fv( sg->lgAxesColor );
				glTranslatef( 0,0, (float)gi->traj->nav->pointPos[2] );
			} else {
				glTranslatef( 0, 0, -(float)gi->datalink->terrainH );
			}

			/* size constraint */
			zoom = ( gsc->viewMode == VIEW_NAV ? -v->zoom*2 : v->zoom );
			if( zoom > 0 ) {
				glGetFloatv( GL_MODELVIEW_MATRIX, &(mvm[0][0]) );
				distance = sqrt( SQ( mvm[3][0] ) + SQ( mvm[3][1] ) + SQ( mvm[3][2] ) );
				enlarge = (float)(MAX( gi->set->waypointR, distance*sg->minWaySize/(2)*zoom/sc->winh ));
			} else {
 				enlarge = (float)(MAX( gi->set->waypointR, -sg->minWaySize/(2)*zoom/sc->winh ));
			}
			glScalef( enlarge, enlarge, enlarge );
			

			glCallList( sg->waypointP_dl );
			glPopMatrix();
			glEnable( GL_DEPTH_TEST );

            if( sg->showPPTarget && step3D && 
                (gsc->viewMode == VIEW_CAMERA || gsc->viewMode == VIEW_CAMERA2 || gsc->viewMode == VIEW_CAMERA3 || gsc->viewMode == VIEW_CAMERA4 )
            ) {
                /* use point pos to draw target in camera mode */

                glDisable( GL_DEPTH_TEST );
                glPushMatrix();
                glTranslatef( (float)(( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0),
                    (float)(hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
                    (float)(-gi->outputs->datumAlt + sc->eyeAlt) );
                glTranslatef( (float)gi->traj->nav->pointPos[0],
                    (float)gi->traj->nav->pointPos[1],
                    (float)gi->traj->nav->pointPos[2] );

                glColor4fv( sg->lgAxesColor );
                glRotatef( sc->eyePsi  *CF_RAD2DEG, 0, 0, 1 );
                glRotatef( sc->eyeTheta*CF_RAD2DEG + 90.0f, 0, 1, 0 );

                /* size constraint */
                zoom = ( gsc->viewMode == VIEW_NAV ? -v->zoom*2 : v->zoom );
                if( zoom > 0 ) {
                    glGetFloatv( GL_MODELVIEW_MATRIX, &(mvm[0][0]) );
                    distance = sqrt( SQ( mvm[3][0] ) + SQ( mvm[3][1] ) + SQ( mvm[3][2] ) );
                    enlarge = (float)(MAX( sg->pptargetR, distance*sg->minPptargetSize/(2)*zoom/sc->winh ));
                } else {
                    enlarge = (float)(MAX( sg->pptargetR, -sg->minWaySize/(2)*zoom/sc->winh ));
                }
                glScalef( enlarge, enlarge, enlarge );

                glCallList( sg->waypointP_dl );
                glPopMatrix();
                glEnable( GL_DEPTH_TEST );

                // draw floating arrow in camera frame
                glDisable( GL_DEPTH_TEST );
                glPushMatrix(); {
                    int i;
                    double r[3], r_v[3];
                    double dcm_vl[3][3];
                    double length;
                    double loc[3] = {10.0f, 0, 0};
                    
                    glRotatef( sc->eyePsi  *CF_RAD2DEG, 0, 0, 1 );
                    glRotatef( sc->eyeTheta*CF_RAD2DEG, 0, 1, 0 );
                    glRotatef( sc->eyePhi  *CF_RAD2DEG, 1, 0, 0 );
                    
                    // get vector from vehicle to target
                    for( i=0; i<3; i++ ) {
                        r[i] = gi->traj->nav->pointPos[i] - o->pos[i];
                    }
                    
                    euler2dcm( (double)sc->eyePhi, (double)sc->eyeTheta, (double)sc->eyePsi, dcm_vl);
                    
                    map_T_vector(dcm_vl, r, r_v);
                    length = MAX( .0001, mat_norm2(r_v, 3) );
                    for( i=0; i<3; i++ )
                        r_v[i] *= 2.0/length;
					
                    glBegin( GL_LINES );
                    glVertex3f( (float)(         loc[0]), (float)(         loc[1]), (float)(         loc[2]) );
                    glVertex3f( (float)(r_v[0] + loc[0]), (float)(r_v[1] + loc[1]), (float)(r_v[2] + loc[2]) );
                    glEnd();
                    
                } glPopMatrix();
                glEnable( GL_DEPTH_TEST );  
                
            }

			glPushMatrix();
			glTranslatef( (float)(( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0),
					(float)(hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(-gi->outputs->datumAlt + sc->eyeAlt) );

			/* planned trajectory */
            if( 1 == gsc->showPlan && gi->set->showPlan ) {
				int start, drewTimeLabels = 0;
				char doingBothPlans;
				glLineWidth( sg->planLineWidth );

				doingBothPlans = (gi->traj->uploadEach[gi->traj->edit] || gi->traj->edit==PLANEDIT_LC ?1:0);

				j = doingBothPlans + 1;
				while( j-- > 0 ) {

					switch( j ) {
					case 0: /* current path */
						if( gi->datalink->m1->lostComm ) fp = gi->traj->flightPlanLCDL;
						else                             fp = gi->traj->flightPlanDL;
						if( step3D ) {
							glColor4fv( sg->planColor );
						}
						break;
					default:
					case 1: /* editing path */
						if( step3D ) {
							if( gi->traj->uploadEach[gi->traj->edit] == 0 )
								glColor4fv( sg->blackColor );
							else
								glColor4fv( sg->selectedWayColor );
						}
						break;
					}

					/* draw path */
					/* get future path */
					memcpy( la, gi->traj->traj, sizeof( struct trajectory_ref ) );
					if( la->safemode ) {  /* plot what would happen if engaged */
						la->currentTransType = TRANS_UNKNOWN;
						la->nextTransType    = TRANS_UNKNOWN;

						la->safemode = 0;
					}

					/* draw the purple line */
					start = MAX( 1, (int)(( sim.time - trajTime[gn][j] )/MAX( sg->planStep, 0.0001 )) + 1 );
					if( step3D ) {
						if( sg->planWidth > 0 ) {
							float dir[2], head, bank;
							double f[3], fmag2;

					        glDisable( GL_CULL_FACE );
							glBegin( GL_QUAD_STRIP );

							if( start < trajNPoint[gn][j] - 1 ) {
								dir[0] = (float)( trajPos[gn][j][start+1][0] - trajPos[gn][j][start][0] );
								dir[1] = (float)( trajPos[gn][j][start+1][1] - trajPos[gn][j][start][1] );
							} else {
								dir[0] = 0;
								dir[1] = 0;
							}
								head = (float)atan2( dir[1], dir[0] );

							f[0] = trajAcc[gn][j][start][0];
							f[1] = trajAcc[gn][j][start][1];
							f[2] = trajAcc[gn][j][start][2] - 32.174;
							fmag2 = SQ( f[0] ) + SQ( f[1] ) + SQ( f[2] );
							if( fmag2 > 1 ) {
								bank = (float)atan2( f[1]*cos( head ) - f[0]*sin( head ), -f[2] );
							} else {
								bank = 0.0;
							}

							glVertex3f( (float)gi->traj->traj->x[0] + gi->set->waypointR*sg->planWidth*(float)(sin( head )*cos( bank )), 
								        (float)gi->traj->traj->x[1] - gi->set->waypointR*sg->planWidth*(float)(cos( head )*cos( bank )), 
										(float)gi->traj->traj->x[2] - gi->set->waypointR*sg->planWidth*(float)(            sin( bank )) );
							glVertex3f( (float)gi->traj->traj->x[0] - gi->set->waypointR*sg->planWidth*(float)(sin( head )*cos( bank )), 
								        (float)gi->traj->traj->x[1] + gi->set->waypointR*sg->planWidth*(float)(cos( head )*cos( bank )), 
										(float)gi->traj->traj->x[2] + gi->set->waypointR*sg->planWidth*(float)(            sin( bank )) );
							glEnd();
					        glEnable( GL_CULL_FACE );
						}

						glBegin( GL_LINE_STRIP );
						glVertex3f( (float)gi->traj->traj->x[0], (float)gi->traj->traj->x[1], (float)gi->traj->traj->x[2] );
						for( i=start; i<trajNPoint[gn][j]; i++ ) {
							glVertex3f( (float)trajPos[gn][j][i][0], (float)trajPos[gn][j][i][1], (float)trajPos[gn][j][i][2] );
						}
						glEnd();

					} else if( gsc->viewMode != VIEW_CAMERA && gsc->viewMode != VIEW_CAMERA2 && gsc->viewMode != VIEW_CAMERA3 && gsc->viewMode != VIEW_CAMERA4 ) {
						glBegin( GL_LINE_STRIP );
						glVertex3f( (float)gi->traj->traj->x[0], (float)gi->traj->traj->x[1], -(float)gi->datalink->terrainH );
						for( i=start; i<trajNPoint[gn][j]; i++ ) {
							glVertex3f( (float)trajPos[gn][j][i][0], (float)trajPos[gn][j][i][1], -(float)gi->datalink->terrainH );
						}
						glEnd();
					}

					if( step3D && sg->timeLabelStep > 0 && !drewTimeLabels && gi->datalink->m0->navStatus == 1 ) {
						double lastT = 0;
						unsigned int hours;
						unsigned int minutes;
						unsigned int seconds;
						drewTimeLabels = 1;
						//glDisable( GL_DEPTH_TEST );
						//glPointSize( sg->planLineWidth );
						glColor4fv( sg->mouseOverWayColor );
						//glBegin( GL_POINTS );
						for( i=start; i<trajNPoint[gn][j]; i++ ) {
							if( trajT[gn][j][i] > lastT + sg->timeLabelStep ) {
								lastT += sg->timeLabelStep;
								hours   = (unsigned int)( lastT/3600);
								minutes = (unsigned int)((lastT - 3600*hours)/60);
								seconds = (unsigned int)( lastT - 3600*hours - 60*minutes);
								//glVertex3f( (float)trajPos[gn][j][i][0], (float)trajPos[gn][j][i][1], (float)trajPos[gn][j][i][2] );
								if( hours ) sprintf( buffer, "%d:%02d:%02d", hours, minutes, seconds );
								else        sprintf( buffer,      "%d:%02d",        minutes, seconds );
								showBitmapMessage( (float)trajPos[gn][j][i][0] - gi->set->waypointR*(float)sin( sc->eyePsi ),
									               (float)trajPos[gn][j][i][1] + gi->set->waypointR*(float)cos( sc->eyePsi ),
												   (float)trajPos[gn][j][i][2], buffer, LIMIT( sg->wayFont, 2, 8 ) );
							}
						}
						//glEnd();
						//glPointSize( 1.0 );
						//glEnable( GL_DEPTH_TEST );
					}
				}
            }

			glPopMatrix();

			glLineWidth( sg->gcsLineWidth );

		} /* 3d */

        /* current position/attitude command (yellow circle) */
        if( gsc->showPlan && gi->set->showPlan && gsc->viewMode != VIEW_CAMERA && gsc->viewMode != VIEW_CAMERA2 && gsc->viewMode != VIEW_CAMERA3 && gsc->viewMode != VIEW_CAMERA4 ) {

				glPushMatrix();
				glTranslatef( (float)(( gi->outputs->latitude - sc->eyeLat )*C_NM2FT*60.0),
					(float)(hmodDeg( gi->outputs->longitude - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
					(float)(-gi->outputs->altitudeMSL + sc->eyeAlt) );
				glColor4fv( sg->wayComColor );
				glBegin( GL_LINES );
				glVertex3f( 0, 0, 0 );
				glVertex3f( 0, 0, (float)o->altitudeAGL );
				glEnd();

				glPopMatrix();

        }

        /* future position/attitude command (bright yellow circle, "rabbit" ) */
        if( gsc->showPlan && gi->set->showPlan && gsc->viewMode != VIEW_CAMERA && gsc->viewMode != VIEW_CAMERA2 && gsc->viewMode != VIEW_CAMERA3 && gsc->viewMode != VIEW_CAMERA4 ) {

				if( gi->traj->traj->safemode == 1 ) {
					int wow;
					memcpy( la, gi->traj->traj, sizeof( struct trajectory_ref ) );
					sc->futureTime = sg->futureTime;
					glPushMatrix();
					glTranslatef( (float)(( gi->outputs->datumLat - sc->eyeLat )*C_NM2FT*60.0),
						(float)(hmodDeg( gi->outputs->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat),
						(float)(-gi->outputs->datumAlt + sc->eyeAlt) );
					glColor4fv( sg->selectedWayTextColor ); /* I find this makes the most sense, but name is inconsistent */
					glBegin( GL_LINE_STRIP );
					if( gi->datalink->m1->lostComm ) fp = gi->traj->flightPlanLCDL;
					else                             fp = gi->traj->flightPlanDL;
					wow = gi->traj->nav->wow;
					for( time = gi->traj->traj->time;
					time <= gi->traj->traj->time + sc->futureTime;
					time += sg->planStep ) {
						gi->traj->nav->wow = 0;  /* assume it will takeoff */
						glVertex3f( (float)la->x[0], (float)la->x[1], (float)la->x[2] );
					}
					gi->traj->nav->wow = wow;
					gi->traj->traj->geofence = la->geofence;
					glEnd();
					glPopMatrix();
				}
        }

	} /* end GCS */

	glDisable( GL_DEPTH_TEST );

	glPopMatrix(); /* back to Etkin */
	glPopMatrix(); /* back to screen */

	/* hud */

	if( v->hudOn &&
		(( gsc->viewMode != VIEW_GROUND ) &&
		 ( gsc->viewMode != VIEW_NAV    ) &&
		 ( gsc->viewMode != VIEW_HOVER  ) &&
		 ( gsc->viewMode != VIEW_CHASE  )) ) {

		double windDir, windMag, heading;
		float stretch;
		struct view_ref *vh = gi->set->cockpit; /* normally just use cockpit for hud, exceptions for camera views */
		char useAlternateHud = 0;
		float hudLW[2];
		int k;

		if( sc == &scenePIP ) {
			if( sc->videoModeDerived ) {
				hudLW[0] = sg->hudVideoLW_PIP[0];
				hudLW[1] = sg->hudVideoLW_PIP[1];
			} else {
				hudLW[0] = sg->hudLW_PIP[0];
				hudLW[1] = sg->hudLW_PIP[1];
			}
		} else {
			if( sc->videoModeDerived ) {
				hudLW[0] = sg->hudVideoLW[0];
				hudLW[1] = sg->hudVideoLW[1];
			} else {
				hudLW[0] = sg->hudLW[0];
				hudLW[1] = sg->hudLW[1];
			}
		}

        /* provisions for video overays, alternate huds */
		if( ( sc->videoModeDerived == 1 && gsc->viewMode != VIEW_COCKPIT ) || /* video overlay and not showing cockpit view */
			gsc->viewMode == VIEW_CAMERA || gsc->viewMode == VIEW_CAMERA2 || gsc->viewMode == VIEW_CAMERA3 || gsc->viewMode == VIEW_CAMERA4 ) { /* any camera view */
			switch( gsc->viewMode ) {
			case VIEW_CAMERA2: vh = gi->set->camera2; break;
			case VIEW_CAMERA3: vh = gi->set->camera3; break;
			case VIEW_CAMERA4: vh = gi->set->camera4; break;
			default:           vh = gi->set->camera;  break;
			}
			if( sc->hudAlternate == 1 )
				useAlternateHud = 1;
		}

		glDisable( GL_DEPTH_TEST );
		glDisable( GL_LIGHTING );
		glShadeModel( GL_FLAT );

		for( k=(hudLW[0]==0.0); k<=(hudLW[1]>0.0); k++ ) {

            glLineWidth( hudLW[k] );
            glPointSize( hudLW[k] );
			glColor4fv( sg->hudColor[k] );

			glMatrixMode( GL_PROJECTION );
			glLoadIdentity();
			gluPerspective( vh->zoom*sc->fovy*sc->hudh/sc->winh,
				(GLfloat)(sc->hudw)/sc->hudh, sg->znear, sg->vis );
			glMatrixMode( GL_MODELVIEW );
			glViewport( winx + (sc->winw-sc->hudw)/2, winy + (sc->winh-sc->hudh)/2,
				sc->hudw, sc->hudh );
			glLoadIdentity();

			if( knowView ) {

				glPushMatrix(); {
					glRotatef( 90.0, 0.0, 1.0, 0.0 ); /* switch to Etkin Axes */
					glRotatef( 90.0, 1.0, 0.0, 0.0 );
					if( useAlternateHud ) {
						float scale;
						scale = (float)(vh->zoom*sc->fovy*sc->hudh/MAX( 1, sc->winh )/25.0*sg->hudAlternateSize);
						glPushMatrix(); {
							glTranslatef( sg->znear*10, 0, 0 );
							glScalef( scale, scale, scale );
							glRotatef( -vh->neckPhi*CF_RAD2DEG,   1.0, 0.0, 0.0 );
							glRotatef( -vh->neckTheta*CF_RAD2DEG, 0.0, 1.0, 0.0 );
							glRotatef( -vh->neckPsi*CF_RAD2DEG,   0.0, 0.0, 1.0 );
							glMultMatrixf( &(o->float_dcm_bl[0][0]) ); /* local geographic */
							glCallList( sg->hud_alternate_dl );
						} glPopMatrix();
					}
					glPushMatrix(); {
						glRotatef( -vh->neckPhi*CF_RAD2DEG,   1.0, 0.0, 0.0 );
						glRotatef( -vh->neckTheta*CF_RAD2DEG, 0.0, 1.0, 0.0 );
						glRotatef( -vh->neckPsi*CF_RAD2DEG,   0.0, 0.0, 1.0 );
						glMultMatrixf( &(o->float_dcm_bl[0][0]) ); /* local geographic */
						if( useAlternateHud == 0 ) {
							glCallList( sg->hud_horizon_dl );
							glPushMatrix(); {
								if( sg->hudBodyAxes ) if( gi->set->rotate90 ) glRotatef( (float)o->psi90,  0.0, 0.0, 1.0 );
													  else                    glRotatef( (float)o->psi,    0.0, 0.0, 1.0 );
								else                                          glRotatef( (float)o->psiw,   0.0, 0.0, 1.0 );
								glCallList( sg->hud_ladder_dl );
							} glPopMatrix();
						}

						/* flight path vector */
						if( SQ( o->vs/60.0 ) + SQ( o->gs*C_KT2FPS ) > 1.0 ) {
							glPushMatrix(); 
							glRotatef( (float)o->track, 0.0, 0.0, 1.0 );
							glRotatef( (float)o->gamma, 0.0, 1.0, 0.0 );

							/* should be phiFPA */
							glRotatef( (float)o->phiw, 1.0, 0.0, 0.0 );
							glCallList( sg->hud_fp_dl ); 
							glPopMatrix();
						}

						if( useAlternateHud == 0 ) {
							/* wind axis vector */
							/*glPushMatrix(); 
								glMultMatrixf( &(o->float_dcm_lw[0][0]) ); 
								glCallList( sg->hud_fp_dl ); 
							glPopMatrix();*/
							glPushMatrix();
							if( sg->hudBodyAxes ) {
								glMultMatrixf( &(o->float_dcm_lb[0][0]) ); /* to body axes */
								if( gi->set->rotate90 ) glRotatef( 90.0, 0.0, 1.0, 0.0 );
							} else {
								glRotatef( (float)o->track, 0.0, 0.0, 1.0 );
								glRotatef( (float)o->gamma, 0.0, 1.0, 0.0 );
								glRotatef( (float)o->phiw,  1.0, 0.0, 0.0 );
							}
							glCallList( sg->hud_bankscale_dl );
							if( sg->hudBodyAxes ) if( gi->set->rotate90 ) glRotatef( -(float)o->phi90,  1.0, 0.0, 0.0 );
												  else                    glRotatef( -(float)o->phi,    1.0, 0.0, 0.0 );
							else                                          glRotatef( -(float)o->phiw,   1.0, 0.0, 0.0 );
							glCallList( sg->hud_bank_dl );
							glPopMatrix();
						}

					} glPopMatrix(); /* back to Etkin */

					glPushMatrix(); {
						glRotatef( (float)(-vh->neckTheta*C_RAD2DEG), 0.0, 1.0, 0.0 );
						glRotatef( (float)(-vh->neckPsi*C_RAD2DEG),   0.0, 0.0, 1.0 );
						if( gi->set->rotate90 ) glRotatef( 90.0, 0.0, 1.0, 0.0 );
						glCallList( sg->hud_reference_dl );

						/*if( o->flightDirector ) {
							glPushMatrix(); {
								glRotatef( o->pitchSteeringBar, 0.0, 1.0, 0.0 );
								glRotatef( o->bankSteeringBar, 1.0, 0.0, 0.0 );
								glCallList( sg->hud_sb_dl );
							} glPopMatrix();
						}*/

					} glPopMatrix();

				} glPopMatrix(); /* back to screen coordinates */

			}

			glMatrixMode( GL_PROJECTION );
			glLoadIdentity();
			glOrtho( -0.5*C_DEG2RAD*sg->hfov*sc->hudw/sc->hudh,
				0.5*C_DEG2RAD*sg->hfov*sc->hudw/sc->hudh,
				-0.5*C_DEG2RAD*sg->hfov,
				0.5*C_DEG2RAD*sg->hfov, -1.0, 1.0 );
			glMatrixMode( GL_MODELVIEW );
			glLoadIdentity();

			if( useAlternateHud == 0 ) {

				double cas;

				/*sprintf( buffer, "%.1f %.1f\0", o->position[0]*C_FT2NM,
				o->position[1]*C_FT2NM );
				showMessage( -sg->hrpx, -0.17, buffer, 1.0 );*/

				/* airspeed and altitude scales */
				glCallList( sg->hud_scales_dl );

                switch( sg->speedUnits ) {
                case SPEED_FPS:
                default:
					cas = o->cas*C_KT2FPS;
					break;
                case SPEED_KNOTS:
					cas = o->cas;
					break;
                case SPEED_KPH:
					cas = o->cas*C_KT2KPH;
					break;
                case SPEED_MPH:
					cas = o->cas*C_KT2MPH;
					break;
                case SPEED_MPS:
					cas = o->cas*C_KT2MPS;
					break;
                }
				sprintf( buffer, "%.0f", cas );
				angle = (float)(C_DEG2RAD*cas*3.6);
				if( cas < 999.5 ) {
					showMessage( -sg->hrpx-0.01f, sg->hrpy-0.005f, buffer, 0.75 );
				} else {
					double limitcas;
					limitcas = LIMIT( cas, -100, 10000 );
					i = (int)(( limitcas + 0.5 )/1000.0);
					sprintf( buffer, "%d", i );
					showMessage( -sg->hrpx - 0.01f, sg->hrpy-0.005f, buffer, 0.75 );
					sprintf( buffer, "%03.0f", ABS( limitcas-i*1000.0 ) );
					showMessage( -sg->hrpx - 0.002f + (int)(log10(i))*0.0075f,
						sg->hrpy-0.0045f, buffer, 0.5 );
				}
				//if( o->fuel > 0.0 )
				//sprintf( buffer, "Th%.0f", o->delt[0]*50 + 50 );
				//else
				//	sprintf( buffer, "nofuel"/*, o->throttleLever*100*/ );
				/*if( ABS( o->speedBrake ) > 0.01 )
				sprintf( buffer, "%s(SB%.0f)", buffer, o->speedBrake*100 );*/
				//showMessage( -sg->hrpx - 0.02f, sg->hrpy - 0.073f, buffer, 0.75 );
				//if( o->fuel > 0.0 ) {
				//	sprintf( buffer, " f%.0f\0", o->fuel*100 );
				//	showMessage( -sg->hrpx - 0.02f, sg->hrpy - 0.085f, buffer, 0.75 );
				//}
				//if( o->model != MODEL_LOGO ) {
				//	sprintf( buffer, "RPM%.0f", o->rpm );
				//	showMessage( -sg->hrpx - 0.02f, sg->hrpy - 0.097f, buffer, 0.75 );
				//}

                switch( sg->speedUnits ) {
                case SPEED_FPS:
                default:
					sprintf( buffer, "GS%.0f FPS", LIMIT( o->gs*C_KT2FPS, -1, 1000000 ) );
					break;
                case SPEED_KNOTS:
					sprintf( buffer, "GS%.0f Kts", LIMIT( o->gs, -1, 1000000 ) );
					break;
                case SPEED_KPH:
					sprintf( buffer, "GS%.0f KPH", LIMIT( o->gs*C_KT2KPH, -1, 1000000 ) );
					break;
                case SPEED_MPH:
					sprintf( buffer, "GS%.0f MPH", LIMIT( o->gs*C_KT2MPH, -1, 1000000 ) );
					break;
                case SPEED_MPS:
					sprintf( buffer, "GS%.0f m/s", LIMIT( o->gs*C_KT2MPS, -1, 1000000 ) );
					break;
                }
				showMessage( -sg->hrpx - 0.02f, sg->hrpy - 0.109f, buffer, 0.75 );
				sprintf( buffer, "%.1fG", LIMIT( o->G, -1000, 1000 ) );
				showMessage( -sg->hrpx - 0.02f, sg->hrpy - 0.121f, buffer, 0.75 );
				glBegin( GL_LINES );
				glVertex3f( -sg->hrpx + 0.009f*(float)sin( angle ),
					sg->hrpy + 0.009f*(float)cos( angle ), 0.0 );
				glVertex3f( -sg->hrpx + 0.016f*(float)sin( angle ),
					sg->hrpy + 0.016f*(float)cos( angle ), 0.0 );
				glEnd();
				if( o->altitudeMSL < 9999.5 ) {
					sprintf( buffer, "%.0f", MAX( o->altitudeMSL, -10000 ) );
					showMessage( sg->hrpx-0.015f, sg->hrpy-0.005f, buffer, 0.75 );
				} else {
					i = (int)(( LIMIT( o->altitudeMSL, -10000, 10000000 ) + 0.5 )/1000.0);
					sprintf( buffer, "%d", i );
					showMessage( sg->hrpx - 0.015f, sg->hrpy - 0.005f, buffer, 0.75 );
					sprintf( buffer, "%03.0f", ABS( LIMIT( o->altitudeMSL, -10000, 10000000 ) -i*1000.0 ) );
					showMessage( sg->hrpx - 0.007f + (int)(log10(i))*0.0075f,
						sg->hrpy - 0.0045f, buffer, 0.5 );
				}
				sprintf( buffer, " VS%.0f", LIMIT( o->vs, -100000, 100000 ) );
				showMessage( sg->hrpx - 0.015f, sg->hrpy - 0.073f, buffer, 0.7f );
				//sprintf( buffer, "AGL%.0f\0", LIMIT( o->altitudeAGL, -10000, 1000000 ) );
				//showMessage( sg->hrpx-0.015, sg->hrpy-0.085, buffer, 0.7 );
				angle = (float)(C_DEG2RAD*o->altitudeMSL*0.36);
				glBegin( GL_LINES );
				glVertex3f( sg->hrpx + 0.0125f*(float)sin( angle ),
					sg->hrpy + 0.0125f*(float)cos( angle ), 0.0 );
				glVertex3f( sg->hrpx + 0.023f*(float)sin( angle ),
					sg->hrpy + 0.023f*(float)cos( angle ), 0.0 );
				glEnd();
				angle = (float)(o->vs*0.036);
				angle = LIMIT( angle, -360.0f, 360.0f );
				glBegin( GL_LINE_STRIP );
				glVertex3f( sg->hrpx - 0.025f, sg->hrpy, 0.0 );
				if( angle > 0.0 ) {
					for( i=0; i<angle; i+=18 )
						glVertex3f( sg->hrpx - 0.025f*(float)cos( C_DEG2RAD*i ),
						sg->hrpy + 0.025f*(float)sin( C_DEG2RAD*i ), 0.0 );
				} else {
					for( i=0; i>angle; i-=18 )
						glVertex3f( sg->hrpx - 0.025f*(float)cos( C_DEG2RAD*i ),
						sg->hrpy + 0.025f*(float)sin( C_DEG2RAD*i ), 0.0 );
				}
				glVertex3f( sg->hrpx - 0.025f*(float)cos( angle*C_DEG2RAD ),
					sg->hrpy + 0.025f*(float)sin( angle*C_DEG2RAD ), 0.0 );
				glEnd();

				heading = o->psi;
				if( gi->set->rotate90 ) heading = o->psi90;
				else                    heading = o->psi;
					/*heading -= o->phi*sin( o->theta*C_DEG2RAD );*/

				/* wind vector */

				windMag = sqrt( SQ( o->wind[0] ) + SQ( o->wind[1] ) )*C_FPS2KT;
				if( windMag > 2.0 ) {
					windDir = atan2( -o->wind[1], -o->wind[0] )*C_RAD2DEG;
					windDir = hmod360( windDir );
					if( windDir < 0.5 ) windDir += 360.0;
					windDir = LIMIT( windDir, 0, 360 );
					windMag = LIMIT( windMag, 0, 1000 );
					stretch = (float)(1.0 - exp( -(float)windMag/10 ));
					glPushMatrix();
					glTranslatef( -sg->hrpx, -0.15f, 0.0 );
					glRotatef( (float)(windDir - heading + 180), 0.0, 0.0, -1.0 );
					glScalef( 1.0, stretch, 1.0 );
					glBegin( GL_LINES );
					glVertex3f(  0.0,    0.02f, 0.0 );
					glVertex3f(  0.0,   -0.02f, 0.0 );
					glVertex3f(  0.0,    0.02f, 0.0 );
					glVertex3f(  0.005f, 0.01f, 0.0 );
					glVertex3f(  0.0,    0.02f, 0.0 );
					glVertex3f( -0.005f, 0.01f, 0.0 );
					glEnd();
					glPopMatrix();
					switch( sg->speedUnits ) {
					case SPEED_FPS:
					default:
						windMag *= C_KT2FPS;
						break;
					case SPEED_KNOTS:
						break;
					case SPEED_KPH:
						windMag *= C_KT2KPH;
						break;
					case SPEED_MPH:
						windMag *= C_KT2MPH;
						break;
					case SPEED_MPS:
						windMag *= C_KT2MPS;
						break;
					}
					sprintf( buffer, "%03.0f@%.0f", windDir, windMag );
					showMessage( -0.022f - sg->hrpx, -0.165f - 0.02f*stretch, buffer, 0.7f );
				}

				/* heading numerically */

				if( heading < 0.5 ) heading += 360.0;
				sprintf( buffer, "%03.0f", LIMIT( heading, 0, 360 ) );
				showMessage( -0.015f, 0.15f, buffer, 1.0 );

				/*if( ( o->brakes[0] > 0.1 ) || ( o->brakes[1] > 0.1 ) ) {
				sprintf( buffer, "BRKS ON\0" );
				showMessage( -0.035, -0.15, buffer, 1.0 );}*/
				/*if( o->unlimitedFuel ) {
				sprintf( buffer, "FREE FL\0" );
				showMessage( -0.035, -0.162, buffer, 1.0 );}*/

			}

		}

		glLineWidth( 1.0 );
		glPointSize( 1.0 );

		glViewport( winx, winy, sc->winw, sc->winh );
	} else {
		glDisable( GL_DEPTH_TEST );
		glDisable( GL_LIGHTING );
		glShadeModel( GL_FLAT );
	}

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glOrtho( 0.0, 1.0, 0.0, 1.0*sc->winh/sc->winw, -1.0, 1.0 );
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	/* screen relative information */

	/* map north */

    if( sg->showNorthOnMap ) {
        if( gsc->viewMode == VIEW_NAV ) {
            double windMag;
	        int k;

			if( ABS( hmodRad( v->neckPsi ) ) > C_DEG2RAD ||
				( gsc->mapUpMode != MAPUP_CONST && v->lookat == VIEW_LOOKAT_VEHICLE ) ) {

				for( k=((sg->hudLW[0]==0.0&&sc->videoModeDerived==0)||sg->hudVideoLW[0]==0.0); k<=(sg->hudLW[1]>0.0); k++ ) {

					if( sc->videoModeDerived ) {
						glLineWidth( sg->hudVideoLW[k] );
					} else {
						glLineWidth( sg->hudLW[k] );
					}
					glColor4fv( sg->hudColor[k] );

					glPushMatrix();
					glTranslatef( (float)(1.0 - ( sg->northOnMapSize + 2.0 )/sc->winw),
								  (float)(      ( sg->northOnMapSize + 2.0 )/sc->winw), 0 );
					if( gsc->showPIP ) {
						if( sg->pipOffsetX < 60 && sg->pipOffsetY < 60 ) {
							glTranslatef( 0, (float)( scenePIP.winh + sg->pipOffsetY + sg->pipE )/sc->winw, 0 );
						}
					}
					glScalef( sg->northOnMapSize/sc->winw, -sg->northOnMapSize/sc->winw, 1 );
					if( sc->show3D ) glRotatef( gsc->angle3D, 1, 0, 0 );
					glRotatef( v->neckPsi*CF_RAD2DEG + 90, 0, 0, -1 );
					glCallList( sg->northOnMap_dl );
					glPopMatrix();
				}

				glLineWidth( 1 );
			}

			/* show wind on map */
			windMag = sqrt( SQ( o->wind[0] ) + SQ( o->wind[1] ) )*C_FPS2KT;
			if( windMag > 2.0 ) {
				double windDir, stretch;

				windDir = atan2( -o->wind[1], -o->wind[0] )*C_RAD2DEG;
				windDir = hmod360( windDir );
				if( windDir < 0.5 ) windDir += 360.0;
				windDir = LIMIT( windDir, 0, 360 );
				windMag = LIMIT( windMag, 0, 1000 );
				stretch = 1.0 - exp( -(float)windMag/10 );

				switch( sg->speedUnits ) {
				case SPEED_FPS:
				default:
					windMag *= C_KT2FPS;
					break;
				case SPEED_KNOTS:
					break;
				case SPEED_KPH:
					windMag *= C_KT2KPH;
					break;
				case SPEED_MPH:
					windMag *= C_KT2MPH;
					break;
				case SPEED_MPS:
					windMag *= C_KT2MPS;
					break;
				}

				for( k=((sg->hudLW[0]==0.0&&sc->videoModeDerived==0)||sg->hudVideoLW[0]==0.0); k<=(sg->hudLW[1]>0.0); k++ ) {

					if( sc->videoModeDerived ) {
						glLineWidth( sg->hudVideoLW[k] );
					} else {
						glLineWidth( sg->hudLW[k] );
					}
					glColor4fv( sg->hudColor[k] );

					glPushMatrix();
					glTranslatef( (float)(1.0 - ( sg->northOnMapSize + 2.0 )/sc->winw),
								  (float)(      ( sg->northOnMapSize + 2.0 )/sc->winw), 0 );
					if( gsc->showPIP ) {
						if( sg->pipOffsetX < 60 && sg->pipOffsetY < 60 ) {
							glTranslatef( 0, (float)( scenePIP.winh + sg->pipOffsetY + sg->pipE )/sc->winw, 0 );
						}
					}
					glScalef( sg->northOnMapSize/sc->winw, -sg->northOnMapSize/sc->winw, 1 );
					sprintf( buffer, "%03.0f@%.0f", windDir, windMag );
					glPushMatrix();
					glRotatef( 180, 0, 0, 1 );
					glRotatef( 180, 0, 1, 0 );
					showMessage( -0.67f, -0.9f, buffer, 20 );
					glPopMatrix();
					if( sc->show3D ) glRotatef( gsc->angle3D, 1, 0, 0 );
					glRotatef( v->neckPsi*CF_RAD2DEG, 0, 0, -1 );
					glRotatef( (float)(windDir), 0.0, 0.0, 1.0 );
					glScalef( 50.0, (float)(50.0*stretch), 50.0 );
					glBegin( GL_LINES );
					glVertex3f(  0.0,    0.02f, 0.0 );
					glVertex3f(  0.0,   -0.02f, 0.0 );
					glVertex3f(  0.0,    0.02f, 0.0 );
					glVertex3f(  0.005f, 0.01f, 0.0 );
					glVertex3f(  0.0,    0.02f, 0.0 );
					glVertex3f( -0.005f, 0.01f, 0.0 );
					glEnd();
					glPopMatrix();
				}
            }
	    }
	}

	/* big red x if datalink down */
	if( gi->panel->inop && sc->showGCS ) {
		glLineWidth( sg->menuTextLW ); /* just make it consistent with data block line width one */
		if( fmod( sim.time, 2*gi->panel->blinkRate ) < gi->panel->blinkRate ) {
			glColor4fv( gi->panel->boxColor[BOX_RED] );
		} else {
			glColor4fv( gi->panel->boxColor[BOX_REDBLINK] );
		}
		glBegin( GL_LINES );
		glVertex3f( 0.35f, 0.5f*sc->winh/sc->winw - 0.15f, 0 );
		glVertex3f( 0.65f, 0.5f*sc->winh/sc->winw + 0.15f, 0 );
		glVertex3f( 0.35f, 0.5f*sc->winh/sc->winw + 0.15f, 0 );
		glVertex3f( 0.65f, 0.5f*sc->winh/sc->winw - 0.15f, 0 );
		glEnd();
	}
}

/**
 * Determines whether to add an additional line to the joystick output for held digital functions
 * @param digiF - the digital function to test
 * @param line - in/out value - may add the line at this point
 * @param lines - in/out value - the current line at which to add text (may be incremented)
 */
static void addDigitalFunction(struct digitalFunction_ref *digiF, unsigned char* line, int* lines)
{
	if((digiF->inUse != 0) && (digiF->secRemaining > 0.0))
	{
		// Create the line
		int secRemaining = (int)(digiF->secHold);
		sprintf(line, "%s %d", digiF->label, (int)ceil(digiF->secRemaining));
		(*lines)++;
	}
}

/* this is stuff that wouldn't go on PIP (menu, data blocks, frame counter...) */
static void redrawSingleScene2( struct sceneGlobal_ref *sg, struct scene_ref *sc ) {

	struct gcs_ref              *g   = &gcs;
	struct gcsInstance_ref      *gi  = gcsActiveInstance( g );
	struct gcsScene_ref         *gsc = whichGcsScene( sc, gi );
	struct vehicleOutputs_ref   *no  = gi->outputs;

	struct vehicleOutputs_ref *o;
	struct view_ref *v;

	unsigned char buffer[BUFFER_SIZE];
	int numberWay, pickWay;
	int i;

	v = whichView( gi->set, gsc->viewMode );

	switch( sc->lookat ) {
	default:
	case LOOKAT_TRUTH:
	case LOOKAT_BOTH:
		o = &vehicleOutputs;
		break;
	case LOOKAT_GCS:
		o = gi->outputs;
		break;
	}

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glOrtho( 0.0, 1.0, 0.0, 1.0*sc->winh/sc->winw, -1.0, 1.0 );
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	/* panel */

	if( sc->showGCS && gsc->showPanel ) {

		int panelWidth, winwForPanel, side = 0;

		winwForPanel = sc->winw;
		if( gsc->showPIP ) {
			if( sg->pipOffsetY < ( gi->panel->nboxes - 1 )/MAX(1,gi->panel->perRow)*gi->panel->boxh + sg->panelUp ) {
				if( sg->pipOffsetX + (int)((float)sc->winh*sg->pipSize*sg->pipRatio)/2 < sc->winw/2 ) {
					winwForPanel = sc->winw - (int)((float)sc->winh*sg->pipSize*sg->pipRatio) - sg->pipOffsetX;
					side = -1;
				} else {
					winwForPanel = sg->pipOffsetX;
					side = 1;
				}
			}
		}

		panelWidth = (int)(MIN( sg->panelWidth, winwForPanel ));
		panelWidth = MIN( panelWidth, gi->panel->nboxes*gi->panel->boxw ); /* in case even a single row doesn't fill it */

		glPushMatrix();
		glScalef( (float)(1.0/sc->winw), (float)(-1.0/sc->winw), 1 );
		if( side == -1 )
			glTranslatef( (float)(( winwForPanel - panelWidth )/2), (float)(-sc->winh), 0 );
		else if( side == 0 )
			glTranslatef( (float)(( sc->winw - panelWidth )/2), (float)(-sc->winh), 0 );
		else /* side = 1 */
			glTranslatef( (float)(sc->winw - ( winwForPanel + panelWidth )/2), (float)(-sc->winh), 0 );
		panelSubDraw( gi->panel, panelWidth, (int)(sc->winh - sg->panelUp) );
		glPopMatrix();

	}

	/* actuator deflections */
	if( sc->showGCS && gsc->showAutopilotDels ) {

		char showHypothetical = 0;

		if ( ( ABS( gi->datalink->m1->time - gi->datalink->autopilotDels->time ) < gi->datalink->autopilotDelsTimeout ) && /* data is good from hypothetical autopilot */
				 ( 0 == gi->datalink->autopilot ) ) { /* autopilot is off */
			showHypothetical = 1;
		}

		if( ( ( 3 == gsc->showAutopilotDels ) ||                               /* we _really_ want to see it */
			  ( 2 == gsc->showAutopilotDels && gsc->virtualJoystick ) ) ||     /* show when virtual joysticks on */
			 ( showHypothetical ) ) {      /* show hypothetical commands and compare to current */

			char showThrottle = 0;

			glLineWidth( sg->gcsLineWidth );

			/* fixed items */
			glColor4fv( sg->virtualJoystickColor );

			glBegin( GL_LINE_LOOP );
			glVertex3f(        sg->joyX              , sg->joyY + sg->joySize, 0 );
			glVertex3f(        sg->joyX              , sg->joyY              , 0 );
			glVertex3f(        sg->joyX + sg->joySize, sg->joyY              , 0 );
			glVertex3f(        sg->joyX + sg->joySize, sg->joyY + sg->joySize, 0 );
			glEnd();

			glBegin( GL_LINE_LOOP );
			glVertex3f( 1.0f - sg->joyX - sg->joySize, sg->joyY + sg->joySize, 0 );
			glVertex3f( 1.0f - sg->joyX - sg->joySize, sg->joyY              , 0 );
			glVertex3f( 1.0f - sg->joyX              , sg->joyY              , 0 );
			glVertex3f( 1.0f - sg->joyX              , sg->joyY + sg->joySize, 0 );
			glEnd();

			if( showThrottle ) {
				glBegin( GL_LINE_STRIP );
				glVertex3f(        sg->joyX + sg->joySize     , sg->joyY + sg->joySize, 0 );
				glVertex3f(        sg->joyX + sg->joySize*1.2f, sg->joyY + sg->joySize, 0 );
				glVertex3f(        sg->joyX + sg->joySize*1.2f, sg->joyY              , 0 );
				glVertex3f(        sg->joyX + sg->joySize     , sg->joyY              , 0 );
				glEnd();
			}

			/* moving items */

			glColor4fv( sg->virtualJoystickOverColor );

			glBegin( GL_LINES );
			/* left */
			glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->m1->delf[0] + 1 ), 0 );
			glVertex3f(        sg->joyX              , sg->joyY + sg->joySize, 0 );
			glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->m1->delf[0] + 1 ), 0 );
			glVertex3f(        sg->joyX              , sg->joyY              , 0 );
			glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->m1->delf[0] + 1 ), 0 );
			glVertex3f(        sg->joyX + sg->joySize, sg->joyY              , 0 );
			glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->m1->delf[0] + 1 ), 0 );
			glVertex3f(        sg->joyX + sg->joySize, sg->joyY + sg->joySize, 0 );
			/* right */
			glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->m1->delm[1] + 1 ), 0 );
			glVertex3f( 1.0f - sg->joyX - sg->joySize, sg->joyY + sg->joySize, 0 );
			glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->m1->delm[1] + 1 ), 0 );
			glVertex3f( 1.0f - sg->joyX - sg->joySize, sg->joyY              , 0 );
			glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->m1->delm[1] + 1 ), 0 );
			glVertex3f( 1.0f - sg->joyX              , sg->joyY              , 0 );
			glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->m1->delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->m1->delm[1] + 1 ), 0 );
			glVertex3f( 1.0f - sg->joyX              , sg->joyY + sg->joySize, 0 );
			/* throttle */
			if( showThrottle ) {
				glVertex3f(        sg->joyX + sg->joySize     , sg->joyY + sg->joySize/2*( +gi->datalink->m1->delt[0] + 1 ), 0 );
				glVertex3f(        sg->joyX + sg->joySize*1.2f, sg->joyY + sg->joySize/2*( +gi->datalink->m1->delt[0] + 1 ), 0 );
				glVertex3f(        sg->joyX + sg->joySize*1.1f, sg->joyY + sg->joySize/2*( +gi->datalink->m1->delt[0] + 1 ), 0 );
				glVertex3f(        sg->joyX + sg->joySize*1.1f, sg->joyY                                                   , 0 );
			}
			glEnd();

			if( showHypothetical ) {
				glColor4fv( sg->autopilotDelsColor );

				glBegin( GL_LINES );
				/* left */
				glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->autopilotDels->c_delf[0] + 1 ), 0 );
				glVertex3f(        sg->joyX              , sg->joyY + sg->joySize, 0 );
				glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->autopilotDels->c_delf[0] + 1 ), 0 );
				glVertex3f(        sg->joyX              , sg->joyY              , 0 );
				glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->autopilotDels->c_delf[0] + 1 ), 0 );
				glVertex3f(        sg->joyX + sg->joySize, sg->joyY              , 0 );
				glVertex3f(        sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[2] + 1 ), sg->joyY + sg->joySize/2*( +gi->datalink->autopilotDels->c_delf[0] + 1 ), 0 );
				glVertex3f(        sg->joyX + sg->joySize, sg->joyY + sg->joySize, 0 );
				/* right */
				glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->autopilotDels->c_delm[1] + 1 ), 0 );
				glVertex3f( 1.0f - sg->joyX - sg->joySize, sg->joyY + sg->joySize, 0 );
				glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->autopilotDels->c_delm[1] + 1 ), 0 );
				glVertex3f( 1.0f - sg->joyX - sg->joySize, sg->joyY              , 0 );
				glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->autopilotDels->c_delm[1] + 1 ), 0 );
				glVertex3f( 1.0f - sg->joyX              , sg->joyY              , 0 );
				glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( +gi->datalink->autopilotDels->c_delm[0] - 1 ), sg->joyY + sg->joySize/2*( -gi->datalink->autopilotDels->c_delm[1] + 1 ), 0 );
				glVertex3f( 1.0f - sg->joyX              , sg->joyY + sg->joySize, 0 );
				/* throttle */
				if( showThrottle ) {
					glVertex3f(        sg->joyX + sg->joySize                                 , sg->joyY + sg->joySize/2*( +gi->datalink->autopilotDels->c_delt[0] + 1 ), 0 );
					glVertex3f(        sg->joyX + sg->joySize*1.2f                            , sg->joyY + sg->joySize/2*( +gi->datalink->autopilotDels->c_delt[0] + 1 ), 0 );
					glVertex3f(        sg->joyX + sg->joySize*1.1f + sg->gcsLineWidth/sc->winw, sg->joyY + sg->joySize/2*( +gi->datalink->autopilotDels->c_delt[0] + 1 ), 0 );
					glVertex3f(        sg->joyX + sg->joySize*1.1f + sg->gcsLineWidth/sc->winw, sg->joyY                                                                , 0 );
				}
				glEnd();
			}

			glLineWidth( 1 );
		}
	}

	/* virtual joysticks */
	if( gsc->virtualJoystick ) {
		struct motionControls_ref *co = &motionControls;
		float joy[4];

		joy[0] = (float)gi->cntrlInput->roll->output;
		joy[2] = (float)gi->cntrlInput->rudder->output;
		joy[1] = (float)(-gi->cntrlInput->pitch->output);
		joy[3] = (float)gi->cntrlInput->throttle->output;

		/*glLineWidth( sg->joyLW );*/
		if( sc->joyLeftMouseOver ) glColor4fv( sg->virtualJoystickOverColor );
		else                       glColor4fv( sg->virtualJoystickColor     );
		/* border line */
		glBegin( GL_QUAD_STRIP );
		glVertex3f(        sg->joyX               + sg->joyLW/2/sc->winw, sg->joyY + sg->joySize - sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX               - sg->joyLW/2/sc->winw, sg->joyY + sg->joySize + sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX               + sg->joyLW/2/sc->winw, sg->joyY               + sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX               - sg->joyLW/2/sc->winw, sg->joyY               - sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX + sg->joySize - sg->joyLW/2/sc->winw, sg->joyY               + sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX + sg->joySize + sg->joyLW/2/sc->winw, sg->joyY               - sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX + sg->joySize - sg->joyLW/2/sc->winw, sg->joyY + sg->joySize - sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX + sg->joySize + sg->joyLW/2/sc->winw, sg->joyY + sg->joySize + sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX               + sg->joyLW/2/sc->winw, sg->joyY + sg->joySize - sg->joyLW/2/sc->winw, 0 );
		glVertex3f(        sg->joyX               - sg->joyLW/2/sc->winw, sg->joyY + sg->joySize + sg->joyLW/2/sc->winw, 0 );
		glEnd();
		/* stick position */
		glBegin( GL_POLYGON );
		glVertex3f(        sg->joyX + sg->joySize/2*( joy[2] + 1 ) - sg->joySS, sg->joyY + sg->joySize/2*( joy[3] + 1 ) + sg->joySS, 0 );
		glVertex3f(        sg->joyX + sg->joySize/2*( joy[2] + 1 ) - sg->joySS, sg->joyY + sg->joySize/2*( joy[3] + 1 ) - sg->joySS, 0 );
		glVertex3f(        sg->joyX + sg->joySize/2*( joy[2] + 1 ) + sg->joySS, sg->joyY + sg->joySize/2*( joy[3] + 1 ) - sg->joySS, 0 );
		glVertex3f(        sg->joyX + sg->joySize/2*( joy[2] + 1 ) + sg->joySS, sg->joyY + sg->joySize/2*( joy[3] + 1 ) + sg->joySS, 0 );
		glEnd();

		if( sc->joyRightMouseOver ) glColor4fv( sg->virtualJoystickOverColor );
		else                        glColor4fv( sg->virtualJoystickColor     );
		/* border line */
		glBegin( GL_QUAD_STRIP );
		glVertex3f( 1.0f - sg->joyX - sg->joySize + sg->joyLW/2/sc->winw, sg->joyY + sg->joySize - sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX - sg->joySize - sg->joyLW/2/sc->winw, sg->joyY + sg->joySize + sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX - sg->joySize + sg->joyLW/2/sc->winw, sg->joyY               + sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX - sg->joySize - sg->joyLW/2/sc->winw, sg->joyY               - sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX               - sg->joyLW/2/sc->winw, sg->joyY               + sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX               + sg->joyLW/2/sc->winw, sg->joyY               - sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX               - sg->joyLW/2/sc->winw, sg->joyY + sg->joySize - sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX               + sg->joyLW/2/sc->winw, sg->joyY + sg->joySize + sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX - sg->joySize + sg->joyLW/2/sc->winw, sg->joyY + sg->joySize - sg->joyLW/2/sc->winw, 0 );
		glVertex3f( 1.0f - sg->joyX - sg->joySize - sg->joyLW/2/sc->winw, sg->joyY + sg->joySize + sg->joyLW/2/sc->winw, 0 );
		glEnd();
		/* stick position */
		glBegin( GL_POLYGON );
		glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( joy[0] - 1 ) - sg->joySS, sg->joyY + sg->joySize/2*( joy[1] + 1 ) + sg->joySS, 0 );
		glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( joy[0] - 1 ) - sg->joySS, sg->joyY + sg->joySize/2*( joy[1] + 1 ) - sg->joySS, 0 );
		glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( joy[0] - 1 ) + sg->joySS, sg->joyY + sg->joySize/2*( joy[1] + 1 ) - sg->joySS, 0 );
		glVertex3f( 1.0f - sg->joyX + sg->joySize/2*( joy[0] - 1 ) + sg->joySS, sg->joyY + sg->joySize/2*( joy[1] + 1 ) + sg->joySS, 0 );
		glEnd();
		/*glLineWidth( 1 );*/

	}

	/* waypoint information */
	if( ( numberWay = numberSelected( sc, &pickWay ) ) > 0 ) {

		unsigned char wayBuffer[MAN_NMANS+1][BUFFER_SIZE];
		int count = 0;
		int width = 0, height, heightL;

		height = messageHeight( sg, sg->wayFont );
		heightL = height + (int)sg->wayTextLH;

		if( sc->usedKeys ) {
			sprintf( wayBuffer[count], "step %.1f", sc->posStep );
			width = MAX( width, getBitmapLength( sg->wayFont, wayBuffer[count] ) );
			count++;
			numberWay++; /* really number of lines of text */
		}

		if( sg->instBackColor[3] > 0 ) {
			glColor4fv( sg->instBackColor );
			glBegin( GL_POLYGON );
			glVertex3f( ( sg->wayTextX )/sc->winw,
				( sg->wayTextY + heightL*( numberWay - 1 ) + height + sg->instBoxE*2 )/sc->winw, 0 );
			glVertex3f( ( sg->wayTextX )/sc->winw,
				( sg->wayTextY )/sc->winw, 0 );
			glVertex3f( ( sg->wayTextX + width + sg->instBoxE*2 )/sc->winw,
				( sg->wayTextY )/sc->winw, 0 );
			glVertex3f( ( sg->wayTextX + width + sg->instBoxE*2 )/sc->winw,
				( sg->wayTextY + heightL*( numberWay - 1 ) + height + sg->instBoxE*2 )/sc->winw, 0 );
			glEnd();
		}

		/* draw all text */
		glColor4fv( sg->selectedWayTextColor );
		for( i=0; i<numberWay; i++ ) {
			showBitmapMessage( ( sg->wayTextX + sg->instBoxE )/sc->winw,
				( sg->wayTextY + sg->instBoxE + heightL*( numberWay - i - 1 ) )/sc->winw,
				0, wayBuffer[i], sg->wayFont );
		}
	}

	/* GCS instrumentation extra text */
	/* note: always uses navigation outputs, never truth data (regardless of scene mode) */

	if( sc->showGCS && gsc->showGCStext ) {

		struct gcsInstance_ref *gis;
		struct gcsSet_ref *set;
		struct vehicleOutputs_ref *nso;
		int ig, igcycle;
		int width, height, heightL;
		float usedWidth = 0, justifyShift;

#define MESSAGECOLOR_NORMAL 0
#define MESSAGECOLOR_CAUTION 1
#define MESSAGECOLOR_WARNING 2
#define MESSAGECOLOR_LABEL 3
#define MESSAGECOLOR_PLAN 4
#define MAXINSTLINES 30

#define JUSTIFY_LEFT 0
#define JUSTIFY_CENTER 1

		for( igcycle=0; igcycle<GCS_MAX_INSTANCES; igcycle++ ) {

			ig = LIMIT( sg->gcsOrder[igcycle], 0, GCS_MAX_INSTANCES-1 );

			gis = gcsGetInstance( g, ig );
			nso = gis->outputs;

			if( gis->run ) {

				int lines = 0;
				unsigned char instBuffer[MAXINSTLINES][BUFFER_SIZE];
				unsigned char instColor[MAXINSTLINES]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
				unsigned char instJustify[MAXINSTLINES] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
				char showJoysticks = 0;
				float joy[4];
				char notActiveGCS;

				set = gis->set;

				if( g->active != ig ) notActiveGCS = 1;
				else                  notActiveGCS = 0;

				/* be careful not to let "lines" exceed MAXINSTLINES!!! */

				/* name of vehicle */

				if( sg->showNameText == 1 ) {
					instColor[lines] = MESSAGECOLOR_LABEL;
					instJustify[lines] = JUSTIFY_CENTER;
					sprintf( instBuffer[lines], "" );
					gcsAddVehicleName( instBuffer[lines++], gis->outputs );
				}

				/* time */

				if( set->showTimeText > notActiveGCS ) {
					int seconds, minutes, hours;
					seconds = (int)( gis->timer->flightTime );
					minutes = seconds/60;
					seconds -= minutes*60;
					hours = minutes/60;
					minutes -= hours*60;
					if( hours )
						sprintf( instBuffer[lines++], "%d:%02d:%02d", hours, minutes, seconds );
					else
						sprintf( instBuffer[lines++], "%d:%02d", minutes, seconds );
				}

				/* airspeed */
				if( set->showAirspeedText > notActiveGCS ) {
					double speed;
					speed = nso->cas;
					switch( sg->speedUnits ) {
					case SPEED_FPS:
					default:
						sprintf( instBuffer[lines++], "%.0f FPS", LIMIT( speed*C_KT2FPS, 0, 10000 ) );
						break;
					case SPEED_KNOTS:
						sprintf( instBuffer[lines++], "%.0f Kts", LIMIT( speed, 0, 10000 )  );
						break;
					case SPEED_KPH:
						sprintf( instBuffer[lines++], "%.0f KPH", LIMIT( speed*C_KT2KPH, 0, 10000 ) );
						break;
					case SPEED_MPH:
						sprintf( instBuffer[lines++], "%.0f MPH", LIMIT( speed*C_KT2MPH, 0, 10000 ) );
						break;
					case SPEED_MPS:
						sprintf( instBuffer[lines++], "%.0f m/s", LIMIT( speed*C_KT2MPS, 0, 10000 ) );
						break;
					}
				}

				/* ground speed */
				if( set->showGroundSpeedText > notActiveGCS ) {
					double speed;
					//speed = sqrt( SQ( nso->gs ) + SQ( nso->vs/60*C_FPS2KT ) );
					switch( sg->speedUnits ) {
					case SPEED_FPS:
					default:
						speed = nso->gs*C_KT2FPS;
						break;
					case SPEED_KNOTS:
						speed = nso->gs;
						break;
					case SPEED_KPH:
						speed = nso->gs*C_KT2KPH;						
						break;
					case SPEED_MPH:
						speed = nso->gs*C_KT2MPH;						
						break;
					case SPEED_MPS:
						speed = nso->gs*C_KT2MPS;						
						break;
					}
					speed = LIMIT( speed, 0, 10000 );
					if( speed < 9.95 ) {
						if( speed < 0.05 ) {
							sprintf( instBuffer[lines++], "GS0" );
						} else {
							sprintf( instBuffer[lines++], "GS%.1f", speed );
						}
					} else {
						sprintf( instBuffer[lines++], "GS%.0f", speed );
					}
				}

				if( set->showVerticalSpeedText > notActiveGCS ) {
					sprintf( instBuffer[lines++], "VS%.0f", nso->vs );
				}

				/* altitude above mean sea level */
				if( set->showMSLtext > notActiveGCS ) {
					sprintf( instBuffer[lines++], set->showMSLFmt, LIMIT( nso->altitudeMSL, -10000, 10000000 ) );
				}

				/* altitude above datum */
				if( set->showADLtext > notActiveGCS ) {
					sprintf( instBuffer[lines++], set->showADLFmt, LIMIT( nso->altitudeMSL - nso->datumAlt, -10000, 10000000 ) );
				}

				/* altitude above ground */
				if( set->showAGLtext > notActiveGCS ) {
					sprintf( instBuffer[lines++], set->showAGLFmt, LIMIT( nso->altitudeAGL, -10000, 1000000 ) );
				}

				/* heading */
				if( set->showHeading > notActiveGCS ) {
					double heading;
					char direction[5];

					if( set->rotate90 ) heading = nso->psi90;
					else                heading = nso->psi;
						/*heading -= no->phi*sin( no->theta*C_DEG2RAD );*/
					heading = hmod360( heading );
					if( heading > 359.5 ) heading = 0;
					if(      heading <  22.5 ) sprintf( direction, "N" );
					else if( heading <  67.5 ) sprintf( direction, "NE" );
					else if( heading < 112.5 ) sprintf( direction, "E" );
					else if( heading < 157.5 ) sprintf( direction, "SE" );
					else if( heading < 202.5 ) sprintf( direction, "S" );
					else if( heading < 247.5 ) sprintf( direction, "SW" );
					else if( heading < 292.5 ) sprintf( direction, "W" );
					else if( heading < 337.5 ) sprintf( direction, "NW" );
					else                       sprintf( direction, "N" );
					sprintf( instBuffer[lines++], "%s %03.0f%c", direction, heading, SCENEFONT_DEGREES );
				}

				/* throttle lever % */
				if( set->showThrottle > notActiveGCS ) {
					sprintf( instBuffer[lines++], "%02.0f%cTh", LIMIT( nso->delt[0]*50 + 50, -1000, 1000 ), 37 );
				}

				/* thrust lever % */
				if( set->showThrust > notActiveGCS ) {
					sprintf( instBuffer[lines++], "%02.0f%c", LIMIT( nso->delf[0]*50 + 50, -1000, 1000 ), 37 );
				}

				/* thrust lever % */
				if( set->showThrustPlusMinus > notActiveGCS ) {
					if( nso->delf[0] > -0.005 ) sprintf( instBuffer[lines++], "%02.0f%c",     LIMIT( +nso->delf[0]*100, 0, 1000 ), 37 );
					else                        sprintf( instBuffer[lines++], "%02.0f%c REV", LIMIT( -nso->delf[0]*100, 0, 1000 ), 37 );
				}

				/* RPM */
				if( set->showRPMtext > notActiveGCS ) {
					sprintf( instBuffer[lines++], "%.0fRPM", LIMIT( nso->rpm, -1000, 100000 ) );
				}

				/* range/bearing from datum */
				if( set->showRangeBearing > notActiveGCS ) {
					double range, bearing, dxrb[2];
					char units[10];

					dxrb[0] =        ( nso->latitude  - nso->datumLat )*C_NM2FT*60.0;
					dxrb[1] = hmodDeg( nso->longitude - nso->datumLon )*C_NM2FT*60.0*sc->cosDatumLat;
					range = sqrt( SQ( dxrb[0] ) + SQ( dxrb[1] ) );
					switch( sg->distanceUnits ) {
					default:
					case DISTANCE_FT:
						sprintf( units, "ft" );
						break;
					case DISTANCE_M:
						sprintf( units, "m" );
						range *= C_FT2M;
						break;
					case DISTANCE_NM:
						sprintf( units, "nm" );
						range *= C_FT2NM;
						break;
					case DISTANCE_KM:
						sprintf( units, "km" );
						range *= C_FT2KM;
						break;
					case DISTANCE_SM:
						sprintf( units, "sm" );
						range *= C_FT2SM;
						break;
					}
					bearing = hmod360( C_RAD2DEG*atan2( dxrb[1], dxrb[0] ) );
					if( bearing > 359.5 ) bearing = 0;

					if(      range < 0.005 ) sprintf( instBuffer[lines++], "%.2f%s/...",      range, units );
					else if( range < 10    ) sprintf( instBuffer[lines++], "%.2f%s/%03.0f%c", range, units, bearing, SCENEFONT_DEGREES );
					else if( range < 100   ) sprintf( instBuffer[lines++], "%.1f%s/%03.0f%c", range, units, bearing, SCENEFONT_DEGREES );
					else if( range < 20000 ) sprintf( instBuffer[lines++], "%.0f%s/%03.0f%c", range, units, bearing, SCENEFONT_DEGREES );
					else                     sprintf( instBuffer[lines++], ">20000%s/%03.0f%c",      units, bearing, SCENEFONT_DEGREES );
				}

				/* battery */
				if( set->showBatt ) {
					char show = 1;
					switch( gis->datalink->m1->batteryStatus ) {
					case 0:
					default:
						if( set->showBatt > notActiveGCS ) {
							instColor[lines] = MESSAGECOLOR_NORMAL;
						} else {
							show = 0;
						}
						break;
					case 1:
					case 2:
						instColor[lines] = MESSAGECOLOR_CAUTION;
						break;
					case 3:
						instColor[lines] = MESSAGECOLOR_WARNING;
						break;
					}
					if( show ) {
						sprintf( instBuffer[lines], "%c%.1fV", SCENEFONT_BATT, 0.001*gis->datalink->m1->battery );
						if( set->showCurrent ) {
							sprintf( instBuffer[lines], "%s %.0fA", instBuffer[lines], 0.001*gis->datalink->m1->current );
						}
						if( set->showAmpHours ) {
							sprintf( instBuffer[lines], "%s %.0fmAh", instBuffer[lines], gis->timer->mAh );
						}
						lines++;
					}
				}

				/* battery time remaining */
				if( set->showBattTime > notActiveGCS ) {
					float minutesRemain;
					int minutesShow;
					int secondsShow;
					minutesRemain = (float)(( 0.001*gis->datalink->m1->battery - set->battMinimumVoltage )*set->battMinutesPerVolt);
					minutesRemain = MAX( 0, minutesRemain );
					minutesShow = (int)(minutesRemain);
					secondsShow = (int)(( minutesRemain - minutesShow )*60);
					sprintf( instBuffer[lines++], "%c%01d:%02d", SCENEFONT_BATT, minutesShow, secondsShow );
				}

				/* fuel level */
				if( set->showFuelLevel ) {
					char show = 1;
					switch( gis->datalink->m1->fuel ) {
					case 0:
					default:
						if( set->showFuelLevel > notActiveGCS ) {
							instColor[lines] = MESSAGECOLOR_NORMAL;
						} else {
							show = 0;
						}
						break;
					case 1:
					case 2:
						instColor[lines] = MESSAGECOLOR_CAUTION;
						break;
					}
				}

				/* number of sats tracked */
				if( set->showGPS ) {
					if( gis->panel->box[BOX_GPS]->state == 0 && set->showGPStextMin ) {
						instColor[lines] = MESSAGECOLOR_WARNING;
						sprintf( instBuffer[lines++], "no GPS" );
					} else {
						if( gis->datalink->m1->numberOfSats < set->showGPStextMin ) {
							sprintf( instBuffer[lines++], "%d Sats", gis->datalink->m1->numberOfSats );
						}
						if( gis->datalink->m1->ubloxSNR && gis->datalink->m1->ubloxSNR < set->showGPSSNRMin ) {
							instColor[lines] = MESSAGECOLOR_CAUTION;
							sprintf( instBuffer[lines++], "s/n=%d", gis->datalink->m1->ubloxSNR );
						}
					}
				}

				/* wind information */
				if( set->showWind > notActiveGCS ) {
					double windMag, windDir;
					windMag = sqrt( SQ( o->wind[0] ) + SQ( o->wind[1] ) );
					if( windMag > 2.0 ) {
						windDir = atan2( -o->wind[1], -o->wind[0] )*C_RAD2DEG;
						windDir = hmod360( windDir );
						if( windDir < 0.5 ) windDir += 360.0;
						windDir = LIMIT( windDir, 0, 360 );
						windMag = LIMIT( windMag, 0, 1000 );
						switch( sg->speedUnits ) {
						case SPEED_FPS:
						default:
							break;
						case SPEED_KNOTS:
							windMag *= C_FPS2KT;
							break;
						case SPEED_KPH:
							windMag *= C_FPS2KPH;
							break;
						case SPEED_MPH:
							windMag *= C_FPS2MPH;
							break;
						case SPEED_MPS:
							windMag *= C_FT2M;
							break;
						}
						sprintf( instBuffer[lines++], "w%03.0f@%.0f", windDir, windMag );
					}
				}

				/* wait time (stop and wait waypoints) */
				if( set->showWaitTime > notActiveGCS ) {
					if( gi->traj->traj->stopTime > 0 ) {
						if( gi->traj->flightPlanDL->man[gi->traj->traj->manIndex]->type == MAN_STOPANDWAIT ) {
							int seconds, minutes, hours;
							seconds = (int)( gi->traj->flightPlanDL->man[gi->traj->traj->manIndex]->extra - gi->traj->traj->stopTime );
							minutes = seconds/60;
							seconds -= minutes*60;
							hours = minutes/60;
							minutes -= hours*60;
							instColor[lines] = MESSAGECOLOR_PLAN;
							if( hours )
								sprintf( instBuffer[lines++], "%d:%02d:%02d wait", hours, minutes, seconds );
							else
								sprintf( instBuffer[lines++], "%d:%02d wait", minutes, seconds );
						}
						if( gi->traj->flightPlanDL->man[gi->traj->traj->manIndex]->type == MAN_DIP ) {
							instColor[lines] = MESSAGECOLOR_PLAN;
							sprintf( instBuffer[lines++], "dip %.0f", gi->traj->traj->stopTime );
						}
					}
				}

				/* joystick indicator */
				if( sg->showJoystick && notActiveGCS==0 ) {
					struct motionControls_ref *co = &motionControls;
					if( gi->cntrlInput->mode == MOTIONINPUT_JOYSTICK ) { /* support only those joysticks really intended for use in GCS */
						if( gi->cntrlInput->joystickMode == JOYSTICK_MODE_LOGITECHRUMBLE2 ||
							gi->cntrlInput->joystickMode == JOYSTICK_MODE_SAITEKFPS ||
							gi->cntrlInput->joystickMode == JOYSTICK_MODE_ESTERLINE ||
							gi->cntrlInput->joystickMode == JOYSTICK_MODE_XBOX360 ||
							gi->cntrlInput->joystickMode == JOYSTICK_MODE_LOGITECH_F310 ||
							gi->cntrlInput->joystickMode == JOYSTICK_MAPPABLE ) {

							// Add held buttons
							addDigitalFunction(gis->cntrlInput->manOverride, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->arm, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->ground, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->groundLanding, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->air, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->airTakeoff, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->safeOff, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->safeOn, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->shutdown, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->suppressStick, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->dash, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->gpsDenied, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->gpsDenRestGps, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->gpsDenRestPos, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->suppressSonar, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->zoomIn, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->zoomOut, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->loadRunPlan, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->planToggle, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->videoToggle, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->joySiMan, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->rtb, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->stopPlan, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->systemSafety, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camStop, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camPause, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camPlay, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camRewind, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camFF, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camRecord, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camPowerOn, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->tglCamFocusManAuto, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camManFocusFarther, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camManFocusCloser, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camModeGeo, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camModeVideo, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camModeStow, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camModeJS, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->camModeToggle, instBuffer[lines], &lines);

							addDigitalFunction(gis->cntrlInput->yellowButton, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->whiteButton, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->blueButton, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->greenButton, instBuffer[lines], &lines);
							addDigitalFunction(gis->cntrlInput->bigRedButton, instBuffer[lines], &lines);

							if( (gsc->virtualJoystick == 0) &&
								(ABS( gis->cntrlInput->roll->output )   > 0.0 || ABS( gis->cntrlInput->pitch->output    ) > 0.0 ||
								ABS( gis->cntrlInput->rudder->output ) > 0.0 || ABS( gis->cntrlInput->throttle->output ) > 0.0 ||
								gis->cntrlInput->manOverride->output ) ) {

								joy[0] = (float)gis->cntrlInput->roll->output;
								joy[2] = (float)gis->cntrlInput->rudder->output;
								joy[1] = (float)(-gis->cntrlInput->pitch->output);
								joy[3] = (float)gis->cntrlInput->throttle->output;

								showJoysticks = 1;
								if( gis->cntrlInput->manOverride->output )
									sprintf( instBuffer[lines++], "c" ); /* indicate override enabled (not necessarily allowed) */
								else
									sprintf( instBuffer[lines++], " " );
							}
						}
					}
				}

				/* draw it all */
				/* determine width/height */
				width = 0;
				if( lines ) {
					for( i=0; i<lines; i++ ) {
						width = MAX( width, getBitmapLength( sg->instFont, instBuffer[i] ) );
					}
					height = messageHeight( sg, sg->instFont );
					heightL = height + (int)sg->instTextLH;
					if( showJoysticks ) width = MAX( width, (int)(height*3) );
				} else {
					height = 0;
					heightL = 0;
				}

				sc->textBox[ig][0][0] = usedWidth + sg->instTextX + 1;
				sc->textBox[ig][0][1] = (float)sc->winh - sg->instTextY;
				sc->textBox[ig][1][0] = usedWidth + sg->instTextX + width + sg->instBoxE*2;
				sc->textBox[ig][1][1] = (float)sc->winh - sg->instTextY - heightL*( lines - 1 ) - height - sg->instBoxE*2;

				if( sg->instBackColor[3] > 0 ) {
					glColor4fv( sg->instBackColor );
					glBegin( GL_POLYGON );
					glVertex3f( sc->textBox[ig][0][0]/sc->winw, sc->textBox[ig][0][1]/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][0][0]/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][1][0]/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][1][0]/sc->winw, sc->textBox[ig][0][1]/sc->winw, 0 );
					glEnd();
				}

				if( ig == g->active && sg->drawLineToText ) { /* highlight active one */
					/*glLineWidth( sg->menuTextLW );*/
					glColor4fv( sg->menuTextColor );
					glBegin( GL_POLYGON );
					glVertex3f( sc->textBox[ig][0][0]/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][0][0]/sc->winw, ( sc->textBox[ig][1][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][1][0]/sc->winw, ( sc->textBox[ig][1][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][1][0]/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glEnd();
					/*glLineWidth( 1 );*/
				}

				if( sg->drawLineToText && sc->rasterPosVehicle[ig][0] >= 0 /*&& notActiveGCS*/ ) { /* line between this text box and vehicle itself */
					switch( gis->panel->status ) {
					default:
						glColor4fv( sg->menuTextColor );
						break;
					case 1:
						glColor4fv( gis->panel->boxColor[BOX_YELLOW] );
						break;
					case 2:
						if( fmod( sim.time, 2*gis->panel->blinkRate ) < gis->panel->blinkRate ) {
							glColor4fv( gis->panel->boxColor[BOX_RED] );
						} else {
							glColor4fv( gis->panel->boxColor[BOX_REDBLINK] );
						}
						break;
					}

					//glLineWidth( sg->menuTextLW );
					//glBegin( GL_LINE_STRIP );
					//glVertex3f( ( sc->textBox[ig][0][0] + sc->textBox[ig][1][0] )/2/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					//glVertex3f( sc->rasterPosVehicle[ig][0]/sc->winw, sc->rasterPosVehicle[ig][1]/sc->winw, 0 );
					glLineWidth( 1 );
					glBegin( GL_POLYGON );
					if( sc->rasterPosVehicle[ig][1] < sc->textBox[ig][1][1] ) {
						glVertex3f( ( sc->textBox[ig][0][0] + sc->textBox[ig][1][0] + sg->menuTextLW*2 )/2/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
						glVertex3f( ( sc->textBox[ig][0][0] + sc->textBox[ig][1][0] - sg->menuTextLW*2 )/2/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
						glVertex3f( sc->rasterPosVehicle[ig][0]/sc->winw, sc->rasterPosVehicle[ig][1]/sc->winw, 0 );
					} else {
						glVertex3f( ( sc->textBox[ig][0][0] + sc->textBox[ig][1][0] - sg->menuTextLW*2 )/2/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
						glVertex3f( ( sc->textBox[ig][0][0] + sc->textBox[ig][1][0] + sg->menuTextLW*2 )/2/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
						glVertex3f( sc->rasterPosVehicle[ig][0]/sc->winw, sc->rasterPosVehicle[ig][1]/sc->winw, 0 );
					}
					glEnd();
					glBegin( GL_LINE_LOOP );
					glVertex3f( ( sc->textBox[ig][0][0] + sc->textBox[ig][1][0] + sg->menuTextLW*2 )/2/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] + sc->textBox[ig][1][0] - sg->menuTextLW*2 )/2/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
					glVertex3f( sc->rasterPosVehicle[ig][0]/sc->winw, sc->rasterPosVehicle[ig][1]/sc->winw, 0 );
					glEnd();
					//glLineWidth( 1 );
				}

				if( sc->textMouseOver[ig] ) { /* highlight if mouse over */
					/*glLineWidth( sg->menuTextLW );*/
					glColor4fv( sg->menuTextColor );
					/*glBegin( GL_LINE_LOOP );
					glVertex3f( sc->textBox[ig][0][0]/sc->winw, sc->textBox[ig][0][1]/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][0][0]/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][1][0]/sc->winw, sc->textBox[ig][1][1]/sc->winw, 0 );
					glVertex3f( sc->textBox[ig][1][0]/sc->winw, sc->textBox[ig][0][1]/sc->winw, 0 );
					glEnd();*/
					glBegin( GL_QUAD_STRIP );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] - sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] - sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][1][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] + sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][1][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] + sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] - sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glEnd();
					/*glLineWidth( 1 );*/
				}

				if( gis->panel->status > 0 /*&& notActiveGCS*/ ) { /* highlight warning/caution */
					switch( gis->panel->status ) {
					case 1:
						glColor4fv( gis->panel->boxColor[BOX_YELLOW] );
						break;
					case 2:
						if( fmod( sim.time, 2*gis->panel->blinkRate ) < gis->panel->blinkRate ) {
							glColor4fv( gis->panel->boxColor[BOX_RED] );
						} else {
							glColor4fv( gis->panel->boxColor[BOX_REDBLINK] );
						}
						break;
					default: break;
					}
					/*glLineWidth( sg->menuTextLW );*/

					/*glBegin( GL_LINE_LOOP );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW/2 )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW/2 )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW/2 )/sc->winw, 0 );
					glEnd();*/
					glBegin( GL_QUAD_STRIP );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0]                  )/sc->winw, ( sc->textBox[ig][0][1]                  )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0]                  )/sc->winw, ( sc->textBox[ig][1][1]                  )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0]                  )/sc->winw, ( sc->textBox[ig][1][1]                  )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0]                  )/sc->winw, ( sc->textBox[ig][0][1]                  )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0]                  )/sc->winw, ( sc->textBox[ig][0][1]                  )/sc->winw, 0 );
					glEnd();
					/*glLineWidth( 1.0 );*/
				}

				if( lines ) {
					for( i=0; i<lines; i++ ) {
						switch( instColor[i] ) {
						case MESSAGECOLOR_LABEL:
							glColor4fv( sg->menuTextColor );
							break;
						case MESSAGECOLOR_WARNING:
							glColor4fv( sg->instTextWarningColor );
							break;
						case MESSAGECOLOR_CAUTION:
							glColor4fv( sg->instTextCautionColor );
							break;
						case MESSAGECOLOR_PLAN:
							glColor4fv( sg->selectedWayColor );
							break;
						case MESSAGECOLOR_NORMAL:
						default:
							glColor4fv( sg->instTextColor );
							break;
						}
						switch( instJustify[i] ) {
						case JUSTIFY_LEFT:
						default:
							justifyShift = 0;
							break;
						case JUSTIFY_CENTER:
							justifyShift = ( width - getBitmapLength( sg->instFont, instBuffer[i] ) )*0.5f;
							break;
						}
						showBitmapMessage( ( usedWidth + sg->instTextX + sg->instBoxE + justifyShift )/sc->winw,
							((float)sc->winh - sg->instTextY - heightL*i - height - sg->instBoxE )/sc->winw,
							0, instBuffer[i], sg->instFont );
					}

					/* joystick deflections */
					if( showJoysticks ) {
						glLineWidth( sg->instTextLW );
						glPointSize( sg->instTextLW );
						glBegin( GL_POINTS );
						glVertex3f( ( usedWidth + sg->instTextX + 1 + height*1.5f + sg->instBoxE )/sc->winw,
							((float)sc->winh - sg->instTextY - heightL*( -1.5f + (float)lines ) - height - sg->instBoxE - 1 )/sc->winw, 0 );
						glVertex3f( ( usedWidth + sg->instTextX + 1 + height*2.5f + sg->instBoxE )/sc->winw,
							((float)sc->winh - sg->instTextY - heightL*( -1.5f + (float)lines ) - height - sg->instBoxE - 1 )/sc->winw, 0 );
						glEnd();
						glBegin( GL_LINES );
						glVertex3f( ( usedWidth + sg->instTextX + 1 + height*1.5f + sg->instBoxE )/sc->winw,
							((float)sc->winh - sg->instTextY - heightL*( -1.5f + (float)lines ) - height - sg->instBoxE - 1 )/sc->winw, 0 );
						glVertex3f( ( usedWidth + sg->instTextX + 1 + height*1.5f + sg->instBoxE + joy[2]*height*0.5f )/sc->winw,
							((float)sc->winh - sg->instTextY - heightL*( -1.5f + (float)lines ) - height - sg->instBoxE + joy[3]*height*0.5f - 1 )/sc->winw, 0 );
						glVertex3f( ( usedWidth + sg->instTextX + 1 + height*2.5f + sg->instBoxE )/sc->winw,
							((float)sc->winh - sg->instTextY - heightL*( -1.5f + (float)lines ) - height - sg->instBoxE - 1 )/sc->winw, 0 );
						glVertex3f( ( usedWidth + sg->instTextX + 1 + height*2.5f + sg->instBoxE + joy[0]*height*0.5f )/sc->winw,
							((float)sc->winh - sg->instTextY - heightL*( -1.5f + (float)lines ) - height - sg->instBoxE + joy[1]*height*0.5f - 1 )/sc->winw, 0 );
						glEnd();
						glLineWidth( 1.0 );
						glPointSize( 1.0 );
					}

				}

				/* big red x if datalink down */
				if( gis->panel->inop ) {
					if( fmod( sim.time, 2*gis->panel->blinkRate ) < gis->panel->blinkRate ) {
						glColor4fv( gis->panel->boxColor[BOX_RED] );
					} else {
						glColor4fv( gis->panel->boxColor[BOX_REDBLINK] );
					}

					glBegin( GL_POLYGON );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW*0.7f )/sc->winw, ( sc->textBox[ig][0][1]                       )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0]                       )/sc->winw, ( sc->textBox[ig][0][1]                       )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0]                       )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW*0.7f )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW*0.7f )/sc->winw, ( sc->textBox[ig][1][1]                       )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0]                       )/sc->winw, ( sc->textBox[ig][1][1]                       )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0]                       )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW*0.7f )/sc->winw, 0 );
					glEnd();

					glBegin( GL_POLYGON );
					glVertex3f( ( sc->textBox[ig][1][0]                       )/sc->winw, ( sc->textBox[ig][0][1] - sg->menuTextLW*0.7f )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0]                       )/sc->winw, ( sc->textBox[ig][0][1]                       )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][1][0] - sg->menuTextLW*0.7f )/sc->winw, ( sc->textBox[ig][0][1]                       )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0]                       )/sc->winw, ( sc->textBox[ig][1][1] + sg->menuTextLW*0.7f )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0]                       )/sc->winw, ( sc->textBox[ig][1][1]                       )/sc->winw, 0 );
					glVertex3f( ( sc->textBox[ig][0][0] + sg->menuTextLW*0.7f )/sc->winw, ( sc->textBox[ig][1][1]                       )/sc->winw, 0 );
					glEnd();

				}

				usedWidth += width + sg->instBoxE*3;

			} else {

				sc->textBox[ig][0][0] = -1; /* make sure it isn't on screen */
				sc->textBox[ig][0][1] = -1;

			}
		}
	}

	/* zoom bar or menu */

	if( sc->yzoom != -1 ) {

		glColor4fv( sg->instBackColor );
		glBegin( GL_POLYGON );
		glVertex3f( (float)(sc->winw - sg->zoomWide)/sc->winw, (float)(sc->winh - 2)/sc->winw, 0 );
		glVertex3f( (float)(sc->winw - sg->zoomWide)/sc->winw, (float)(2)/sc->winw,            0 );
		glVertex3f( (float)(sc->winw - 2)/sc->winw,            (float)(2)/sc->winw,            0 );
		glVertex3f( (float)(sc->winw - 2)/sc->winw,            (float)(sc->winh - 2)/sc->winw, 0 );
		glEnd();

	} else if( gsc->showMenu ) {

		int height, heightL;
		float modeMenuX, customMenuX;

		height = messageHeight( sg, sg->menuFont );
		heightL = height + (int)sg->menuH;

		glLineWidth( sg->menuTextLW );

		if( sc->viewMenuOpen ) {
			unsigned char menuBuffer[VIEWMENUITEMS][BUFFER_SIZE];
			char inverse[VIEWMENUITEMS] = {1,0,0,0,0};
			switch( gsc->viewMode ) {
			default:
				break;
			case VIEW_CAMERA:
				inverse[1] = 1;
				break;
			case VIEW_NAV:
				inverse[2] = 1;
				break;
			case VIEW_CHASE:
				inverse[3] = 1;
				break;
			case VIEW_COCKPIT:
				inverse[4] = 1;
				break;
			}
			sprintf( menuBuffer[0], "view" );
			sprintf( menuBuffer[1], "camera" );
			sprintf( menuBuffer[2], "map" );
			sprintf( menuBuffer[3], "chase" );
			sprintf( menuBuffer[4], "PFD" );
			for( i=0; i<VIEWMENUITEMS; i++ ) {
			drawMenuItem( sg, sc,
				((float)sc->winw - sg->viewMenuX*sg->menuW - sg->menuX )/sc->winw,
				((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
				menuBuffer[i], (sc->viewMenuMouseOver==i?1:0), inverse[i], 1 );
			}
		} else {
			switch( gsc->viewMode ) {
			default:
				sprintf( buffer, "view" );
				break;
			case VIEW_COCKPIT:
				sprintf( buffer, "PFD" );
				break;
			case VIEW_NAV:
				sprintf( buffer, "map" );
				break;
			case VIEW_CHASE:
				sprintf( buffer, "chase" );
				break;
			case VIEW_GROUND:
				sprintf( buffer, "ground" );
				break;
			case VIEW_HOVER:
				sprintf( buffer, "hover" );
				break;
			case VIEW_CAMERA:
				sprintf( buffer, "camera" );
				break;
			case VIEW_CAMERA2:
				sprintf( buffer, "camera2" );
				break;
			case VIEW_CAMERA3:
				sprintf( buffer, "camera3" );
				break;
			case VIEW_CAMERA4:
				sprintf( buffer, "camera4" );
				break;
			case VIEW_IPRESULTS:
				sprintf( buffer, "IP" );
				break;
			}
			drawMenuItem( sg, sc,
				((float)sc->winw - sg->viewMenuX*sg->menuW - sg->menuX )/sc->winw,
				((float)sc->winh - sg->menuY)/sc->winw,
				buffer, (sc->viewMenuMouseOver==0?1:0), 0, 1 );
			if( gsc->viewMode == VIEW_NAV ) {
				int index, mapUp;
				if( gsc->show3Dmap )
					sprintf( buffer, "3D" );
				else {
					sprintf( buffer, "2D" );
				}
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->viewMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY - heightL )/sc->winw,
					buffer, (sc->viewMenuMouseOver==1?1:0), 0, 1 );
				if( sg->autoscaleNav ) {
					sprintf( buffer, "auto" );
				} else {
					switch( v->lookat ) {
					case VIEW_LOOKAT_NOTHING:  sprintf( buffer, "fixed" );  break;
					case VIEW_LOOKAT_VEHICLE:  sprintf( buffer, "move"  );  break;
					}
				}
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->viewMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY - heightL*2 )/sc->winw,
					buffer, (sc->viewMenuMouseOver==2?1:0), 0, 1 );
				mapUp = gsc->mapUpMode;
				if( v->lookat == VIEW_LOOKAT_NOTHING && sg->autoscaleNav == 0 ) mapUp = MAPUP_CONST;
				switch( mapUp ) {
				case MAPUP_CONST: /* or not looking at vehicle */
				default:
					if( gsc->mapUpAngle >= 0 ) index =     ((int)( +gsc->mapUpAngle*4.0f/CF_PI + 0.5f ))%8;
					else                       index = 8 - ((int)( -gsc->mapUpAngle*4.0f/CF_PI + 0.5f ))%8;
					switch( index ) {
					default:
					case 0:  sprintf( buffer, "N up"  );  break;
					case 1:  sprintf( buffer, "NE up" );  break;
					case 2:  sprintf( buffer, "E up"  );  break;
					case 3:  sprintf( buffer, "SE up" );  break;
					case 4:  sprintf( buffer, "S up"  );  break;
					case 5:  sprintf( buffer, "SW up" );  break;
					case 6:  sprintf( buffer, "W up"  );  break;
					case 7:  sprintf( buffer, "NW up" );  break;
					}
					break;
				case MAPUP_HEADING:
					sprintf( buffer, "fwd up" );
					break;
				case MAPUP_TRACK:
					sprintf( buffer, "trk up" );
					break;
				}
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->viewMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY - heightL*3 )/sc->winw,
					buffer, (sc->viewMenuMouseOver==3?1:0), 0, (v->lookat == VIEW_LOOKAT_VEHICLE||sg->autoscaleNav?1:0) );
			}
		}

		if( sc->visMenuOpen ) {
			unsigned char menuBuffer[VISMENUITEMS][BUFFER_SIZE];
			char inverse[VISMENUITEMS] = {1,0,0,0,0,0,0,0,0,0,0,0};
			sprintf( menuBuffer[0], "show" );
			sprintf( menuBuffer[1], "clear" );
			sprintf( menuBuffer[2], "PIP %c", SCENEFONT_PIP );
			inverse[2] = gsc->showPIP;
			sprintf( menuBuffer[3], "controls" );
			inverse[3] = gsc->virtualJoystick;
			sprintf( menuBuffer[4], "HUD" );
			inverse[4] = v->hudOn;
			sprintf( menuBuffer[5], "trace" );
			inverse[5] = sc->showTraj;
			sprintf( menuBuffer[6], "plan" );
			if( 1 == gsc->showPlan && gi->set->showPlan ) inverse[6] = 1;
			sprintf( menuBuffer[7], "grid %c", SCENEFONT_GRID );
			inverse[7] = sc->showGrid;
			sprintf( menuBuffer[8], "FOV" );
			inverse[8] = gsc->showCameraFOV;
			sprintf( menuBuffer[9], "overlay" );
			inverse[9] = gsc->showTex;
			sprintf( menuBuffer[10], "video" );
			inverse[10] = gsc->videoMode;
			sprintf( menuBuffer[11], "%c List ", SCENEFONT_CHECKED );
			inverse[11] = sc->showingChecklist;
			for( i=0; i<VISMENUITEMS; i++ ) {
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->visMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
					menuBuffer[i], (sc->visMenuMouseOver==i?1:0), inverse[i], 1 );
			}
		} else {
			sprintf( buffer, "show..." );
			drawMenuItem( sg, sc,
				((float)sc->winw - sg->visMenuX*sg->menuW - sg->menuX )/sc->winw,
				((float)sc->winh - sg->menuY)/sc->winw,
				buffer, (sc->visMenuMouseOver==0?1:0), 0, 1 );
		}

		if( sc->wayMenuOpen ) {
			unsigned char menuBuffer[WAYMENUITEMS][BUFFER_SIZE];
			char inverse[WAYMENUITEMS] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			char active[WAYMENUITEMS]  = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
			switch( gi->traj->edit ) {
				default:
				case PLANEDIT_A:
					sprintf( menuBuffer[0], "plan A" );
					break;
				case PLANEDIT_B:
					sprintf( menuBuffer[0], "plan B" );
					break;
				case PLANEDIT_LC:
					sprintf( menuBuffer[0], "LostC" );
					break;
			}
			sprintf( menuBuffer[1],  "%cplan", SCENEFONT_CW );
			sprintf( menuBuffer[2],  "%cprev", SCENEFONT_LEFTARROW );
			sprintf( menuBuffer[3],  "next%c", SCENEFONT_RIGHTARROW );
			sprintf( menuBuffer[4],  "select" );
			inverse[4] = sc->waySelectMode;
			switch( sc->wayInsertMode ) {
				case 0:
				default:
					sprintf( menuBuffer[5], "ins" );
					break;
				case 1:
					sprintf( menuBuffer[5], "ins 1" );
					break;
				case 2:
					sprintf( menuBuffer[5], "ins %c", SCENEFONT_INFINITY );
					break;
			} 
			if( sc->wayInsertMode ) inverse[5] = 1;
			sprintf( menuBuffer[6], "del" );
			sprintf( menuBuffer[7], "%cback", SCENEFONT_LEFTARROW);
			sprintf( menuBuffer[8], "fwd%c",  SCENEFONT_RIGHTARROW);
			/*if( numberSelected( sc, &pickWay ) == 0 && trajectoryWork.lastIndex > 0 ) active[6] = 0;*/
			sprintf( menuBuffer[9],  "repeat" );
			inverse[9] = 0;
			sprintf( menuBuffer[10],  "open..." );
			sprintf( menuBuffer[11],  "save..." );
			sprintf( menuBuffer[12], "dnload" );
			sprintf( menuBuffer[13], "Upload" );
			inverse[13] = gi->traj->uploadEach[gi->traj->edit];
			if( gi->traj->traj->safemode != 1 ) {
                     
					sprintf( menuBuffer[14], "STOP" );
				
				/*inverse[12] = 1;*/
			} else {
				sprintf( menuBuffer[14], "GO %c", SCENEFONT_TRIANGLE );
				active[14] = !inverse[13];
			}
			/*inverse[12] = ( gi->traj->traj->safemode == 0 || gi->traj->traj->safemode == 2 );*/
			sprintf( menuBuffer[15], "GO TO" );
			active[15] = ( numberSelected( sc, &pickWay ) == 1 && pickWay != MAN_NMANS && !inverse[11] );

			for( i=0; i<WAYMENUITEMS; i++ ) {
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->wayMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
					menuBuffer[i], (sc->wayMenuMouseOver==i?1:0), inverse[i], active[i] );
			}
		} else {
			switch( gi->traj->edit ) {
			case PLANEDIT_A:
			default:
				sprintf( buffer, "planA..." );
				break;
			case PLANEDIT_B:
				sprintf( buffer, "planB..." );
				break;
			case PLANEDIT_LC:
				sprintf( buffer, "LostC..." );
				break;
			}
			drawMenuItem( sg, sc,
				((float)sc->winw - sg->wayMenuX*sg->menuW - sg->menuX )/sc->winw,
				((float)sc->winh - sg->menuY)/sc->winw,
				buffer, (sc->wayMenuMouseOver==0?1:0), 0, 1 );
		}

		if( numberSelected( sc, &pickWay ) > 0 ) {

			if( sc->typeMenuOpen ) {
				unsigned char menuBuffer[TYPEMENUITEMS][BUFFER_SIZE];
				char inverse[TYPEMENUITEMS] = {1,0,0,0,0,0,0,0,0};
				char active[TYPEMENUITEMS]  = {1,1,1,1,1,1,1,1,1};
				sprintf( menuBuffer[0], "type" );
				sprintf( menuBuffer[1], "cut" );
				sprintf( menuBuffer[2], "through" );
				sprintf( menuBuffer[3], "wait" );
				sprintf( menuBuffer[4], "climb" );
				sprintf( menuBuffer[5], "land" );
				sprintf( menuBuffer[6], "chase" );
				sprintf( menuBuffer[7], "formatn" );
				sprintf( menuBuffer[8], "extSeq" );
				/*sprintf( menuBuffer[8], "dip" );
				if( gi->set->controlType != CONTROLTYPE_HELI ) active[8] = 0;*/

				for( i=0; i<TYPEMENUITEMS; i++ ) {
					drawMenuItem( sg, sc,
						((float)sc->winw - sg->typeMenuX*sg->menuW - sg->menuX )/sc->winw,
						((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
						menuBuffer[i], (sc->typeMenuMouseOver==i?1:0), inverse[i], active[i] );
				}
			} else {
				sprintf( buffer, "type..." );
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->typeMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY)/sc->winw,
					buffer, (sc->typeMenuMouseOver==0?1:0), 0, 1 );
			}

			if( sc->velMenuOpen ) {
				unsigned char menuBuffer[VELMENUITEMS][BUFFER_SIZE];
				char inverse[VELMENUITEMS] = {1,0,0,0,0,0,0,0,0,0,0,0};
				sprintf( menuBuffer[0], "vel" );
                switch( sg->speedUnits ) {
                case SPEED_FPS:
                default:
                    sprintf( menuBuffer[1], "%.0f fps", gi->set->velMenuValue[0] );
                    break;
                case SPEED_KNOTS:
                    sprintf( menuBuffer[1], "%.0f Kts", gi->set->velMenuValue[0] );
                    break;
                case SPEED_KPH:
                    sprintf( menuBuffer[1], "%.0f kph", gi->set->velMenuValue[0] );
                    break;
                case SPEED_MPH:
                    sprintf( menuBuffer[1], "%.0f mph", gi->set->velMenuValue[0] );
                    break;
                case SPEED_MPS:
                    sprintf( menuBuffer[1], "%.0f m/s", gi->set->velMenuValue[0] );
                    break;
                }
				sprintf( menuBuffer[2], "%.0f", gi->set->velMenuValue[1] );
				sprintf( menuBuffer[3], "%.0f", gi->set->velMenuValue[2] );
				sprintf( menuBuffer[4], "%.0f", gi->set->velMenuValue[3] );
				sprintf( menuBuffer[5], "%.0f", gi->set->velMenuValue[4] );
				sprintf( menuBuffer[6], "+%.0f", gi->set->velMenuNudge );
				sprintf( menuBuffer[7], "-%.0f", gi->set->velMenuNudge );

					sprintf( menuBuffer[8], "acc+%.0f", gi->set->velMenuAccNudge );
					sprintf( menuBuffer[9], "acc-%.0f", gi->set->velMenuAccNudge );

				sprintf( menuBuffer[10], "wt+%.0f", gi->set->velMenuWaitNudge );
				sprintf( menuBuffer[11], "wt-%.0f", gi->set->velMenuWaitNudge );
				for( i=0; i<VELMENUITEMS; i++ ) {
					drawMenuItem( sg, sc,
						((float)sc->winw - sg->velMenuX*sg->menuW - sg->menuX )/sc->winw,
						((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
						menuBuffer[i], (sc->velMenuMouseOver==i?1:0), inverse[i], 1 );
				}
			} else {
				sprintf( buffer, "vel..." );
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->velMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY)/sc->winw,
					buffer, (sc->velMenuMouseOver==0?1:0), 0, 1 );
			}

			if( sc->altMenuOpen ) {
				unsigned char menuBuffer[ALTMENUITEMS][BUFFER_SIZE];
				char inverse[ALTMENUITEMS] = {1,0,0,0,0,0,0,0,0,0,0};
				sprintf( menuBuffer[0], "alt" );
				sprintf( menuBuffer[1], "%.0f", gi->set->altMenuValue[0] );
				sprintf( menuBuffer[2], "%.0f", gi->set->altMenuValue[1] );
				sprintf( menuBuffer[3], "%.0f", gi->set->altMenuValue[2] );
				sprintf( menuBuffer[4], "%.0f", gi->set->altMenuValue[3] );
				sprintf( menuBuffer[5], "%.0f", gi->set->altMenuValue[4] );
				sprintf( menuBuffer[6], "%.0f", gi->set->altMenuValue[5] );
				sprintf( menuBuffer[7], "%.0f", gi->set->altMenuValue[6] );
				sprintf( menuBuffer[8], "up %.0f",   gi->set->altMenuNudge );
				sprintf( menuBuffer[9], "down %.0f", gi->set->altMenuNudge );
				sprintf( menuBuffer[10], "at veh" );
				for( i=0; i<ALTMENUITEMS; i++ ) {
					drawMenuItem( sg, sc,
						((float)sc->winw - sg->altMenuX*sg->menuW - sg->menuX )/sc->winw,
						((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
						menuBuffer[i], (sc->altMenuMouseOver==i?1:0), inverse[i], 1 );
				}
			} else {
				sprintf( buffer, "alt..." );
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->altMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY)/sc->winw,
					buffer, (sc->altMenuMouseOver==0?1:0), 0, 1 );
			}

			if( sc->headMenuOpen ) {
				unsigned char menuBuffer[HEADMENUITEMS][BUFFER_SIZE];
				char inverse[HEADMENUITEMS] = {1,0,0,0,0,0,0,0,0,0};
				char active[HEADMENUITEMS]  = {1,1,1,1,1,1,1,1,1,1};
				sprintf( menuBuffer[0], "head" );
				sprintf( menuBuffer[1], "path" );
				if( allFormationWaypointsHeading( sc ) ) {
					sprintf( menuBuffer[2], "For" );
					sprintf( menuBuffer[3], "Aft" );
					sprintf( menuBuffer[4], "Starbd" );
					sprintf( menuBuffer[5], "Port" );
				} else {
					sprintf( menuBuffer[2], "North" );
					sprintf( menuBuffer[3], "South" );
					sprintf( menuBuffer[4], "East" );
					sprintf( menuBuffer[5], "West" );
				}

				sprintf( menuBuffer[6], "%c10%c", SCENEFONT_CW,  SCENEFONT_DEGREES );
				sprintf( menuBuffer[7], "%c10%c", SCENEFONT_CCW, SCENEFONT_DEGREES );
				sprintf( menuBuffer[8], "point" );
				sprintf( menuBuffer[9], "stick" );
				for( i=0; i<HEADMENUITEMS; i++ ) {
					drawMenuItem( sg, sc,
						((float)sc->winw - sg->headMenuX*sg->menuW - sg->menuX )/sc->winw,
						((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
						menuBuffer[i], (sc->headMenuMouseOver==i?1:0), inverse[i], active[i] );
				}
			} else {
				sprintf( buffer, "head..." );
				drawMenuItem( sg, sc,
					((float)sc->winw - sg->headMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY)/sc->winw,
					buffer, (sc->headMenuMouseOver==0?1:0), 0, 1 );
			}

			modeMenuX = sg->modeMenuX;
			customMenuX = sg->customMenuX;
			if( gsc->virtualJoystick == 0 ) customMenuX--;
		} else {
			modeMenuX = sg->modeMenuX - 4; /* shift it over */
			customMenuX = sg->customMenuX - 4; /* shift it over */
			if( gsc->virtualJoystick == 0 ) customMenuX--;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen = 0;
			sc->altMenuOpen = 0;
			sc->headMenuOpen = 0;
		}

		if( gsc->virtualJoystick ) {
			if( sc->modeMenuOpen ) {
				unsigned char menuBuffer[MODEMENUITEMS][BUFFER_SIZE];
				char inverse[MODEMENUITEMS] = {1,0,0,0,0,0,0};
				char active[MODEMENUITEMS]  = {1,1,1,1,1,1,1};
				sprintf( menuBuffer[0], "mode" );
				sprintf( menuBuffer[1], "GPSoff" );
				if( gi->datalink->navStatus == 3 ) inverse[1] = 1;
				sprintf( menuBuffer[2], "GPSon1" );
				if( gi->datalink->navStatus == 1 && gi->panel->box[BOX_AUTO]->state == 6 ) inverse[2] = 1;
				sprintf( menuBuffer[3], "GPSon" );
				if( gi->datalink->navStatus == 1 && gi->panel->box[BOX_AUTO]->state == 2 || gi->panel->box[BOX_AUTO]->state == 3 ||
					gi->panel->box[BOX_AUTO]->state == 4 || gi->panel->box[BOX_AUTO]->state == 5 ) inverse[3] = 1;
				sprintf( menuBuffer[4], "CntrChk" );
				inverse[4] = (char)gi->cntrlInput->manOverride->output;
				active[4] = 0;
				if( gi->cntrlInput->manOverride->output ) {
					active[4] = 1;
				} else if( gi->datalink->m0->motor == 0 ||
					gi->cntrlInput->arm->output ) {
					active[4] = 1;
				}
				sprintf( menuBuffer[5], "Init" );
				if( gi->datalink->m0->navStatus == 0 || gi->datalink->m0->navStatus == 2 ) inverse[5] = 1;
				else                                                                         inverse[5] = 0;
				if( gi->datalink->m0->wow == 1 && gi->datalink->m0->motor == 0 ) active[5] = 1;
				else                                                                active[5] = 0;
				sprintf( menuBuffer[6], "Motor" );
				if( gi->datalink->m0->motor == 0 ||
					( gi->datalink->m0->wow == 0 && gi->cntrlInput->arm->output ) ) inverse[6] = 0;
				else                                                                 inverse[6] = 1;
				if( gsc->virtualJoystick == 0 ) active[6] = 0;

				for( i=0; i<MODEMENUITEMS; i++ ) {
					drawMenuItem( sg, sc,
						((float)sc->winw - modeMenuX*sg->menuW - sg->menuX )/sc->winw,
						((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
						menuBuffer[i], (sc->modeMenuMouseOver==i?1:0), inverse[i], active[i] );
				}
			} else {
				sprintf( buffer, "mode..." );
				drawMenuItem( sg, sc,
					((float)sc->winw - modeMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY)/sc->winw,
					buffer, (sc->modeMenuMouseOver==0?1:0), 0, 1 );
			}
		}

		if( sg->customMenu->on ) {
			if( sc->customMenuOpen ) {
				struct sceneCustomMenu_ref *cm = sg->customMenu;
				unsigned char menuBuffer[CUSTOMMENUITEMS][BUFFER_SIZE];
				char inverse[CUSTOMMENUITEMS] = {1,0,0,0,0,0,0,0,0,0,0};
				char active[CUSTOMMENUITEMS]  = {1,1,1,1,1,1,1,1,1,1,1};
				sprintf( menuBuffer[0], cm->nameOpen );
				for( i=1; i<CUSTOMMENUITEMS; i++ ) {
					sprintf( menuBuffer[i], cm->item[i-1]->name );
					if( strlen( cm->item[i-1]->command ) ) {
						if( gi->datalink->remoteCommandInputFileState == 0 && gi->datalink->remoteCommandInputFileSuccess ) cm->item[i-1]->acting = 0;
						inverse[i] = cm->item[i-1]->acting;
					} else {
						active[i] = 0;
					}
				}
				for( i=0; i<CUSTOMMENUITEMS; i++ ) {
					drawMenuItem( sg, sc,
						((float)sc->winw - customMenuX*sg->menuW - sg->menuX )/sc->winw,
						((float)sc->winh - sg->menuY - heightL*i )/sc->winw,
						menuBuffer[i], (sc->customMenuMouseOver==i?1:0), inverse[i], active[i] );
				}
			} else {
				sprintf( buffer, sg->customMenu->nameClosed );
				drawMenuItem( sg, sc,
					((float)sc->winw - customMenuX*sg->menuW - sg->menuX )/sc->winw,
					((float)sc->winh - sg->menuY)/sc->winw,
					buffer, (sc->customMenuMouseOver==0?1:0), 0, 1 );
			}
		}

		glLineWidth( 1 );

	}

	sc->showingChecklist = 0;
	if( sc->showGCS && sc->showChecklist ) {

		struct sceneChecklist_ref *c = sg->checklist;

		if( c->activeItem < SCENE_CHECKLIST_MAXITEMS ) {
			if( strlen(c->item[c->activeItem]->text) ) { /* an active item */

				int width, height;
				float alpha;
				char buffer[256];

				struct sceneChecklistItem_ref *item = c->item[c->activeItem];

				sprintf( buffer, "%c   %s   (Step %d)",
					(sc->checklistMouseOver?SCENEFONT_CHECKED:SCENEFONT_UNCHECKED), item->text, c->activeItem + 1 );

				/* determine width/height */
				width = getBitmapLength( sg->instFont, buffer );
				height = messageHeight( sg, sg->instFont );

				sc->checklistBox[0][0] = (float)sc->winw/2 - width/2 - sg->instBoxE;
				sc->checklistBox[0][1] = (float)sc->winh*sg->checklistY - sg->instTextY + height/2 + sg->instBoxE;
				sc->checklistBox[1][0] = (float)sc->winw/2 + width/2 + sg->instBoxE;
				sc->checklistBox[1][1] = (float)sc->winh*sg->checklistY - sg->instTextY - height/2 - sg->instBoxE;

				if( sg->instBackColor[3] > 0 ) {
					glColor4f( sg->instBackColor[0], sg->instBackColor[1], sg->instBackColor[2], sg->instBackColor[3] );
					glBegin( GL_POLYGON );
					glVertex3f( sc->checklistBox[0][0]/sc->winw, sc->checklistBox[0][1]/sc->winw, 0 );
					glVertex3f( sc->checklistBox[0][0]/sc->winw, sc->checklistBox[1][1]/sc->winw, 0 );
					glVertex3f( sc->checklistBox[1][0]/sc->winw, sc->checklistBox[1][1]/sc->winw, 0 );
					glVertex3f( sc->checklistBox[1][0]/sc->winw, sc->checklistBox[0][1]/sc->winw, 0 );
					glEnd();
				}

				glColor4f( sg->checklistColor[0], sg->checklistColor[1], sg->checklistColor[2], sg->checklistColor[3] );
				showBitmapMessage( MAX( (float)sc->winw/2 - width/2, 0 )/sc->winw,
					( (float)sc->winh*sg->checklistY - height/2 - height/3 )/sc->winw,
					0, buffer, sg->instFont );
				sc->showingChecklist = 1;

				if( sc->checklistMouseOver ) { /* highlight if mouse over */
					glLineWidth( sg->menuTextLW );
					glColor4fv( sg->menuTextColor );
					glBegin( GL_LINE_LOOP );
					glVertex3f( sc->checklistBox[0][0]/sc->winw, sc->checklistBox[0][1]/sc->winw, 0 );
					glVertex3f( sc->checklistBox[0][0]/sc->winw, sc->checklistBox[1][1]/sc->winw, 0 );
					glVertex3f( sc->checklistBox[1][0]/sc->winw, sc->checklistBox[1][1]/sc->winw, 0 );
					glVertex3f( sc->checklistBox[1][0]/sc->winw, sc->checklistBox[0][1]/sc->winw, 0 );
					glEnd();
					glLineWidth( 1 );
				}

				/* draw fancy stuff */

				for( i=-3; i<=3; i++ ) {
					if( c->activeItem + i >= 0 && c->activeItem + i < SCENE_CHECKLIST_MAXITEMS && i != 0 ) {
						if( strlen( c->item[c->activeItem+i]->text ) ) {
							alpha = 1.0f - (float)(ABS( i ))*0.25f;
							if( i<0 ) {
								glColor4f( sg->instTextColor[0], sg->instTextColor[1], sg->instTextColor[2], sg->instTextColor[3]*alpha );
								sprintf( buffer, "%c %s", SCENEFONT_CHECKED, c->item[c->activeItem+i]->text );
							} else {
								glColor4f( sg->checklistColor[0], sg->checklistColor[1], sg->checklistColor[2], sg->checklistColor[3]*alpha );
								sprintf( buffer, "%c %s", SCENEFONT_UNCHECKED, c->item[c->activeItem+i]->text );
							}
							width = getBitmapLength( sg->instFont, buffer );
							showBitmapMessage( MAX( (float)sc->winw/2 - width/2, 0 )/sc->winw,
								( (float)sc->winh*sg->checklistY - height/2 - height/3 - height*2*i )/sc->winw,
								0, buffer, sg->instFont );
						}
					}
				}

			}
		}
	}

	if( sc->showGCS && sc->showMessages ) {

		int ii;
		char *messageText;
		double messageTime;

		for( ii=0; ii<4; ii++ ) {
			switch( ii ) {
			default:
			case 0:  messageText = sc->messageText0;  messageTime = sc->messageTime0;  break;
			case 1:  messageText = sc->messageText1;  messageTime = sc->messageTime1;  break;
			case 2:  messageText = sc->messageText2;  messageTime = sc->messageTime2;  break;
			case 3:  messageText = sc->messageText3;  messageTime = sc->messageTime3;  break;
			}

			/* draw it */
			if( sim.time < messageTime ) {
				/* determine width/height */
				int width, height;
				float alpha;
				width = getBitmapLength( sg->instFont, messageText );
				height = messageHeight( sg, sg->instFont );

				if( sim.mode == SIM_MODE_INIT ) {
					alpha = 1;
				} else {
					if( sim.time < messageTime - sg->messageFadeTime ) {
						alpha = 1;
					} else {
						alpha = (float)(( messageTime - sim.time )/sg->messageFadeTime);
					}
				}

				if( sg->instBackColor[3] > 0 ) {
					glColor4f( sg->instBackColor[0], sg->instBackColor[1], sg->instBackColor[2], sg->instBackColor[3]*alpha );
					glBegin( GL_POLYGON );
					glVertex3f( ( (float)sc->winw/2 - width/2 - sg->instBoxE )/sc->winw,
						((float)sc->winh*sg->messageY - sg->instTextY + height/2 + sg->instBoxE + ( height + 3*sg->instBoxE )*ii )/sc->winw, 0 );
					glVertex3f( ( (float)sc->winw/2 - width/2 - sg->instBoxE )/sc->winw,
						((float)sc->winh*sg->messageY - sg->instTextY - height/2 - sg->instBoxE + ( height + 3*sg->instBoxE )*ii )/sc->winw, 0 );
					glVertex3f( ( (float)sc->winw/2 + width/2 + sg->instBoxE )/sc->winw,
						((float)sc->winh*sg->messageY - sg->instTextY - height/2 - sg->instBoxE + ( height + 3*sg->instBoxE )*ii )/sc->winw, 0 );
					glVertex3f( ( (float)sc->winw/2 + width/2 + sg->instBoxE )/sc->winw,
						((float)sc->winh*sg->messageY - sg->instTextY + height/2 + sg->instBoxE + ( height + 3*sg->instBoxE )*ii )/sc->winw, 0 );
					glEnd();
				}

				glColor4f( sg->instTextColor[0], sg->instTextColor[1], sg->instTextColor[2], sg->instTextColor[3]*alpha );
				showBitmapMessage( ( (float)sc->winw/2 - width/2 )/sc->winw,
					( (float)sc->winh*sg->messageY - height/2 - height/3 + ( height + 3*sg->instBoxE )*ii )/sc->winw,
					0, messageText, sg->instFont );
			}
		}
	}

	/* add watermark */
	if( sc->showWatermark ) {
		int width;
		glColor4fv( sg->watermarkColor );
		/*showMessage( sg->watermarkPos[0], sg->watermarkPos[1], sg->watermarkText, 1.0 );*/
	    width = getBitmapLength( sg->watermarkFont, sg->watermarkText );
		showBitmapMessage((float)( sc->winw - sg->watermarkPos[0] - width )/sc->winw,
						  (float)( sg->watermarkPos[1] )/sc->winw, 0, sg->watermarkText, sg->watermarkFont );
	}

	/* capture the with-everything scene if desired */

	captureImage( sg, sc, &sceneCapture, SCENECAPTURE_ALL );

	/* frame rate indicator */

	if( sc->showFrameRate && numberSelected( sc, &pickWay ) == 0 ) {
		glColor4fv( sg->blackColor );
		sprintf( buffer, "%d", sim.framesPerSec );

		if( sceneCapture.mode > 0 ) {
			if( ( sc == &scene0 && sceneCapture.window == 0 ) ||
				( sc == &scene1 && sceneCapture.window == 1 ) ||
				( sc == &scene2 && sceneCapture.window == 2 ) ) {
				strcat( buffer, "/REC" );
			}
		}

		showMessage( 0.01f, 0.01f, buffer, 1.0 );
	}

	glEnable( GL_LIGHTING );

}


int redrawScene( void ) {

	struct sceneGlobal_ref *sg = &sceneGlobal;
	struct scene_ref       *sc;
	char finishSwapScene = 0;

	struct gcs_ref              *g   = &gcs;
	struct gcsInstance_ref      *gi  = gcsActiveInstance( g );
	struct gcsScene_ref         *gsc;

	sc = whichScene();
	gsc = whichGcsScene( sc, gi );

	if( sg->forceWidthDivisibleBy4 ) while( sc->winw%4 ) sc->winw--; /* make sure width is divisible by 4 */

	switch( gsc->viewMode ) {

	default:

		/* don't really need to clear the color buffer here, but it's a work-
		around of a bug on home machine (Riva TNT card) */
		if( sc->videoModeDerived == 1 )
			glClearColor( sg->videoSideColor[0], sg->videoSideColor[1], sg->videoSideColor[2], 1 );
		else if( gsc->viewMode == VIEW_NAV )
			glClearColor( sg->navColor[0], sg->navColor[1], sg->navColor[2], 1 );
		else
			glClearColor( 0, 0, 0, 1 );
		glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );

		glViewport( 0, 0, sc->winw, sc->winh );
		redrawSingleScene( sg, sc, sc, 0, 0 );

		if( gsc->showPIP ) {

			int winw, winh, pipOffsetX, pipOffsetY;

			/* get all the window sizes right */
			scenePIP.winh = (int)((float)sc->winh*sg->pipSize);
			scenePIP.winw = (int)((float)sc->winh*sg->pipSize*sg->pipRatio);

			/* deal with animating the switch */
			if( sc->animatePIP > 0 ) {
				if( sim.mode == SIM_MODE_PAUSE ) {
					finishSwapScene = 1;
				} else {
					sc->animatePIP -= sg->animatePIPspeed;
					if( sc->animatePIP < sg->animatePIPspeed ) {
						finishSwapScene = 1;
					}
				}
			}
			if( sc->animatePIP > 0 ) {
				scenePIP.winw = (int)( ( 1.0f - sc->animatePIP )*sc->winw + sc->animatePIP*scenePIP.winw );
				scenePIP.winh = (int)( ( 1.0f - sc->animatePIP )*sc->winh + sc->animatePIP*scenePIP.winh );
				pipOffsetX = (int)( sc->animatePIP*sg->pipOffsetX );
				pipOffsetY = (int)( sc->animatePIP*sg->pipOffsetY );
			} else {
				pipOffsetX = sg->pipOffsetX;
				pipOffsetY = sg->pipOffsetY;
			}
			winw = scenePIP.winw;
			winh = scenePIP.winh;

			scenePIP.hudh = (int)(scenePIP.winh*sg->hfov/scenePIP.fovy);
			scenePIP.hudw = scenePIP.hudh;
			if( scenePIP.hudw > scenePIP.winw ) scenePIP.hudw = scenePIP.winw;

			glClear( GL_DEPTH_BUFFER_BIT );

			/* draw border box */
			glDisable( GL_LIGHTING );
		    glShadeModel( GL_SMOOTH );
			glPushMatrix();
			glScalef( (float)(1.0/sc->winw), (float)(1.0/sc->winw), 1 );
			if( sg->instBackColor[3] > 0 ) {
				glBegin( GL_QUAD_STRIP );
				glColor4f( 0, 0, 0, 0 );
				glVertex3i( sc->winw - winw - pipOffsetX - sg->pipE, pipOffsetY + winh + sg->pipE, 0 );
				glColor4fv( sg->instBackColor );
				glVertex3i( sc->winw - winw - pipOffsetX           , pipOffsetY + winh           , 0 );
				glColor4f( 0, 0, 0, 0 );
				glVertex3i( sc->winw        - pipOffsetX + sg->pipE, pipOffsetY + winh + sg->pipE, 0 );
				glColor4fv( sg->instBackColor );
				glVertex3i( sc->winw        - pipOffsetX           , pipOffsetY + winh           , 0 );
				glColor4f( 0, 0, 0, 0 );
				glVertex3i( sc->winw        - pipOffsetX + sg->pipE, pipOffsetY        - sg->pipE, 0 );
				glColor4fv( sg->instBackColor );
				glVertex3i( sc->winw        - pipOffsetX           , pipOffsetY                  , 0 );
				glColor4f( 0, 0, 0, 0 );
				glVertex3i( sc->winw - winw - pipOffsetX - sg->pipE, pipOffsetY        - sg->pipE, 0 );
				glColor4fv( sg->instBackColor );
				glVertex3i( sc->winw - winw - pipOffsetX           , pipOffsetY                  , 0 );
				glColor4f( 0, 0, 0, 0 );
				glVertex3i( sc->winw - winw - pipOffsetX - sg->pipE, pipOffsetY + winh + sg->pipE, 0 );
				glColor4fv( sg->instBackColor );
				glVertex3i( sc->winw - winw - pipOffsetX           , pipOffsetY + winh           , 0 );
				glEnd();
			}
			if( sc->mouseOverPIP && sc->animatePIP == 0 ) { /* highlight PIP if mouse over it */
				glLineWidth( sg->menuTextLW );
				glColor4fv( sg->menuTextColor );
				glBegin( GL_LINE_LOOP );
				glVertex3i( sc->winw - winw - pipOffsetX - (int)(sg->menuTextLW/2), pipOffsetY + winh + (int)(sg->menuTextLW/2), 0 );
				glVertex3i( sc->winw        - pipOffsetX + (int)(sg->menuTextLW/2), pipOffsetY + winh + (int)(sg->menuTextLW/2), 0 );
				glVertex3i( sc->winw        - pipOffsetX + (int)(sg->menuTextLW/2), pipOffsetY        - (int)(sg->menuTextLW/2), 0 );
				glVertex3i( sc->winw - winw - pipOffsetX - (int)(sg->menuTextLW/2), pipOffsetY        - (int)(sg->menuTextLW/2), 0 );
				glEnd();
				glLineWidth( 1 );
			}

			/* don't really need to clear the color buffer here, but it's a work-
			around of a bug on home machine (Riva TNT card) */
			if( gi->set->scenePIP->viewMode != VIEW_NAV || sg->seeThroughMap == 0 || gsc->viewMode != VIEW_CAMERA && gsc->viewMode != VIEW_CAMERA2 && gsc->viewMode != VIEW_CAMERA3 && gsc->viewMode != VIEW_CAMERA4 ) {
				if( gi->set->scenePIP->viewMode == VIEW_NAV )
					glColor4fv( sg->navColor );
				else
					glColor4f( 0, 0, 0, 1 );
				glBegin( GL_POLYGON );
				glVertex3i( sc->winw - winw - pipOffsetX, pipOffsetY + winh, 0 );
				glVertex3i( sc->winw - winw - pipOffsetX, pipOffsetY       , 0 );
				glVertex3i( sc->winw        - pipOffsetX, pipOffsetY       , 0 );
				glVertex3i( sc->winw        - pipOffsetX, pipOffsetY + winh, 0 );
				glEnd();
			}
			glPopMatrix();
			glEnable( GL_LIGHTING );
		    glShadeModel( GL_FLAT );

			glViewport( sc->winw - winw - pipOffsetX, pipOffsetY, winw, winh );
            glScalef( 1, (float)(winw)/winh/sc->winw*sc->winh, 1 );
			redrawSingleScene( sg, &scenePIP, sc, sc->winw - winw - pipOffsetX, pipOffsetY );

			/* put viewport back to draw the rest */
			glViewport( 0, 0, sc->winw, sc->winh );

			/* change size of PIP */
			if( sc->mouseOverPIP && sc->animatePIP == 0 && sg->pipResizeBox < winh ) { /* highlight PIP if mouse over it */
				glPushMatrix();
	            glScalef( 1, (float)(winh)/winw/sc->winh*sc->winw, 1 );
				glScalef( (float)(1.0/sc->winw), (float)(1.0/sc->winw), 1 );
				glLineWidth( sg->menuTextLW );
				glColor4fv( sg->menuTextColor );
				glBegin( GL_LINE_STRIP );
				glVertex3i( sc->winw - winw - pipOffsetX + sg->pipResizeBox       , pipOffsetY + winh + (int)(sg->menuTextLW/2), 0 );
				glVertex3i( sc->winw - winw - pipOffsetX + sg->pipResizeBox       , pipOffsetY + winh - sg->pipResizeBox       , 0 );
				glVertex3i( sc->winw - winw - pipOffsetX - (int)(sg->menuTextLW/2), pipOffsetY + winh - sg->pipResizeBox       , 0 );
				glEnd();
				glLineWidth( 1 );
				glPopMatrix();
			}
		}

		redrawSingleScene2( sg, sc );

	}

	/*if(sg->camgrab->run && sc->videoModeDerived == 1 && sg->camgrab->drawtex == 3) {
		camgrabSwapBuffers(sg->camgrab,sc);
	} else {*/
	glFinish();
	glutSwapBuffers();
	/*}*/

	if( finishSwapScene ) {
		finishSwapScenePIP( sc );
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
	}

	return 0;

}


void redrawSceneCallback( void ) {

	redrawScene();

}


void sceneReshape( int w, int h ) {

	struct scene_ref *sc;
	struct sceneGlobal_ref *sg = &sceneGlobal;
	sc = whichScene();

	sc->winw = w;
	sc->winh = h;

	sc->hudh = (int)(sc->winh*sg->hfov/sc->fovy);
	sc->hudw = sc->hudh;
	if( sc->hudw > sc->winw ) sc->hudw = sc->winw;

	redrawScene();

}


void sceneMouseButton( int button, int state, int x, int y ) {

	struct gcs_ref *g = &gcs;
	struct gcsInstance_ref *gi = gcsActiveInstance( g );
	struct gcsScene_ref *gsc;
	struct scene_ref *sc;
	struct sceneGlobal_ref *sg = &sceneGlobal;
	struct imViewer_ref *view = &imViewer;
	struct pfOpInterface_ref *pfOI = &pfOpInterface;
	struct maneuver_ref *m;
	int oldorient, oldmove, oldmovePIP, oldyzoom;
	int pickWay, holdMouseOver;
	int i;

	int enoughTime;
	static double lastTime;
	unsigned char buffer[BUFFER_SIZE];
	char gn;

	sc = whichScene();
	gsc = whichGcsScene( sc, gi );

	sc->modifiers = glutGetModifiers();
	gn = LIMIT( g->active, 0, GCS_MAX_INSTANCES-1 );

	if( sim.time < lastTime + sg->minClickTime && sim.time > lastTime ) {
		enoughTime = 0;
	} else {
		if( state == GLUT_UP )
			lastTime = sim.time;
		enoughTime = 1;
	}

	//printf( "button %d, state %d, x %d, y %d\n", button, state, x, y );

	oldorient = sg->orient;
	oldmove   = sg->move;
	oldmovePIP = sg->movePIP;
	oldyzoom  = sc->yzoom;
	sc->yzoom = -1;
	sg->orient = 0;
	sg->move   = 0;
	sg->movePIP = 0;

	/* menu selections */
	if( sc->viewMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->viewMenuMouseOver == 0 && sc->viewMenuOpen == 0 ) {
			sc->viewMenuOpen = 1;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen  = 0;
			sc->altMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->visMenuOpen  = 0;
			sc->modeMenuOpen = 0;
			sc->customMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->viewMenuMouseOver == 0 ) {
				sc->viewMenuOpen = 0;
			} else if( state == GLUT_UP && sc->viewMenuOpen && sc->viewMenuMouseOver > 0 && enoughTime ) {
				switch( sc->viewMenuMouseOver ) {
				default: break;
				case 2:
					gsc->viewMode = VIEW_NAV;
					sceneAddMessage( sg, sc, "Map View Selected" );
					break;
				case 3:
					gsc->viewMode = VIEW_CHASE;
					sceneAddMessage( sg, sc, "Chase View Selected" );
					break;
				case 4:
					gsc->viewMode = VIEW_COCKPIT;
					sceneAddMessage( sg, sc, "PFD View Selected" );
					break;
				}
				sc->viewMenuOpen = 0;
				sc->viewMenuMouseOver = 0;
			} else if( state == GLUT_UP && sc->viewMenuOpen == 0 &&
				sc->viewMenuMouseOver == 2 && gsc->viewMode == VIEW_NAV && enoughTime ) {
				struct view_ref *v;
				struct vehicleOutputs_ref *o;
				double dx, dy;
				v = whichView( gi->set, gsc->viewMode );
				switch( sc->lookat ) {
				default:
				case LOOKAT_TRUTH:
				case LOOKAT_BOTH:
					o = &vehicleOutputs;
					break;
				case LOOKAT_GCS:
					o = gi->outputs;
					break;
				}
				if( sg->autoscaleNav ) {
					sg->autoscaleNav = 0;
				} else {
					v->lookat = !v->lookat;
				}
				/* this makes it so center does not change when changing mode */
				if( v->lookat == VIEW_LOOKAT_VEHICLE ) {
					if( numberSelected( sc, &pickWay ) == 0 ) {
						dx = ( sc->eyeLat - o->latitude  )*C_NM2FT*60;
						dy = hmodDeg( sc->eyeLon - o->longitude )*C_NM2FT*60*sc->cosDatumLat;
						v->seat[0] = (float)( dx*cos( v->neckPsi ) + dy*sin( v->neckPsi ) );
						v->seat[1] = (float)( dy*cos( v->neckPsi ) - dx*sin( v->neckPsi ) );
						v->seat[2] = (float)( o->altitudeMSL - sc->eyeAlt );

						if( sg->resetViewWhenSwitchToFollow ) {
							if( v->seat[0] > +v->zoom                   ) v->seat[0] = +v->zoom*0.67f;
							if( v->seat[0] < -v->zoom                   ) v->seat[0] = -v->zoom*0.67f;
							if( v->seat[1] > +v->zoom*sc->winw/sc->winh ) v->seat[1] = +v->zoom*sc->winw/sc->winh*0.67f;
							if( v->seat[1] < -v->zoom*sc->winw/sc->winh ) v->seat[1] = -v->zoom*sc->winw/sc->winh*0.67f;
						}
					} else {
						v->seat[0] = 0;
						v->seat[1] = 0;
						v->seat[2] = 0;
					}
					sceneAddMessage( sg, sc, "View Follows Aircraft" );
				} else { /* looking at nothing */
					dx = ( sc->eyeLat - o->datumLat )*C_NM2FT*60;
					dy = hmodDeg( sc->eyeLon - o->datumLon )*C_NM2FT*60*sc->cosDatumLat;
					if( numberSelected( sc, &pickWay ) == 0 ) { /* complexity so ownship doesn't move */
						dx -= v->seat[0]*cos( v->neckPsi ) - v->seat[1]*sin( v->neckPsi );
						dy -= v->seat[1]*cos( v->neckPsi ) + v->seat[0]*sin( v->neckPsi );
						determineMapUp( gi->set, sg, gsc, sc, o );
						dx += v->seat[0]*cos( v->neckPsi ) - v->seat[1]*sin( v->neckPsi );
						dy += v->seat[1]*cos( v->neckPsi ) + v->seat[0]*sin( v->neckPsi );
					}
					v->seat[0] = (float)( dx );
					v->seat[1] = (float)( dy );
					v->seat[2] = (float)( o->datumAlt - sc->eyeAlt );
					sceneAddMessage( sg, sc, "View Following Stopped" );
				}
			} else if( state == GLUT_UP && sc->viewMenuOpen == 0 &&
				sc->viewMenuMouseOver == 1 && gsc->viewMode == VIEW_NAV && enoughTime ) {
				gsc->show3Dmap = !gsc->show3Dmap;
			} else if( state == GLUT_UP && sc->viewMenuOpen == 0 &&
				sc->viewMenuMouseOver == 3 && gsc->viewMode == VIEW_NAV && enoughTime &&
				( gi->set->nav->lookat == VIEW_LOOKAT_VEHICLE || sg->autoscaleNav ) ) {
				if( gsc->mapUpMode == MAPUP_CONST ) {
					gsc->mapUpMode = MAPUP_HEADING;
					sceneAddMessage( sg, sc, "Map Heading Up" );
				} else {
					gsc->mapUpMode = MAPUP_CONST;
					sceneAddMessage( sg, sc, "Map Orientation Fixed" );
				}
			} else if( state == GLUT_UP && sc->viewMenuOpen == 0 &&
				sc->viewMenuMouseOver == 1 && ( gsc->viewMode == VIEW_CAMERA || gsc->viewMode == VIEW_CAMERA2 || gsc->viewMode == VIEW_CAMERA3 || gsc->viewMode == VIEW_CAMERA4 ) && enoughTime ) {
					gi->cntrlInput->videoToggle->output = -1;
				sceneAddMessage( sg, sc, "Video Source Swapped" );
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->visMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->visMenuMouseOver == 0 && sc->visMenuOpen == 0 ) {
			sc->visMenuOpen  = 1;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen  = 0;
			sc->altMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->viewMenuOpen = 0;
			sc->modeMenuOpen = 0;
			sc->customMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->visMenuMouseOver == 0 ) {
				sc->visMenuOpen = 0;
			} else if( state == GLUT_UP && sc->visMenuOpen && sc->visMenuMouseOver > 0 && enoughTime ) {
				struct view_ref *v;
				v = whichView( gi->set, gsc->viewMode );
				switch( sc->visMenuMouseOver ) {
				default: break;
				case 1:
					sg->ntpoints     = 0;
					sg->ntpoints_nav[gn] = 0;
					sg->ntpoints_other[gn] = 0;
                    sg->ntpoints_gps[gn] = 0;
					gi->timer->prevTime = 0;
					gi->timer->mAh = 0;
					sceneAddMessage( sg, sc, "Trace Cleared and Flight Timer Reset" );
					break;
				case 2:
					gsc->showPIP = !gsc->showPIP;
					if( gsc->showPIP ) sceneAddMessage( sg, sc, "Picture In Picture On" );
					else               sceneAddMessage( sg, sc, "Picture In Picture Off" );
					break;
				case 3:  gsc->virtualJoystick = !gsc->virtualJoystick;
					if( gsc->virtualJoystick == 0 ) {
						gi->cntrlInput->leftVirtualJoystickInUse  = 0;
						gi->cntrlInput->rightVirtualJoystickInUse = 0;
					}
					break;
				case 4:  v->hudOn = !v->hudOn;
					if( v->hudOn ) sceneAddMessage( sg, sc, "HUD On" );
					else           sceneAddMessage( sg, sc, "HUD Off" );
					break;
				case 5:  sc->showTraj = !sc->showTraj;
					if( sc->showTraj ) sceneAddMessage( sg, sc, "Showing Trace" );
					else               sceneAddMessage( sg, sc, "Hiding Trace" );
					break;
				case 6:
					if( gi->set->showPlan ) {
						if( 1 == gsc->showPlan ) {
							gsc->showPlan = 2;
							sceneAddMessage( sg, sc, "Hiding Plan" );
						} else {
							gsc->showPlan = 1;
							sceneAddMessage( sg, sc, "Showing Plan" );
						}
					} else {
						gi->set->showPlan = 1;
						gsc->showPlan = 1;
						sceneAddMessage( sg, sc, "Showing Plan" );
					}
					break;
				case 7:  sc->showGrid = !sc->showGrid;
					if( sc->showGrid ) sceneAddMessage( sg, sc, "Showing Grid" );
					else               sceneAddMessage( sg, sc, "Hiding Grid" );
					break;
				case 8:  gsc->showCameraFOV = !gsc->showCameraFOV;
					if( gsc->showCameraFOV ) sceneAddMessage( sg, sc, "Showing Camera Field Of View" );
					else                     sceneAddMessage( sg, sc, "Hiding Camera Field Of View" );
					break;
				case 9:  gsc->showTex = !gsc->showTex;
					if( gsc->showTex ) sceneAddMessage( sg, sc, "Showing Map Overlays" );
					else               sceneAddMessage( sg, sc, "Hiding Map Overlays" );
					break;
				case 10:  gsc->videoMode = !gsc->videoMode;
					if( gsc->videoMode ) sceneAddMessage( sg, sc, "Showing Video Feed (if in Camera View)" );
					else                 sceneAddMessage( sg, sc, "Hiding Video Feed (current view)" );
					break;
				case 11:
					if( sc->showingChecklist ) {
						sceneChecklistClear( sg );
						sceneAddMessage( sg, sc, "Checklist Closed" );
					} else {
						commandExecute( "pickFile @%s checklist_*.inp" );
						sceneAddMessage( sg, sc, "Opening Checklist" );
					}
					break;
				}
				sc->visMenuOpen = 0;
				sc->visMenuMouseOver = 0;
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->wayMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->wayMenuMouseOver == 0 && sc->wayMenuOpen == 0 ) {
			sc->wayMenuOpen  = 1;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen  = 0;
			sc->altMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->visMenuOpen  = 0;
			sc->viewMenuOpen = 0;
			sc->modeMenuOpen = 0;
			sc->customMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->wayMenuMouseOver == 0 ) {
				sc->wayMenuOpen = 0;
			} else if( state == GLUT_UP && sc->wayMenuOpen && sc->wayMenuMouseOver > 0 && enoughTime  ) {
				switch( sc->wayMenuMouseOver ) {
				default: break;
				case 1:
					commandExecute( "trajEdit" );
					switch( gi->traj->edit ) {
					case 0:
						sceneAddMessage( sg, sc, "Editing Flight Plan A" );
						break;
					case 1:
						sceneAddMessage( sg, sc, "Editing Flight Plan B" );
						break;
					case 2:
						sceneAddMessage( sg, sc, "Editing Lost Com Flight Plan" );
						break;
					default:
						break;
					}
					break;
				case 2:
					if( sc->waySelectMode ) sceneKeyboard( '_', 0, 0 );
					else                    sceneKeyboard( '-', 0, 0 );
					break;
				case 3:
					if( sc->waySelectMode ) sceneKeyboard( '+', 0, 0 );
					else                    sceneKeyboard( '=', 0, 0 );
					break;
				case 4:
					sc->waySelectMode = !sc->waySelectMode;
					if( sc->waySelectMode ) sc->wayInsertMode = 0;
					/* double click for all waypoints */
					if( sim.time - sg->lastClickSelectTime < sg->doubleClickTime && sim.time > sg->lastClickSelectTime ) {
						sc->waySelectMode = 0;
					}
					sg->lastClickSelectTime = sim.time;
					break;
				case 5:
					switch( sc->wayInsertMode ) {
					case 0:
						sc->wayInsertMode = 1;
						break;
					case 1:
						sc->wayInsertMode = 2;
						break;
					default:
						sc->wayInsertMode = 0;
					}
					if( sc->wayInsertMode ) sc->waySelectMode = 0;
					/*sceneSpecialKeys( GLUT_KEY_INSERT, 0, 0 );*/
					break;
				case 6:
					sceneKeyboard( 127 /* delete */, 0, 0 );
					break;
				case 7: /* move waypoint up */
					if (!sc->waySelected[0] && numberSelected(sc, &pickWay) == 1) {
						struct maneuver_ref temp_maneuver;
						memcpy(&temp_maneuver, &maneuver[pickWay], sizeof(struct maneuver_ref));
						memcpy(&maneuver[pickWay], &maneuver[pickWay - 1], sizeof(struct maneuver_ref));
						memcpy(&maneuver[pickWay - 1], &temp_maneuver, sizeof(struct maneuver_ref));
						selectNone(sc);
						sc->waySelected[pickWay - 1] = 1;
						gi->traj->uploadEach[gi->traj->edit] = 1;
					}
					break;
				case 10:
					commandExecute( "pickFile @%s plan_*.inp" );
					//commandExecute( "@tempplan" );
					sceneAddMessage( sg, sc, "Opening Flight Plan" );
					break;
				case 11:
					commandExecute( "pickFile trajSave %s plan_*.inp" );
					sceneAddMessage( sg, sc, "Saving Flight Plan" );
					break;
				case 12:
					commandExecute( "trajDownload" );
					sceneAddMessage( sg, sc, "Flight Plan Requested" );
					break;
				case 13:
					commandExecute( "trajUpload" );
					sc->headMenuOpen = 0;
					sc->typeMenuOpen = 0;
					sc->velMenuOpen  = 0;
					sc->altMenuOpen  = 0;
					switch( gi->traj->edit ) {
					case 0:
						sceneAddMessage( sg, sc, "Flight Plan A Sent" );
						break;
					case 1:
						sceneAddMessage( sg, sc, "Flight Plan B Sent" );
						break;
					case 2:
						sceneAddMessage( sg, sc, "Lost Com Flight Plan Sent" );
						break;
					default:
						break;
					}
					break;
				case 14:
					/* trajStop should always work */

					if( !gi->traj->uploadEach[gi->traj->edit] ) {
						if( gi->traj->traj->safemode != 0 && gi->traj->traj->safemode != 2 ) {
							sceneAddMessage( sg, sc, "Go Command Sent" );
							commandExecute( "trajGo" ); /* I resisted having this for 5 years... */
						}
						sc->headMenuOpen = 0;
						sc->typeMenuOpen = 0;
						sc->velMenuOpen  = 0;
						sc->altMenuOpen  = 0;
					}
					break;
				case 15:
					if( !gi->traj->uploadEach[gi->traj->edit] &&
						numberSelected( sc, &pickWay ) == 1 &&
						pickWay != MAN_NMANS ) {
						if( numberSelected( sc, &pickWay ) == 1 ) {
							if( pickWay != MAN_NMANS ) {
								sceneAddMessage( sg, sc, "Go Command Sent" );
								sprintf( buffer, "trajGo %d", pickWay );
								commandExecute( buffer );
							}
						} else {
							sceneAddMessage( sg, sc, "INFO: select single waypoint first" );
						}
						sc->headMenuOpen = 0;
						sc->typeMenuOpen = 0;
						sc->velMenuOpen  = 0;
						sc->altMenuOpen  = 0;
					}
					break;
				}
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->typeMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->typeMenuMouseOver == 0 && sc->typeMenuOpen == 0 ) {
			sc->typeMenuOpen = 1;
			sc->velMenuOpen  = 0;
			sc->altMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->visMenuOpen  = 0;
			sc->viewMenuOpen = 0;
			sc->modeMenuOpen = 0;
			sc->customMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->typeMenuMouseOver == 0 ) {
				sc->typeMenuOpen = 0;
			} else if( state == GLUT_UP && sc->typeMenuOpen && sc->typeMenuMouseOver > 0 && enoughTime  ) {
				if( numberSelected( sc, &pickWay ) ) {
					holdMouseOver = sc->typeMenuMouseOver;
					sc->typeMenuOpen = 0;
					sc->typeMenuMouseOver = 0;
					gi->traj->uploadEach[gi->traj->edit] = 1;
				}
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->velMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->velMenuMouseOver == 0 && sc->velMenuOpen == 0 ) {
			sc->velMenuOpen  = 1;
			sc->typeMenuOpen = 0;
			sc->altMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->visMenuOpen  = 0;
			sc->viewMenuOpen = 0;
			sc->modeMenuOpen = 0;
			sc->customMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->velMenuMouseOver == 0 ) {
				sc->velMenuOpen = 0;
			} else if( state == GLUT_UP && sc->velMenuOpen && sc->velMenuMouseOver > 0 && enoughTime ) {
				if( numberSelected( sc, &pickWay ) ) {
					holdMouseOver = sc->velMenuMouseOver;
					gi->traj->uploadEach[gi->traj->edit] = 1;
				}
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->altMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->altMenuMouseOver == 0 && sc->altMenuOpen == 0 ) {
			sc->altMenuOpen  = 1;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->visMenuOpen  = 0;
			sc->viewMenuOpen = 0;
			sc->modeMenuOpen = 0;
			sc->customMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->altMenuMouseOver == 0 ) {
				sc->altMenuOpen = 0;
			} else if( state == GLUT_UP && sc->altMenuOpen && sc->altMenuMouseOver > 0 && enoughTime ) {
				if( numberSelected( sc, &pickWay ) ) {
					double move;
					double mx[3];
					m = &maneuver[pickWay];
					waypointLocate( gi, pickWay, mx );
					move = gi->datalink->m0->pos[2] - mx[2];
					holdMouseOver = sc->altMenuMouseOver;
					gi->traj->uploadEach[gi->traj->edit] = 1;
				}
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->headMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->headMenuMouseOver == 0 && sc->headMenuOpen == 0 ) {
			sc->headMenuOpen = 1;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen  = 0;
			sc->altMenuOpen  = 0;
			sc->visMenuOpen  = 0;
			sc->viewMenuOpen = 0;
			sc->modeMenuOpen = 0;
			sc->customMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->headMenuMouseOver == 0 ) {
				sc->headMenuOpen = 0;
			} else if( state == GLUT_UP && sc->headMenuOpen && sc->headMenuMouseOver > 0 && enoughTime ) {
				if( numberSelected( sc, &pickWay ) ) {
					holdMouseOver = sc->headMenuMouseOver;
					gi->traj->uploadEach[gi->traj->edit] = 1;
				}
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->modeMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->modeMenuMouseOver == 0 && sc->modeMenuOpen == 0 ) {
			sc->modeMenuOpen = 1;
			sc->visMenuOpen  = 0;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen  = 0;
			sc->altMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->viewMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->modeMenuMouseOver == 0 ) {
				sc->modeMenuOpen = 0;
			} else if( state == GLUT_UP && sc->modeMenuOpen && sc->modeMenuMouseOver > 0 && enoughTime ) {
				switch( sc->modeMenuMouseOver ) {
				default: break;
				case 1:
					commandExecute( "gpsDenied on"    );
					sceneAddMessage( sg, sc, "GPS Denied Mode Activated" );
					break;
				case 2:
					commandExecute( "gpsDenied gpson" );
					sceneAddMessage( sg, sc, "GPS Enabled, Still Flying Without GPS" );
					break;
				case 3:
					commandExecute( "gpsDenied off"   );
					sceneAddMessage( sg, sc, "GPS Denied Mode Off" );
					break;
				case 4:
					if( gi->cntrlInput->manOverride->output ) {
						gi->cntrlInput->manOverride->output = 0;
						sceneAddMessage( sg, sc, "Controls Check Mode Off" );
					} else if( gi->datalink->m0->motor == 0 ||
						gi->cntrlInput->arm->output ) { // Arm
						gi->cntrlInput->manOverride->output = 1; // Manual override
						sceneAddMessage( sg, sc, "Controls Check Mode On" );
					}
					break;
				case 5:
					if( gi->datalink->m0->wow == 1 && gi->datalink->m0->motor == 0 ) {
						commandExecute( "navInit" );
						sceneAddMessage( sg, sc, "Initialization Requested" );
					}
					break;
				case 6:
					if( gsc->virtualJoystick ) {
						if( gi->datalink->m0->motor == 0 ) {
							sceneAddMessage( sg, sc, "Motor Start Requested" );
							commandExecute( "motorStart" );
						} else {
							sceneAddMessage( sg, sc, "Motor Shutdown Arm Requested" );
							commandExecute( "motorStop" );
						}
					}
					break;
				}
				/*sc->visModeOpen = 0;*/
				sc->modeMenuMouseOver = 0;
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	if( sc->customMenuMouseOver >= 0 && button == GLUT_LEFT_BUTTON && !sc->modifiers ) {
		if( state == GLUT_DOWN && sc->customMenuMouseOver == 0 && sc->customMenuOpen == 0 ) {
			sc->customMenuOpen = 1;
			sc->visMenuOpen  = 0;
			sc->typeMenuOpen = 0;
			sc->velMenuOpen  = 0;
			sc->altMenuOpen  = 0;
			sc->headMenuOpen = 0;
			sc->viewMenuOpen = 0;
			sc->modeMenuOpen = 0;
		} else {
			if( state == GLUT_DOWN && sc->customMenuMouseOver == 0 ) {
				sc->customMenuOpen = 0;
			} else if( state == GLUT_UP && sc->customMenuOpen && sc->customMenuMouseOver > 0 && sc->customMenuMouseOver < CUSTOMMENUITEMS && enoughTime ) {
				if( strlen( sg->customMenu->item[sc->customMenuMouseOver - 1]->command ) ) {
					if( strncmp( sg->customMenu->item[sc->customMenuMouseOver - 1]->command, "rc@", 3 ) == 0 ) { /* this is an rc@ command */
						sg->customMenu->item[sc->customMenuMouseOver - 1]->acting = 1;
					} else {
						sg->customMenu->item[sc->customMenuMouseOver - 1]->acting = 0;
					}
					commandExecute( sg->customMenu->item[sc->customMenuMouseOver - 1]->command );
				}
				sc->customMenuMouseOver = 0;
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	/* clicking on checklist */
	if( sc->checklistMouseOver && button == GLUT_LEFT_BUTTON && state == GLUT_UP ) {
		if( enoughTime ) {
			if( sc->modifiers == GLUT_ACTIVE_SHIFT ) {
				sg->checklist->activeItem = MAX( 0, sg->checklist->activeItem - 1 );
			} else if( !sc->modifiers ) {
				char buffer[BUFFER_SIZE];
				sg->checklist->activeItem = LIMIT( sg->checklist->activeItem, 0, SCENE_CHECKLIST_MAXITEMS - 1 );
				sprintf( buffer, "checked '%s'", sg->checklist->item[sg->checklist->activeItem]->text );
				gcsAddLine( gi, buffer );
				if( strlen( sg->checklist->item[sg->checklist->activeItem]->command ) ) commandExecute( sg->checklist->item[sg->checklist->activeItem]->command );
				sg->checklist->activeItem++;
			}
		}
		if( sim.mode == SIM_MODE_PAUSE )
			glutPostWindowRedisplay( sc->win );
		return;
	}

	/* clicking on a GCS text box */
	for( i=0; i<GCS_MAX_INSTANCES; i++ ) {
		if( sc->textMouseOver[i] && button == GLUT_LEFT_BUTTON && state == GLUT_UP ) {
			if( enoughTime ) {
				int igcycle, ig;
				for( igcycle=0; igcycle<GCS_MAX_INSTANCES; igcycle++ ) {
					ig = LIMIT( sg->gcsOrder[igcycle], 0, GCS_MAX_INSTANCES-1 );
					if( gcsGetInstance( g, ig )->run ) {
						if( ig == i ) {
							char buffer[50];
							sprintf( buffer, "gcsSelect %d", i );
							commandExecute( buffer );
						}
					}
				}
			}
			if( sim.mode == SIM_MODE_PAUSE )
				glutPostWindowRedisplay( sc->win );
			return;
		}
	}

	/* mouse zooming */
	if( button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN && !sc->modifiers ) {
		sc->yzoom = y;
	}

	/* press left button */
	if( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN ) {

	    /* for touch screen, need to pick mouseOverWay */
	    redrawScene();

		if( sc->joyLeftMouseOver ) {
			sg->joyLeft = 1;
			sceneMouseMotion( x, y );
		} else {
			if( sc->joyRightMouseOver ) {
				sg->joyRight = 1;
				sceneMouseMotion( x, y );
			} else {
				if( gsc->showPIP && sc->mouseOverPIP ) { /* click in PIP */
					sg->movePIP = 1;
					sg->xorient = x;
					sg->yorient = y;
				} else {
					if( sc->mouseOverWay == -1 && !sc->modifiers ) {
						if( x < sc->winw - sg->zoomWide ) {
							sg->xorient = x;
							sg->yorient = y;
							sg->orient = 1;
						} else {
							sc->yzoom = y;
						}
					} else {
						sg->move = 1;
						sg->xorient = x;
						sg->yorient = y;
						sg->holdMouseOverWay = sc->mouseOverWay;
						if( !sc->modifiers && !sc->waySelectMode ) {
							if( sc->mouseOverWay > -1 ) if( sc->waySelected[sc->mouseOverWay] == 0 ) if( sg->allowWaySelectingWithNoShiftMouse ) { /* select point if it's a new one */
								selectNone( sc );
								sc->waySelected[sc->mouseOverWay] = 1;
								sc->noInsert = 1;
							}
						}
					}
				}
			}
		}
	}

	/* picking things */
	if( button == GLUT_LEFT_BUTTON && state == GLUT_UP && ( sg->joyLeft || sg->joyRight ) ) {
		struct motionControls_ref *co = &motionControls;
		sg->joyLeft  = 0;
		sg->joyRight = 0;
		sc->joyLeftMouseOver  = 0;
		sc->joyRightMouseOver = 0;
		if( gsc->virtualJoystick /*&& co->mode == MOTIONINPUT_MOUSE*/ ) {
			gi->cntrlInput->throttle->output = 0; // throttle
			gi->cntrlInput->rudder->output = 0; // rudder
			gi->cntrlInput->pitch->output = 0; // pitch
			gi->cntrlInput->roll->output = 0; // roll
			gi->cntrlInput->leftVirtualJoystickInUse  = 0;
			gi->cntrlInput->rightVirtualJoystickInUse = 0;
		}
	} else if( button == GLUT_LEFT_BUTTON && state == GLUT_UP && enoughTime ) {
		if( sc->modifiers == (GLUT_ACTIVE_SHIFT+GLUT_ACTIVE_CTRL) ) {
			commandExecute( "ipLookat" );
		} else if ( ( sc->modifiers & GLUT_ACTIVE_SHIFT ||
			( sc->waySelectMode && !( sc->modifiers & GLUT_ACTIVE_CTRL ) ) ) &&
			( sg->holdMouseOverWay > -1 && oldmove < 2 ) ) {
				sc->waySelected[sg->holdMouseOverWay] = !sc->waySelected[sg->holdMouseOverWay];
				sg->holdMouseOverWay = -1;
		} else if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			sc->pick = 3; /* pan/tilt/roll position command */
		} else {
			if( oldorient < 2 && oldmove < 2 && oldyzoom == -1 && oldmovePIP < 2 ) {
				if( gsc->showPIP && sc->mouseOverPIP ) { /* click in PIP */
					swapScenePIP( sg, sc );
				} else {
					if( sc->wayInsertMode && sc->noInsert == 0 ) {
						sc->pick = 4;
						if( sc->wayInsertMode == 1 ) sc->wayInsertMode = 0;
					}
					else {
						/* in case we were just changing view orientation, not picking */
						if( oldorient == 1 && !sc->waySelectMode ) {
							if( numberSelected( sc, &pickWay ) == 0 ) sc->pick = 1;
							else                                      selectNone( sc );
						} else {
							sc->pick = 1; /* pick off and display lat/long */
						}
					}
				}
			}
		}
		sc->noInsert = 0;
	}

    if( sim.mode == SIM_MODE_PAUSE )
        glutPostWindowRedisplay( sc->win );

}


void sceneMouseMotion( int x, int y ) {

	struct gcs_ref *g = &gcs;
	struct gcsInstance_ref *gi = gcsActiveInstance( g );
	struct gcsScene_ref *gsc;
    struct motionControls_ref *co = &motionControls;
    struct scene_ref *sc;
    struct sceneGlobal_ref *sg = &sceneGlobal;
	struct vehicleSet_ref  *set = &vehicleSet;
	int menux, menuy;
	int i;
	int pickWay, menuMouseOver, textMouseOver;

    sc = whichScene();
	gsc = whichGcsScene( sc, gi );

	sc->mx = x;
	sc->my = y;

	/* menu highlighting */

	menux = sc->winw - x;  /* menu comes from right side */
	menuy = y;

	sc->viewMenuMouseOver = -1;
	sc->visMenuMouseOver  = -1;
	sc->wayMenuMouseOver  = -1;
	sc->typeMenuMouseOver = -1;
	sc->velMenuMouseOver  = -1;
	sc->altMenuMouseOver  = -1;
	sc->headMenuMouseOver = -1;
	sc->modeMenuMouseOver = -1;
	sc->customMenuMouseOver = -1;

	sc->checklistMouseOver = 0;
	for( i=0; i<GCS_MAX_INSTANCES; i++ ) sc->textMouseOver[i] = 0;

	/* checklist selection */

	if( sc->showingChecklist ) {
		if( sc->yzoom == -1 && sg->orient < 2 && sg->move == 0 && sg->movePIP == 0 && sg->joyLeft == 0 && sg->joyRight == 0 ) {
			if( sc->mx >= sc->checklistBox[0][0] ) {
				if( sc->mx <= sc->checklistBox[1][0] ) {
					if( sc->my >= sc->winh - sc->checklistBox[0][1] ) {
						if( sc->my <= sc->winh - sc->checklistBox[1][1] ) {
							sc->checklistMouseOver = 1;
						}
					}
				}
			}
		}
	}

	/* menu selections */
	if( sc->yzoom == -1 && sg->orient < 2 && sg->move == 0 && sg->movePIP == 0 && sg->joyLeft == 0 && sg->joyRight == 0 && gsc->showMenu && sc->checklistMouseOver == 0 ) {

		int height, heightL;
		float modeMenuX;
		float customMenuX;

		height = messageHeight( sg, sg->menuFont );
		heightL = height + (int)sg->menuH;

		/* view menu */
		if( menuy < sg->menuY + heightL*VIEWMENUITEMS ) {
			if( menux < sg->viewMenuX*sg->menuW + sg->menuX  )
				if( menuy > sg->menuY )
					if( menux > sg->viewMenuX*sg->menuW + sg->menuX - sg->menuW )
						sc->viewMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
		}

		/* vis menu */
		if( menuy < sg->menuY + heightL*VISMENUITEMS ) {
			if( menux < sg->visMenuX*sg->menuW + sg->menuX )
				if( menuy > sg->menuY )
					if( menux > sg->visMenuX*sg->menuW + sg->menuX - sg->menuW )
						sc->visMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
		}
		if( sc->visMenuOpen == 0 && sc->visMenuMouseOver > 0 )
			sc->visMenuMouseOver = -1;

		/* way menu */
		if( menuy < sg->menuY + heightL*WAYMENUITEMS ) {
			if( menux < sg->wayMenuX*sg->menuW + sg->menuX )
				if( menuy > sg->menuY )
					if( menux > sg->wayMenuX*sg->menuW + sg->menuX - sg->menuW )
						sc->wayMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
		}
		if( sc->wayMenuOpen == 0 && sc->wayMenuMouseOver > 0 )
			sc->wayMenuMouseOver = -1;

		if( numberSelected( sc, &pickWay ) > 0 ) {

			/* way type menu */
			if( menuy < sg->menuY + heightL*TYPEMENUITEMS ) {
				if( menux < sg->typeMenuX*sg->menuW + sg->menuX )
					if( menuy > sg->menuY )
						if( menux > sg->typeMenuX*sg->menuW + sg->menuX - sg->menuW )
							sc->typeMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
			}
			if( sc->typeMenuOpen == 0 && sc->typeMenuMouseOver > 0 )
				sc->typeMenuMouseOver = -1;

			/* way vel menu */
			if( menuy < sg->menuY + heightL*VELMENUITEMS ) {
				if( menux < sg->velMenuX*sg->menuW + sg->menuX )
					if( menuy > sg->menuY )
						if( menux > sg->velMenuX*sg->menuW + sg->menuX - sg->menuW )
							sc->velMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
			}
			if( sc->velMenuOpen == 0 && sc->velMenuMouseOver > 0 )
				sc->velMenuMouseOver = -1;

			/* way alt menu */
			if( menuy < sg->menuY + heightL*ALTMENUITEMS ) {
				if( menux < sg->altMenuX*sg->menuW + sg->menuX )
					if( menuy > sg->menuY )
						if( menux > sg->altMenuX*sg->menuW + sg->menuX - sg->menuW )
							sc->altMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
			}
			if( sc->altMenuOpen == 0 && sc->altMenuMouseOver > 0 )
				sc->altMenuMouseOver = -1;

			/* way head menu */
			if( menuy < sg->menuY + heightL*HEADMENUITEMS ) {
				if( menux < sg->headMenuX*sg->menuW + sg->menuX )
					if( menuy > sg->menuY )
						if( menux > sg->headMenuX*sg->menuW + sg->menuX - sg->menuW )
							sc->headMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
			}
			if( sc->headMenuOpen == 0 && sc->headMenuMouseOver > 0 )
				sc->headMenuMouseOver = -1;

			modeMenuX = sg->modeMenuX;
			customMenuX = sg->customMenuX;
			if( gsc->virtualJoystick == 0 ) customMenuX--;
		} else {
			modeMenuX = sg->modeMenuX - 4;
			customMenuX = sg->customMenuX - 4;
			if( gsc->virtualJoystick == 0 ) customMenuX--;
		}

		/* mode menu */
		if( gsc->virtualJoystick ) {
			if( menuy < sg->menuY + heightL*MODEMENUITEMS ) {
				if( menux < modeMenuX*sg->menuW + sg->menuX )
					if( menuy > sg->menuY )
						if( menux > modeMenuX*sg->menuW + sg->menuX - sg->menuW )
							sc->modeMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
			}
			if( sc->modeMenuOpen == 0 && sc->modeMenuMouseOver > 0 )
				sc->modeMenuMouseOver = -1;
		}

		/* custom menu */
		if( sg->customMenu->on ) {
			if( menuy < sg->menuY + heightL*CUSTOMMENUITEMS ) {
				if( menux < customMenuX*sg->menuW + sg->menuX )
					if( menuy > sg->menuY )
						if( menux > customMenuX*sg->menuW + sg->menuX - sg->menuW )
							sc->customMenuMouseOver = (char)((float)( menuy - sg->menuY )/heightL);
			}
			if( sc->customMenuOpen == 0 && sc->customMenuMouseOver > 0 )
				sc->customMenuMouseOver = -1;
		}
	}
	menuMouseOver = 1;
	if( sc->viewMenuMouseOver == -1 && sc->wayMenuMouseOver  == -1 && sc->visMenuMouseOver == -1 &&
		sc->altMenuMouseOver  == -1 && sc->typeMenuMouseOver == -1 && sc->velMenuMouseOver == -1 &&
		sc->headMenuMouseOver == -1 && sc->modeMenuMouseOver == -1 && sc->customMenuMouseOver == -1 ) { /* not on a menu item */
		menuMouseOver = 0;
	}

	textMouseOver = 0;
	if( gsc->showGCStext && sg->enableGCSSelect ) {
		if( sc->yzoom == -1 && sg->orient < 2 && sg->move == 0 && sg->movePIP == 0 && sg->joyLeft == 0 && sg->joyRight == 0 && menuMouseOver == 0 && sc->checklistMouseOver == 0 ) {
			for( i=0; i<GCS_MAX_INSTANCES; i++ ) {
				if( sc->mx >= sc->textBox[i][0][0] ) {
					if( sc->mx <= sc->textBox[i][1][0] ) {
						if( sc->my >= sc->winh - sc->textBox[i][0][1] ) {
							if( sc->my <= sc->winh - sc->textBox[i][1][1] ) {
								sc->textMouseOver[i] = 1;
								textMouseOver = 1;
							}
						}
					}
				}
			}
		}
	}

	/* mouse zoom */
	if( sc->yzoom != -1 ) {
		struct view_ref  *v;
		float dcm[4][4], gain, dx[3], dist;

		v = whichView( gi->set, gsc->viewMode );
		switch( gsc->viewMode ) {
		default:
			break;
		case VIEW_COCKPIT:
		case VIEW_NAV:
		case VIEW_GROUND:
			v->zoom *= (float)pow( 1.1, (float)(y - sc->yzoom)*0.1 );
			if( gsc->viewMode != VIEW_NAV ) v->zoom = (float)LIMIT( v->zoom, pow( 1.1, sg->zoomMin ), pow( 1.1, sg->zoomMax ) );
			else                            v->zoom = (float)LIMIT( v->zoom, gi->set->waypointR*0.5, sg->zoomMaxMap );
			break;
		case VIEW_CHASE:
			euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
			dist = (float)sqrt( SQ( v->seat[0] ) + SQ( v->seat[1] ) + SQ( v->seat[2] ) );
			gain = dist*5*sc->fovy*v->zoom/sc->winh*CF_DEG2RAD;
			dx[0] = -dcm[0][0]*v->viewStep*(float)(y - sc->yzoom)*gain;
			dx[1] = -dcm[1][0]*v->viewStep*(float)(y - sc->yzoom)*gain;
			dx[2] = -dcm[2][0]*v->viewStep*(float)(y - sc->yzoom)*gain;
			if( ( dx[0]*v->seat[0] + dx[1]*v->seat[1] + dx[2]*v->seat[2] )*(float)( y - sc->yzoom ) > 0 ) {
				v->seat[0] += dx[0];
				v->seat[1] += dx[1];
				v->seat[2] += dx[2];
				dist = (float)sqrt( SQ( v->seat[0] ) + SQ( v->seat[1] ) + SQ( v->seat[2] ) );
				if( dist < gi->set->waypointR ) {
					float scale;
					scale = gi->set->waypointR/dist;
					v->seat[0] *= scale;
					v->seat[1] *= scale;
					v->seat[2] *= scale;
				}
			}
			break;
		}
		sc->yzoom = y;
	}

	/* virtual joysticks */
	sc->joyLeftMouseOver  = 0;
	sc->joyRightMouseOver = 0;
	if( gsc->virtualJoystick ) {
		if( sg->joyLeft ) {
			float joy[2];
			sc->joyLeftMouseOver = 1;
			gi->cntrlInput->leftVirtualJoystickInUse = 1;
			joy[0] = +( (float)             x  /sc->winw - ( sg->joyX + sg->joySize/2 ) )/( sg->joySize/2 );
			joy[1] = +( (float)( sc->winh - y )/sc->winw - ( sg->joyY + sg->joySize/2 ) )/( sg->joySize/2 );
			joy[0] = LIMIT( joy[0], -1, 1 );
			joy[1] = LIMIT( joy[1], -1, 1 );
			gi->cntrlInput->rudder->output = joy[0]; // rudder
			gi->cntrlInput->rudder->output = MAX( ABS( gi->cntrlInput->rudder->output  ) - gi->cntrlInput->rudder->deadBand, 0.0 )*
				SIGN( gi->cntrlInput->rudder->output  )/( 1.0 - gi->cntrlInput->rudder->deadBand );
			gi->cntrlInput->throttle->output = joy[1];
			gi->cntrlInput->throttle->output = MAX( ABS( gi->cntrlInput->throttle->output ) - gi->cntrlInput->throttle->deadBand, 0.0 )*
				SIGN( gi->cntrlInput->throttle->output )/( 1.0 - gi->cntrlInput->throttle->deadBand );
			// co->analogFunction[3] = 0.5 + 0.5*co->analogFunction[3];
		} else {
			if( sg->movePIP ==0 && sg->orient == 0 && sg->move == 0 && sc->yzoom == -1 && sc->mouseOverWay == -1 && menuMouseOver == 0 && sc->checklistMouseOver == 0 && textMouseOver == 0 ) { /* not already doing something */
				if( x < ( sg->joyX + sg->joySize )*sc->winw ) {
					if( sc->winh - y < ( sg->joyY + sg->joySize )*sc->winw ) {
						if( x > sg->joyX*sc->winw ) {
							if( sc->winh - y > sg->joyY*sc->winw ) {
								sc->joyLeftMouseOver = 1;
							}
						}
					}
				}
			}
		}
		if( sg->joyRight ) {
			float joy[2];
			sc->joyRightMouseOver = 1;
			gi->cntrlInput->rightVirtualJoystickInUse = 1;
			joy[0] = -( (float)( sc->winw - x )/sc->winw - ( sg->joyX + sg->joySize/2 ) )/( sg->joySize/2 );
			joy[1] = +( (float)( sc->winh - y )/sc->winw - ( sg->joyY + sg->joySize/2 ) )/( sg->joySize/2 );
			joy[0] = LIMIT( joy[0], -1, 1 );
			joy[1] = LIMIT( joy[1], -1, 1 );
			gi->cntrlInput->roll->output = joy[0]; // roll
			gi->cntrlInput->roll->output  = MAX( ABS( gi->cntrlInput->roll->output  ) - gi->cntrlInput->roll->deadBand, 0.0 )*
				SIGN( gi->cntrlInput->roll->output  )/( 1.0 - gi->cntrlInput->roll->deadBand );
			gi->cntrlInput->pitch->output = -joy[1]; // pitch
			gi->cntrlInput->pitch->output = MAX( ABS( gi->cntrlInput->pitch->output ) - gi->cntrlInput->pitch->deadBand, 0.0 )*
		        SIGN( gi->cntrlInput->pitch->output )/( 1.0 - gi->cntrlInput->pitch->deadBand );
		} else {
			if( sg->movePIP ==0 && sg->orient == 0 && sg->move == 0 && sc->yzoom == -1 && sc->mouseOverWay == -1 && menuMouseOver == 0 && sc->checklistMouseOver == 0 && textMouseOver == 0 ) { /* not already doing something */
				if( sc->winw - x < ( sg->joyX + sg->joySize )*sc->winw ) {
					if( sc->winh - y < ( sg->joyY + sg->joySize )*sc->winw ) {
						if( sc->winw - x > sg->joyX*sc->winw ) {
							if( sc->winh - y > sg->joyY*sc->winw ) {
								sc->joyRightMouseOver = 1;
							}
						}
					}
				}
			}
		}
	}

	/* PIP box */
	if( sg->movePIP ) { /* move it around */
		if( sg->movePIP == 1 ) {
			if( ( ABS( y - sg->yorient ) > sg->deadbandMouse ) ||
				( ABS( x - sg->xorient ) > sg->deadbandMouse ) ) {
				sg->movePIP = 2; /* if not in a corner, move it */
				if(     sg->xorient < sc->winw - scenePIP.winw - sg->pipOffsetX + sg->pipResizeBox ) {
					if( sg->yorient < sc->winh - scenePIP.winh - sg->pipOffsetY + sg->pipResizeBox ) sg->movePIP = 3;
				}
			}
		} else {
			switch( sg->movePIP ) {
			case 2: /* move */
				sg->pipOffsetX = LIMIT( sg->pipOffsetX - ( x - sg->xorient ),
					10 + sg->zoomWide - (int)(sg->pipSize*sc->winh*sg->pipRatio), sc->winw - 10 );
				sg->pipOffsetY = LIMIT( sg->pipOffsetY - ( y - sg->yorient ),
					10                - (int)(sg->pipSize*sc->winh)             , sc->winh - 10 );
				break;
			case 3: /* upper left corner */
				sg->pipSize = LIMIT( (float)( sg->pipSize*sc->winw +
					(float)( - x + sg->xorient + ( - y + sg->yorient )*sg->pipRatio )/2 ), 10, sc->winw - 20 )/sc->winw;
				break;
			default:
				break;
			}
			sg->xorient = x;
			sg->yorient = y;
		}
	} else { /* figure out if mouse is over it */
		sc->mouseOverPIP = 0;
		if( gsc->showPIP ) {
			if( sg->orient == 0 && sg->move == 0 && sc->yzoom == -1 && sc->mouseOverWay == -1 && sc->joyLeftMouseOver == 0 && sc->joyRightMouseOver == 0 && menuMouseOver == 0 && sc->checklistMouseOver == 0 && textMouseOver == 0 ) { /* not already doing something */
				if( x > sc->winw - scenePIP.winw - sg->pipOffsetX ) {
					if( y > sc->winh - scenePIP.winh - sg->pipOffsetY ) {
						if( x < sc->winw - sg->zoomWide ) {
							if( x < sc->winw - sg->pipOffsetX ) {
								if( y < sc->winh - sg->pipOffsetY ) {
									sc->mouseOverPIP = 1;
								}
							}
						}
					}
				}
			}
		}
	}

	/* mouse moving/orientation */
    if( sg->orient ) {
		if( sg->orient == 1 ) {
			if(      ABS( y - sg->yorient ) > sg->deadbandMouse )
				sg->orient = 2;
			else if( ABS( x - sg->xorient ) > sg->deadbandMouse )
				sg->orient = 2;
		} else {
			struct view_ref  *v;
			float dcm[4][4], gain;

			v = whichView( gi->set, gsc->viewMode );
			switch( gsc->viewMode ) {
			default:
				break;
			case VIEW_COCKPIT:
				/* this is a PFD, leave it alone */
				break;
			case VIEW_GROUND:
				v->neckPsi   -= (float)( x - sg->xorient )*sc->fovy*v->zoom/sc->winh*CF_DEG2RAD;
				v->neckTheta += (float)( y - sg->yorient )*sc->fovy*v->zoom/sc->winh*CF_DEG2RAD;
				break;
			case VIEW_NAV:
				if( v->lookat == VIEW_LOOKAT_NOTHING ) {
					v->seat[0] += (float)(sin( v->neckPsi )*(float)( x - sg->xorient )*2*v->zoom/sc->winh);
					v->seat[1] -= (float)(cos( v->neckPsi )*(float)( x - sg->xorient )*2*v->zoom/sc->winh);
					v->seat[0] += (float)(cos( v->neckPsi )*(float)( y - sg->yorient )*2*v->zoom/sc->winh/cos( gsc->angle3D*C_DEG2RAD*sc->show3D ));
					v->seat[1] += (float)(sin( v->neckPsi )*(float)( y - sg->yorient )*2*v->zoom/sc->winh/cos( gsc->angle3D*C_DEG2RAD*sc->show3D ));
				} else {
					v->seat[1] -= (float)( (float)( x - sg->xorient )*2*v->zoom/sc->winh);
					v->seat[0] += (float)( (float)( y - sg->yorient )*2*v->zoom/sc->winh/cos( gsc->angle3D*C_DEG2RAD*sc->show3D ));
				}
				break;
			case VIEW_CHASE:
				euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
				gain = (float)(sqrt( SQ( v->seat[0] ) + SQ( v->seat[1] ) + SQ( v->seat[2] ) )
					*2*sc->fovy*v->zoom/sc->winh*C_DEG2RAD);
				v->seat[0] -= dcm[0][1]*v->viewStep*(float)( x - sg->xorient )*gain;
				v->seat[1] -= dcm[1][1]*v->viewStep*(float)( x - sg->xorient )*gain;
				v->seat[2] -= dcm[2][1]*v->viewStep*(float)( x - sg->xorient )*gain;
				v->seat[0] -= dcm[0][2]*v->viewStep*(float)( y - sg->yorient )*gain;
				v->seat[1] -= dcm[1][2]*v->viewStep*(float)( y - sg->yorient )*gain;
				v->seat[2] -= dcm[2][2]*v->viewStep*(float)( y - sg->yorient )*gain;
				/* should really add something here that concerves the norm of v->seat */
				break;
			}
			sg->xorient = x;
			sg->yorient = y;
		}
	}

	/* mouse waypoint moving, dragging */
    if( sg->move ) {
		if( sg->move == 1 ) {
			if(      ABS( y - sg->yorient ) > sg->deadbandMouse )
				sg->move = 2;
			else if( ABS( x - sg->xorient ) > sg->deadbandMouse )
				sg->move = 2;
		} else {
			struct view_ref  *v;
			struct vehicleOutputs_ref *o;
			int moved = 0;
			float dcm[4][4]; //, dcm2[4][4], dcm3[4][4];
			double dist, move[2] = {0,0};
			double mx[3];

			v = whichView( gi->set, gsc->viewMode );
			switch( sc->lookat ) {
			default:
			case LOOKAT_TRUTH:
			case LOOKAT_BOTH:
				o = &vehicleOutputs;
				break;
			case LOOKAT_GCS:
				o = gi->outputs;
				break;
			}

			switch( gsc->viewMode ) {
			default:
				break;
			case VIEW_NAV:
				move[0] -= sin( v->neckPsi )*(float)( x - sg->xorient )*2*v->zoom/sc->winh;
				move[1] += cos( v->neckPsi )*(float)( x - sg->xorient )*2*v->zoom/sc->winh;
				move[0] -= cos( v->neckPsi )*(float)( y - sg->yorient )*2*v->zoom/sc->winh/cos( gsc->angle3D*C_DEG2RAD*sc->show3D );
				move[1] -= sin( v->neckPsi )*(float)( y - sg->yorient )*2*v->zoom/sc->winh/cos( gsc->angle3D*C_DEG2RAD*sc->show3D );
				/* change view location a bit so mouse movement makes sense in nav mode when following the waypoint */
				if( v->followWaypoints ) {
					switch( v->lookat ) {
					case VIEW_LOOKAT_VEHICLE:
						v->seat[0] += (float)((float)( y - sg->yorient )*2*v->zoom/sc->winh/cos( gsc->angle3D*C_DEG2RAD*sc->show3D ));
						v->seat[1] -= (float)((float)( x - sg->xorient )*2*v->zoom/sc->winh);
						break;
					}
				}
				moved = 1;
				break;
			case VIEW_CHASE:
			case VIEW_GROUND:
			case VIEW_COCKPIT:
			case VIEW_CAMERA:
			case VIEW_CAMERA2:
			case VIEW_CAMERA3:
			case VIEW_CAMERA4:
				euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
				/*euler2dcmf( 0, - ( (float)sc->my/sc->winh - 0.5f )*sc->fovy*v->zoom*CF_DEG2RAD, + ( (float)sc->mx/sc->winw - 0.5f )*sc->fovy*v->zoom*(GLfloat)sc->winw/sc->winh*CF_DEG2RAD, dcm2 );*/
				/*euler2dcmf( 0, sc->vec[1], sc->vec[0], dcm2 );*/
				/*matf_mult( (float *)dcm,  4, 4,
					       (float *)dcm2, 4, 4,
						   (float *)dcm3 );*/
				/* could really reorient this dcm to direction clicked rather than center of screen */
				/*m = &maneuver[sg->holdMouseOverWay];*/
				waypointLocate( gi, sg->holdMouseOverWay, mx );
				dist = sqrt( SQ( (double)(mx[0]) + ( o->datumLat - sc->eyeLat )*C_NM2FT*60.0 ) +
					SQ( (double)(mx[1]) + hmodDeg( o->datumLon - sc->eyeLon )*C_NM2FT*60.0*sc->cosDatumLat ) +
					SQ( (double)(mx[2]) - o->datumAlt + sc->eyeAlt ) );
				move[0] += dcm[0][1]*dist*(float)( x - sg->xorient )*sc->fovy*v->zoom/sc->winh*C_DEG2RAD;
				move[1] += dcm[1][1]*dist*(float)( x - sg->xorient )*sc->fovy*v->zoom/sc->winh*C_DEG2RAD;
				move[0] += dcm[0][2]*dist*(float)( y - sg->yorient )*sc->fovy*v->zoom/sc->winh*C_DEG2RAD/MAX( 0.1, SQ( dcm[2][0] ) );
				move[1] += dcm[1][2]*dist*(float)( y - sg->yorient )*sc->fovy*v->zoom/sc->winh*C_DEG2RAD/MAX( 0.1, SQ( dcm[2][0] ) );
				moved = 1;
				break;
			}

			if( moved ) {
				gi->traj->uploadEach[gi->traj->edit] = 1;
			}
			sg->xorient = x;
			sg->yorient = y;
		}
	}


#if 0
	/* mouse control */
	/* this is the old way, replaced with virtual joystick */

    if( co->mode == MOTIONINPUT_MOUSE ) {

		if( sg->yawMode ) {
			co->r_rollStick = 0.0;
			co->r_rudderPedal = -1.0*( sc->winw - 2.0*x )/sc->winw;
		} else {
			co->r_rollStick = -1.0*( sc->winw - 2.0*x )/sc->winw;
			co->r_rudderPedal = 0.0;
		}

		co->r_rollStick  = MAX( ABS( co->r_rollStick  ) - co->deadBand[0], 0.0 )*
			SIGN( co->r_rollStick  )/( 1.0 - co->deadBand[0] );

		co->r_pitchStick = -1.0*( sc->winh - 2.0*y )/sc->winh;

		co->r_pitchStick = MAX( ABS( co->r_pitchStick ) - co->deadBand[1], 0.0 )*
		        SIGN( co->r_pitchStick )/( 1.0 - co->deadBand[1] );

		co->r_pitchStick  = LIMIT( co->r_pitchStick,  -1.0, 1.0 );
        co->r_rollStick   = LIMIT( co->r_rollStick,   -1.0, 1.0 );
        co->r_rudderPedal = LIMIT( co->r_rudderPedal, -1.0, 1.0 );

	}
#endif

    if( sim.mode == SIM_MODE_PAUSE )
        glutPostWindowRedisplay( sc->win );

}


void sceneMenuState( int status ) {

	if( status == GLUT_MENU_IN_USE )
	{
		sim.skipTime = 1;
	}

}


void sceneKeyboard( unsigned char key, int x, int y ) {

	/*struct vehicleOutputs_ref *o = &vehicleOutputs;*/
	struct sceneGlobal_ref *sg = &sceneGlobal;
	struct scene_ref  *sc;
	struct view_ref   *v;
	struct motionControls_ref *co = &motionControls;
	struct gcs_ref *g = &gcs;
	struct gcsInstance_ref *gi = gcsActiveInstance( g );
	struct gcsScene_ref *gsc;
	struct maneuver_ref *m;

	float dcm[4][4];
	double move[3];

	int i, j;
	int pickWay;
	char gn;

	//printf( "key = %d\n", key );

	sc = whichScene();
	sc->modifiers = glutGetModifiers();
	gn = LIMIT( g->active, 0, GCS_MAX_INSTANCES-1 );
	gsc = whichGcsScene( sc, gi );
	v = whichView( gi->set, gsc->viewMode );

	switch( key ) {
	case 'a':
		gi->cntrlInput->throttle->output -= 0.1; // throttle
		break;
	case 's':
		gi->cntrlInput->throttle->output += 0.1; // throttle
		break;
	case 'w':
			if( numberSelected( sc, &pickWay ) > 0 ) {
				gi->traj->uploadEach[gi->traj->edit] = 1;
			}
		break;

	case 'i':
		if( gsc->showPIP && ( gsc->viewMode == VIEW_CAMERA || gsc->viewMode == VIEW_CAMERA2 || gsc->viewMode == VIEW_CAMERA3 || gsc->viewMode == VIEW_CAMERA4 ) && gi->set->scenePIP->viewMode == VIEW_NAV ) {
			gi->set->nav->zoom /= 1.1f;
			sg->autoscaleNav = 0;
			if( gi->set->nav->lookat == VIEW_LOOKAT_VEHICLE ) {
				gi->set->nav->seat[0] = 0;
				gi->set->nav->seat[1] = 0;
				gi->set->nav->seat[2] = 0;
			}
		} else {
			v->zoom /= 1.1f;
			if( gsc->viewMode != VIEW_NAV ) v->zoom = (float)LIMIT( v->zoom, pow( 1.1, sg->zoomMin ), pow( 1.1, sg->zoomMax ) );
			if( gsc->viewMode == VIEW_NAV ) {
				sg->autoscaleNav = 0;
				if( v->lookat == VIEW_LOOKAT_VEHICLE ) {
					v->seat[0] = 0;
					v->seat[1] = 0;
					v->seat[2] = 0;
				}
			}
		}
		break;
	case 'o':
		if( gsc->showPIP && ( gsc->viewMode == VIEW_CAMERA || gsc->viewMode == VIEW_CAMERA2 || gsc->viewMode == VIEW_CAMERA3 || gsc->viewMode == VIEW_CAMERA4 ) && gi->set->scenePIP->viewMode == VIEW_NAV ) {
			gi->set->nav->zoom *= 1.1f;
			sg->autoscaleNav = 0;
			if( gi->set->nav->lookat == VIEW_LOOKAT_VEHICLE ) {
				gi->set->nav->seat[0] = 0;
				gi->set->nav->seat[1] = 0;
				gi->set->nav->seat[2] = 0;
			}
		} else {
			v->zoom *= 1.1f;
			if( gsc->viewMode != VIEW_NAV ) v->zoom = (float)LIMIT( v->zoom, pow( 1.1, sg->zoomMin ), pow( 1.1, sg->zoomMax ) );
			if( gsc->viewMode == VIEW_NAV ) {
				sg->autoscaleNav = 0;
				if( v->lookat == VIEW_LOOKAT_VEHICLE ) {
					v->seat[0] = 0;
					v->seat[1] = 0;
					v->seat[2] = 0;
				}
			}
		}
		break;
    case 'O':
    case 'I':
        if( gsc->viewMode == VIEW_NAV ) sg->autoscaleNav = 1;
		else if( gsc->showPIP && ( gsc->viewMode == VIEW_CAMERA || gsc->viewMode == VIEW_CAMERA2 || gsc->viewMode == VIEW_CAMERA3 || gsc->viewMode == VIEW_CAMERA4 ) && gi->set->scenePIP->viewMode == VIEW_NAV ) sg->autoscaleNav = 1;
		break;
	case '[':
		v->viewStep /= 1.414f;
		break;
	case ']':
		v->viewStep *= 1.414f;
		break;
	case 'f':
		switch( gsc->viewMode ) {
		default:
		case VIEW_COCKPIT:
		case VIEW_CAMERA:
        case VIEW_CAMERA2:
        case VIEW_CAMERA3:
        case VIEW_CAMERA4:
			v->seat[0] += v->viewStep;
			break;
		case VIEW_NAV:
			v->zoom /= 1.1f;
			break;
		case VIEW_CHASE:
		case VIEW_GROUND:
		case VIEW_HOVER:
			euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
			v->seat[0] += dcm[0][0]*v->viewStep;
			v->seat[1] += dcm[1][0]*v->viewStep;
			v->seat[2] += dcm[2][0]*v->viewStep;
			break;
		}
		break;
	case 'b':
		switch( gsc->viewMode ) {
		default:
		case VIEW_COCKPIT:
		case VIEW_CAMERA:
        case VIEW_CAMERA2:
        case VIEW_CAMERA3:
        case VIEW_CAMERA4:
			v->seat[0] -= v->viewStep;
			break;
		case VIEW_NAV:
			v->zoom *= 1.1f;
			break;
		case VIEW_CHASE:
		case VIEW_GROUND:
		case VIEW_HOVER:
			euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
			v->seat[0] -= dcm[0][0]*v->viewStep;
			v->seat[1] -= dcm[1][0]*v->viewStep;
			v->seat[2] -= dcm[2][0]*v->viewStep;
			break;
		}
		break;
	case 'r':
		switch( gsc->viewMode ) {
		case VIEW_NAV:
			break;
		default:
		case VIEW_COCKPIT:
		case VIEW_CAMERA:
		case VIEW_CAMERA2:
		case VIEW_CAMERA3:
		case VIEW_CAMERA4:
			v->seat[1] += v->viewStep;
			break;
		case VIEW_CHASE:
		case VIEW_GROUND:
		case VIEW_HOVER:
			euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
			v->seat[0] += dcm[0][1]*v->viewStep;
			v->seat[1] += dcm[1][1]*v->viewStep;
			v->seat[2] += dcm[2][1]*v->viewStep;
			break;
		}
		break;
	case 'l':
		switch( gsc->viewMode ) {
		case VIEW_NAV:
			break;
		default:
		case VIEW_COCKPIT:
		case VIEW_CAMERA:
        case VIEW_CAMERA2:
        case VIEW_CAMERA3:
        case VIEW_CAMERA4:
			v->seat[1] -= v->viewStep;
			break;
		case VIEW_CHASE:
		case VIEW_GROUND:
		case VIEW_HOVER:
			euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
			v->seat[0] -= dcm[0][1]*v->viewStep;
			v->seat[1] -= dcm[1][1]*v->viewStep;
			v->seat[2] -= dcm[2][1]*v->viewStep;
			break;
		}
		break;
	case 'd':
		switch( gsc->viewMode ) {
		case VIEW_NAV:
			if( sc->show3D ) v->seat[2] += v->viewStep;
			break;
		default:
		case VIEW_COCKPIT:
		case VIEW_CAMERA:
        case VIEW_CAMERA2:
        case VIEW_CAMERA3:
        case VIEW_CAMERA4:
			v->seat[2] += v->viewStep;
			break;
		case VIEW_CHASE:
		case VIEW_GROUND:
		case VIEW_HOVER:
			euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
			v->seat[0] += dcm[0][2]*v->viewStep;
			v->seat[1] += dcm[1][2]*v->viewStep;
			v->seat[2] += dcm[2][2]*v->viewStep;
			break;
		}
		break;
	case 'u':
		switch( gsc->viewMode ) {
		case VIEW_NAV:
			if( sc->show3D ) v->seat[2] -= v->viewStep;
			break;
		default:
		case VIEW_COCKPIT:
		case VIEW_CAMERA:
        case VIEW_CAMERA2:
        case VIEW_CAMERA3:
        case VIEW_CAMERA4:
			v->seat[2] -= v->viewStep;
			break;
		case VIEW_CHASE:
		case VIEW_GROUND:
		case VIEW_HOVER:
			euler2dcmf( sc->eyePhi, sc->eyeTheta, sc->eyePsi, dcm );
			v->seat[0] -= dcm[0][2]*v->viewStep;
			v->seat[1] -= dcm[1][2]*v->viewStep;
			v->seat[2] -= dcm[2][2]*v->viewStep;
			break;
		}
		break;
	case 'X':
		sg->ntpoints     = 0;
		for( i=0; i<GCS_MAX_INSTANCES; i++ ) {
			sg->ntpoints_nav[i] = 0;
			sg->ntpoints_other[i] = 0;
			sg->ntpoints_gps[i] = 0;
		}
		sceneAddMessage( sg, sc, "Trace Cleared" );
		break;
	case 'h':
		if( 0 == v->hudOn ) {
			v->hudOn = 1;  sc->hudAlternate = 0;  sceneAddMessage( sg, sc, "HUD Normal" );
		} else if( 0 == sc->hudAlternate ) {
			sc->hudAlternate = 1;  sceneAddMessage( sg, sc, "HUD Minimal" );
		} else {
			v->hudOn = 0;  sceneAddMessage( sg, sc, "HUD Off" );
		}
		break;
	case 't':
		sc->showTraj = !sc->showTraj;
		if( sc->showTraj ) sceneAddMessage( sg, sc, "Showing Trace" );
		else               sceneAddMessage( sg, sc, "Hiding Trace" );
		break;

	/* waypoint editing */

	case '=':
		if( numberSelected( sc, &pickWay ) == 0 )
			sc->waySelected[0] = 1;
		else {
		}
		break;
	case '-':
		if( numberSelected( sc, &pickWay ) == 0 )
			sc->waySelected[0] = 1;
		else {
			selectNone( sc );
				pickWay = MAX( pickWay - 1, 0 );
			sc->waySelected[pickWay] = 1;
		}
		break;
	case '+':
		if( numberSelected( sc, &pickWay ) > 0 ) {
		} else {
			sc->waySelected[0] = 1;
		}
		break;
	case '_':
		if( numberSelected( sc, &pickWay ) > 0 ) {
			sc->waySelected[MAX(pickWay - 1,0)] = 1;
		} else {
			sc->waySelected[0] = 1;
		}
		break;
	case ' ':
		selectNone( sc );
		if( sc->showGCS && sc->showChecklist ) {
			if( sc->modifiers == GLUT_ACTIVE_SHIFT ) {
				sg->checklist->activeItem = MAX( 0, sg->checklist->activeItem - 1 );
			} else {
				sg->checklist->activeItem++;
			}
		}
		break;
	case '\r': /* enter */
		if( gsc->showMenu && sg->menuKeys && !sc->modifiers ) {
			if( sc->viewMenuMouseOver > -1 || sc->wayMenuMouseOver > -1 || sc->visMenuMouseOver > -1 ||
				sc->typeMenuMouseOver > -1 || sc->velMenuMouseOver > -1 || sc->altMenuMouseOver > -1 ||
				sc->headMenuMouseOver > -1 || sc->modeMenuMouseOver > -1 || sc->customMenuMouseOver > -1 ) {
				sceneMouseButton( GLUT_LEFT_BUTTON, GLUT_DOWN, -1, -1 );
				sceneMouseButton( GLUT_LEFT_BUTTON, GLUT_UP,   -1, -1 );
			} else {
				sc->viewMenuMouseOver = 0;
			}
		} else if( sc->modifiers == GLUT_ACTIVE_ALT ) {
			if( sc->fullScreen ) {
				glutReshapeWindow( sc->oldw, sc->oldh );
				sc->fullScreen = 0;
			    sceneAddMessage( sg, sc, "Window Mode" );
			} else {
				sc->oldw = sc->winw;
				sc->oldh = sc->winh;
				glutFullScreen();
				sc->fullScreen = 1;
			    sceneAddMessage( sg, sc, "Full Screen Mode" );
			}
		}
		break;
	case 'p': /* swap PIP */
		if( gsc->showPIP ) {
			swapScenePIP( sg, sc );
		} else {
			gsc->showPIP = 1;
			sceneAddMessage( sg, sc, "Picture In Picture On" );
		}
		break;
	case 'P':
		if( numberSelected( sc, &pickWay ) > 0 ) {
			double mx[3];
			m = &maneuver[pickWay];
			waypointLocate( gi, pickWay, mx );
			if( m->hdgMode == HDG_CONST ) {
				if( m->type == MAN_FORMATION && m->derived > 0 ) {
					struct gcsInstance_ref *other;
					switch( gi->datalink->following ) {
					default:
					case 0:  other = &gcs0Instance;  break;
					}
					m->psi = gi->outputs->psi - other->outputs->psi;
				} else {
					m->psi = gi->outputs->psi;
				}
			}
			for( j=0; j<3; j++ )
				move[j] = gi->datalink->m0->pos[j] - mx[j];
			gi->traj->uploadEach[gi->traj->edit] = 1;
		}
		break;
	case '{':
		if( sc->usedKeys ) {
			sc->posStep /= 2;
			sc->altStep /= 2;
		}
		sc->usedKeys = 1;
		break;
	case '}':
		if( sc->usedKeys ) {
			sc->posStep *= 2;
			sc->altStep *= 2;
		}
		sc->usedKeys = 1;
		break;
	case 20: /* ctrl-T */
		commandExecute( "resetTimer" );
		sceneAddMessage( sg, sc, "Flight Timer Reset" );
		break;
	case '@':
		commandExecute( "@" );
		break;

	case 0x1b:
		sceneClose( sc, gi );
		return;
		break;

	default:
		break;
  }
  glutPostWindowRedisplay( sc->win );

}


void sceneSpecialKeys( int key, int x, int y ) {

	struct sceneGlobal_ref *sg = &sceneGlobal;
	struct scene_ref *sc;
	struct view_ref *v;
	struct gcs_ref *g = &gcs;
	struct gcsInstance_ref *gi = gcsActiveInstance( g );
	struct gcsScene_ref *gsc;
	int pickWay;

	//printf( "key = %d\n", key );

	sc = whichScene();
	sc->modifiers = glutGetModifiers();
	gsc = whichGcsScene( sc, gi );
	v = whichView( gi->set, gsc->viewMode );

	switch( key ) {
	case GLUT_KEY_F1:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f1 );
		} else if( sc->modifiers & GLUT_ACTIVE_SHIFT ) {
				switch( gsc->viewMode ) {
				case VIEW_CAMERA4:
				default:
					gsc->viewMode = VIEW_CAMERA;
					sceneAddMessage( sg, sc, "Camera View Selected" );
					break;
				case VIEW_CAMERA:
					gsc->viewMode = VIEW_CAMERA2;
					sceneAddMessage( sg, sc, "Camera 2 View Selected" );
					break;
				case VIEW_CAMERA2:
					gsc->viewMode = VIEW_CAMERA3;
					sceneAddMessage( sg, sc, "Camera 3 View Selected" );
					break;
				case VIEW_CAMERA3:
					gsc->viewMode = VIEW_CAMERA4;
					sceneAddMessage( sg, sc, "Camera 4 View Selected" );
					break;
				}
		} else {
			gsc->viewMode = VIEW_CAMERA;
			/*if( sc->videoMode ) motionControls.videoToggle = 1;*/
			sceneAddMessage( sg, sc, "Camera View Selected" );
		}
		break;
	case GLUT_KEY_F2:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f2 );
		} else {
			gsc->viewMode = VIEW_NAV;
			sceneAddMessage( sg, sc, "Map View Selected" );
		}
		break;
	case GLUT_KEY_F3:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f3 );
		} else {
			gsc->viewMode = VIEW_CHASE;
			sceneAddMessage( sg, sc, "Chase View Selected" );
		}
		break;
	case GLUT_KEY_F4:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f4 );
		} else {
	        gsc->viewMode = VIEW_COCKPIT;
			sceneAddMessage( sg, sc, "PFD View Selected" );
		}
		break;
	case GLUT_KEY_F5:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f5 );
		} else {
			gsc->viewMode = VIEW_GROUND;
			sceneAddMessage( sg, sc, "Ground Observer View Selected" );
		}
		break;
	case GLUT_KEY_F6:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f6 );
		} else {
			gsc->viewMode = VIEW_HOVER;
			sceneAddMessage( sg, sc, "Action View Selected" );
		}
		break;
	case GLUT_KEY_F7:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f7 );
		} 
		break;
	case GLUT_KEY_F8:
		if( sc->modifiers & GLUT_ACTIVE_CTRL ) {
			commandExecute( sg->cf->f8 );
		} else {
			if( sceneCapture.mode ) {
				sceneCapture.mode = SCENECAPTURE_OFF;
				sceneAddMessage( sg, sc, "Image Record OFF" );
			} else {
				if(      sc == &scene0 ) sceneCapture.window = 0;
				else if( sc == &scene1 ) sceneCapture.window = 1;
				else if( sc == &scene2 ) sceneCapture.window = 2;
				sceneCapture.mode = SCENECAPTURE_ALL;
				sceneAddMessage( sg, sc, "Image Record ON" );
			}
		}
		break;

	case GLUT_KEY_F9:
		gsc->virtualJoystick = !gsc->virtualJoystick;
		if( gsc->virtualJoystick == 0 ) {
			gi->cntrlInput->leftVirtualJoystickInUse  = 0;
			gi->cntrlInput->rightVirtualJoystickInUse = 0;
		}
		break;
	case GLUT_KEY_F10:
		if( gsc->showPIP ) {
			if( sc->modifiers & GLUT_ACTIVE_SHIFT ) {
				swapScenePIP( sg, sc );
			} else {
				gsc->showPIP = !gsc->showPIP;
				sceneAddMessage( sg, sc, "Picture In Picture On" );
			}
		} else {
			gsc->showPIP = !gsc->showPIP;
			sceneAddMessage( sg, sc, "Picture In Picture Off" );
		}
		break;
	case GLUT_KEY_F11:
		if( sc->modifiers & GLUT_ACTIVE_SHIFT ) {
			switch( sg->overlaySource ) {
			case OVERLAY_KML:
			/*	sg->overlaySource = OVERLAY_GOOGLETYPE1;
				sceneAddMessage( sg, sc, "Overlays:  Google" );
				break;
			case OVERLAY_GOOGLETYPE1:
				sg->overlaySource = OVERLAY_GOOGLETYPE2;
				sceneAddMessage( sg, sc, "Overlays:  Google with labels" );
				break;
			case OVERLAY_GOOGLETYPE2:
				sg->overlaySource = OVERLAY_GOOGLETYPE1_AND_KML;
				sceneAddMessage( sg, sc, "Overlays:  Google and" );
				break;
			case OVERLAY_GOOGLETYPE1_AND_KML:
				sg->overlaySource = OVERLAY_BINGAERIAL;
				sceneAddMessage( sg, sc, "Overlays:  Bing" );
				break;
			case OVERLAY_BINGAERIAL:*/
				sg->overlaySource = OVERLAY_BINGAERIALWITHLABELS;
				sceneAddMessage( sg, sc, "Overlays:  Bing with labels" );
				break;
			case OVERLAY_BINGAERIALWITHLABELS:
				sg->overlaySource = OVERLAY_BING_AND_KML;
				sceneAddMessage( sg, sc, "Overlays:  Bing and KML" );
				break;
			default:
				sg->overlaySource = OVERLAY_KML;
				sceneAddMessage( sg, sc, "Overlays:  KML" );
				break;
			}
		} else {
			gsc->showTex = !gsc->showTex;
			if( gsc->showTex ) sceneAddMessage( sg, sc, "Showing Map Overlays" );
			else               sceneAddMessage( sg, sc, "Hiding Map Overlays" );
		}
		break;
	case GLUT_KEY_F12:
		gsc->videoMode = !gsc->videoMode;
		if( gsc->videoMode ) sceneAddMessage( sg, sc, "Showing Video Feed (if in Camera View)" );
		else                 sceneAddMessage( sg, sc, "Hiding Video Feed (current view)" );
		break;

	case GLUT_KEY_LEFT:
		if( gsc->showMenu && sg->menuKeys && !sc->modifiers ) {
			if( sc->modeMenuMouseOver > -1 && sg->customMenu->on && gsc->virtualJoystick == 1 ) {
				sc->modeMenuMouseOver = -1;
				sc->customMenuMouseOver = 0;
			}
			if( numberSelected( sc, &pickWay ) > 0 ) {
				if( sc->headMenuMouseOver > -1 && sg->customMenu->on && gsc->virtualJoystick == 0 ) {
					sc->headMenuMouseOver = -1;
					sc->customMenuMouseOver = 0;
				}
				if( sc->headMenuMouseOver > -1 && gsc->virtualJoystick ) {
					sc->headMenuMouseOver = -1;
					sc->modeMenuMouseOver = 0;
				}
				if( sc->altMenuMouseOver > -1 ) {
					sc->altMenuMouseOver = -1;
					sc->headMenuMouseOver = 0;
				}
				if( sc->velMenuMouseOver > -1 ) {
					sc->velMenuMouseOver = -1;
					sc->altMenuMouseOver = 0;
				}
				if( sc->typeMenuMouseOver > -1 ) {
					sc->typeMenuMouseOver = -1;
					sc->velMenuMouseOver = 0;
				}
				if( sc->visMenuMouseOver > -1 ) {
					sc->visMenuMouseOver = -1;
					sc->typeMenuMouseOver = 0;
				}
			} else {
				if( sc->visMenuMouseOver > -1 && sg->customMenu->on && gsc->virtualJoystick == 0 ) {
					sc->visMenuMouseOver = -1;
					sc->customMenuMouseOver = 0;
				}
				if( sc->modeMenuMouseOver > -1 && sg->customMenu->on && gsc->virtualJoystick == 1 ) {
					sc->modeMenuMouseOver = -1;
					sc->customMenuMouseOver = 0;
				}
				if( sc->visMenuMouseOver > -1 && gsc->virtualJoystick ) {
					sc->visMenuMouseOver = -1;
					sc->modeMenuMouseOver = 0;
				}
			}
			if( sc->wayMenuMouseOver > -1 ) {
				sc->wayMenuMouseOver = -1;
				sc->visMenuMouseOver = 0;
			}
			if( sc->viewMenuMouseOver > -1 ) {
				sc->viewMenuMouseOver = -1;
				sc->wayMenuMouseOver = 0;
			}
		} else {
			switch( gsc->viewMode ) {
			case VIEW_COCKPIT:
			case VIEW_CHASE:
			case VIEW_GROUND:
				v->neckPsi -= v->viewAngStep;
				break;
			case VIEW_NAV:
				if( v->lookat == VIEW_LOOKAT_NOTHING || gsc->mapUpMode == MAPUP_CONST ) gsc->mapUpAngle += v->viewAngStep;
				break;
			default:
				break;
			}
		}
		break;
	case GLUT_KEY_RIGHT:
		if( gsc->showMenu && sg->menuKeys && !sc->modifiers ) {
			if( sc->wayMenuMouseOver > -1 ) {
				sc->wayMenuMouseOver = -1;
				sc->viewMenuMouseOver = 0;
			}
			if( sc->visMenuMouseOver > -1 ) {
				sc->visMenuMouseOver = -1;
				sc->wayMenuMouseOver = 0;
			}
			if( sc->typeMenuMouseOver > -1 ) {
				sc->typeMenuMouseOver = -1;
				sc->visMenuMouseOver = 0;
			}
			if( sc->velMenuMouseOver > -1 ) {
				sc->velMenuMouseOver = -1;
				sc->typeMenuMouseOver = 0;
			}
			if( sc->altMenuMouseOver > -1 ) {
				sc->altMenuMouseOver = -1;
				sc->velMenuMouseOver = 0;
			}
			if( sc->headMenuMouseOver > -1 ) {
				sc->headMenuMouseOver = -1;
				sc->altMenuMouseOver = 0;
			}
			if( sc->modeMenuMouseOver > -1 && gsc->virtualJoystick ) {
				sc->modeMenuMouseOver = -1;
				if( numberSelected( sc, &pickWay ) > 0 ) {
					sc->headMenuMouseOver = 0;
				} else {
					sc->visMenuMouseOver = 0;
				}
			}
			if( sc->customMenuMouseOver > -1 && sg->customMenu->on ) {
				sc->customMenuMouseOver = -1;
				if( gsc->virtualJoystick ) {
					sc->modeMenuMouseOver = 0;
				} else {
					if( numberSelected( sc, &pickWay ) > 0 ) {
						sc->headMenuMouseOver = 0;
					} else {
						sc->visMenuMouseOver = 0;
					}
				}
			}
		} else {
			switch( gsc->viewMode ) {
			case VIEW_COCKPIT:
			case VIEW_CHASE:
			case VIEW_GROUND:
				v->neckPsi += v->viewAngStep;
				break;
			case VIEW_NAV:
				if( v->lookat == VIEW_LOOKAT_NOTHING || gsc->mapUpMode == MAPUP_CONST ) gsc->mapUpAngle -= v->viewAngStep;
				break;
			default:
				break;
			}
		}
		break;
	case GLUT_KEY_UP:
		if( gsc->showMenu && sg->menuKeys && !sc->modifiers ) {
			if( sc->viewMenuMouseOver > -1 ) {
				sc->viewMenuMouseOver = MAX( 0, sc->viewMenuMouseOver - 1 );
			}
			if( sc->visMenuMouseOver > -1 ) {
				sc->visMenuMouseOver = MAX( 0, sc->visMenuMouseOver - 1 );
			}
			if( sc->wayMenuMouseOver > -1 ) {
				sc->wayMenuMouseOver = MAX( 0, sc->wayMenuMouseOver - 1 );
			}
			if( sc->typeMenuMouseOver > -1 ) {
				sc->typeMenuMouseOver = MAX( 0, sc->typeMenuMouseOver - 1 );
			}
			if( sc->velMenuMouseOver > -1 ) {
				sc->velMenuMouseOver = MAX( 0, sc->velMenuMouseOver - 1 );
			}
			if( sc->altMenuMouseOver > -1 ) {
				sc->altMenuMouseOver = MAX( 0, sc->altMenuMouseOver - 1 );
			}
			if( sc->headMenuMouseOver > -1 ) {
				sc->headMenuMouseOver = MAX( 0, sc->headMenuMouseOver - 1 );
			}
			if( sc->modeMenuMouseOver > -1 ) {
				sc->modeMenuMouseOver = MAX( 0, sc->modeMenuMouseOver - 1 );
			}
			if( sc->customMenuMouseOver > -1 ) {
				sc->customMenuMouseOver = MAX( 0, sc->customMenuMouseOver - 1 );
			}
		} else {
			switch( gsc->viewMode ) {
			case VIEW_COCKPIT:
			case VIEW_CHASE:
			case VIEW_GROUND:
				v->neckTheta += v->viewAngStep;
				break;
			case VIEW_NAV:
				gi->set->angle3D = MAX( gi->set->angle3D - v->viewAngStep*CF_RAD2DEG, 0 );
				gsc->angle3D     = MAX( gsc->angle3D     - v->viewAngStep*CF_RAD2DEG, 0 );
				break;
			default:
				break;
			}
		}
		break;
	case GLUT_KEY_DOWN:
		if( gsc->showMenu && sg->menuKeys && !sc->modifiers ) {
			if( sc->viewMenuMouseOver > -1 ) {
					sc->viewMenuMouseOver = MIN( (sc->viewMenuOpen?VIEWMENUITEMS-1:(gsc->viewMode==VIEW_NAV?3:0)), sc->viewMenuMouseOver + 1 );
				
			}
			if( sc->visMenuMouseOver > -1 ) {
				sc->visMenuMouseOver = MIN( (sc->visMenuOpen?VISMENUITEMS-1:0), sc->visMenuMouseOver + 1 );
			}
			if( sc->wayMenuMouseOver > -1 ) {
				sc->wayMenuMouseOver = MIN( (sc->wayMenuOpen?WAYMENUITEMS-1:0), sc->wayMenuMouseOver + 1 );
			}
			if( sc->typeMenuMouseOver > -1 ) {
				sc->typeMenuMouseOver = MIN( (sc->typeMenuOpen?TYPEMENUITEMS-1:0), sc->typeMenuMouseOver + 1 );
			}
			if( sc->velMenuMouseOver > -1 ) {
				sc->velMenuMouseOver = MIN( (sc->velMenuOpen?VELMENUITEMS-1:0), sc->velMenuMouseOver + 1 );
			}
			if( sc->altMenuMouseOver > -1 ) {
				sc->altMenuMouseOver = MIN( (sc->altMenuOpen?ALTMENUITEMS-1:0), sc->altMenuMouseOver + 1 );
			}
			if( sc->headMenuMouseOver > -1 ) {
				sc->headMenuMouseOver = MIN( (sc->headMenuOpen?HEADMENUITEMS-1:0), sc->headMenuMouseOver + 1 );
			}
			if( sc->modeMenuMouseOver > -1 ) {
				sc->modeMenuMouseOver = MIN( (sc->modeMenuOpen?MODEMENUITEMS-1:0), sc->modeMenuMouseOver + 1 );
			}
			if( sc->customMenuMouseOver > -1 ) {
				sc->customMenuMouseOver = MIN( (sc->customMenuOpen?CUSTOMMENUITEMS-1:0), sc->customMenuMouseOver + 1 );
			}
		} else {
			switch( gsc->viewMode ) {
			case VIEW_COCKPIT:
			case VIEW_CHASE:
			case VIEW_GROUND:
				v->neckTheta -= v->viewAngStep;
				break;
			case VIEW_NAV:
				gi->set->angle3D = MIN( gi->set->angle3D + v->viewAngStep*CF_RAD2DEG, 90 );
				gsc->angle3D     = MIN( gsc->angle3D     + v->viewAngStep*CF_RAD2DEG, 90 );
				break;
			default:
				break;
			}
		}
		break;

	
	case GLUT_KEY_HOME:
		selectNone( sc );
		sc->waySelected[0] = 1;
		break;
	case GLUT_KEY_PAGE_UP:
		/*commandExecute("ipAbort");*/
		sceneKeyboard( 'i', x, y );
		break;
	case GLUT_KEY_PAGE_DOWN:
		/*commandExecute("ipGo");*/
		sceneKeyboard( 'o', x, y );
		break;

	default:
		break;
	}

	glutPostWindowRedisplay( sc->win );

}


void sceneWaypointMenu( int value ) {

	switch( value ) {
	case 1:
		sceneSpecialKeys( GLUT_KEY_INSERT, 0, 0 );
		break;
	case 2:
		sceneKeyboard( 127 /*delete*/, 0, 0 );
		break;
	case 3:
		sceneKeyboard( 'T', 0, 0 );
		break;
	case 4:
		sceneKeyboard( 'M', 0, 0 );
		break;
	default:
		break;
	}

}


void sceneSimMenu( int value ) {

	switch( value ) {
	case 1:
		initSim();
		break;
	case 2:
		stopSim();
		break;
	case 3:
		startSim();
		break;
	case 4:
		commandExecute( "quit" );
		break;
	default:
		break;
	}

}


void sceneViewMenu( int value ) {

	struct scene_ref *sc;
	sc = whichScene();

	whichGcsScene( sc, gcsActiveInstance(&gcs) )->viewMode = value;

	glutPostWindowRedisplay( sc->win );

}


void sceneLookatMenu( int value ) {

	struct scene_ref *sc;
	sc = whichScene();

	sc->lookat = value;

	switch( value ) {

	case LOOKAT_TRUTH:
		sc->showTruth = 1;
		sc->showGCS = 0;
		break;
	case LOOKAT_GCS:
		sc->showTruth = 0;
		sc->showGCS = 1;
		break;
	case LOOKAT_BOTH:
		sc->showTruth = 1;
		sc->showGCS = 1;
		break;
	default:
		break;
	}

	selectNone( sc );
	sc->mouseOverWay = -1;
	glutPostWindowRedisplay( sc->win );

}


void sceneVisualizeMenu( int value ) {

	struct sceneGlobal_ref *sg = &sceneGlobal;
	struct scene_ref *sc;
	struct view_ref *v;
	struct gcsInstance_ref* gi = gcsActiveInstance(&gcs);
	struct gcsScene_ref *gsc;
	sc = whichScene();
	gsc = whichGcsScene( sc, gi );
	v = whichView( gi->set, gsc->viewMode );

	switch( value ) {
	case 2:
		sc->showBodyAxes = !sc->showBodyAxes;
		break;
	case 3:
		sc->showLgAxes = !sc->showLgAxes;
		break;
	case 1:
		sc->showVel = !sc->showVel;
		break;
	case 4:
		sc->showOmega = !sc->showOmega;
		break;
	case 5:
		sc->showAngMom = !sc->showAngMom;
		break;
	case 6:
		sc->showMag = !sc->showMag;
		break;
	case 7:
		sc->showTraj = !sc->showTraj;
		break;
	case 8:
		sc->showTrack = !sc->showTrack;
		break;
	case 9:
		sc->showGrid = !sc->showGrid;
		break;
	case 10:
		v->hudOn = !v->hudOn;
		break;
	case 11:
		sc->showThreats = !sc->showThreats;
		break;
	case 12:
		gsc->showTex = !gsc->showTex;
		break;
	case 13:
		sc->showNavIP = !sc->showNavIP;
		break;
	case 14:
		sc->showVisionFormation = !sc->showVisionFormation;
		break;
	case 15:
		gsc->showCameraFOV = !gsc->showCameraFOV;
		break;
	case 16:
		gsc->showGCStext = !gsc->showGCStext;
		break;
	case 17:
		gsc->showMenu = !gsc->showMenu;
		break;
	case 18:
		sg->fog = !sg->fog;
		break;
	case 19:
		gsc->showPanel = !gsc->showPanel;
		break;
    case 20:
		gsc->videoMode = !gsc->videoMode;
		break;
    case 21:
        sc->showMapFP = !sc->showMapFP;
        break;
    case 22:
        sc->showSVInfo = !sc->showSVInfo;
        break;
    case 23:
        sc->showScan = !sc->showScan;
        break;
    case 24:
        sc->showGraph = !sc->showGraph;
        break;
    case 25:
        sc->show2dCov = !sc->show2dCov;
        break;
    case 26:
        sc->showPspFilament = !sc->showPspFilament;
        break;
	case 27:
		gsc->showPIP = !gsc->showPIP;
		break;
	case 28:
		swapScenePIP( sg, sc );
		break;
	case 29:
		sc->showLabels = !sc->showLabels;
		break;
	case 30:
		gsc->virtualJoystick = !gsc->virtualJoystick;
		if( gsc->virtualJoystick == 0 ) {
			gi->cntrlInput->leftVirtualJoystickInUse  = 0;
			gi->cntrlInput->rightVirtualJoystickInUse = 0;
		}
		break;
    case 31:
        sc->showWorldPoints = !sc->showWorldPoints;
        break;
	case 32:
		sc->showSLAM = !sc->showSLAM;
		break;
	case 33:
		sc->showWDB = !sc->showWDB;
		break;
	case 34:
		sc->showWatermark = !sc->showWatermark;
		break;
	case 35:
		sc->showTraffic = !sc->showTraffic;
		break;
	case 36:
		sc->showEvimap = !sc->showEvimap;
		break;
    case 37:
        sc->showMBZIRC = !sc->showMBZIRC;
        break;
	case 38:
		gsc->showGeofence = !gsc->showGeofence;
		break;
	default:
		break;
	}

	glutPostWindowRedisplay( sc->win );

}


void sceneMainMenu( int value ) {

	struct scene_ref *sc;
	struct view_ref *v;
	struct gcsInstance_ref *gi;
    struct gcsScene_ref *gsc;
	sc = whichScene();
	gi = gcsActiveInstance( &gcs );
    gsc = whichGcsScene( sc, gi );
	v = whichView( gi->set, gsc->viewMode );

	switch( value ) {
	case 1:
		v->hudOn = !v->hudOn;
		break;
	case 3:
		if( sc->fullScreen ) {
			glutReshapeWindow( sc->oldw, sc->oldh );
			sc->fullScreen = 0;
		} else {
			sc->oldw = sc->winw;
			sc->oldh = sc->winh;
			glutFullScreen();
			sc->fullScreen = 1;
		}
		break;
	case 4:
		sc->redraw = 1;
		glutPostWindowRedisplay( sc->win );
		break;
	case 5:
		sceneClose( sc, gi );
		return;
		break;
	default:
		break;
	}

	glutPostWindowRedisplay( sc->win );

}


void sceneVisibility( int visibility ) {

	struct scene_ref *sc;
	sc = whichScene();

	sc->visibility = visibility;

}


static void savePoseKML( struct vehicleOutputs_ref *o, struct sceneGlobal_ref *sg ) {

	FILE *poseFileID;

	poseFileID = fopen( "gevehicle.kml", "w" );
	if( poseFileID == NULL ) {
		logWarning( "savePose: unable to open file to write" );
	} else {
		fprintf( poseFileID, "<?xml version=%c1.0%c encoding=%cUTF-8%c?>\n<kml xmlns=%chttp://www.opengis.net/kml/2.2%c>\n", 34, 34, 34, 34, 34, 34 );
		fprintf( poseFileID, "  <Placemark>\n" );
		fprintf( poseFileID, "    <name>GUST Vehicle Pose</name>\n" );
		fprintf( poseFileID, "    <Model id=%cheli%c>\n", 34, 34 );
		fprintf( poseFileID, "      <altitudeMode>absolute</altitudeMode>\n" );
		fprintf( poseFileID, "      <Location>\n" );
		fprintf( poseFileID, "        <longitude>%.13lf</longitude>\n", o->longitude );
		fprintf( poseFileID, "        <latitude>%.13lf</latitude>\n", o->latitude );
		fprintf( poseFileID, "        <altitude>%.2f</altitude>\n", o->altitudeMSL*C_FT2M );
		fprintf( poseFileID, "      </Location>\n" );
		fprintf( poseFileID, "      <Orientation>\n" );
		fprintf( poseFileID, "        <heading>%.2f</heading>\n", 180 + o->psi );
		fprintf( poseFileID, "        <tilt>%.2f</tilt>\n", o->theta );
		fprintf( poseFileID, "        <roll>%.2f</roll>\n", o->phi );
		fprintf( poseFileID, "      </Orientation>\n" );
		fprintf( poseFileID, "      <Scale><x>%f</x><y>%f</y><z>%f</z></Scale>\n", sg->io->poseKMLScale, sg->io->poseKMLScale, sg->io->poseKMLScale );
		fprintf( poseFileID, "      <Link><href>%s</href></Link>\n", sg->io->poseKMLFileName );
		fprintf( poseFileID, "    </Model>\n" );
		fprintf( poseFileID, "    <Camera id = %ccamera%c>\n", 34, 34 );
		fprintf( poseFileID, "      <altitudeMode>absolute</altitudeMode>\n" );
		fprintf( poseFileID, "      <longitude>%.13lf</longitude>\n", o->longitude );
		fprintf( poseFileID, "      <latitude>%.13lf</latitude>\n", o->latitude );
		fprintf( poseFileID, "      <altitude>%.2f</altitude>\n", o->altitudeMSL*C_FT2M + 50 );
		/*fprintf( poseFileID, "      <heading>%.2f</heading>\n", o->psi );
		fprintf( poseFileID, "      <tilt>%.2f</tilt>\n", 90 + o->theta );
		fprintf( poseFileID, "      <roll>%.2f</roll>\n", o->phi );*/
		fprintf( poseFileID, "    </Camera>\n" );
		fprintf( poseFileID, "  </Placemark>\n" );
		fprintf( poseFileID, "</kml>\n" );
		fclose( poseFileID );
	}

}


static void savePathKML( char *fileName, char saveTruth ) {

	FILE *pathFileID;
	char buffer[256];
	int i;
	unsigned char gn;

	gn = LIMIT( gcs.active, 0, GCS_MAX_INSTANCES-1 );

	pathFileID = fopen( fileName, "w" );
	if( pathFileID == NULL ) {
		logWarning( "savePath: unable to open file to write" );
	} else {
		fprintf( pathFileID, "<?xml version=%c1.0%c encoding=%cUTF-8%c?>\n<kml xmlns=%chttp://www.opengis.net/kml/2.2%c>\n", 34, 34, 34, 34, 34, 34 );
		fprintf( pathFileID, "  <Document>\n    <name>%s</name>\n", fileName );
		fprintf( pathFileID, "    <description>trajectory imported from GUST</description>\n" );
		fprintf( pathFileID, "    <Style id=%cyellowLineGreenPoly%c>\n", 34, 34 );
		fprintf( pathFileID, "      <LineStyle>\n" );
		fprintf( pathFileID, "        <color>7f00ffff</color>\n" );
		fprintf( pathFileID, "        <width>4</width>\n" );
		fprintf( pathFileID, "      </LineStyle>\n" );
		fprintf( pathFileID, "      <PolyStyle>\n" );
		fprintf( pathFileID, "        <color>7f00ff00</color>\n" );
		fprintf( pathFileID, "      </PolyStyle>\n" );
		fprintf( pathFileID, "    </Style>\n    <Placemark>\n" );
		fprintf( pathFileID, "      <name>Recorded Trajectory</name>\n" );
		fprintf( pathFileID, "      <styleUrl>#yellowLineGreenPoly</styleUrl>\n" );
		fprintf( pathFileID, "      <LineString>\n" );
		fprintf( pathFileID, "        <extrude>1</extrude>\n" );
		fprintf( pathFileID, "        <tessellate>1</tessellate>\n" );
		fprintf( pathFileID, "        <altitudeMode>absolute</altitudeMode>\n" );
		fprintf( pathFileID, "        <coordinates>\n" );
		if( saveTruth ) {
			for( i=0; i<sceneGlobal.ntpoints; i++ )
				fprintf( pathFileID, "          %.13lf,%.13lf,%.2f\n", tpoints[i].lon, tpoints[i].lat, tpoints[i].alt*C_FT2M );
		} else {
			for( i=0; i<sceneGlobal.ntpoints_nav[gn]; i++ )
				fprintf( pathFileID, "          %.13lf,%.13lf,%.2f\n", tpoints_nav[gn][i].lon, tpoints_nav[gn][i].lat, tpoints_nav[gn][i].alt*C_FT2M );
		}
		fprintf( pathFileID, "        </coordinates>\n" );
		fprintf( pathFileID, "      </LineString>\n" );
		fprintf( pathFileID, "    </Placemark>\n" );
		fprintf( pathFileID, "  </Document>\n" );
		fprintf( pathFileID, "</kml>\n" );
		fclose( pathFileID );
		sprintf( buffer, "savePath: path saved to %s", fileName );
		logInfo( buffer );
	}

}


static void updateScene( struct scene_ref *sc ) {

	if( sc->open )
		if( sc->visibility == GLUT_VISIBLE )
			glutPostWindowRedisplay( sc->win );

}


void updateScenes( void ) {

	struct gcs_ref *g = &gcs;
	struct gcsInstance_ref *gi;
	struct vehicleOutputs_ref *no;
	struct vehicleOutputs_ref *vo = &vehicleOutputs;
	struct sceneGlobal_ref    *sg = &sceneGlobal;
	struct tpoint *t;
	char gn;

	/* storing past trajectory */

	/* truth */
	if( sim.mode == SIM_MODE_INIT ) {
		sg->ntpoints = 0;
		tpointTime = 0;
		scene0.messageTime0 = -100;
		scene0.messageTime1 = -100;
		scene0.messageTime2 = -100;
		scene0.messageTime3 = -100;
		scene1.messageTime0 = -100;
		scene1.messageTime1 = -100;
		scene1.messageTime2 = -100;
		scene1.messageTime3 = -100;
		scene2.messageTime0 = -100;
		scene2.messageTime1 = -100;
		scene2.messageTime2 = -100;
		scene2.messageTime3 = -100;
	} else {
		if( tpointTime + sg->dtTraj <= vo->time && sg->ntpoints < MAXTPOINTS-1 ) {
			t = &tpoints[sg->ntpoints];
			tpointTime = vo->time;
			t->lat     = vo->latitude;
			t->lon     = vo->longitude;
			t->alt     = vo->altitudeMSL;
			sg->ntpoints++;
		}
	}

	/* nav */
	for( gn=0; gn<GCS_MAX_INSTANCES; gn++ ) {

		gi = gcsGetInstance( g, gn );
		no = gi->outputs;

		if( sim.mode == SIM_MODE_INIT ) {
			sg->ntpoints_nav[gn]   = 0;
			tpointTime_nav[gn]     = 0;
		} else {
			if( no->time < tpointTime_nav[gn] - 5 ) { /* onboard was reset */
				tpointTime_nav[gn] = no->time - sg->dtTraj_nav;    /* store the new point */
			}
			if( tpointTime_nav[gn] + sg->dtTraj_nav <= no->time && sg->ntpoints_nav[gn] < MAXTPOINTS-1 ) {
				t = &tpoints_nav[gn][sg->ntpoints_nav[gn]];
				tpointTime_nav[gn] = no->time;
				t->lat             = no->latitude;
				t->lon             = no->longitude;
				t->alt             = no->altitudeMSL;
				sg->ntpoints_nav[gn]++;
			}
		}

	}

	/* update each individual window */

	updateScene( &scene0 );
	updateScene( &scene1 );
	updateScene( &scene2 );

	/* save the pose KML file */

	if( sg->outputGoogleEarth ) {
		if( sim.wallTime > sg->io->poseKMLLastTime + sg->io->poseKMLdt || sim.wallTime < sg->io->poseKMLLastTime ) {
			no = g->instance0->outputs;
			savePoseKML( no, sg );
			sg->io->poseKMLLastTime = sim.wallTime;
		}
	}

}


static void closeSceneCmd( int window ) {

	struct scene_ref *sc = NULL;
	/*struct sceneGlobal_ref *sg = &sceneGlobal;*/

	switch( window ) {
	case 0:
		sc = &scene0;
		break;
	case 1:
		sc = &scene1;
		break;
	case 2:
		sc = &scene2;
		break;
	default:
		break;
	}

	if( sc != NULL ) {
		if( sc->open ) {
			sceneClose( sc, gcsActiveInstance( &gcs ) );
		}
	}

}

static void swapSceneCmd( int window ) {

	struct scene_ref *sc = NULL;
	struct sceneGlobal_ref *sg = &sceneGlobal;

	switch( window ) {
	case 0:
		sc = &scene0;
		break;
	case 1:
		sc = &scene1;
		break;
	case 2:
		sc = &scene2;
		break;
	default:
		break;
	}

	if( sc != NULL ) {
		if( sc->open ) {
			swapScenePIP( sg, sc );
		}
	}

}

static void openSceneCmd( int window ) {

	struct scene_ref *sc = NULL;
	/*struct sceneGlobal_ref *sg = &sceneGlobal;*/
	int simMenu, viewMenu, lookatMenu, visualizeMenu, waypointMenu;

	switch( window ) {
	case 0:
		sc = &scene0;
		break;
	case 1:
		sc = &scene1;
		break;
	case 2:
		sc = &scene2;
		break;
	default:
		if(      scene0.open == 0 ) sc = &scene0;
		else if( scene1.open == 0 ) sc = &scene1;
		else if( scene2.open == 0 ) sc = &scene2;
		break;
	}

	if( sc != NULL ) {

		if( sc->open ) {
			sceneClose( sc, gcsActiveInstance( &gcs ) );
		}

		sc->open = 1;

		/* glut initialize */

		glutInitWindowSize( sc->winw, sc->winh );
		glutInitWindowPosition( sc->x, sc->y );
		glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE );
		sc->win = glutCreateWindow( sc->name );
		if( sc->fullScreen ) {
			sc->oldw = sc->winw;
			sc->oldh = sc->winh;
			glutFullScreen();
		}

		/* top level call-backs */

		glutDisplayFunc( redrawSceneCallback );
		glutReshapeFunc( sceneReshape );
		glutMouseFunc( sceneMouseButton );
		glutMotionFunc( sceneMouseMotion );
		glutPassiveMotionFunc( sceneMouseMotion );
		glutKeyboardFunc( sceneKeyboard );
		glutSpecialFunc( sceneSpecialKeys );
		glutVisibilityFunc( sceneVisibility );

		/* menu setup */

		if( sc->allowRightClickMenu ) {

			waypointMenu = glutCreateMenu( sceneWaypointMenu );
			glutAddMenuEntry( "Insert <ins>",   1 );
			glutAddMenuEntry( "Delete <del>",   2 );
			glutAddMenuEntry( "Transition <T>", 3 );
			glutAddMenuEntry( "Maneuver <M>",   4 );

			simMenu = glutCreateMenu( sceneSimMenu );
			glutAddMenuEntry( "Init",  1 );
			glutAddMenuEntry( "Pause", 2 );
			glutAddMenuEntry( "Start", 3 );
			glutAddMenuEntry( "Quit",  4 );

			visualizeMenu = glutCreateMenu( sceneVisualizeMenu );
			glutAddMenuEntry( "GCS Data (Text)",          16 );
			glutAddMenuEntry( "Menu",                     17 );
			glutAddMenuEntry( "HUD <h>",                  10 );
			glutAddMenuEntry( "Status Panel",             19 );
			glutAddMenuEntry( "Picture in Picture <F10>", 27 );
			glutAddMenuEntry( "Swap PIP <p>",             28 );
			glutAddMenuEntry( "Virtual Joysticks <F9>",   30 );
			glutAddMenuEntry( "Body Axes",                2  );
			glutAddMenuEntry( "LG/NED Axes",              3  );
			glutAddMenuEntry( "Velocity",                 1  );
			glutAddMenuEntry( "Angular Velocity",         4  );
			glutAddMenuEntry( "Angular Momentum",         5  );
			glutAddMenuEntry( "Axis Labels (if on)",      29 );
			glutAddMenuEntry( "Magnetic Field",           6  );
			glutAddMenuEntry( "Trace <t>",                7  );
			glutAddMenuEntry( "Grid on Ground",           9  );
			glutAddMenuEntry( "Overlays <F11>",           12 );
			glutAddMenuEntry( "Video <F12>",              20 );
			glutAddMenuEntry( "Fog",                      18 );
			glutAddMenuEntry( "Camera FOV",               15 );
			glutAddMenuEntry( "ADS-B Traffic",            35 );
			glutAddMenuEntry( "Vision Tracking",          8  );
			glutAddMenuEntry( "NavIP",                    13 );
			glutAddMenuEntry( "Vision Formation",         14 );
			glutAddMenuEntry( "MapFP",                    21 );
			glutAddMenuEntry( "World Points",             31 );
			glutAddMenuEntry( "GPS Sats",                 22 );
			glutAddMenuEntry( "Laser Scan",               23 );
			glutAddMenuEntry( "Slam Guidance",            24 );
			glutAddMenuEntry( "2D Error Ellipse",         25 );
			glutAddMenuEntry( "Psp Filament",             26 );
			glutAddMenuEntry( "SLAM map",                 32 );
			glutAddMenuEntry( "Evidence map",             36 );
			glutAddMenuEntry( "MBZIRC",                   37 );
			glutAddMenuEntry( "World database (wdb)",     33 );
			glutAddMenuEntry( "Watermark",                34 );
			glutAddMenuEntry( "Geofence",                 38 );

			viewMenu = glutCreateMenu( sceneViewMenu );
			glutAddMenuEntry( "Camera <F1>",      VIEW_CAMERA    );
			glutAddMenuEntry( "Map Display <F2>", VIEW_NAV       );
			glutAddMenuEntry( "Chase <F3>",       VIEW_CHASE     );
			glutAddMenuEntry( "PFD <F4>",         VIEW_COCKPIT   );
			glutAddMenuEntry( "Ground <F5>",      VIEW_GROUND    );
			glutAddMenuEntry( "Action <F6>",      VIEW_HOVER     );
			glutAddMenuEntry( "Camera 2",         VIEW_CAMERA2   );
			glutAddMenuEntry( "Camera 3",         VIEW_CAMERA3   );
			glutAddMenuEntry( "Camera 4",         VIEW_CAMERA4   );

			lookatMenu = glutCreateMenu( sceneLookatMenu );
			glutAddMenuEntry( "Truth", LOOKAT_TRUTH );
			glutAddMenuEntry( "GCS",   LOOKAT_GCS   );
			glutAddMenuEntry( "Both",  LOOKAT_BOTH  );

			glutCreateMenu( sceneMainMenu );
			glutAddSubMenu( "Waypoint", waypointMenu );
			/*glutAddMenuEntry( "HUD <h>",       1 );*/
			glutAddSubMenu( "Show",    visualizeMenu );
			glutAddSubMenu( "View",    viewMenu      );
			glutAddSubMenu( "Lookat",  lookatMenu    );
			glutAddMenuEntry( "Full screen",   3 );
			glutAddMenuEntry( "Refresh",       4 );
			glutAddSubMenu( "Sim",     simMenu       );
			glutAddMenuEntry( "Close Window",  5 );

			glutMenuStateFunc( sceneMenuState );
			glutAttachMenu( GLUT_RIGHT_BUTTON );
		}

		sc->redraw = 1;

		sceneReshape( sc->winw, sc->winh );

	}

}


void commandScene( int argc, char **argv ) {

	if(        argc == 3 && !strcmp( argv[2], "close" )  ) {
		closeSceneCmd( atoi( argv[1] ) );
	} else if( argc == 3 && !strcmp( argv[2], "swap" )  ) {
		swapSceneCmd( atoi( argv[1] ) );
	} else if( argc == 2 ) {
		openSceneCmd( atoi( argv[1] ) );
	} else {
		openSceneCmd( -1 );
	}

}


void commandSavePath( int argc, char **argv ) {

	char fileName[100], saveTruth = 0;

	if( argc == 3 && !strcmp( argv[2], "truth" ) ) {
		saveTruth = 1;
		strcpy( fileName, argv[1] );
	} else if( argc == 2 ) {
		strcpy( fileName, argv[1] );
	} else {
		sprintf( fileName, "path" );
	}

	/* write the file */
	strcat( fileName, ".kml" );
	savePathKML( fileName, saveTruth );

}


void commandCapture( int argc, char **argv ) {

	if( argc == 1 ) {

		sceneCapture.singleShot = 1;

		if( sim.mode == SIM_MODE_PAUSE ) {
			switch( sceneCapture.window ) {
			case 0:
				if( scene0.open ) glutPostWindowRedisplay( scene0.win );
				break;
			case 1:
				if( scene1.open ) glutPostWindowRedisplay( scene1.win );
				break;
			case 2:
				if( scene2.open ) glutPostWindowRedisplay( scene2.win );
				break;
			}
		}

		logInfo( "sceneCapture: single image" );

	} else if( argc == 2 && !strcmp( argv[1], "on" ) ) {

		sceneCapture.mode = SCENECAPTURE_ALL;
		logInfo( "sceneCapture is ON" );

	} else if( argc == 2 && !strcmp( argv[1], "off" ) ) {

		sceneCapture.mode = SCENECAPTURE_OFF;
		logInfo( "sceneCapture is OFF" );

	} else {

		logInfo( "usage: sceneCapture [on/off]" );

	}

}


void commandChecklistClear( int argc, char **argv ) {

	struct sceneGlobal_ref *sg = &sceneGlobal;

	sceneChecklistClear( sg );

	logInfo( "Electronic Checklist Cleared" );
	/*if( scene0.open ) sceneAddMessage( sg, &scene0, "Checklist Cleared" );
	if( scene1.open ) sceneAddMessage( sg, &scene1, "Checklist Cleared" );
	if( scene2.open ) sceneAddMessage( sg, &scene2, "Checklist Cleared" );*/

}


void commandSceneMessage( int argc, char **argv ) {

	struct sceneGlobal_ref *sg = &sceneGlobal;

	if( argc > 1 ) {

		char buffer[200];
		int i;

		sprintf( buffer, argv[1] );
		for( i=2; i<argc; i++ ) {
			sprintf( buffer, "%s %s", buffer, argv[i] );
		}

		if( scene0.open ) {
			sceneAddMessage( sg, &scene0, buffer );
			glutPostWindowRedisplay( scene0.win );
		}
		if( scene1.open ) {
			sceneAddMessage( sg, &scene1, buffer );
			glutPostWindowRedisplay( scene1.win );
		}
		if( scene2.open ) {
			sceneAddMessage( sg, &scene2, buffer );
			glutPostWindowRedisplay( scene2.win );
		}

	} else {

		logInfo( "usage: sceneMessage <message text>" );

	}


}


int sceneGrabberStart( int win ) {

	int ret = 0;

	switch( win ) {
	case 0:  scene0.openglGrabber = 1;  break;
	case 1:  scene1.openglGrabber = 1;  break;
	case 2:  scene2.openglGrabber = 1;  break;
	default:  ret = 1;  break;
	}

	return ret;

}


int sceneGrabberShutdown( int win ) {

	int ret = 0;
	struct scene_ref *sc = NULL;

	switch( win ) {
	case 0:  sc = &scene0;  break;
	case 1:  sc = &scene1;  break;
	case 2:  sc = &scene2;  break;
	default:  ret = 1;  break;
	}

	if( sc != NULL ) { 
		sc->openglGrabber = 0;
		if( sc->grabberAllocatedSize ) {
			free( sc->grabberData );
			sc->grabberAllocatedSize = 0;
		}
	}

	return ret;

}


int sceneGrabberFrame( int win, int *winw, int *winh, void **data ) {

	/* see if data is ready for opengl frame grabber, and send pointer if it is */

	int ret = 1;
	struct scene_ref *sc = &scene0;

	switch( win ) {
	case 0:  sc = &scene0;  break;
	case 1:  sc = &scene1;  break;
	case 2:  sc = &scene2;  break;
	default:  ret = 1;  break;
	}

	if( sc->open ) {
		if( sc->openglGrabber ) {
			*winw = sc->winw;
			*winh = sc->winh;
			if( sc->winw*sc->winh*3 == sc->grabberAllocatedSize ) {
				*data = sc->grabberData;
				ret = 0;
			}
		}
	}

	return ret;
}


int initScenes( void ) {

	char in[BSIZE][BSIZE]= { /* init */
   			"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
			"bb                            bb",
			"b                              b",
			"b                              b",
			"b     bbbbbbbbbbbbbbbbbbbb     b",
			"b     b                  b     b",
			"b     b wwwwwwwwwwwwwwww b     b",
			"b     b wwwwwwwwwwwwwwww b     b",
			"b     b wwwwwwwwbbwwwwww b     b",
			"b     b wwwwwwbbwwwwwwbb b     b",
			"b     b wwwwbbwwwwwwbbbb b     b",
			"b     b wwwwwwwwwbbbbbbb b     b",
			"b     b wwwwwwwbbbbbbbbb b     b",
			"b     b wwwwwbbbbbbbbbbb b     b",
			"b     b wwwbbbbbbbbwwbbb b     b",
			"b     b wbbbbbbbbwwbbbbb b     b",
			"b     b bbbbbbbwwbbbbbbb b     b",
			"b     b bbbbbbbbbbbbbbbb b     b",
			"b     b bbbbbbbbbbbbbbbb b     b",
			"b     b                  b     b",
			"b     bbbbbbbbbbbbbbbbbbbb     b",
			"b     bbbbbbbbbbbbbbbbbbbb     b",
			"b                              b",
			"b                              b",
			"b   bbb  bb  bbbb b   b bbbb   b",
			"b  b    b  b b    bb  b b      b",
			"b   bb  b    bb   b b b bbb    b",
			"b     b b  b b    b  bb b      b",
			"b  bbb   bb  bbbb b   b bbbb   b",
			"b                              b",
			"bb                            bb",
			"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
    };

	addCnslButton( "scene", in );
	commandLoad( "scene",          commandScene,          "scene: open a scene window" );
	commandLoad( "pathSave",       commandSavePath,       "scene: save path flown as kml file" );
	commandLoad( "sceneCapture",   commandCapture,        "scene: capture image as file" );
	commandLoad( "sceneMessage",   commandSceneMessage,   "scene: show message on scene windows" );
	commandLoad( "checklistClear", commandChecklistClear, "scene: clear electronic checklist" );

	sceneInitFonts();

	return 0;

}



void pickPlanter(double n, double e ) {
	//plant poles of random heights and widths with the picker in the scene
	char buf[2048];

	//wdbadd building latlong lat long  lt   alt  length width  height heading
	FILE *f = 0;

	double ranHeight = 40*randne();	//why doesnt abs work?
	if (ranHeight < 0) ranHeight = -ranHeight;
	f = fopen("pickPlanter.inp","a");
	if( !f ) {
		logError("cannot open pickPlanter.inp to write");
		return;
	}


	sprintf(buf, "wdbadd pole ned %f %f 0 0 %f %f\n",n,e, ceil(ranHeight), ceil(ranHeight/3));
	fwrite(buf,strlen(buf),1,f);

	fclose(f);
	logInfo("Wrote to pickPlanter.inp");
}

void drawCylinder( float radius, float height ) {

    float angle;
    float cosine, sine;

    glBegin( GL_POLYGON );
    glNormal3f( 0, -1.0, 0 );
    for( angle = 0; angle<C_PI*2; angle += (float)C_PI/10.0f ) {
        cosine = (float) cos(angle);
        sine   = (float) sin(angle);
        glVertex3f( radius*cosine, -height/2.0f, radius*sine );
    }
    glEnd();

    glBegin( GL_POLYGON );
    glNormal3f( 0, 1.0, 0 );
    for( angle = 0; angle<C_PI*2; angle += (float)C_PI/10 ) {
        cosine = (float) cos(angle);
        sine   = (float) sin(angle);
        glVertex3f( radius*cosine, +height/2, -radius*sine );
    }
    glEnd();

	glBegin( GL_QUAD_STRIP );
    for( angle = 0; angle<C_PI*2 + C_PI/20; angle += (float)C_PI/10 ) {
        cosine = (float) cos(angle);
        sine   = (float) sin(angle);
        glNormal3f( cosine, 0.0f, sine );
        glVertex3f( radius*cosine, -height/2, radius*sine);
        glVertex3f( radius*cosine, +height/2, radius*sine );
    }
	glEnd();

}
