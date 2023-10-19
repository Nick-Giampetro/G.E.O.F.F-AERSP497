#ifndef RMAX_TASKS_H
#define RMAX_TASKS_H


// C function prototypes */
#ifdef __cplusplus
extern "C" {
#endif
// any idle functions
extern void simTick( void );
extern void drawTick( void );


#ifdef __cplusplus
}
#endif

int simTask( int argc, char **argv );

#endif // RMAX_TASKS_H

