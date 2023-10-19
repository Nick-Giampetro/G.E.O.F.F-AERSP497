#ifndef rmax_panel_h
#define rmax_panel_h
#if defined(__cplusplus)
extern "C"
{
#endif

/* Copyright (c) Eric N. Johnson, 1998.  */

int initPanel( void );
void updatePanel( void );
void panelSubDraw( struct panel_ref *p, int winw, int winh );
void readPanelStates( struct gcs_ref *g, struct gcsInstance_ref *gi );

#if defined(__cplusplus)
}
#endif
#endif
