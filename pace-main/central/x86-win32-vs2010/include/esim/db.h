#ifndef esim_db_h
#define esim_db_h

#if defined(__cplusplus)
extern "C"
{
#endif

#define MAXDIRS 16384
#define VARNAMELENGTH 180
#define VALUELENGTH 256   /* needs to be less than or equal to EDITBUFFERLENGTH in brwin.db */
#define MAXDIM 4

#include "types.h"

typedef struct option_ref {
  char *name;
  int value;
} Option;

typedef struct enum_ref {
  char *name;
  int noptions;
  Option *options;
} Enum;

typedef struct Var_ref {
  char type;  /* variable type */
  void *data; /* memory of stored parameter value */
  char *name; /* name of var */
  char *comment; /* comment for var */
  struct dir_ref *child;   /* if this is a dir */
  struct dir_ref *parent;  /* parent dir */
  Enum *enumMe; /* if it is one */
  char unexpandedArray;
} Var;

typedef struct dir_ref {
  char name[VARNAMELENGTH]; /* name of dir */
  struct dir_ref *parent;  /* parent dir */
  Var *vars;    /* variables in directory array */
  void *data;   /* memory of c-structure */
  int size;     /* number of vars in dir */
  int nbytes;   /* number of bytes for c-structure */
  char type[VARNAMELENGTH]; /* string type */
} Dir;

struct dir_hlist_element {
  char name[VARNAMELENGTH];
  Dir *dir;
};

extern struct dir_hlist_element dir_hlist[MAXDIRS];

Dir *findDir( const char *dirName );
Var *findVar( const char *varName );
Var *findVarInDir( const Dir *owner, const char *varName );
void initVar( Var *v );

#if defined(__cplusplus)
}
#endif

#endif //esim_db_h
