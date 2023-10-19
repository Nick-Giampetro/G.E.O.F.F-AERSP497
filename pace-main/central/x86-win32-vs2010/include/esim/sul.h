#ifndef SUL_SUL_H
#define SUL_SUL_H

/** Copyright 2003 Suresh K. Kannan */

#include "esim/platform.h"


#define SUL_TYPE_CHAR                  1
#define SUL_TYPE_SHORT                 2
#define SUL_TYPE_INT                   3
#define SUL_TYPE_LONG                  4
#define SUL_TYPE_DOUBLE                5
#define SUL_TYPE_UCHAR                 6
#define SUL_TYPE_USHORT                7
#define SUL_TYPE_UINT                  8
#define SUL_TYPE_ULONG                 9




#define SUL_OK 0
#define SUL_NOTOK -1
#define SUL_MAX_WARN_MSG_LEN        1024
#define SUL_MAX_ERR_MSG_LEN         1024




#define SUL_MAXHOSTNAMELEN 256



/********************* Error and Printing utilities **************************/
#if defined(__cplusplus)
extern "C"
{
#endif
	/** warns user about something */
	void sulWarn (const char* func, const char* msg, ...);
	/** prints out a fatal error */
	void sulError(const char* func, const char* msg, ...);
#if defined(__cplusplus)
}
#endif



/** An element in a list */
typedef struct sulListElmt_ref {
	void                     *data;
	struct sulListElmt_ref   *next;
} sulListElmt;

/** A linked list containing sulListElmt as its elements */
typedef struct sulList_ref {
	int                size;
	int                (*match)(const void *key1, const void *key2);
	void               (*destroy)(void *data);
	sulListElmt         *head;
	sulListElmt         *tail;
} sulList;

#if defined(__cplusplus)
extern "C"
{
#endif
	/** sulList public interface */
	void sulListInit(sulList *list, void (*destroy)(void *data));
	
	void sulListDestroy(sulList *list);
	
	int sulListInsNext(sulList *list, sulListElmt *element, const void *data);
	
	int sulListRemNext(sulList *list, sulListElmt *element, void **data);
	
#define sulListSize(list) ((list)->size)
	
#define sulListHead(list) ((list)->head)
	
#define sulListTail(list) ((list)->tail)
	
#define sulListIsHead(list, element) ((element) == (list)->head ? 1 : 0)
	
#define sulListIsTail(element) ((element)->next == NULL ? 1 : 0)
	
#define sulListData(element) ((element)->data)
	
#define sulListNext(element) ((element)->next)
	
#if defined(__cplusplus)
}
#endif



/*********************** Map Functions ***************************************/


typedef struct sulMap_ref {
	int                buckets;
	int                (*h)(const void *key);
	int                (*match)(const void *key1, const void *key2);
	void               (*destroy)(void *data);
	int                size;
	sulList            *table;
	
} sulMap;

#if defined(__cplusplus)
extern "C"
{
#endif
	
	int sulMapInit(sulMap *htbl, int buckets, int (*h)(const void *key), int
		(*match)(const void *key1, const void *key2), void (*destroy)(void *data));
	
	void sulMapDestroy(sulMap *htbl);
	
	int sulMapInsert(sulMap *htbl, const void *data);
	
	int sulMapRemove(sulMap *htbl, void **data);
	
	int sulMapLookup(const sulMap *htbl, void **data);
	
#define sulMapSize(htbl) ((htbl)->size)
	
#if defined(__cplusplus)
}
#endif

/************************* Sockets *******************************************/
#if defined(__cplusplus)
extern "C"
{
#endif
	/** Makes a sockaddr from given host and port number */
	int sulSockaddrMake ( const char *host,   short port, struct sockaddr_in *sin );
	/** Sets either socket to be blocking or non blocking */
	void sulSockBlocking ( int fd, int blocking );
	/** Initializes the sockets subsystem, really only needed for windows */
	int sulSocketsInit (void);
	/** Shutsdown the sockets subsystem, really only needed for windows*/
	int sulSocketsShutdown (void);
	/** Creates a socket */
	int sulSockCreate(const char *protocol);
	/** Connects a socket */
	int sulSockConnect(int fd, const struct sockaddr_in *addr);
	/** Sends data to a socket */
	int sulSockSend(int fd, const char *buf, int n, int flags);
	/** Closes a socket */
	int sulSockClose(int fd);
	
	
#if defined(__cplusplus)
}
#endif


/*************************** Timing ******************************************/
#if defined(__cplusplus)
extern "C"
{
#endif
	
	/** sleep for given seconds */
	void sulSleep(unsigned long secs);
	/** sleep for given milliseconds*/
	void sulSleepM(unsigned long msecs);
	
#if defined(__cplusplus)
}
#endif


#endif /* SUL_SUL_H */

