/***
 * $Id: ether.cpp,v 1.7 2005-09-19 08:09:41 ejohnson Exp $
 * contains platform specific code to read and write serial and udp packets
 * this file mostly cotains win32 code
 * unix version is contained in ether2.cpp
 ***/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define _WIN32_WINNT 0x0400 /* this allows use of SwitchThreadTo */

#include <windows.h>      /* Standard Windows include for system defines */
#include <winbase.h>      /* Has WIN32 Comm library and structure DEFS */
#include <winsock.h>

#include "esim/util.h"
#include "rmax/onboard_ref.h"
#include "rmax/ether.h"


int openCom(struct serialPort_ref *s)
{
	SerialDevice *sd;
#ifdef POSIX
	sd = new PosixSerial();
#else
	sd = new WinSerial();
#endif
	sd->open(s);
	s->serialDevice = sd;
	return s->fd;
}

int closeCom(struct serialPort_ref *s)
{
	SerialDevice *sd = (SerialDevice*)s->serialDevice;
	if(sd)
	{
		sd->close();
		delete sd;
		s->serialDevice = NULL;
	}
	return 0;
}

int readCom(struct serialPort_ref *s, void *buf, int nbytes)
{
	return ((SerialDevice*)s->serialDevice)->read(buf, nbytes);
}

int writeCom(struct serialPort_ref *s, void *buf, int nbytes)
{
	return ((SerialDevice*)s->serialDevice)->write(buf, nbytes);
}


int openSock(struct serialPort_ref *s)
{
	SerialDevice *sd;
#ifdef WIN32
	sd = new WinBsdUDP();
#else
	sd = new WinBsdUDP();
#endif

	sd->open(s);
	s->sockDevice = sd;
	return (int)(s->sockDevice);
}

int closeSock(struct serialPort_ref *s)
{
	SerialDevice *sd = (SerialDevice*)s->sockDevice;
	if(sd)
	{
		sd->close();
		delete sd;
		s->sockDevice = NULL;
	}
	return 0;
}

int readSock(struct serialPort_ref *s, void *buf, int nbytes)
{
	return ((SerialDevice*)s->sockDevice)->read(buf, nbytes);
}

int writeSock(struct serialPort_ref *s, void *buf, int nbytes)
{
	return ((SerialDevice*)s->sockDevice)->write(buf, nbytes);
}



#ifdef POSIX
int PosixSerial::open(serialPort_ref *serial)
{
	int modemControl, parity, characterSize, stopbits;
	modemControl = 0; parity = 0; characterSize = 0; stopbits = 0;

	s = serial;
	s->fd = ::open( s->name, O_RDWR | O_NONBLOCK, 0 );

	if( s->fd ) {
		// for constants see sioLib.h
		// set baud rate
		ioctl( s->fd, FIOBAUDRATE, s->baud );
#if 1
		modemControl = CLOCAL | CREAD ;	// disable lines and enable receiver
		// set character size (default is 8 bits per character )
		switch(s->termios[0]) {
		case '7':
			characterSize = CSIZE | CS7;
			break;
		default:
		case '8':
			characterSize = CSIZE | CS8;
			break;
		};

		// set parity (default is no parity)
		switch(s->termios[1]) {
		case 'e':
		case 'E':
			parity = PARENB;	// default is even
			break;
		case 'o':
		case 'O':
			parity = PARENB | PARODD;
			break;
		default:
		case 'n':
		case 'N':
			parity = 0;
			break;
		};

		// set stop bits
		switch(s->termios[2]) {
		case '2':
			stopbits = STOPB;
			break;
		default:
		case '1':
			stopbits = 0;	// this corresponds to 1 stop bit (default in vxworks)
			break;
		};

		int serialflags = 0;
		ioctl(s->fd, SIO_HW_OPTS_GET, (int)&serialflags);
		serialflags |= 0*modemControl | parity | characterSize | stopbits;

		/* not sure if this is right
		serialflags &= ~(IXON | IXOFF | IXANY);
		*/

		ioctl(s->fd, SIO_HW_OPTS_SET, serialflags);
			/*
		ioctl( s->fd, SIO_HW_OPTS_SET, modemControl |
			parity       |
			characterSize|
			stopbits);
			*/

		//ioctl( s->fd, FIOSETOPTIONS, OPT_TANDEM | OPT_MON_TRAP );

		//ioctl( s->fd, FIOFLUSH, 0 );
#endif

	}



	if(s->fd == 0 || s->fd == -1) {
		s->portIsOpen = 0;
		printf( " serial: Comport %s Open FAIL!\n", s->name );
	}
	else {
		s->portIsOpen = 1;
		printf( " serial: Comport %s Open Success %s (handle %d)\n", s->name, s->termios, s->fd );
	}

	return s->fd;
}

int PosixSerial::close()
{
	if(s->fd)
		::close(s->fd);
	s->fd = 0;
}

static void sec2tv(double t, struct timeval *tv) {
	tv->tv_sec = (long)t;
	tv->tv_usec = (long)( (t - tv->tv_sec)*1000000 );

};

int PosixSerial::read (void *buf, int nbytes)
{
	int newBytes;
	fd_set rfds;
    struct timeval tv;
    int retval;
	double t = s->blockTimeout; // make it shorter for easier reference

	if( nbytes <= 0 ) return 0;

	if(s->blockingRead) {
		s->blockDeadlineExpired = 0;
		// Watch s->fdto see when it has input.
		FD_ZERO(&rfds);
		FD_SET(s->fd, &rfds);
		if(s->blockTimeout > 0 ) {
			// give timeout is > 0 so set timeout to that
			tv.tv_sec = (long)t;
			tv.tv_usec = (long)( (t - tv.tv_sec)*1000000 );
			retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
		} else {
			// given time out < 0 implying wait for ever (this does not work verywell)
			t = 99999;
			tv.tv_sec = (long)t;
			tv.tv_usec = (long)( (t - tv.tv_sec)*1000000 );
			retval = select(FD_SETSIZE, &rfds, NULL, NULL, NULL);
		}
		if(retval == 0)
			s->blockDeadlineExpired = 1;
		else
			s->blockDeadlineExpired = 0;

	}

	ioctl( s->fd, FIONREAD, (int)(&newBytes) );
	if( newBytes ) {
		newBytes = MIN( newBytes, nbytes);
		::read( s->fd, (char*)buf, newBytes );
		s->received += MAX( 0, newBytes );
	}

	return newBytes;
}

int PosixSerial::write(void *buf, int nbytes)
{
	int written;
	written = ::write( s->fd, (char*)buf, nbytes );
	/*tcflush(s->fd, TCOFLUSH);*/
	if( written != nbytes ) {
		s->dropwrite += nbytes - written;
		printf( " serial: write error on %s\n", s->name );
	}
	s->sent += written;
	return written;
}




#endif // posix



#ifdef WIN32

#define UCHARP unsigned char*
/* Reader func */
DWORD  readerFunc (LPVOID  arg) {

	COMSTAT comstat;
	DWORD   comerr;
	DWORD   bytestoread;
	struct serialPort_ref *s = (struct serialPort_ref *)(arg);
	WinSerial* ws = (WinSerial*)(s->serialDevice);


	while(ws->runReader)
	{
		/* Check the Input buffer  */
		ClearCommError( (HANDLE)(s->fd), &comerr, &comstat);
		if( comstat.cbInQue > 0 ) {
			bytestoread = MIN(comstat.cbInQue, BUFFERSIZE);
			ReadFile( (HANDLE)(s->fd), ws->readbuf, bytestoread, &(ws->ireadbuf), NULL );
			s->received += ws->ireadbuf;

			WaitForSingleObject( ws->readLock, INFINITE);
			/* see if newly read stuff will fit */
			if(ws->ireadablebuf + ws->ireadbuf <= BUFFERSIZE)
			{
				memcpy(ws->readablebuf + ws->ireadablebuf, ws->readbuf, ws->ireadbuf);
				ws->ireadablebuf += ws->ireadbuf;
				ws->ireadbuf = 0;
			}
			else
			{
				// normally you would move as much as is necessary but
				// here if tacking on newly read bytes will result in buffer overflow
				// we just tack it on the beginning ofthe buffer
				memcpy(ws->readablebuf, ws->readbuf, ws->ireadbuf);
				s->dropread += ws->ireadablebuf;
				ws->ireadablebuf = ws->ireadbuf;
				ws->ireadbuf = 0;
			}
			ReleaseMutex( ws->readLock );

		} else {
			//SwitchToThread();
			ws->ireadbuf = 0;
		}

		SleepEx(10,1);		// max rate at which this thread runs
		ws->ireader++;
	}
	return 0;
}

/* Writer Func */
DWORD  writerFunc(LPVOID  arg) {

	DWORD byteswritten;
	struct serialPort_ref *s = (struct serialPort_ref *)arg;
	WinSerial *ws = (WinSerial*)(s->serialDevice);

	while(ws->runWriter)
	{

		WaitForSingleObject( ws->writeLock, INFINITE);
		if ( ws->iwriteablebuf > 0)
		{
			memcpy(ws->writebuf,ws->writeablebuf,ws->iwriteablebuf);
			ws->iwritebuf = ws->iwriteablebuf;
			ws->iwriteablebuf = 0;
		}
		else
		{
			ws->iwritebuf = 0;
		}
		ReleaseMutex( ws->writeLock );

		if(ws->iwritebuf > 0)
		{
			WriteFile( (HANDLE)s->fd, ws->writebuf, ws->iwritebuf, &byteswritten, 0 );
			if(byteswritten != ws->iwritebuf)
			{
				s->dropwrite += ws->iwritebuf - byteswritten;
				ws->iwritebuf = 0;
			}
			s->sent += byteswritten;
		}

		SleepEx(10,1);	// max rate at which this thread runs
		ws->iwriter++;
	}
	return 0;
}





WinSerial::WinSerial()
{
	ireader = 0;
	iwriter = 0;
	runReader = 0;
	runWriter = 0;
	readThread = 0;
	writeThread = 0;
	readLock = 0;
	writeLock = 0;
	iwriteablebuf = 0;
	ireadbuf = 0;
	iwritebuf = 0;
	ireadablebuf = 0;
	s = NULL;
}


int WinSerial::open(serialPort_ref *serial)
{
	s = serial;

	/********* DO COM PORT OPENING STUFF HERE */
	DWORD dcbbaud;
	char fn0[20], fn[10];
	DCB     dcb;
	char mutexname[50];

	/* Range check the Comport number */
	if( s->port > 255 )
		return -1;

	/* Make a COMPORT name from a number */
	sprintf( fn0, "\\\\.\\COM%i", s->port );
	sprintf( fn, "COM%i", s->port );
	sprintf( s->name, "%d", s->port );

	/* Convert the Baud Rate to the Com library define for the DCB */
	switch( s->baud ) {
	case 110:
		dcbbaud = 110;
		break;
	case 300:
		dcbbaud = CBR_300;
		break;
	case 600:
		dcbbaud = CBR_600;
		break;
	case 1200:
		dcbbaud = CBR_1200;
		break;
	case 2400:
		dcbbaud = CBR_2400;
		break;
	case 4800:
		dcbbaud = CBR_4800;
		break;
	case 9600:
		dcbbaud = CBR_9600;
		break;
	case 19200:
		dcbbaud = CBR_19200;
		break;
	case 38400:
		dcbbaud = CBR_38400;
		break;
	case 57600:
		dcbbaud = CBR_57600;
		break;
	case 115200:
		dcbbaud = CBR_115200;
		break;
	case 230400:
    	dcbbaud = 230400;
    	break;
	case 921600:
		dcbbaud = 921600;
		break;
	default:
		return -1;
	}

	s->fd = (int)CreateFile( fn0, GENERIC_READ | GENERIC_WRITE,
		0, NULL, OPEN_EXISTING, 0, NULL );

	if(s->fd == 0 || (HANDLE)(s->fd) == INVALID_HANDLE_VALUE) {
		s->portIsOpen = 0;
		printf( " serial: Comport %d Open FAIL!\n", s->port );
		return -1;
	} else {
		s->portIsOpen = 1;
		printf( " serial: Comport %02d Open Success %s\n", s->port, s->termios);
	}


	/* The SetupComm() function establishes the Transmit and Receive buffer sizes.*/
	SetupComm( (void*)s->fd, 2*BUFFERSIZE, 2*BUFFERSIZE );

	/* Obtain the current DCB structure. this can be saved away for restore  8N1 */
	GetCommState( (void*)s->fd, &dcb );
	dcb.BaudRate = dcbbaud;

	// set character size (default is 8 bits per character )
	switch(s->termios[0]) {
	case '7':
		dcb.ByteSize = 7;
		break;
	default:
	case '8':
		dcb.ByteSize = 8;
		break;
	};

	// set parity (default is no parity )
	switch(s->termios[1]) {
	case 'e':
	case 'E':
		dcb.Parity = EVENPARITY;
		break;
	case 'o':
	case 'O':
		dcb.Parity = ODDPARITY;
		break;
	default:
	case 'n':
	case 'N':
		dcb.Parity = NOPARITY;
		break;
	};

	// set stop bits
	switch(s->termios[2]) {
	case '2':
		dcb.StopBits = TWOSTOPBITS;
		break;
	default:
	case '1':
		dcb.StopBits = ONESTOPBIT;	// this corresponds to 1 stop bit (default in vxworks)
		break;
	};

	/* turn off flow control */
	dcb.fOutX = FALSE;
	dcb.fInX  = FALSE;

	/* Configure the comport with our new DCB */
	SetCommState( (HANDLE)(s->fd), &dcb );
	SetCommMask( (HANDLE)(s->fd), EV_TXEMPTY );






	/******** THIS CREATES READER AND WRITER THREADS ********/

	// Create reader thread and suspend it
    this->readThread = CreateThread(
        NULL,								/* no security attributes  */
        0,									/* use default stack size  */
        (LPTHREAD_START_ROUTINE)readerFunc,               /*thread function */
        s,							/* argument to thread function */
        CREATE_SUSPENDED,           /* create it suspended */
        NULL);				/* returns the thread identifier */

	// Create writer thread and suspend it
    this->writeThread = CreateThread(
        NULL,                        /* no security attributes */
        0,                           /* use default stack size  */
        (LPTHREAD_START_ROUTINE)writerFunc,               /* thread function */
        s,							/* argument to thread function */
        CREATE_SUSPENDED,			/* create it suspended */
        NULL);			/* returns the thread identifier */

	if(this->readThread == NULL || this->writeThread == NULL)
	{
		printf(" WinSerial::open: error creating serial port thread for %s\n",fn);
		return -1;
	}


	// Create a mutex objects for controlling reading and writing to the work buffers
	sprintf(mutexname,"ReadProtection%s",fn);
	this->readLock = CreateMutex( NULL,	/* no security attributes */
		FALSE,							/* initially not owned */
		mutexname/*"ReadProtection"*/);						/* name of mutex */

	sprintf(mutexname,"WriteProtection%s",fn);
	this->writeLock = CreateMutex(NULL,	/* no security attributes */
		FALSE,							/* initially not owned */
		mutexname/*"WriteProtection"*/);						/* name of mutex */

	if(this->readLock == NULL || this->writeLock == NULL)
	{
		printf(" WinSerial::open: error creating serial port lock mutexes for %s\n", fn);
		return -1;
	}

	this->runReader = 1;
	this->runWriter = 1;

	ResumeThread(this->readThread);
	ResumeThread(this->writeThread);

	return s->fd;
}

int WinSerial::close()
{
	this->runReader = 0;
	this->runWriter = 0;
	SleepEx(500,0);	// maybe 500 ms is too long this allows threads to exit
	if(this->readThread) CloseHandle(this->readThread);
	if(this->writeThread) CloseHandle(this->writeThread);
	if(this->readLock) CloseHandle(this->readLock);
	if(this->writeLock) CloseHandle(this->writeLock);
	if(this->s->fd) CloseHandle((HANDLE)s->fd);
	this->readThread = NULL;
	this->writeThread = NULL;
	this->readLock = NULL;
	this->writeLock = NULL;
	s->fd = 0;
	return 0;
}

int WinSerial::read (void *buf, int nbytes)
{
	int ret;

	if( nbytes <= 0 ) return 0;

	WaitForSingleObject( this->readLock, INFINITE);
	if(this->ireadablebuf > 0)
	{
		if(this->ireadablebuf > (DWORD)nbytes)
		{
			ret = nbytes;
			memcpy(buf, this->readablebuf, ret);
			memmove(this->readablebuf, (UCHARP)this->readablebuf + ret, this->ireadablebuf - ret);
			this->ireadablebuf -= ret;

		}
		else
		{
			ret = this->ireadablebuf;
			memcpy(buf, this->readablebuf, ret);
			this->ireadablebuf = 0;
		}

	}
	else
	{
		this->ireadablebuf= 0; /* set to 0 should not be necessary but do anyway, just in case */
		ret = 0;
	}
	ReleaseMutex( this->readLock);

	return ret;
}

int WinSerial::write(void *buf, int nbytes)
{
	int ret;
	WaitForSingleObject( this->writeLock, INFINITE);

	/* if writing will overlflow the buffer */
	if(this->iwriteablebuf + nbytes > BUFFERSIZE)
	{
		/* This case we drop the most recent packet */
		s->dropwrite += nbytes;
		ret = 0;

	}
	else
	{
		ret = nbytes;
		memcpy((UCHARP)this->writeablebuf + this->iwriteablebuf, buf, ret);
		this->iwriteablebuf += ret;
	}

	ReleaseMutex( this->writeLock );
	return ret;

}

#endif //win32


#ifdef WIN32
int WinBsdUDP::winsockInitialized = 0;
#endif

WinBsdUDP::WinBsdUDP() {
	sd = NULL;
	ma = (struct sockaddr_in*)calloc(1,sizeof(struct sockaddr_in));
	ra = (struct sockaddr_in*)calloc(1,sizeof(struct sockaddr_in));
	receivedFrom = (struct sockaddr_in*)calloc(1,sizeof(struct sockaddr_in));
}

WinBsdUDP::~WinBsdUDP() {
	if(ma) free(ma);
	if(ra) free(ra);
	if(receivedFrom) free(receivedFrom);
}


int WinBsdUDP::getaddr(struct sockaddr_in *ad, char *hostname, unsigned short port,
						struct sockaddr_in* dest)
{
#ifdef WIN32
	struct hostent *hp;
	struct in_addr testAddr;
	memset(ad, 0, sizeof(struct sockaddr_in)); /* clear our address */
	hp = gethostbyname(hostname);                 /* get our address info */
	if (hp == NULL) {                             /* we don't exist !? */
		printf(" WinBsdUDP::getaddr Lookup of %s failed\n",hostname);
		return(-1);
	}
	if( dest == NULL )
	{
		// Grab the default
		memcpy((char *)(&(ad->sin_addr)), hp->h_addr, hp->h_length);   /* set address */
	}
	else if(dest->sin_addr.s_addr == inet_addr("127.0.0.1"))
	{
		ad->sin_addr.s_addr = inet_addr("127.0.0.1");
	}
	else
	{
		bool found = false;
		for(int index = 0; ((hp->h_addr_list[index] != 0) && !found); ++index)
		{
			memcpy(&testAddr, hp->h_addr_list[index], sizeof(struct in_addr));
			// Look for first 3 octets to line up (note: we should really determine the netmask if we can)
			if((testAddr.S_un.S_un_b.s_b1 == dest->sin_addr.S_un.S_un_b.s_b1) &&
				(testAddr.S_un.S_un_b.s_b2 == dest->sin_addr.S_un.S_un_b.s_b2) &&
				(testAddr.S_un.S_un_b.s_b3 == dest->sin_addr.S_un.S_un_b.s_b3))
			{
				// Use this address
				memcpy((char *)(&(ad->sin_addr)), hp->h_addr_list[index], sizeof(struct in_addr));   /* set address */
				found = true;
			}
		}
		if( !found )
		{
			// Use the first (default) address
			memcpy((char *)(&(ad->sin_addr)), hp->h_addr, hp->h_length);   /* set address */
		}
	}
	ad->sin_family = hp->h_addrtype;                /* this is our host address */
	ad->sin_port   = htons(port);					/* this is our port number */

	return 0;
#else
	memset(ad, 0, sizeof(struct sockaddr_in)); /* clear our address */
	ad->sin_len = sizeof(struct sockaddr_in);
	ad->sin_family = AF_INET;
	ad->sin_addr.s_addr = hostGetByName(hostname);
	ad->sin_port = htons(port);
	return 0;
#endif
}



int WinBsdUDP::open(struct serialPort_ref *serial) {

	unsigned long one = 1;

	// save the serail structure
	s = serial;

#ifdef WIN32
	// Do one time initialization of winsock, the flag is static
	WSADATA info;
	if(!WinBsdUDP::winsockInitialized)
	{
		if (WSAStartup(MAKEWORD(1,1), &info) != 0)
			MessageBox(NULL, "WinBsdUDP::open Cannot initialize WinSock!", "WSAStartup", MB_OK);
	}
#endif

	// formulate addresses of my port and remote ports
	if(s->isServer) {
		s->myport	  = BSD_BASE_PORT + s->portNum + 1000;
		s->remoteport = BSD_BASE_PORT + s->portNum;
	}
	else {
		int tmp = BSD_BASE_PORT;
		s->myport     = BSD_BASE_PORT + s->portNum;
		s->remoteport = BSD_BASE_PORT + s->portNum + 1000;
	}

	// is isServer is actually set to 99 then set myport = remoteport = portNum
	if( s->isServer == 99 ) {
		s->myport     = s->portNum;
		s->remoteport = s->port;  
	}

	if(s->connectionMode == MODE_TCP) {
		s->remoteport = s->portNum;
	}

	if( !s->setHostnameManually )
    	gethostname(s->myname,sizeof(s->myname));

	strcpy(s->remotename,s->connectTo);

	// get their sockaddr's
	if (getaddr(ra, s->remotename, s->remoteport) != 0)
	{
		printf(" serial: Cannot get remote port address\n");
		return -1;
	}
	if( getaddr(ma,s->myname,s->myport, ra) != 0 )
	{
		printf(" serial: Cannot get local port address\n");
		return -1;
	}

	// create the socket
	if(s->connectionMode == MODE_TCP) {
		sd = socket(AF_INET, SOCK_STREAM, 0);
	} else {
		sd = socket(AF_INET, SOCK_DGRAM, 0);
	}
#ifdef WIN32
	if (sd == INVALID_SOCKET)
#else
	if (sd == ERROR )
#endif
	{
		printf(" WinBsdUDP::open Cannot make socket\n");
		return -1;
	}

	// make socket nonblocking
	if( s->blocking != 1 ) {
#ifdef WIN32
		ioctlsocket(sd, FIONBIO , (unsigned long*)&one);
#else
		int err = ioctl(sd, FIONBIO, (int)&one);
		int errcode = ERROR;
#endif
	}

	if(s->connectionMode == MODE_TCP) {
		if(!this->tryConnect()) {
			printf( " serial: Comport still waiting (TCP attempting to connect to %s:%d)\n", s->connectTo, s->portNum );
		}

	} else {
		// bind the socket to the internet address
#ifdef WIN32
		if (bind(sd, (struct sockaddr *)ma, sizeof(struct sockaddr_in)) == 	SOCKET_ERROR)
#else
		if (bind(sd, (struct sockaddr *)ma, sizeof(struct sockaddr_in)) == 	ERROR)
#endif
		{
			this->closeSocket();
			printf(" WinBsdUDP::open Cannot bind socket to %s:%d\n",s->myname,s->myport);
			return(-1);
		}

		printf( " ether: Comport %02d Open Success (UDP Sockets listening at %s:%d to %s:%d)\n",
			s->port,s->myname,s->myport,s->remotename,s->remoteport);

/* Restricts from where the data is allowed to come.
#ifdef WIN32
			if (connect(sd, (struct sockaddr *)ra, sizeof(struct sockaddr_in)) == 	SOCKET_ERROR)
#else
			if (connect(sd, (struct sockaddr *)ra, sizeof(struct sockaddr_in)) == 	ERROR)
#endif
			{
				this->closeSocket();
				printf(" WinBsdUDP::open Cannot connect socket to %s:%d\n",remotename,remoteport);
				return(-1);
			}
*/
	}
	s->portIsOpen = 1;

	return 0;
}


int WinBsdUDP::close() {
	this->closeSocket();
	s->portIsOpen = 0;
	s->connectState = 0;
	return 0;
}


void WinBsdUDP::closeSocket() {
#ifdef WIN32
	closesocket(sd);
#else
	::close(sd);
#endif
	sd = NULL;
}

int WinBsdUDP::handleSockError()
{
	int err;
#ifdef WIN32
	err = WSAGetLastError();
	switch(err)
	{
	case WSAEADDRNOTAVAIL:
		printf(" WinBsdUDP: Requested address is not valid\n");
		break;
	case WSAENOTCONN:
		printf(" WinBsdUDP: Not connected\n");
		break;
	case WSAENETRESET:
		printf(" WinBsdUDP: Connection has been broken\n");
		break;
	case WSAESHUTDOWN:
		printf(" WinBsdUDP: Socket already shut down\n");
		break;
	case WSAEINVAL:
		printf(" WinBsdUDP: An invalid argument was supplied\n");
		break;
	case WSAECONNABORTED:
		printf(" WinBsdUDP: Aborted\n");
		break;
	case WSAETIMEDOUT:
		printf(" WinBsdUDP: Timeout\n");
		break;
	case WSAEFAULT:
		printf(" WinBsdUDP: Detected an invalid pointer address in attempting to use a pointer argument in a call\n");
		break;
	case WSAECONNRESET:
		//this->closeSocket();
		printf(" WinBsdUDP: Connection was closed by the remote host\n");
		break;
	default:
		break;
	};

#else

	err = errno;

#endif

	return err;

}


int WinBsdUDP::read (void *buf, int nbytes) {
	unsigned long toread;
	int ret;
	int sizeofsockaddr = sizeof(struct sockaddr_in);
	int status;

	fd_set rfds;
    struct timeval tv;
    int retval;
	double t = s->blockTimeout; // make it shorter for easier reference

	if(!this->tryConnect()) return 0;

	if(s->blockingRead) {
		// Watch s->fdto see when it has input.
		FD_ZERO(&rfds);
		FD_SET(sd, &rfds);
		if(s->blockTimeout > 0 ) {
			// give timeout is > 0 so set timeout to that
			tv.tv_sec = (long)t;
			tv.tv_usec = (long)( (t - tv.tv_sec)*1000000 );
			retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
		} else {
			// given time out < 0 implying wait for ever (this does not work verywell)
			t = 99999;
			tv.tv_sec = (long)t;
			tv.tv_usec = (long)( (t - tv.tv_sec)*1000000 );
			retval = select(FD_SETSIZE, &rfds, NULL, NULL, NULL);
		}
		if(retval == 0)
			s->blockDeadlineExpired = 1;
		else
			s->blockDeadlineExpired = 0;


	}

	// if socket is connected
	if(s->portIsOpen) {

#ifdef WIN32
		status = ioctlsocket(sd, FIONREAD, &toread);
		if(status == SOCKET_ERROR) {
			printf(" WinBsdUDP: problem receiving from socket\n");
			return 0;
		}
#else
		status = ioctl(sd,FIONREAD,(int)&toread);
		if(status == ERROR) {
			printf("\nproblem receiving from socket");
			return 0;
		}
#endif

		//ret = recvfrom(sd, (char*)buf, MIN(toread,(unsigned long)nbytes),  0, (struct sockaddr*)ra, &sizeofsockaddr );
		if(s->connectionMode == MODE_TCP) {
			ret = recv(sd,(char*)buf, MIN(toread,(unsigned long)nbytes),  0);
		} else {
			ret = recvfrom(sd, (char*)buf, MIN(toread,(unsigned long)nbytes),  0, (struct sockaddr*)receivedFrom, &sizeofsockaddr );
		}


#ifdef WIN32
		if (ret == SOCKET_ERROR)
#else
		if (ret == ERROR)
#endif
		{
			s->errorNumberRx = this->handleSockError();
			return 0;
		}
		else {
			s->received += ret;
			return ret;
		}
	}
	return 0;
}

int WinBsdUDP::write(void *buf, int nbytes) {
	int ret;

	if(!this->tryConnect()) return 0;

	// if socket is connected
	if(s->portIsOpen) {
		if(s->connectionMode == MODE_TCP) {
			ret = send(sd,(char*)buf, nbytes,	0);
		} else {
			ret = sendto(sd, (char*)buf, nbytes,	0, (struct sockaddr*)ra,	sizeof(struct sockaddr_in) );
		}
#ifdef WIN32
		if (ret == SOCKET_ERROR)
#else
		if (ret == ERROR)
#endif
		{
			s->errorNumberTx = this->handleSockError();
			s->dropwrite += nbytes;
			return 0;
		}
		else {
			s->sent += ret;
			return ret;
		}
	}
	return 0;
}

bool WinBsdUDP::tryConnect() {

	if( s->connectionMode == MODE_TCP ) {
		if( s->connectState == 1 ) {
			return true;
		} else {
			int ret = ::connect(sd,(const sockaddr*)(ra),sizeof(struct sockaddr_in));
			if(ret == SOCKET_ERROR) {
				s->errorNumberRx = WSAGetLastError();
				if(this->isBlockingError()) {
					return false; // still connecting
				} else if(this->isAlreadyConnectedError()) {
					printf( " serial: Comport Open Success (TCP Sockets connected to %s:%d)\n",s->connectTo,s->portNum);
					s->connectState = 1;
					return true;
				} else if(this->isConnReset()) {
					printf(" serial: TCP Connection to %s:%d was reset, so closing port\n",s->connectTo,s->portNum);
					this->close();
					return false;
				}
			} else {
				printf( " serial: Comport Open Success (TCP Sockets connected to %s:%d)\n",s->connectTo,s->portNum);
				s->connectState = 1;
				return true;
			}
			return false;
		}
	} else {
		if( s->portIsOpen ) {
			return true;
		} else {
			return false;
		}
	}

}

bool WinBsdUDP::isAlreadyConnectedError() {
	if(WSAGetLastError() == WSAEISCONN) {
		return true;
	} else {
		return false;
	}
}
bool WinBsdUDP::isBlockingError() {
	switch (WSAGetLastError()) {
	case WSAEWOULDBLOCK: // always == NET_EAGAIN?
	case WSAEALREADY:
	case WSAEINPROGRESS:
		return true;
	}
	return false;
}

bool WinBsdUDP::isConnReset() {
	switch (WSAGetLastError()) {
	case WSAECONNRESET:
		return true;
	}
	return false;
}
