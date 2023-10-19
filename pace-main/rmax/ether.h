#ifndef rmax_ether_h
#define rmax_ether_h

/** Must declare with C linkage because it is serial.c not .cpp */

#ifdef __cplusplus
extern "C" {
#endif


	int openCom(struct serialPort_ref *s);
	int closeCom(struct serialPort_ref *s);
	int readCom(struct serialPort_ref *s, void *buf, int nbytes);
	int writeCom(struct serialPort_ref *s, void *buf, int nbytes);

	int openSock(struct serialPort_ref *s);
	int closeSock(struct serialPort_ref *s);
	int readSock(struct serialPort_ref *s, void *buf, int nbytes);
	int writeSock(struct serialPort_ref *s, void *buf, int nbytes);

#ifdef __cplusplus
}
#endif

class SerialDevice
{
public:
	virtual ~SerialDevice() {}
	virtual int open(struct serialPort_ref *s) = 0;
	virtual int close() = 0;
	virtual int read (void *buf, int nbytes) = 0;
	virtual int write(void *buf, int nbytes) = 0;
	struct serialPort_ref *s;
};


class Bsd : public SerialDevice
{

};


#ifdef WIN32
#include <windows.h>
class WinSerial : public SerialDevice
{
public:
	WinSerial();
	virtual ~WinSerial() {}
	int open(struct serialPort_ref *s);
	int close();
	int read (void *buf, int nbytes);
	int write(void *buf, int nbytes);

	long ireader;
	long iwriter;
	int runReader;
	int runWriter;
	HANDLE readThread;
	HANDLE writeThread;
	HANDLE readLock;
	HANDLE writeLock;
	DWORD iwriteablebuf;
	DWORD ireadbuf;
	DWORD iwritebuf;
	DWORD ireadablebuf;
	unsigned char readablebuf[BUFFERSIZE];
	unsigned char writeablebuf[BUFFERSIZE];
	unsigned char readbuf[BUFFERSIZE];
	unsigned char writebuf[BUFFERSIZE];
};

#endif //win32

#ifdef POSIX
class PosixSerial : public SerialDevice
{
public:
	virtual ~PosixSerial() {}
	int open(struct serialPort_ref *s);
	int close();
	int read (void *buf, int nbytes);
	int write(void *buf, int nbytes);
};

#endif //posix

//#endif // c++


class WinBsdUDP : public Bsd
{
public:
	WinBsdUDP();
	virtual ~WinBsdUDP();
	int open(struct serialPort_ref *s);
	int close();
	int read (void *buf, int nbytes);
	int write(void *buf, int nbytes);

#ifdef WIN32
	SOCKET sd;	// socket descriptor for doing actual talking
	static int winsockInitialized;
#else
	int sd;
#endif

	int handleSockError();
	// Provide dest if using a multi-homed host in order to match to the correct NIC
	int getaddr(struct sockaddr_in *ad, char *hostname, unsigned short port, struct sockaddr_in* dest = NULL);
	void closeSocket();
	bool tryConnect();
	bool isAlreadyConnectedError();
	bool isBlockingError();
	bool isConnReset();

	struct sockaddr_in *ma;	// my address
	struct sockaddr_in *ra;	// remote address
	struct sockaddr_in *receivedFrom;
};




#endif // rmax_ether_h

