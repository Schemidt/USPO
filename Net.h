#ifndef __NET_VOICE_H__
#define __NET_VOICE_H__

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <WinSock.h>

class NETVOICE {
	WSADATA wsaData;
	sockaddr_in addr;
	int addrsize;
	int port;
	char *buf;
	int buflen;
	SOCKET sock;
public:

	int Select(timeval *timeout);
	bool Wait(DWORD ms);
	bool InitNet(void *fr, int lenFr);
	void StopNet();
	bool NetReceiveUDP();
};

bool InitNetVoice(void *fr, int lenFr);
void StopNetVoice();
bool NetReceiveVoice();

#endif