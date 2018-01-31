#include <math.h>
#include <winsock.h>
#include <stdio.h>
#include "Net.h"

NETVOICE nv;

double getTimeMs()
{
	static long double Sc = 0;
	LARGE_INTEGER cnt;
	if(Sc == 0){
		LARGE_INTEGER fr;
		if(QueryPerformanceFrequency(&fr) == 0)
			return GetTickCount();
		Sc = 1000/(long double)fr.QuadPart;
	}
	QueryPerformanceCounter(&cnt);
	return (long double)cnt.QuadPart*Sc;
}

void delayMs(double ms)
{
	double t0 = getTimeMs();
	while(1) {
		if((getTimeMs()-t0) >= ms)
			break;
	}
}

int NETVOICE::Select(timeval *timeout)
{
	fd_set rfd;
	int max_fd;
	FD_ZERO(&rfd);
	max_fd = 0;
	FD_SET(sock,&rfd);
	if((int)sock > max_fd)
		max_fd = (int)sock;
	return select(max_fd + 1,&rfd,NULL,NULL,timeout);
}

bool NETVOICE::Wait(DWORD ms)
{
	timeval timeout = {0,ms*1000};
	return Select(&timeout) > 0;
}

bool InitNetVoice(void *fr, int lenFr)
{
	return nv.InitNet(fr, lenFr);
}

bool NETVOICE::InitNet(void *fr, int bufl)
{

	char str[256];
	char tIP[20];
	int port;

	FILE *fi = fopen("init.txt", "r");
		fgets(str, 256, fi);
		strcpy(tIP, str);
		fgets(str, 256, fi);
		sscanf(str, "%d", &port);
	fclose(fi);	
	
	addrsize = sizeof(addr);
	buf = (char *)fr;
	buflen = bufl;

	
	int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
	if (iResult != NO_ERROR) 
		return false;

	
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET) {
		WSACleanup();
		return false;
	}
	
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = inet_addr(tIP);
	int aaa;
	aaa = bind(sock, (SOCKADDR *) &addr, sizeof(addr));

	return true;
}

void StopNetVoice()
{
	nv.StopNet();
}

void NETVOICE::StopNet()
{
	closesocket(sock);
	WSACleanup();
}

bool NetReceiveVoice()
{
	return nv.NetReceiveUDP();
}

double getTimeMs();

bool NETVOICE::NetReceiveUDP()
{
	double tm0 = getTimeMs();
	int retS;
	if(Wait(1)) {
		retS = recvfrom(sock, buf, buflen, 0, (SOCKADDR *)&addr, &addrsize);
		if(retS != buflen)
			return false;
		return true;
	}
	return false;
}


