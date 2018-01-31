#ifndef __RUN_TIME__
#define __RUN_TIME__

#pragma	once

#include <WinSock.h>
//#include "Def.h"

void InitRealTime(DWORD  Step);
void StopRealTime();

void delayMs(double ms);
double getTimeMs();

class CRITICALSECTIONS {
	CRITICAL_SECTION cs;
public:
	CRITICALSECTIONS();
	~CRITICALSECTIONS();
	void Lock();
	void Unlock();
};

class REALTIME {
public:
	DWORD  step;					// ��� � �������������
	double time;					// ���������� �����, ��
	double Step;					// ��� � ���
	double timeS;					// ���������� �����, �
	double dT;						// ����� ���������� �� ����
	long int Nteak;					// ���������� �����
	int SetEvent;
	int pExchOK;					// ������� ��������� ������

	REALTIME();
};

extern REALTIME rt;
extern CRITICALSECTIONS crts;

#pragma comment(lib,"winmm.lib")

#endif