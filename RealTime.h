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
	DWORD  step;					// Шаг в миллисекундах
	double time;					// Внутреннее время, мс
	double Step;					// Шаг в сек
	double timeS;					// Внутреннее время, с
	double dT;						// Время вычисления на шаге
	long int Nteak;					// Количество тиков
	int SetEvent;
	int pExchOK;					// Признак успешного обмена

	REALTIME();
};

extern REALTIME rt;
extern CRITICALSECTIONS crts;

#pragma comment(lib,"winmm.lib")

#endif