#include <stdio.h>
#include <conio.h>
#include <math.h>
#include "RealTime.h"
#include "Net.h"
#include "Memory.h"

REALTIME rt;

CRITICALSECTIONS crts;

CRITICALSECTIONS::CRITICALSECTIONS() {
	InitializeCriticalSection(&cs); 
}

CRITICALSECTIONS::~CRITICALSECTIONS()
{
	DeleteCriticalSection(&cs);
}

void CRITICALSECTIONS::Lock() {
		EnterCriticalSection(&cs);
}

void CRITICALSECTIONS::Unlock() {
	LeaveCriticalSection(&cs);
};


REALTIME::REALTIME()
{
	timeS = 0;
}

void CALLBACK RealTimeFunc(UINT,UINT,DWORD st,DWORD,DWORD)
{
	static int flag=1;
	double tm0 = getTimeMs();

	crts.Lock();

	rt.pExchOK = NetReceiveVoice();

	memcpy(&shaData->sr, (void*)&soundFFT, sizeof(SOUNDFFT));

	rt.timeS += rt.Step;

	crts.Unlock();
	double tm1 = getTimeMs();
	rt.dT = tm1 - tm0;
}

void InitRealTime(DWORD  STEP)
{
	rt.step = STEP;
	rt.Step = rt.step * 0.001;
	rt.timeS = 0;
	rt.SetEvent = timeSetEvent(rt.step,1,RealTimeFunc,rt.step,TIME_PERIODIC);
	
}

void StopRealTime()
{
	if(rt.SetEvent){
		timeKillEvent(rt.SetEvent);
		rt.SetEvent = 0;
	}
}
