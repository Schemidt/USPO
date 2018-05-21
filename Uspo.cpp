#include <stdio.h>
#include <tchar.h>
#include "Uspo.h"
#include "Memory.h"
#include "Net.h"
#include "RealTime.h"
#include "fstream"
#include "Helicopter.h"
#include "regex"
#include "iostream"
#include "string.h"
#include "tlhelp32.h"

#define ANSAT_ENG_REV_TURN 54.00
#define ANSAT_1ENG_TURN 73.00
#define VSU_MAX_TURN 100.00


class point {
public:
	double x;
	double y;

	point()
	{
		x = 0;
		y = 0;
	}

	point(double x, double y)
	{
		this->x = x;
		this->y = y;
	}
};

class testChunk
{
public:
	int number;
	double start;
	double end;
};

void delayMs(double ms);

void kbHit();

double getTimeMs();

double interpolation(point p1, point p2, double x);

double interpolation(point p1, point p2, point p3, double x);

double getOffset(string filename, double parameter);

double getParameterFromFile(string filename, double offset);

double getParameterFromVector(vector<double> &value, vector<double> &time, double offset);

double getParameterFromVector(vector<point> &value, double offset);

int binSer(vector<double> &time, double offset);

int binSer(vector<point> &time, double offset);

using namespace std;

SOUNDFFT soundFFT;
Helicopter helicopter;

double test = 0;
double delta = 0;
double offsetTest = 0;
double testTimeEnd = 0;
double testTimeStart = 0;
double pause = 0;
bool timeReset = 0;
bool vectload = 0;

string statusEng1;
string statusEng2;
string statusRed;

double turn = 0;
double turnMg1 = 0;
double turnMg2 = 0;
double turnAvt = 0;
double turnMgEng1 = 0;
double turnAvtEng1 = 0;
double turnMgEng2 = 0;
double turnAvtEng2 = 0;
double turnRevRedAnsat = 0;
double offsetMgEng1 = 0;
double offsetAvtEng1 = 0;
double offsetMgEng2 = 0;
double offsetAvtEng2 = 0;
double offsetMg1 = 0;
double offsetMg2 = 0;
double offsetAvt = 0;
double offsetAnsatRev = 0;
bool standAlone = 0;//По умолчанию - ждем данных от модели

bool vsuOn = 0;
bool vsuHp = 0;
bool vsuOff = 0;

bool Eng1On = 0;
bool Eng1Hp = 0;
bool Eng1Off = 0;

bool Eng2On = 0;
bool Eng2Hp = 0;
bool Eng2Off = 0;

bool eng1hpbl = 0;
bool eng2hpbl = 0;
bool vsuhpbl = 0;

int main(int argc, char* argv[])
{
	//Только 1на копия приложения может быть запущена одновременно
	HANDLE hMutex = OpenMutex(
		MUTEX_ALL_ACCESS, 0, L"USPO");

	if (!hMutex)
		// Mutex doesn’t exist. This is
		// the first instance so create
		// the mutex.
		hMutex =
		CreateMutex(0, 0, L"USPO");
	else
		// The mutex exists so this is the
		// the second instance so return.
	{
		cout << "Application already exist!" << endl;
		return 1;
	}

	vector <string> helicoptersNames = { "mi_8_mtv5","mi_8_amtsh","mi_26","mi_28","ka_226","ansat","ka_27","ka_29" };
	string model;
	for (size_t i = 1; i < argc; i++)// если передаем аргументы, то argc будет больше 1(в зависимости от кол-ва аргументов)
	{
		for (size_t j = 0; j < helicoptersNames.size(); j++)
		{
			if (regex_match(argv[i], regex("^(" + helicoptersNames[j] + ")")))
			{
				model = argv[i];
				helicopter.setParam(model);
			}
		}
		if (argv[i] == (string)"standalone")
		{
			standAlone = 1;
		}
	}
	if (model.empty())
	{
		helicopter.setParam("ka_29");
	}
	system("cls");
	cout << " Using " << helicopter.modelName << endl;
	helicopter.setPath(helicopter.modelName + "/");

	//Cетевое взаимодействие
	if (!InitNetVoice((void*)&soundFFT, sizeof(SOUNDFFT)))
	{
		cout << "Not InitNetVoice" << endl;
		return 0;
	}

	soundFFT.reduktor_gl_obor = 0;
	soundFFT.eng1_obor = 0;
	soundFFT.eng2_obor = 0;
	soundFFT.master_gain = 1;
	soundFFT.time = 0;
	soundFFT.p_na_vpp = 1;
	soundFFT.p_eng1_rkorr = 0;
	soundFFT.p_eng2_rkorr = 0;
	soundFFT.p_eng1_lkorr = 1;
	soundFFT.p_eng2_lkorr = 1;
	soundFFT.p_eng1_ostanov = 1;
	soundFFT.p_eng2_ostanov = 1;

	if (!shaInit())				// Инициализация общей памяти 
		return 0;
	InitRealTime(1);

	bool hovering = 0;
	bool skv = 0;

	double currentTime = 0;
	double output = 0;

	vector<vector <point>> vectorPar(7);
	vector<point> vectorVy;

	while (true)
	{
		delta = rt.timeS - currentTime;
		currentTime = rt.timeS;

		if (!rt.pExchOK)
		{
			//Функция обработки нажатий клавиш	  
			kbHit();
		}

		//Код для автономных тестов - без модели вертолета 
		if (standAlone)
		{
			//Блок проверки запуска (и холодной прокрутки) двигателей, редуктора, всу
			if (!test)
			{
				//Условие автомата
				bool avtOn = soundFFT.p_eng2_rkorr & soundFFT.p_eng1_rkorr;
				if (eng1hpbl | eng2hpbl)
				{
					if (Eng1Hp)
					{
						if (soundFFT.eng1_obor < 25)
						{
							soundFFT.p_eng1_hp = 1;
						}
						else
						{
							soundFFT.p_eng1_hp = 0;
						}
						if (soundFFT.eng1_obor < 25)
							soundFFT.eng1_obor += 25 / 23. * (delta);
						soundFFT.eng1_obor = (soundFFT.eng1_obor > 25) ? 25 : soundFFT.eng1_obor;
					}
					if (Eng2Hp)
					{
						if (soundFFT.eng2_obor < 25)
						{
							soundFFT.p_eng2_hp = 1;
						}
						else
						{
							soundFFT.p_eng2_hp = 0;
						}
						if (soundFFT.eng2_obor < 25)
							soundFFT.eng2_obor += 25 / 23. * (delta);
						soundFFT.eng2_obor = (soundFFT.eng2_obor > 25) ? 25 : soundFFT.eng2_obor;
					}
					if (Eng1Off)
					{
						if (soundFFT.eng1_obor > 0)
						{
							soundFFT.p_eng1_ostanov = 1;
						}
						else
						{
							eng1hpbl = 0;
							soundFFT.p_eng1_ostanov = 0;
						}

						soundFFT.eng1_obor -= 25 / 35. * (delta);
						soundFFT.eng1_obor = (soundFFT.eng1_obor < 0) ? 0 : soundFFT.eng1_obor;
					}
					if (Eng2Off)
					{
						if (soundFFT.eng2_obor > 0)
						{
							soundFFT.p_eng2_ostanov = 1;
						}
						else
						{
							eng2hpbl = 0;
							soundFFT.p_eng2_ostanov = 0;
						}

						soundFFT.eng2_obor -= 25 / 35. * (delta);
						soundFFT.eng2_obor = (soundFFT.eng2_obor < 0) ? 0 : soundFFT.eng2_obor;
					}
				}
				else
				{
					if (helicopter.modelName == "ansat")
					{
						bool case1 = soundFFT.eng1_obor >= ANSAT_ENG_REV_TURN & soundFFT.eng2_obor >= ANSAT_ENG_REV_TURN;//Подъем или спуск
																														 //bool case2 = soundFFT.eng1_obor < ANSAT_ENG_REV_TURN & soundFFT.eng2_obor < ANSAT_ENG_REV_TURN;//Нормальный подъем
																														 //bool case3 = (soundFFT.eng1_obor < ANSAT_ENG_REV_TURN & soundFFT.eng2_obor >= ANSAT_ENG_REV_TURN) | (soundFFT.eng1_obor >= ANSAT_ENG_REV_TURN & soundFFT.eng2_obor < ANSAT_ENG_REV_TURN);//Нормальный подъем

																														 //Двигатель 1
						if (Eng1On)
						{
							if (!avtOn)
							{
								if (soundFFT.eng1_obor > helicopter.engTurnoverMg)
								{
									if ((statusEng1 != "eng_mg_avt") && (statusEng1 != "eng_avt_mg"))
									{
										if (soundFFT.eng2_obor > ANSAT_ENG_REV_TURN)
										{
											if (statusEng1 != "ansatFirstEng")
											{
												offsetMgEng1 = getOffset(helicopter.fullName["ansatFirstEng"], soundFFT.eng1_obor);
												statusEng1 = "ansatFirstEng";
											}
											turnMgEng1 = getParameterFromFile(helicopter.fullName["ansatFirstEng"], offsetMgEng1);
											offsetMgEng1 += delta;
										}
										else
										{
											if (statusEng1 != "eng_on")
											{
												offsetMgEng1 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng1_obor);
												statusEng1 = "eng_on";
											}
											turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng1);
											offsetMgEng1 += delta;
										}
									}
									else
									{
										if (statusEng1 != "eng_avt_mg")
										{
											offsetAvtEng1 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng1_obor);
											statusEng1 = "eng_avt_mg";
										}
										turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng1);
										offsetAvtEng1 += delta;
									}
								}
								else
								{
									if (case1)
									{
										if (statusEng1 != "ansatSecondEng")
										{
											offsetMgEng1 = getOffset(helicopter.fullName["ansatSecondEng"], soundFFT.eng1_obor);
											statusEng1 = "ansatSecondEng";
										}
										turnMgEng1 = getParameterFromFile(helicopter.fullName["ansatSecondEng"], offsetMgEng1);
										offsetMgEng1 += delta;
									}
									else
									{
										if (statusEng1 != "eng_on")
										{
											offsetMgEng1 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng1_obor);
											statusEng1 = "eng_on";
										}
										turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng1);
										offsetMgEng1 += delta;
									}
								}
							}
							else
							{
								if (statusEng1 != "eng_mg_avt")
								{
									offsetMgEng1 = getOffset(helicopter.fullName["ansatSecondEng"], soundFFT.eng1_obor);
									offsetAvtEng1 = getOffset(helicopter.fullName["eng_mg_avt"], soundFFT.eng1_obor);
									statusEng1 = "eng_mg_avt";
								}
								turnMgEng1 = getParameterFromFile(helicopter.fullName["ansatSecondEng"], offsetMgEng1);
								offsetMgEng1 += delta;
								turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng1) * turnMgEng1 / helicopter.engTurnoverMg;
								offsetAvtEng1 += delta;
							}
						}
						else if (Eng1Off & soundFFT.eng1_obor > 0)
						{
							if (statusEng1 != "eng_off")
							{
								offsetAvtEng1 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng1_obor);
								offsetMgEng1 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng1_obor);
								statusEng1 = "eng_off";
							}
							turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng1);
							offsetAvtEng1 += delta;
							turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng1) * turnAvtEng1 / helicopter.engTurnoverMg;
							offsetMgEng1 += delta;
						}
						//Двигатель 2
						if (Eng2On)
						{
							if (!avtOn)
							{
								if (soundFFT.eng2_obor > helicopter.engTurnoverMg)
								{
									if ((statusEng2 != "eng_mg_avt") && (statusEng2 != "eng_avt_mg"))
									{

										if (soundFFT.eng1_obor > ANSAT_ENG_REV_TURN)
										{
											if (statusEng2 != "ansatFirstEng")
											{
												offsetMgEng2 = getOffset(helicopter.fullName["ansatFirstEng"], soundFFT.eng2_obor);
												statusEng2 = "ansatFirstEng";
											}
											turnMgEng2 = getParameterFromFile(helicopter.fullName["ansatFirstEng"], offsetMgEng2);
											offsetMgEng2 += delta;
										}
										else
										{
											if (statusEng2 != "eng_on")
											{
												offsetMgEng2 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng2_obor);
												statusEng2 = "eng_on";
											}
											turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng2);
											offsetMgEng2 += delta;
										}
									}
									else
									{
										if (statusEng2 != "eng_avt_mg")
										{
											offsetAvtEng2 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng2_obor);
											statusEng2 = "eng_avt_mg";
										}
										turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng2);
										offsetAvtEng2 += delta;
									}
								}
								else
								{
									if (case1)
									{
										if (statusEng2 != "ansatSecondEng")
										{
											offsetMgEng2 = getOffset(helicopter.fullName["ansatSecondEng"], soundFFT.eng2_obor);
											statusEng2 = "ansatSecondEng";
										}
										turnMgEng2 = getParameterFromFile(helicopter.fullName["ansatSecondEng"], offsetMgEng2);
										offsetMgEng2 += delta;
									}
									else
									{
										if (statusEng2 != "eng_on")
										{
											offsetMgEng2 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng2_obor);
											statusEng2 = "eng_on";
										}
										turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng2);
										offsetMgEng2 += delta;
									}
								}
							}
							else
							{
								if (statusEng2 != "eng_mg_avt")
								{
									offsetMgEng2 = getOffset(helicopter.fullName["ansatSecondEng"], soundFFT.eng2_obor);
									offsetAvtEng2 = getOffset(helicopter.fullName["eng_mg_avt"], soundFFT.eng2_obor);
									statusEng2 = "eng_mg_avt";
								}
								turnMgEng2 = getParameterFromFile(helicopter.fullName["ansatSecondEng"], offsetMgEng2);
								offsetMgEng2 += delta;
								turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng2) * turnMgEng2 / helicopter.engTurnoverMg;
								offsetAvtEng2 += delta;
							}
						}
						else if (Eng2Off & soundFFT.eng2_obor > 0)
						{
							if (statusEng2 != "eng_off")
							{
								offsetAvtEng2 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng2_obor);
								offsetMgEng2 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng2_obor);
								statusEng2 = "eng_off";
							}
							turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng2);
							offsetAvtEng2 += delta;
							turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng2) * turnAvtEng2 / helicopter.engTurnoverMg;
							offsetMgEng2 += delta;
						}

						//Редуктор
						if (Eng1On | Eng2On)
						{
							if (avtOn)
							{
								if (statusRed != "red_mg_avt")
								{
									offsetMg1 = getOffset(helicopter.fullName["red_on"], soundFFT.reduktor_gl_obor);
									offsetAvt = getOffset(helicopter.fullName["red_mg_avt"], soundFFT.reduktor_gl_obor);
									statusRed = "red_mg_avt";
								}
								turnMg1 = getParameterFromFile(helicopter.fullName["red_on"], offsetMg1);
								offsetMg1 += delta;
								turnAvt = getParameterFromFile(helicopter.fullName["red_mg_avt"], offsetAvt) * turnMg1 / helicopter.redTurnoverMg1;
								offsetAvt += delta;
							}
							else
							{
								if ((statusRed != "red_mg_avt") && (statusRed != "red_avt_mg"))
								{
									if (case1)
									{
										if (statusRed != "ansatRed")
										{
											offsetMg1 = getOffset(helicopter.fullName["red_on"], soundFFT.reduktor_gl_obor);
											offsetAnsatRev = getOffset(helicopter.fullName["ansatRed"], soundFFT.reduktor_gl_obor);
											statusRed = "ansatRed";
										}
										turnRevRedAnsat = getParameterFromFile(helicopter.fullName["ansatRed"], offsetAnsatRev);
										turnMg1 = getParameterFromFile(helicopter.fullName["red_on"], offsetMg1) / helicopter.redTurnoverMg1 * turnRevRedAnsat;
										offsetAnsatRev += delta;
										offsetMg1 += delta;
									}
									else
									{
										if (statusRed != "red_on")
										{
											offsetMg1 = getOffset(helicopter.fullName["red_on"], soundFFT.reduktor_gl_obor);
											statusRed = "red_on";
										}
										turnMg1 = getParameterFromFile(helicopter.fullName["red_on"], offsetMg1);
										offsetMg1 += delta;
									}
								}
								else
								{
									if (statusRed != "red_avt_mg")
									{
										offsetAvt = getOffset(helicopter.fullName["red_avt_mg"], soundFFT.reduktor_gl_obor);
										statusRed = "red_avt_mg";
									}
									turnAvt = getParameterFromFile(helicopter.fullName["red_avt_mg"], offsetAvt);
									offsetAvt += delta;
								}
							}
						}
						else
						{
							if (statusRed != "red_off")
							{
								offsetAvt = getOffset(helicopter.fullName["red_avt_mg"], soundFFT.reduktor_gl_obor);
								offsetMg1 = getOffset(helicopter.fullName["red_off"], soundFFT.reduktor_gl_obor);
								statusRed = "red_off";
							}
							turnAvt = getParameterFromFile(helicopter.fullName["red_avt_mg"], offsetAvt);
							offsetAvt += delta;
							turnMg1 = getParameterFromFile(helicopter.fullName["red_off"], offsetMg1) * turnAvt / helicopter.redTurnoverMg1;
							offsetMg1 += delta;
						}

						if (statusEng1 == "ansatFirstEng" || statusEng1 == "eng_off" || statusEng1 == "eng_on" || statusEng1 == "ansatSecondEng")
							soundFFT.eng1_obor = turnMgEng1;
						if (statusEng1 == "eng_mg_avt" || statusEng1 == "eng_avt_mg")
							soundFFT.eng1_obor = turnAvtEng1;
						if (statusEng2 == "ansatFirstEng" || statusEng2 == "eng_off" || statusEng2 == "eng_on" || statusEng2 == "ansatSecondEng")
							soundFFT.eng2_obor = turnMgEng2;
						if (statusEng2 == "eng_mg_avt" || statusEng2 == "eng_avt_mg")
							soundFFT.eng2_obor = turnAvtEng2;

						if (statusRed == "ansatRed" || statusRed == "red_off" || statusRed == "red_on")
							soundFFT.reduktor_gl_obor = turnMg1;
						if (statusRed == "red_mg_avt" || statusRed == "red_avt_mg")
							soundFFT.reduktor_gl_obor = turnAvt;

						soundFFT.reduktor_gl_obor = (soundFFT.reduktor_gl_obor < 0) ? 0 : soundFFT.reduktor_gl_obor;
						soundFFT.eng1_obor = (soundFFT.eng1_obor < 0) ? 0 : soundFFT.eng1_obor;
						soundFFT.eng2_obor = (soundFFT.eng2_obor < 0) ? 0 : soundFFT.eng2_obor;
					}
					else
					{

						bool oneEng = (((Eng1On & (!Eng2On | Eng2Off))
							^ (Eng2On & (!Eng1On | Eng1Off)))
							| (Eng1On & Eng2On & soundFFT.eng1_obor < (helicopter.engTurnoverMg - 15)
								& soundFFT.eng2_obor < (helicopter.engTurnoverMg - 15)) | (((soundFFT.eng1_obor >= (helicopter.engTurnoverMg - 15)
									&& soundFFT.eng2_obor < (helicopter.engTurnoverMg / 2.5)) | (soundFFT.eng2_obor >= (helicopter.engTurnoverMg - 15)
										&& soundFFT.eng1_obor < (helicopter.engTurnoverMg / 2.5))) & Eng1On & Eng2On));
						bool twoEng = (((soundFFT.eng1_obor >= (helicopter.engTurnoverMg - 15)
							&& soundFFT.eng2_obor >= (helicopter.engTurnoverMg / 2.5)) | (soundFFT.eng2_obor >= (helicopter.engTurnoverMg - 15)
								&& soundFFT.eng1_obor >= (helicopter.engTurnoverMg / 2.5))) & Eng1On & Eng2On);

						//Двигатель 1
						if (Eng1On)
						{

							if (avtOn)
							{
								if (statusEng1 != "eng_mg_avt")
								{
									offsetMgEng1 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng1_obor);
									offsetAvtEng1 = getOffset(helicopter.fullName["eng_mg_avt"], soundFFT.eng1_obor);
									statusEng1 = "eng_mg_avt";
								}
								turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng1);
								offsetMgEng1 += delta;
								turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng1) * turnMgEng1 / helicopter.engTurnoverMg;
								offsetAvtEng1 += delta;
							}
							else
							{
								if (soundFFT.eng1_obor > helicopter.engTurnoverMg)
								{
									if (statusEng1 != "eng_avt_mg")
									{
										offsetAvtEng1 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng1_obor);
										statusEng1 = "eng_avt_mg";
									}
									turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng1);
									offsetAvtEng1 += delta;
								}
								else
								{
									if (statusEng1 != "eng_on")
									{
										offsetMgEng1 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng1_obor);
										statusEng1 = "eng_on";
									}
									turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng1);
									offsetMgEng1 += delta;
								}
							}
						}
						else if (Eng1Off)
						{
							if (statusEng1 != "eng_off")
							{
								offsetAvtEng1 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng1_obor);
								offsetMgEng1 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng1_obor);
								statusEng1 = "eng_off";
							}
							turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng1);
							offsetAvtEng1 += delta;
							turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng1) * turnAvtEng1 / helicopter.engTurnoverMg;
							offsetMgEng1 += delta;
						}
						//Двигатель 2
						if (Eng2On)
						{

							if (avtOn)
							{
								if (statusEng2 != "eng_mg_avt")
								{
									offsetMgEng2 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng2_obor);
									offsetAvtEng2 = getOffset(helicopter.fullName["eng_mg_avt"], soundFFT.eng2_obor);
									statusEng2 = "eng_mg_avt";
								}
								turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng2);
								offsetMgEng2 += delta;
								turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng2) * turnMgEng2 / helicopter.engTurnoverMg;
								offsetAvtEng2 += delta;
							}
							else
							{
								if (soundFFT.eng2_obor > helicopter.engTurnoverMg)
								{
									if (statusEng2 != "eng_avt_mg")
									{
										offsetAvtEng2 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng2_obor);
										statusEng2 = "eng_avt_mg";
									}
									turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng2);
									offsetAvtEng2 += delta;
								}
								else
								{
									if (statusEng2 != "eng_on")
									{
										offsetMgEng2 = getOffset(helicopter.fullName["eng_on"], soundFFT.eng2_obor);
										statusEng2 = "eng_on";
									}
									turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_on"], offsetMgEng2);
									offsetMgEng2 += delta;
								}
							}
						}
						else if (Eng2Off)
						{
							if (statusEng2 != "eng_off")
							{
								offsetAvtEng2 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng2_obor);
								offsetMgEng2 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng2_obor);
								statusEng2 = "eng_off";
							}
							turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng2);
							offsetAvtEng2 += delta;
							turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng2) * turnAvtEng2 / helicopter.engTurnoverMg;
							offsetMgEng2 += delta;
						}

						//Редуктор
						if (Eng2On | Eng1On)
						{
							if (avtOn)
							{
								if (statusRed != "red_mg_avt")
								{
									offsetMg1 = getOffset(helicopter.fullName["red_on"], soundFFT.reduktor_gl_obor);
									offsetMg2 = getOffset(helicopter.fullName["red_on_mg"], soundFFT.reduktor_gl_obor);
									offsetAvt = getOffset(helicopter.fullName["red_mg_avt"], soundFFT.reduktor_gl_obor);
									statusRed = "red_mg_avt";
								}
								turnMg1 = getParameterFromFile(helicopter.fullName["red_on"], offsetMg1);
								offsetMg1 += delta;
								turnMg2 = getParameterFromFile(helicopter.fullName["red_on_mg"], offsetMg2) * (turnMg1 / helicopter.redTurnoverMg1);
								offsetMg2 += delta;
								turnAvt = getParameterFromFile(helicopter.fullName["red_mg_avt"], offsetAvt) * (turnMg2 / helicopter.redTurnoverMg2);
								offsetAvt += delta;
							}
							else
							{
								if (soundFFT.reduktor_gl_obor > helicopter.redTurnoverMg2)
								{
									if (statusRed != "red_avt_mg")
									{
										offsetAvt = getOffset(helicopter.fullName["red_avt_mg"], soundFFT.reduktor_gl_obor);
										statusRed = "red_avt_mg";
									}
									turnAvt = getParameterFromFile(helicopter.fullName["red_avt_mg"], offsetAvt);
									offsetAvt += delta;
								}
								else
								{
									if (twoEng)
									{
										if (statusRed != "red_on_mg")
										{
											offsetMg1 = getOffset(helicopter.fullName["red_on"], soundFFT.reduktor_gl_obor);
											offsetMg2 = getOffset(helicopter.fullName["red_on_mg"], soundFFT.reduktor_gl_obor);
											statusRed = "red_on_mg";
										}
										turnMg1 = getParameterFromFile(helicopter.fullName["red_on"], offsetMg1);
										offsetMg1 += delta;
										turnMg2 = getParameterFromFile(helicopter.fullName["red_on_mg"], offsetMg2) * (turnMg1 / helicopter.redTurnoverMg1);
										offsetMg2 += delta;
									}
									else if (oneEng)
									{
										if (soundFFT.reduktor_gl_obor > helicopter.redTurnoverMg1)
										{
											if (statusRed != "red_off")
											{
												offsetAvt = getOffset(helicopter.fullName["red_avt_mg"], soundFFT.reduktor_gl_obor);
												offsetMg1 = getOffset(helicopter.fullName["red_off"], soundFFT.reduktor_gl_obor);
												statusRed = "red_off";
											}
											turnAvt = getParameterFromFile(helicopter.fullName["red_avt_mg"], offsetAvt);
											offsetAvt += delta;
											turnMg1 = getParameterFromFile(helicopter.fullName["red_off"], offsetMg1) * turnAvt / helicopter.redTurnoverMg2;
											offsetMg1 += delta;
										}
										else
										{
											if (statusRed != "red_on")
											{
												offsetMg1 = getOffset(helicopter.fullName["red_on"], soundFFT.reduktor_gl_obor);
												statusRed = "red_on";
											}
											turnMg1 = getParameterFromFile(helicopter.fullName["red_on"], offsetMg1);
											offsetMg1 += delta;
										}
									}
								}
							}
						}
						else
						{
							if (statusRed != "red_off")
							{
								offsetAvt = getOffset(helicopter.fullName["red_avt_mg"], soundFFT.reduktor_gl_obor);
								offsetMg1 = getOffset(helicopter.fullName["red_off"], soundFFT.reduktor_gl_obor);
								statusRed = "red_off";
							}
							turnAvt = getParameterFromFile(helicopter.fullName["red_avt_mg"], offsetAvt);
							offsetAvt += delta;
							turnMg1 = getParameterFromFile(helicopter.fullName["red_off"], offsetMg1) * turnAvt / helicopter.redTurnoverMg2;
							offsetMg1 += delta;
						}

						if (statusEng1 == "eng_off" || statusEng1 == "eng_on")
							soundFFT.eng1_obor = turnMgEng1;
						if (statusEng1 == "eng_mg_avt" || statusEng1 == "eng_avt_mg")
							soundFFT.eng1_obor = turnAvtEng1;
						if (statusEng2 == "eng_off" || statusEng2 == "eng_on")
							soundFFT.eng2_obor = turnMgEng2;
						if (statusEng2 == "eng_mg_avt" || statusEng2 == "eng_avt_mg")
							soundFFT.eng2_obor = turnAvtEng2;

						if (statusRed == "red_off" || statusRed == "red_on")
							soundFFT.reduktor_gl_obor = turnMg1;
						if (statusRed == "red_on_mg")
							soundFFT.reduktor_gl_obor = turnMg2;
						if (statusRed == "red_mg_avt" || statusRed == "red_avt_mg")
							soundFFT.reduktor_gl_obor = turnAvt;

						soundFFT.reduktor_gl_obor = (soundFFT.reduktor_gl_obor < 0) ? 0 : soundFFT.reduktor_gl_obor;
						soundFFT.eng1_obor = (soundFFT.eng1_obor < 0) ? 0 : soundFFT.eng1_obor;
						soundFFT.eng2_obor = (soundFFT.eng2_obor < 0) ? 0 : soundFFT.eng2_obor;
					}

					if (Eng1On)
					{
						if (soundFFT.eng1_obor < helicopter.engTurnoverMg * 0.9333)
						{
							soundFFT.p_eng1_zap = 1;
						}
						else
						{
							soundFFT.p_eng1_zap = 0;
						}
					}
					if (Eng1Off)
					{
						soundFFT.p_eng1_ostanov = 1;
					}
					if (Eng2On)
					{
						if (soundFFT.eng2_obor < helicopter.engTurnoverMg * 0.9333)
						{
							soundFFT.p_eng2_zap = 1;
						}
						else
						{
							soundFFT.p_eng2_zap = 0;
						}
					}
					if (Eng2Off)
					{
						soundFFT.p_eng2_ostanov = 1;
					}
				}

				//Холодная прокрутка ВСУ
				if (vsuHp)
				{
					if (soundFFT.vsu_obor < VSU_MAX_TURN * 0.35 * 0.95)
					{
						soundFFT.p_vsu_hp = 1;
					}
					else
					{
						soundFFT.p_vsu_hp = 0;
					}
					if (soundFFT.vsu_obor < (VSU_MAX_TURN * 0.35))
						soundFFT.vsu_obor += (VSU_MAX_TURN * 0.35) / 5. * (delta);
					soundFFT.vsu_obor = (soundFFT.vsu_obor > (VSU_MAX_TURN * 0.35)) ? (VSU_MAX_TURN * 0.35) : soundFFT.vsu_obor;
				}
				//Запуск ВСУ
				if (vsuOn)
				{
					if (soundFFT.vsu_obor < VSU_MAX_TURN * 0.95)
					{
						soundFFT.p_vsu_zap = 1;
					}
					else
					{
						soundFFT.p_vsu_zap = 0;
					}
					soundFFT.p_vsu_ostanov = 0;
					if (soundFFT.vsu_obor < VSU_MAX_TURN)
						soundFFT.vsu_obor += VSU_MAX_TURN / 5. * (delta);
					soundFFT.vsu_obor = (soundFFT.vsu_obor > VSU_MAX_TURN) ? VSU_MAX_TURN : soundFFT.vsu_obor;
				}
				//Остановка ВСУ
				if (vsuOff)
				{
					if (!vsuhpbl)
					{
						if (soundFFT.vsu_obor > 0)
						{
							soundFFT.p_vsu_ostanov = 1;
						}
						else
						{
							soundFFT.p_vsu_ostanov = 0;
						}
						soundFFT.vsu_obor -= VSU_MAX_TURN / 11. * (delta);
						//Обороты ВСУ не должны падать ниже 0
						soundFFT.vsu_obor = (soundFFT.vsu_obor < 0) ? 0 : soundFFT.vsu_obor;
					}
					else
					{
						if (soundFFT.vsu_obor > 0)
						{
							soundFFT.p_vsu_ostanov = 1;
						}
						else
						{
							soundFFT.p_vsu_ostanov = 0;
							vsuhpbl = 0;
						}
						soundFFT.vsu_obor -= (VSU_MAX_TURN * 0.35) / 5. * (delta);
						soundFFT.vsu_obor = (soundFFT.vsu_obor < 0) ? 0 : soundFFT.vsu_obor;
					}
				}

				cout.precision(3);
				cout << fixed
					<< " TIME: " << soundFFT.time
					<< " VSU(on): " << soundFFT.p_vsu_zap
					<< " VSU(off): " << soundFFT.p_vsu_ostanov
					<< " VSU(hp): " << soundFFT.p_vsu_hp
					<< " VSU(%): " << soundFFT.vsu_obor
					<< " ENG1(on): " << soundFFT.p_eng1_zap
					<< " ENG1(off): " << soundFFT.p_eng1_ostanov
					<< " ENG1(hp): " << soundFFT.p_eng1_hp
					<< " ENG1(%): " << soundFFT.eng1_obor
					<< " ENG2(on): " << soundFFT.p_eng2_zap
					<< " ENG2(off): " << soundFFT.p_eng2_ostanov
					<< " ENG2(hp): " << soundFFT.p_eng2_hp
					<< " ENG2(%): " << soundFFT.eng2_obor
					<< " RED(%): " << soundFFT.reduktor_gl_obor
					<< "\t\r";
			}
			//тестовые циклограммы полетов для некоторых вертолетов
			else
			{
				vector <testChunk> tests;

				//Сброс параметров в начале теста
				if (!timeReset)
				{
					string Selector;
					system("cls");
					printf(" Choose type:\n 1) Standart\n 2) Hovering\n 3) SKV\n");

					while (!std::regex_match(Selector, regex("[1-3]")))//повторяем ввод пока не будет цифра от 1 до 4
						Selector = getch();//считываем буфер ввода

					switch (Selector[0])
					{
					case '1':
						hovering = 0;
						skv = 0;
						break;
					case '2':
						hovering = 1;
						skv = 0;
						break;
					case '3':
						hovering = 0;
						skv = 1;
						break;
					}

					if (helicopter.modelName == "mi_8_mtv5")
					{
						if (hovering)
						{
							tests =
							{
								{ 1, 0, 0 }
							};
						}
						else if (skv)
						{
							tests =
							{
								{ 1, 0, 0 }
							};
						}
						else
						{
							tests =
							{
								{ 1, 0, 75 },
								{ 2, 76, 276 },
								{ 3, 277, 437 },
								{ 4, 438, 568 },
								{ 5, 569, 689 },
								{ 6, 690, 965 },
								{ 7, 966, 1276 },
								{ 8, 966, 1276 },
								{ 9, 1277, 1587 },
								{ 10, 1588, 1763 }
							};
						}

						soundFFT.p_model_stop = 1;
						system("cls");

						for (int i = 0; i < tests.size(); i++)
						{
							cout.precision(0);
							cout << tests[i].number << ": " << tests[i].start << " - " << tests[i].end << endl;
						}

						int d;
						cin >> d;//считываем буфер ввода

						for (int i = 0; i < tests.size(); i++)
						{
							if (d == tests[i].number)
							{
								testTimeStart = tests[i].start;
								testTimeEnd = tests[i].end;
							}
						}
						if (d == 0)
						{
							system("cls");
							printf(" Enter range (in seconds): [start] [end]\n ");
							cin >> testTimeStart;
							cin >> testTimeEnd;
						}
					}
					if (helicopter.modelName == "mi_8_amtsh")
					{
						//printf(" TEST:\n 1) 0 - 200\n 2) 201 - 291\n 3) 292 - 432\n 4) 433 - 583\n 5) 584 - 684\n 6) 685 - 1095\n 7) 1096 - 1316\n 8) 1317 - 1582\n 9) [custom]\n");
					}
					if (helicopter.modelName == "mi_28")
					{
						// skv printf(" TEST:\n 1) 0 - 260\n 2) 261 - 691\n 3) 692 - 992\n 4) [custom time]\n");

						// std printf(" TEST:\n 1) 0 - 100\n 2) 101 - 441\n 3) 442 - 852\n 4) 853 - 1283\n 5) 1284 - 1484\n 6) [custom time]\n");
					}
					if (helicopter.modelName == "mi_26")
					{
						// skv printf(" TEST:\n 1) 0 - 260\n 2) 261 - 691\n 3) 692 - 992\n 4) [custom time]\n");

							// std
							//cout << " TEST:\n 1) 21 - 649\n 2) 650 - 760\n 3) 761 - 906\n 4) 907 - 1062\n";
							//cout << " 5) 1063 - 1383\n 6) 1384 - 1914\n 7) 1915 - 2175\n 8) 2176 - 2366\n";
							//cout << " 9) 2367 - 2647\n 10) 2648 - 3018\n 11) 3019 - 3179\n 12) 3180 - 3300\n 13) 3301 - 3641\n";
							//cout << " 0) [custom time]\n";
					}
					if (helicopter.modelName == "ka_29")
					{
						if (hovering)
						{
							tests =
							{
								{ 1, 0, 540 }
							};
						}
						else if (skv)
						{
							tests =
							{
								{ 1, 0, 0 }
							};
						}
						else
						{
							tests =
							{
								{ 1, 0, 75 },
								{ 2, 77, 429 },
								{ 3, 431, 701 },
								{ 4, 703, 858 },
								{ 5, 860, 920 },
								{ 6, 922, 1387 },
								{ 7, 1389, 1919 }
							};
						}

						soundFFT.p_model_stop = 1;
						system("cls");

						for (int i = 0; i < tests.size(); i++)
						{
							cout.precision(0);
							cout << tests[i].number << ": " << tests[i].start << " - " << tests[i].end << endl;
						}

						int d;
						cin >> d;//считываем буфер ввода

						for (int i = 0; i < tests.size(); i++)
						{
							if (d == tests[i].number)
							{
								testTimeStart = tests[i].start;
								testTimeEnd = tests[i].end;
							}
						}
						if (d == 0)
						{
							system("cls");
							printf(" Enter range (in seconds): [start] [end]\n ");
							cin >> testTimeStart;
							cin >> testTimeEnd;
						}
					}
					if (helicopter.modelName == "ka_27")
					{

					}
					if (helicopter.modelName == "ka_226")
					{
						// std printf(" TEST:\n 1) 0 - 120\n 2) 121 - 271\n 3) 272 - 442\n 4) 443 - 553\n 5) 554 - 814\n 6) 815 - 985\n 7) 815 - 1136\n 8) 1137 - 1377\n 9) 1378 - 1728\n 10) 1729 - 1879\n 11) 1880 - 2080\n 0) [custom]\n ");
					}
					if (helicopter.modelName == "ansat")
					{
						tests =
						{
							{ 1, 0, 180 },
							{ 2, 181, 341 },
							{ 3, 342, 682 },
							{ 4, 683, 823 },
							{ 5, 824, 1014 },
							{ 6, 1015, 1145 },
							{ 7, 1146, 1326 },
							{ 8, 1327, 1447 },
							{ 9, 1448, 1709 },
							{ 10, 1710, 1820 },
							{ 11, 1821, 1961 },
							{ 12, 1962, 2082 },
							{ 13, 2083, 2228 },
							{ 14, 2229, 2559 },
							{ 15, 2560, 2880 },
							{ 16, 2881, 3172 },
							{ 17, 3173, 3253 },
							{ 18, 3254, 3504 },
							{ 19, 3505, 3845 },
							{ 20, 3846, 4046 }
						};

						soundFFT.p_model_stop = 1;
						system("cls");

						for (int i = 0; i < tests.size(); i++)
						{
							cout.precision(0);
							cout << tests[i].number << ": " << tests[i].start << " - " << tests[i].end << endl;
						}

						int d;
						cin >> d;//считываем буфер ввода

						for (int i = 0; i < tests.size(); i++)
						{
							if (d == tests[i].number)
							{
								testTimeStart = tests[i].start;
								testTimeEnd = tests[i].end;
							}
						}
						if (d == 0)
						{
							system("cls");
							printf(" Enter range (in seconds): [start] [end]\n ");
							cin >> testTimeStart;
							cin >> testTimeEnd;
						}
					}

					offsetTest = testTimeStart;
					soundFFT.time = 0;
					rt.timeS = 0;
					currentTime = 0;
					delta = 0;
					system("cls");
					timeReset = 1;
				}

				if (!vectload)
				{
					string filename;
					if (hovering)
					{
						filename = "test/" + helicopter.modelName + "/Hovering/test.txt";
					}
					else if (skv)
					{
						filename = "test/" + helicopter.modelName + "/Skv/test.txt";
					}
					else
					{
						filename = "test/" + helicopter.modelName + "/Standart/test.txt";
					}

					ifstream base(filename);
					while (!base.eof())
					{
						string str; 
						double timeCol = 0;
						double HighCol = 0;
						double tangazCol = 0;
						double VelocityCol = 0;
						double StepCol = 0;
						double Eng1Col = 0;
						double Eng2Col = 0;
						double RedCol = 0;
						getline(base, str);
						//T H TNG VELX STEP ENG1 ENG2 RED
						sscanf(str.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &timeCol, &HighCol, &tangazCol, &VelocityCol, &StepCol, &Eng1Col, &Eng2Col, &RedCol);
						vectorPar[0].push_back({ timeCol,Eng1Col });
						vectorPar[1].push_back({ timeCol,Eng2Col });
						vectorPar[2].push_back({ timeCol,RedCol });
						vectorPar[3].push_back({ timeCol,HighCol });
						vectorPar[4].push_back({ timeCol,tangazCol });
						vectorPar[5].push_back({ timeCol,StepCol });
						vectorPar[6].push_back({ timeCol,VelocityCol });
					}
					base.close();

					//Получаем вектор вертикальной скорости из высоты
					vectorVy = vectorPar[3];
					for (int i = 1; i < vectorPar[3].size(); i++)
					{
						vectorVy[i].y = vectorPar[3][i].y - vectorPar[3][i - 1].y;
					}

					vectload = 1;
				}

				//Признак работы теста
				soundFFT.p_model_stop = 0;

				double spd = 0;

				offsetTest += delta;
				soundFFT.eng1_obor = getParameterFromVector(vectorPar[0], offsetTest);//дв1
				soundFFT.eng2_obor = getParameterFromVector(vectorPar[1], offsetTest);//дв2
				soundFFT.reduktor_gl_obor = getParameterFromVector(vectorPar[2], offsetTest);//редуктор
				soundFFT.vy = getParameterFromVector(vectorVy, offsetTest);//высота
				soundFFT.tangaz = getParameterFromVector(vectorPar[4], offsetTest);//тангаж
				soundFFT.step = getParameterFromVector(vectorPar[5], offsetTest);//шаг
				soundFFT.hight = getParameterFromVector(vectorPar[3], offsetTest);//шаг
				soundFFT.hight = (soundFFT.hight < 0) ? 0 : soundFFT.hight;
				if (soundFFT.hight == 0)
				{
					soundFFT.v_surf_x = getParameterFromVector(vectorPar[6], offsetTest);//скорость
					soundFFT.v_atm_x = 0;
					spd = soundFFT.v_surf_x;
					soundFFT.obj_hv = 0.5;
					soundFFT.obj_nos = 0.5;
					soundFFT.obj_l = 0.75;
					soundFFT.obj_r = 0.75;
				}
				else
				{
					soundFFT.v_atm_x = getParameterFromVector(vectorPar[6], offsetTest);//скорость
					soundFFT.v_surf_x = 0;
					spd = soundFFT.v_atm_x;
					soundFFT.obj_hv = 0;
					soundFFT.obj_nos = 0;
					soundFFT.obj_l = 0;
					soundFFT.obj_r = 0;
				}
				soundFFT.p_eng1_lkorr = 0;//Правая - левая коррекция
				soundFFT.p_eng2_lkorr = 0;
				soundFFT.p_eng1_rkorr = 1;
				soundFFT.p_eng2_rkorr = 1;

				cout.precision(3);
				cout << fixed
					<< " TIME: " << soundFFT.time
					<< " OFFS: " << offsetTest
					<< " ENG1(%): " << soundFFT.eng1_obor
					<< " ENG2(%): " << soundFFT.eng2_obor
					<< " REDO(%): " << soundFFT.reduktor_gl_obor
					<< " VELY(%): " << soundFFT.vy
					<< " TANG(%): " << soundFFT.tangaz
					<< " STEP(%): " << soundFFT.step
					<< " VELX(%): " << spd
					<< " HIGH(%): " << soundFFT.hight
					<< "\t\r";

				//Тест закончился
				if (soundFFT.time + testTimeStart > testTimeEnd)
				{
					//Признак работы теста
					soundFFT.p_model_stop = 1;
					system("cls");
					cout << "Test ended..." << endl;
					cout << "Continue? [y/n]" << endl;
					string Selector;
					while (!std::regex_match(Selector, regex("[yn]")))//повторяем ввод пока не будет цифра от 1 до 4
						Selector = getch();//считываем буфер ввода
					switch (Selector[0])
					{
					case 'y':
						timeReset = 0;
						break;
					case 'n':
						StopRealTime();
						StopNetVoice();
						return 0;
						break;
					}
				}
			}
			soundFFT.time = currentTime;
		}

		if (rt.pExchOK)
		{
			rt.pExchOK = 0;
		}
	}

	StopRealTime();
	StopNetVoice();
	return 0;
}

void kbHit()
{
	unsigned char c = ' ';
	if (_kbhit()) {
		c = _getch_nolock();

		switch (c)
		{
		case '0':
			soundFFT.p_crash = !soundFFT.p_crash;//Столкновение с препятствием
			break;
		case '1':
			soundFFT.obj_nos = !soundFFT.obj_nos;//Обжатие левой передней стойки шасси
			soundFFT.obj_hv = !soundFFT.obj_hv;//Обжатие правой передней стойки шасси
			soundFFT.obj_l = !soundFFT.obj_l;//Обжатие левой стойки шасси
			soundFFT.obj_r = !soundFFT.obj_r;//Обжатие правой стойки шасси
			break;
		case '2':
			soundFFT.p_eng1_pomp = !soundFFT.p_eng1_pomp;//Помпаж 1го двигателя
			break;
		case '3':
			soundFFT.p_eng2_pomp = !soundFFT.p_eng2_pomp;//Помпаж 2го двигателя
			break;
		case '4':
			soundFFT.p_ur_ataka = !soundFFT.p_ur_ataka;//Признак применения УР
			break;
		case '5':
			soundFFT.p_spo_upk = !soundFFT.p_spo_upk;//Признак применения СПО УПК
			break;
		case 'q':
			soundFFT.p_pts = !soundFFT.p_pts;//ПТС
			break;
		case 'J':
			soundFFT.p_vu1 = !soundFFT.p_vu1;//ВУ
			break;
		case 'S':
			soundFFT.p_vu3 = !soundFFT.p_vu3;//Свист опционально
			break;
		case 'w':
			//
			break;
		case 'e':		//ВСУ запуск
			soundFFT.p_vsu_hp = 0;
			vsuOn = 1;
			soundFFT.p_vsu_ostanov = 0;
			vsuOff = 0;
			vsuHp = 0;
			break;
		case 'r':		//ВСУ останов
			vsuOff = 1;
			soundFFT.p_vsu_zap = 0;
			soundFFT.p_vsu_hp = 0;
			vsuOn = 0;
			vsuHp = 0;
			break;
		case 'd':		//ВСУ ХП
			vsuhpbl = 1;
			vsuHp = 1;
			soundFFT.p_vsu_zap = 0;
			soundFFT.p_vsu_ostanov = 0;
			vsuOff = 0;
			vsuOn = 0;
			break;
		case 'y':		//Дв1 запуск
			eng1hpbl = 0;
			Eng1On = 1;
			soundFFT.p_eng1_hp = 0;
			soundFFT.p_eng1_ostanov = 0;
			Eng1Off = 0;
			Eng1Hp = 0;
			break;
		case 't':		//Дв1 hp
			eng1hpbl = 1;
			Eng1Hp = 1;
			soundFFT.p_eng1_zap = 0;
			soundFFT.p_eng1_ostanov = 0;
			Eng1On = 0;
			Eng1Off = 0;
			break;
		case 'u':		//Дв1 останов
			Eng1Off = 1;
			soundFFT.p_eng1_zap = 0;
			soundFFT.p_eng1_hp = 0;
			Eng1On = 0;
			Eng1Hp = 0;
			break;
		case 'g':		//Дв2 hp
			eng2hpbl = 1;
			Eng2Hp = 1;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng2_ostanov = 0;
			Eng2On = 0;
			Eng2Off = 0;
			break;
		case 'h':		//Дв2 запуск
			Eng2On = 1;
			eng2hpbl = 0;
			soundFFT.p_eng2_hp = 0;
			soundFFT.p_eng2_ostanov = 0;
			Eng2Off = 0;
			Eng2Hp = 0;
			break;
		case 'j':		//Дв2 останов
			Eng2Off = 1;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng2_hp = 0;
			Eng2On = 0;
			Eng2Hp = 0;
			break;
		case 'a':
			soundFFT.eng1_obor += .5;//Ручное регулирование оборотов двигателей (временно не работает)
			soundFFT.eng2_obor += .5;
			break;
		case 'z':
			soundFFT.eng1_obor -= .5;//Ручное регулирование оборотов двигателей (временно не работает)
			soundFFT.eng2_obor -= .5;
			break;
		case 'c':
			soundFFT.master_gain -= .01f;//Уменьшить громкость
			break;
		case 'v':
			soundFFT.master_gain += .01f;//Прибавить громкость
			break;
		case 'n':
			soundFFT.master_gain = 0;//Убрать звук
			break;
		case 'm':
			soundFFT.master_gain = 1;//Звук на максимум
			break;
		case 'b':
			soundFFT.tormoz_vint = !soundFFT.tormoz_vint;//Тормоз винта
			break;
			/*case 'B':
				soundFFT.rez_10 = !soundFFT.rez_10;
				break;*/
		case 'K':
			soundFFT.stove = !soundFFT.stove;//КО-50(обогреватель)
			break;
		case 'o':
			soundFFT.p_kran_perekr_1 = !soundFFT.p_kran_perekr_1;//Левый кран
			break;
		case 'p':
			soundFFT.p_kran_perekr_2 = !soundFFT.p_kran_perekr_2;//Правый кран
			break;
		case 's':
			soundFFT.v += 0.277;//Увеличить скорость
			soundFFT.v = (soundFFT.v > 100.) ? 100. : soundFFT.v;
			break;
		case 'x':
			soundFFT.v -= 0.277;//Уменьшить скорость
			soundFFT.v = (soundFFT.v < 0.) ? 0. : soundFFT.v;
			break;
		case 'k':
			soundFFT.p_reduktor_gl_crash = !soundFFT.p_reduktor_gl_crash;//Неисправность главного редуктора
			break;
		case ';':
			soundFFT.p_nar_s8 = !soundFFT.p_nar_s8;//НАР 8
			break;
		case 'l':
			soundFFT.p_nar_s13 = !soundFFT.p_nar_s13;//НАР 13
			break;
		case '[':
			soundFFT.p_spo_ppu = !soundFFT.p_spo_ppu;//СПО ППУ
			break;
		case ']':
			soundFFT.p_tormoz = !soundFFT.p_tormoz;//Признак тормоз шасси
			break;
		case 'f':
			soundFFT.p_rocket_hit = !soundFFT.p_rocket_hit;//Признак попадания ракетой
			break;
		case '6':
			soundFFT.p_eng1_lkorr = !soundFFT.p_eng1_lkorr;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = !soundFFT.p_eng2_lkorr;
			soundFFT.p_eng1_rkorr = !soundFFT.p_eng1_lkorr;
			soundFFT.p_eng2_rkorr = !soundFFT.p_eng2_lkorr;
			break;
		case '&':
			soundFFT.p_nasos_podk_1 = !soundFFT.p_nasos_podk_1;//Топливный насос левый
			break;
		case '*':
			soundFFT.p_nasos_podk_2 = !soundFFT.p_nasos_podk_2;//Топливный насос правый
			break;
		case 'Z':
			soundFFT.p_rain = !soundFFT.p_rain;//Дождь
			break;
		case '/':
			soundFFT.accumulator = !soundFFT.accumulator;//Аккумулятор
			break;
		case 'Q':
			soundFFT.p_trans_36_osn = !soundFFT.p_trans_36_osn;//36В
			break;
		case 'W':
			soundFFT.p_po500 = !soundFFT.p_po500;//115В
			break;
		case 'N':
			soundFFT.ground_power_supply = !soundFFT.ground_power_supply;//НИП
			break;
		case 'T':
			soundFFT.dis_tank_pump = !soundFFT.dis_tank_pump;//Насос расходного бака
			break;
		case '{':
			soundFFT.p_kran_poj_l = !soundFFT.p_kran_poj_l;//ПОЖ кран л
			break;
		case '}':
			soundFFT.p_kran_poj_r = !soundFFT.p_kran_poj_r;//ПОЖ кран п
			break;
		case '9':
			soundFFT.p_skv_on = !soundFFT.p_skv_on;//СКВ
			break;
		case '-':
			soundFFT.zoomer = !soundFFT.zoomer;//бибиби
			break;
		case '^':
			standAlone = !standAlone;
			break;
			//case '7':
			//	soundFFT.rez_5 = !soundFFT.rez_5;//хз 3
			//	break;
		case '8':
			soundFFT.undefined = !soundFFT.undefined;//хз 2
			break;
		case 'V':
			soundFFT.p_nasos = !soundFFT.p_nasos;//Насосная станция
			break;
		case 'i':
			soundFFT.p_kran_perekr_vsu = !soundFFT.p_kran_perekr_vsu;//потребитель
			break;
		case 'I':
			soundFFT.p_kran_kolcev = !soundFFT.p_kran_kolcev;//потребитель
			break;
		case '$':
			soundFFT.p_ur_igla = !soundFFT.p_ur_igla;//
			break;
		case 'F':
			test = !test;//Полет
			break;
		case 'P'://пауза (shift + s)
			pause = !pause;
			/*if (pause)
				PauseRealTime();
			else
				RewindRealTime();*/
			break;
		case 'M'://вернуться в начало (shift + m)
			soundFFT.reduktor_gl_obor = 0;
			soundFFT.eng1_obor = 0;
			soundFFT.eng2_obor = 0;
			soundFFT.p_eng1_zap = 0;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng1_ostanov = 1;
			soundFFT.p_eng2_ostanov = 1;
			soundFFT.p_eng1_lkorr = 1;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 1;
			soundFFT.p_eng1_rkorr = 0;
			soundFFT.p_eng2_rkorr = 0;
			soundFFT.p_vsu_ostanov = 0;
			soundFFT.vsu_obor = 0;
			Eng1On = 0;
			Eng1Hp = 0;
			Eng2Hp = 0;
			Eng2On = 0;
			Eng1Off = 0;
			Eng2Off = 0;
			eng1hpbl = 0;
			eng2hpbl = 0;
			statusEng1 = "NULL";
			statusEng2 = "NULL";
			statusRed = "NULL";
			break;
		case '<'://режим мг - редуктора на 1 дв (shift + ,)
			soundFFT.reduktor_gl_obor = helicopter.redTurnoverMg1;
			soundFFT.eng1_obor = helicopter.engTurnoverMg;
			soundFFT.eng2_obor = 0;
			soundFFT.p_eng1_zap = 0;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng1_ostanov = 0;
			soundFFT.p_eng2_ostanov = 0;
			soundFFT.p_eng1_lkorr = 1;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 1;
			soundFFT.p_eng1_rkorr = 0;
			soundFFT.p_eng2_rkorr = 0;
			Eng1On = 1;
			Eng1Hp = 0;
			Eng2Hp = 0;
			Eng2On = 0;
			Eng1Off = 0;
			Eng2Off = 0;
			eng1hpbl = 0;
			eng2hpbl = 0;
			statusEng1 = "NULL";
			statusEng2 = "NULL";
			statusRed = "NULL";
			break;
		case '>'://режим мг - редуктора на 2 дв(shift + .)
			soundFFT.reduktor_gl_obor = helicopter.redTurnoverMg2;
			soundFFT.eng1_obor = helicopter.engTurnoverMg;
			soundFFT.eng2_obor = helicopter.engTurnoverMg;
			soundFFT.p_eng1_zap = 0;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng1_ostanov = 0;
			soundFFT.p_eng2_ostanov = 0;
			soundFFT.p_eng1_lkorr = 1;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 1;
			soundFFT.p_eng1_rkorr = 0;
			soundFFT.p_eng2_rkorr = 0;
			Eng1On = 1;
			Eng1Hp = 0;
			Eng2Hp = 0;
			Eng2On = 1;
			Eng1Off = 0;
			Eng2Off = 0;
			eng1hpbl = 0;
			eng2hpbl = 0;
			statusEng1 = "NULL";
			statusEng2 = "NULL";
			statusRed = "NULL";
			break;
		case '?'://режим автомат - редуктора на 2 дв(shift + /)
			soundFFT.reduktor_gl_obor = helicopter.redTurnoverAvt;
			soundFFT.eng1_obor = helicopter.engTurnoverAvt;
			soundFFT.eng2_obor = helicopter.engTurnoverAvt;
			soundFFT.p_eng1_zap = 0;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng1_ostanov = 0;
			soundFFT.p_eng2_ostanov = 0;
			soundFFT.p_eng1_lkorr = 0;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 0;
			soundFFT.p_eng1_rkorr = 1;
			soundFFT.p_eng2_rkorr = 1;
			Eng1On = 1;
			Eng1Hp = 0;
			Eng2Hp = 0;
			Eng2On = 1;
			Eng1Off = 0;
			Eng2Off = 0;
			eng1hpbl = 0;
			eng2hpbl = 0;
			statusEng1 = "NULL";
			statusEng2 = "NULL";
			statusRed = "NULL";
			break;
		};
	};
}

double interpolation(point p1, point p2, double x)
{
	if (p1.x < p2.x && x > p2.x)
	{
		return p2.y;
	}
	if (p1.x < p2.x && x < p1.x)
	{
		return p1.y;
	}
	if (p1.x > p2.x && x < p2.x)
	{
		return p2.y;
	}
	if (p1.x > p2.x && x > p1.x)
	{
		return p1.y;
	}

	return	p1.y + ((p2.y - p1.y) / (p2.x - p1.x))*(x - p1.x);
}

double interpolation(point p1, point p2, point p3, double x)
{
	if (p1.x < p3.x && x > p3.x)
	{
		return p3.y;
	}
	if (p1.x < p3.x && x < p1.x)
	{
		return p1.y;
	}
	if (p1.x > p3.x && x < p3.x)
	{
		return p3.y;
	}
	if (p1.x > p3.x && x > p1.x)
	{
		return p1.y;
	}

	//если квадратичная интерполяция не работает - берем линейную
	if (p2.x == p1.x | p3.x == p2.x)
	{
		return	interpolation(p1, p2, x);

	}
	else
	{
		double fx, a0, a1, a2;
		a2 = ((p3.y - p1.y) / ((p3.x - p1.x)*(p3.x - p2.x))) - ((p2.y - p1.y) / ((p2.x - p1.x)*(p3.x - p2.x)));
		a1 = ((p2.y - p1.y) / (p2.x - p1.x)) - (a2*(p2.x + p1.x));
		a0 = p1.y - a1 * p1.x - a2 * p1.x*p1.x;
		return fx = a0 + a1 * x + a2*x*x;

	}

}

double getParameterFromFile(string filename, double offset)
{
	vector <double> time, value;

	//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)
	ifstream base(filename);
	while (!base.eof())
	{
		string str;
		double t = 0;
		double v = 0;
		getline(base, str);
		sscanf(str.c_str(), "%lf %lf", &t, &v);
		time.push_back(t);
		value.push_back(v);
	}
	base.close();

	return getParameterFromVector(value, time, offset);
}

double getPitch(double offset, string filename, double parameter)
{
	double turn = 0;
	vector <double> time, value;

	//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)

	ifstream base(filename);
	while (!base.eof())
	{
		double t = 0;
		double v = 0;
		string str;
		getline(base, str);
		sscanf(str.c_str(), "%lf %lf", &t, &v);
		time.push_back(t);
		value.push_back(v);
	}
	base.close();

	double x, a0, a1, a2;
	point p1, p2, p3;
	int n = time.size();

	for (int i = 0; i < n; i++)
	{
		if (offset < time[0])
		{
			turn = value[0];//достаем обороты из базы
			break;
		}
		if (offset == time[i])//реальная отметка времени совпала с отметкой из бд
		{
			turn = value[i];//достаем обороты из базы
			break;
		}
		if (offset > time[n - 1])//отметка не совпала с базой
		{
			turn = value[n - 1];//достаем обороты из базы
			break;
		}
		if (offset > time[i] && offset < time[i + 1])//отметка не совпала с базой
		{

			//квадратичная интерполяция
			if (i - 1 == -1 || i + 1 == n)
			{
				if (i - 1 == -1)
				{
					p1.x = time[i]; p1.y = value[i]; p2.x = time[i + 1]; p2.y = value[i + 1]; p3.x = time[i + 2]; p3.y = value[i + 2];
				}
				if (i + 1 == n)
				{
					p1.x = time[i - 2]; p1.y = value[i - 2]; p2.x = time[i - 1]; p2.y = value[i - 1]; p3.x = time[i]; p3.y = value[i];
				}
			}
			else
			{
				p1.x = time[i - 1]; p1.y = value[i - 1]; p2.x = time[i]; p2.y = value[i]; p3.x = time[i + 1]; p3.y = value[i + 1];
			}

			turn = interpolation(p1, p2, p3, offset);
		}
	}

	return  (turn <= 0) ? 1 : parameter / turn;	//вычисляем результирующий Pitch на основе отношениz настоящего уровня оборотов к базовому (получен при записи аудио-файла)
}

double getOffset(string filename, double parameter)
{
	double new_offset = 0;
	double turn = 0;

	vector <double> time, value;
	//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)
	ifstream base(filename);
	while (!base.eof())
	{
		string str;
		double t = 0;
		double v = 0;
		getline(base, str);
		sscanf(str.c_str(), "%lf %lf", &t, &v);
		time.push_back(t);
		value.push_back(v);
	}
	base.close();
	int n = time.size();

	if (parameter < 0)
		turn = 0;
	else
		turn = parameter;

	double x, a0, a1, a2;

	point p1, p2, p3;

	if (value[0] <= value[n - 1])
	{
		for (int i = 0; i < n; i++)
		{
			if (turn < value[0])
			{
				new_offset = time[0];//достаем обороты из базы
				break;
			}
			if (turn == value[i])//реальная отметка времени совпала с отметкой из бд
			{
				new_offset = time[i];//достаем обороты из базы
				break;
			}
			if (turn > value[n - 1])//отметка не совпала с базой
			{
				new_offset = time[n - 1];//достаем обороты из базы
				break;
			}
			if (turn > value[i] && turn < value[i + 1])//отметка не совпала с базой
			{

				//квадратичная интерполяция
				if (i + 2 == n || i + 1 == n)
				{
					if (i + 2 == n)
					{
						p1.x = value[i - 1]; p1.y = time[i - 1]; p2.x = value[i]; p2.y = time[i]; p3.x = value[i + 1]; p3.y = time[i + 1];
					}
					if (i + 1 == n)
					{
						p1.x = value[i - 2]; p1.y = time[i - 2]; p2.x = value[i - 1]; p2.y = time[i - 1]; p3.x = value[i]; p3.y = time[i];
					}
				}
				else
				{
					p1.x = value[i]; p1.y = time[i]; p2.x = value[i + 1]; p2.y = time[i + 1]; p3.x = value[i + 2]; p3.y = time[i + 2];
				}

				new_offset = interpolation(p1, p2, p3, turn);
			}

		}

	}
	else
	{
		for (int i = 0; i < n; i++)
		{
			if (turn > value[0])
			{
				new_offset = time[0];//достаем обороты из базы
				break;
			}
			if (turn == value[i])//реальная отметка времени совпала с отметкой из бд
			{
				new_offset = time[i];//достаем обороты из базы
				break;
			}
			if (turn < value[n - 1])//отметка не совпала с базой
			{
				new_offset = time[n - 1];//достаем обороты из базы
				break;
			}
			if (turn < value[i] && turn > value[i + 1])//отметка не совпала с базой
			{

				//квадратичная интерполяция
				if (i + 2 == n || i + 1 == n)
				{
					if (i + 2 == n)
					{
						p1.x = value[i - 1]; p1.y = time[i - 1]; p2.x = value[i]; p2.y = time[i]; p3.x = value[i + 1]; p3.y = time[i + 1];
					}
					if (i + 1 == n)
					{
						p1.x = value[i - 2]; p1.y = time[i - 2]; p2.x = value[i - 1]; p2.y = time[i - 1]; p3.x = value[i]; p3.y = time[i];
					}
				}
				else
				{
					p1.x = value[i]; p1.y = time[i]; p2.x = value[i + 1]; p2.y = time[i + 1]; p3.x = value[i + 2]; p3.y = time[i + 2];
				}

				new_offset = interpolation(p1, p2, p3, turn);
			}

		}
	}

	return (new_offset <= 0) ? 0 : new_offset;
}

double getParameterFromVector(vector<double> &value, vector<double> &time, double offset)
{
	int n = time.size();
	point p1, p2, p3;
	double x, a0, a1, a2;

	if (offset < time[0])
	{
		return value[0];//достаем обороты из базы
	}
	else if (offset > time[n - 1])//отметка не совпала с базой
	{
		return value[n - 1];//достаем обороты из базы
	}
	else
	{
		n = binSer(time, offset);
	}
	//Выбираем 3 точки (вариант -1 0 +1)
	if (n - 1 == -1)
	{
		p1.x = time[n]; p1.y = value[n]; p2.x = time[n + 1]; p2.y = value[n + 1]; p3.x = time[n + 2]; p3.y = value[n + 2];
	}
	else if (n + 1 == time.size())
	{
		p1.x = time[n - 2]; p1.y = value[n - 2]; p2.x = time[n - 1]; p2.y = value[n - 1]; p3.x = time[n]; p3.y = value[n];
	}
	else
	{
		p1.x = time[n - 1]; p1.y = value[n - 1]; p2.x = time[n]; p2.y = value[n]; p3.x = time[n + 1]; p3.y = value[n + 1];
	}

	return interpolation(p1, p2, p3, offset);
}

double getParameterFromVector(vector<point> &value, double offset)
{
	int n = value.size();
	point p1, p2, p3;
	double x, a0, a1, a2;

	if (offset < value[0].x)
	{
		return value[0].y;//достаем обороты из базы
	}
	else if (offset > value[n - 1].x)//отметка не совпала с базой
	{
		return value[n - 1].y;//достаем обороты из базы
	}
	else
	{
		n = binSer(value, offset);
	}
	//Выбираем 3 точки (вариант -1 0 +1)
	if (n - 1 == -1)
	{
		p1 = value[n]; p2 = value[n + 1]; p3 = value[n + 2];
	}
	else if (n + 1 == value.size())
	{
		p1 = value[n - 2]; p2 = value[n - 1]; p3 = value[n];
	}
	else
	{
		p1 = value[n - 1]; p2 = value[n]; p3 = value[n + 1];
	}

	return interpolation(p1, p2, p3, offset);
}

int binSer(vector<double> &time, double offset)
{
	int l = 0;
	int n = time.size() - 1;
	int r = n;
	while (abs(l - r) >= 2)
	{
		if (offset == time[n])
		{
			return n;
		}
		else if (offset < time[n])
		{
			r = n;
		}
		else
		{
			l = n;
		}
		n = (l + r) / 2;
	}
	return n;
}

int binSer(vector<point> &time, double offset)
{
	int l = 0;
	int n = time.size() - 1;
	int r = n;
	while (abs(l - r) >= 2)
	{
		if (offset == time[n].x)
		{
			return n;
		}
		else if (offset < time[n].x)
		{
			r = n;
		}
		else
		{
			l = n;
		}
		n = (l + r) / 2;
	}
	return n;
}

