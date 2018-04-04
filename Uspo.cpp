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

#define ANSAT_ENG_REV_TURN 54.00
#define VSU_MAX_TURN 100.00

using namespace std;

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

SOUNDFFT soundFFT;
Helicopter helicopter;

double test = 0;
double delta = 0;
double offsetTest = 0;
double timeEnd = 0;
double timeStart = 0;
double pause = 0;
double timeReset = 0;
int vectload = 0;

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

int main(int argc, char* argv[])
{
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

	/*else
	{
		string ch;
		string Type;
		printf(" Types of Armed Forces:\n 1) VVS\n 2) VMF\n");

		while (!std::regex_match(ch, regex("[1-2]")))//повторяем ввод пока не будет цифра от 1 до 4
			ch = getch();//считываем буфер ввода

		switch (ch[0])
		{
		case '1':
			Type = "VVS";
			break;
		case '2':
			Type = "VMF";
			break;
		}
		//Type = "VVS";
		system("cls");
		if (Type == "VVS")
		{
			ch = "NULL";
			printf(" Choose Helicopter:\n 1) Mi-8 MTV-5\n 2) Mi-8 AMTSH\n 3) Mi-26\n 4) Mi-28\n 5) Ka-226\n 6) ANSAT\n");

			while (!std::regex_match(ch, regex("[1-6]")))//повторяем ввод пока не будет цифра от 1 до 4
				ch = getch();//считываем буфер ввода

			switch (ch[0])
			{
			case '1':
				helicopter = mi_8_mtv5;
				break;
			case '2':
				helicopter = mi_8_amtsh;
				break;
			case '3':
				helicopter = mi_26;
				break;
			case '4':
				helicopter = mi_28;
				break;
			case '5':
				helicopter = ka_226;
				break;
			case '6':
				helicopter = ansat;
				break;
			}
		}
		else if (Type == "VMF")
		{
			ch = "NULL";
			printf(" Choose Helicopter:\n 1) Ka-27M\n 2) Ka-29\n");

			while (!std::regex_match(ch, regex("[1-2]")))//повторяем ввод пока не будет цифра от 1 до 4
				ch = getch();//считываем буфер ввода

			switch (ch[0])
			{
			case '1':
				helicopter = ka_27;
				break;
			case '2':
				helicopter = ka_29;
				break;
			}
		}
	}*/

	int lg = sizeof(SOUNDFFT);
	if (!InitNetVoice((void*)&soundFFT, lg)) {
		cout << "Not InitNetVoice" << endl;
		getch();
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

	if (!shaInit())				// Инициализация общей памяти 
		return 0;
	InitRealTime(1);

	bool hovering = 0;
	bool skv = 0;

	double currentTime = 0;
	double output = 0;

	vector<vector <point>> vectorPar(7);
	string filename[7];

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
			//Программа не поставлена на паузу
			if (!pause)
			{
				//Блок проверки запуска (и холодной прокрутки) двигателей, редуктора, всу
				if (!test)
				{
					//Условие автомата
					bool avtOn = soundFFT.p_eng2_rkorr & soundFFT.p_eng1_rkorr;

					if (helicopter.modelName == "ansat")
					{
						bool case1 = soundFFT.eng1_obor >= ANSAT_ENG_REV_TURN & soundFFT.eng2_obor >= ANSAT_ENG_REV_TURN;//Подъем или спуск
						bool case2 = soundFFT.eng1_obor < ANSAT_ENG_REV_TURN & soundFFT.eng2_obor < ANSAT_ENG_REV_TURN;//Нормальный подъем
						bool case3 = (soundFFT.eng1_obor < ANSAT_ENG_REV_TURN & soundFFT.eng2_obor >= ANSAT_ENG_REV_TURN) | (soundFFT.eng1_obor >= ANSAT_ENG_REV_TURN & soundFFT.eng2_obor < ANSAT_ENG_REV_TURN);//Нормальный подъем

																																																				 //Двигатель 1
						if (soundFFT.p_eng1_zap)
						{
							if (!avtOn)
							{
								if (soundFFT.eng1_obor > helicopter.eng_obor_mg)
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
								turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng1) * turnMgEng1 / helicopter.eng_obor_mg;
								offsetAvtEng1 += delta;
							}
						}
						else if (soundFFT.p_eng1_ostanov & soundFFT.eng1_obor > 0)
						{
							if (statusEng1 != "eng_off")
							{
								offsetAvtEng1 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng1_obor);
								offsetMgEng1 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng1_obor);
								statusEng1 = "eng_off";
							}
							turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng1);
							offsetAvtEng1 += delta;
							turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng1) * turnAvtEng1 / helicopter.eng_obor_mg;
							offsetMgEng1 += delta;
						}
						//Двигатель 2
						if (soundFFT.p_eng2_zap)
						{
							if (!avtOn)
							{
								if (soundFFT.eng2_obor > helicopter.eng_obor_mg)
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
								turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng2) * turnMgEng2 / helicopter.eng_obor_mg;
								offsetAvtEng2 += delta;
							}
						}
						else if (soundFFT.p_eng2_ostanov & soundFFT.eng2_obor > 0)
						{
							if (statusEng2 != "eng_off")
							{
								offsetAvtEng2 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng2_obor);
								offsetMgEng2 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng2_obor);
								statusEng2 = "eng_off";
							}
							turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng2);
							offsetAvtEng2 += delta;
							turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng2) * turnAvtEng2 / helicopter.eng_obor_mg;
							offsetMgEng2 += delta;
						}

						//Редуктор
						if (soundFFT.p_eng1_zap | soundFFT.p_eng2_zap)
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
								turnAvt = getParameterFromFile(helicopter.fullName["red_mg_avt"], offsetAvt) * turnMg1 / helicopter.red_obor_mg1;
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
										turnMg1 = getParameterFromFile(helicopter.fullName["red_on"], offsetMg1) / helicopter.red_obor_mg1 * turnRevRedAnsat;
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
							turnMg1 = getParameterFromFile(helicopter.fullName["red_off"], offsetMg1) * turnAvt / helicopter.red_obor_mg1;
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

						bool oneEng = (((soundFFT.p_eng1_zap & (!soundFFT.p_eng2_zap | soundFFT.p_eng2_ostanov))
							^ (soundFFT.p_eng2_zap & (!soundFFT.p_eng1_zap | soundFFT.p_eng1_ostanov)))
							| (soundFFT.p_eng1_zap & soundFFT.p_eng2_zap & soundFFT.eng1_obor < (helicopter.eng_obor_mg - 15)
								& soundFFT.eng2_obor < (helicopter.eng_obor_mg - 15)) | (((soundFFT.eng1_obor >= (helicopter.eng_obor_mg - 15)
									&& soundFFT.eng2_obor < (helicopter.eng_obor_mg / 2.5)) | (soundFFT.eng2_obor >= (helicopter.eng_obor_mg - 15)
										&& soundFFT.eng1_obor < (helicopter.eng_obor_mg / 2.5))) & soundFFT.p_eng1_zap & soundFFT.p_eng2_zap));
						bool twoEng = (((soundFFT.eng1_obor >= (helicopter.eng_obor_mg - 15)
							&& soundFFT.eng2_obor >= (helicopter.eng_obor_mg / 2.5)) | (soundFFT.eng2_obor >= (helicopter.eng_obor_mg - 15)
								&& soundFFT.eng1_obor >= (helicopter.eng_obor_mg / 2.5))) & soundFFT.p_eng1_zap & soundFFT.p_eng2_zap);

						//Двигатель 1
						if (soundFFT.p_eng1_zap)
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
								turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng1) * turnMgEng1 / helicopter.eng_obor_mg;
								offsetAvtEng1 += delta;
							}
							else
							{
								if (soundFFT.eng1_obor > helicopter.eng_obor_mg)
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
						else if (soundFFT.p_eng1_ostanov)
						{
							if (statusEng1 != "eng_off")
							{
								offsetAvtEng1 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng1_obor);
								offsetMgEng1 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng1_obor);
								statusEng1 = "eng_off";
							}
							turnAvtEng1 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng1);
							offsetAvtEng1 += delta;
							turnMgEng1 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng1) * turnAvtEng1 / helicopter.eng_obor_mg;
							offsetMgEng1 += delta;
						}
						//Двигатель 2
						if (soundFFT.p_eng2_zap)
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
								turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_mg_avt"], offsetAvtEng2) * turnMgEng2 / helicopter.eng_obor_mg;
								offsetAvtEng2 += delta;
							}
							else
							{
								if (soundFFT.eng2_obor > helicopter.eng_obor_mg)
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
						else if (soundFFT.p_eng2_ostanov)
						{
							if (statusEng2 != "eng_off")
							{
								offsetAvtEng2 = getOffset(helicopter.fullName["eng_avt_mg"], soundFFT.eng2_obor);
								offsetMgEng2 = getOffset(helicopter.fullName["eng_off"], soundFFT.eng2_obor);
								statusEng2 = "eng_off";
							}
							turnAvtEng2 = getParameterFromFile(helicopter.fullName["eng_avt_mg"], offsetAvtEng2);
							offsetAvtEng2 += delta;
							turnMgEng2 = getParameterFromFile(helicopter.fullName["eng_off"], offsetMgEng2) * turnAvtEng2 / helicopter.eng_obor_mg;
							offsetMgEng2 += delta;
						}

						//Редуктор
						if (soundFFT.p_eng2_zap | soundFFT.p_eng1_zap)
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
								turnMg2 = getParameterFromFile(helicopter.fullName["red_on_mg"], offsetMg2) * (turnMg1 / helicopter.red_obor_mg1);
								offsetMg2 += delta;
								turnAvt = getParameterFromFile(helicopter.fullName["red_mg_avt"], offsetAvt) * (turnMg2 / helicopter.red_obor_mg2);
								offsetAvt += delta;
							}
							else
							{
								if (soundFFT.reduktor_gl_obor > helicopter.red_obor_mg2)
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
										turnMg2 = getParameterFromFile(helicopter.fullName["red_on_mg"], offsetMg2) * (turnMg1 / helicopter.red_obor_mg1);
										offsetMg2 += delta;
									}
									else if (oneEng)
									{
										if (soundFFT.reduktor_gl_obor > helicopter.red_obor_mg1)
										{
											if (statusRed != "red_off")
											{
												offsetAvt = getOffset(helicopter.fullName["red_avt_mg"], soundFFT.reduktor_gl_obor);
												offsetMg1 = getOffset(helicopter.fullName["red_off"], soundFFT.reduktor_gl_obor);
												statusRed = "red_off";
											}
											turnAvt = getParameterFromFile(helicopter.fullName["red_avt_mg"], offsetAvt);
											offsetAvt += delta;
											turnMg1 = getParameterFromFile(helicopter.fullName["red_off"], offsetMg1) * turnAvt / helicopter.red_obor_mg2;
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
							turnMg1 = getParameterFromFile(helicopter.fullName["red_off"], offsetMg1) * turnAvt / helicopter.red_obor_mg2;
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

					//Холодная прокрутка ВСУ
					if (soundFFT.p_vsu_hp & !soundFFT.p_vsu_zap)
					{
						if (soundFFT.vsu_obor < (VSU_MAX_TURN * 0.35))
							soundFFT.vsu_obor += (VSU_MAX_TURN * 0.35) / 5. * (delta);
						soundFFT.vsu_obor = (soundFFT.vsu_obor > (VSU_MAX_TURN * 0.35)) ? (VSU_MAX_TURN * 0.35) : soundFFT.vsu_obor;
					}
					//Холодная прокрутка ВСУ выкл
					if (!soundFFT.p_vsu_hp & !soundFFT.p_vsu_zap)
					{

						soundFFT.vsu_obor -= (VSU_MAX_TURN * 0.35) / 5. * (delta);
						soundFFT.vsu_obor = (soundFFT.vsu_obor < 0) ? 0 : soundFFT.vsu_obor;
					}
					//Запуск ВСУ
					if (soundFFT.p_vsu_zap)
					{
						soundFFT.p_vsu_ostanov = 0;
						if (soundFFT.vsu_obor < VSU_MAX_TURN)
							soundFFT.vsu_obor += VSU_MAX_TURN / 5. * (delta);
						soundFFT.vsu_obor = (soundFFT.vsu_obor > VSU_MAX_TURN) ? VSU_MAX_TURN : soundFFT.vsu_obor;
					}
					//Остановка ВСУ
					if (soundFFT.p_vsu_ostanov && soundFFT.vsu_obor != 0)
					{
						soundFFT.vsu_obor -= VSU_MAX_TURN / 11. * (delta);
						//Обороты ВСУ не должны падать ниже 0
						soundFFT.vsu_obor = (soundFFT.vsu_obor < 0) ? 0 : soundFFT.vsu_obor;
					}
					//Возвращение в исходное состояние всу
					if (soundFFT.vsu_obor == 0)
						soundFFT.p_vsu_ostanov = 0;

					//Холодная прокрутка двигателя 1
					if (soundFFT.p_eng1_hp && soundFFT.eng1_obor < 20 && !soundFFT.p_eng1_ostanov && !soundFFT.p_eng1_zap)
					{
						statusEng1 = "eng_hp";
						soundFFT.eng1_obor += (helicopter.eng_obor_mg * 0.3) / 23.*(delta);
					}
					//Холодная прокрутка двигателя 1 выкл
					if (!soundFFT.p_eng1_hp && soundFFT.eng1_obor > 0 && !soundFFT.p_eng1_ostanov && !soundFFT.p_eng1_zap)
					{
						statusEng1 = "eng_hp";
						soundFFT.eng1_obor -= (helicopter.eng_obor_mg * 0.3) / 35.*(delta);
						soundFFT.eng1_obor = (soundFFT.eng1_obor < 0) ? 0 : soundFFT.eng1_obor;
					}
					//Холодная прокрутка двигателя 2
					if (soundFFT.p_eng2_hp && soundFFT.eng2_obor < 20 && !soundFFT.p_eng2_ostanov && !soundFFT.p_eng2_zap)
					{
						statusEng2 = "eng_hp";
						soundFFT.eng2_obor += (helicopter.eng_obor_mg * 0.3) / 23.*(delta);
					}
					//Холодная прокрутка двигателя 2 выкл
					if (!soundFFT.p_eng2_hp && soundFFT.eng2_obor > 0 && !soundFFT.p_eng2_ostanov && !soundFFT.p_eng2_zap)
					{
						statusEng2 = "eng_hp";
						soundFFT.eng2_obor -= (helicopter.eng_obor_mg * 0.3) / 35.*(delta);
						soundFFT.eng2_obor = (soundFFT.eng2_obor < 0) ? 0 : soundFFT.eng2_obor;
					}
					//Возвращение в исходное состояние двигателей
					if (soundFFT.eng1_obor == 0)
					{
						soundFFT.p_eng1_ostanov = 0;
					}
					if (soundFFT.eng2_obor == 0)
					{
						soundFFT.p_eng2_ostanov = 0;
					}
				}
				//тестовые циклограммы полетов для некоторых вертолетов
				else
				{
					string ch;

					//Сброс параметров в начале теста
					if (timeReset == 0)
					{
						if (helicopter.modelName == "mi_8_mtv5")
						{
							soundFFT.p_model_stop = 1;
							system("cls");
							printf(" TEST:\n 1) 0 - 75\n 2) 76 - 276\n 3) 277 - 437\n 4) 438 - 568\n 5) 569 - 689\n 6) 690 - 965\n 7) 966 - 1276\n 8) 1277 - 1587\n 9) 1588 - 1763\n 0) [custom]\n");

							while (!std::regex_match(ch, regex("[0-9]")))//повторяем ввод пока не будет цифра от 1 до 4
								ch = getch();//считываем буфер ввода

							switch (ch[0])
							{
							case '1':
								offsetTest = 0;
								timeEnd = 75;
								break;
							case '2':
								offsetTest = 76;
								timeEnd = 276;
								break;
							case '3':
								offsetTest = 277;
								timeEnd = 437;
								break;
							case '4':
								offsetTest = 438;
								timeEnd = 568;
								break;
							case '5':
								offsetTest = 569;
								timeEnd = 689;
								break;
							case '6':
								offsetTest = 690;
								timeEnd = 965;
								break;
							case '7':
								offsetTest = 966;
								timeEnd = 1276;
								break;
							case '8':
								offsetTest = 1277;
								timeEnd = 1587;
								break;
							case '9':
								offsetTest = 1588;
								timeEnd = 1763;
								break;
							case '0':
								system("cls");
								printf(" Enter range (in seconds): [start] [end]\n ");
								cin >> offsetTest;
								cin >> timeEnd;
								break;
							}
							timeStart = offsetTest;
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							timeReset = 1;
							system("cls");

						}
						if (helicopter.modelName == "mi_8_amtsh")
						{
							soundFFT.p_model_stop = 1;
							system("cls");
							printf(" TEST:\n 1) 0 - 200\n 2) 201 - 291\n 3) 292 - 432\n 4) 433 - 583\n 5) 584 - 684\n 6) 685 - 1095\n 7) 1096 - 1316\n 8) 1317 - 1582\n 9) [custom]\n");

							while (!std::regex_match(ch, regex("[1-9]")))//повторяем ввод пока не будет цифра от 1 до 4
								ch = getch();//считываем буфер ввода

							switch (ch[0])
							{
							case '1':
								offsetTest = 0;
								timeEnd = 200;
								break;
							case '2':
								offsetTest = 201;
								timeEnd = 291;
								break;
							case '3':
								offsetTest = 292;
								timeEnd = 432;
								break;
							case '4':
								offsetTest = 433;
								timeEnd = 583;
								break;
							case '5':
								offsetTest = 584;
								timeEnd = 684;
								break;
							case '6':
								offsetTest = 685;
								timeEnd = 1095;
								break;
							case '7':
								offsetTest = 1096;
								timeEnd = 1316;
								break;
							case '8':
								offsetTest = 1317;
								timeEnd = 1582;
								break;
							case '9':
								system("cls");
								printf(" Enter range (in seconds): [start] [end]\n ");
								cin >> offsetTest;
								cin >> timeEnd;
								break;
							}
							timeStart = offsetTest;
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							timeReset = 1;
							system("cls");
						}
						if (helicopter.modelName == "mi_28")
						{
							string ch1;
							system("cls");
							printf(" Choose type:\n 1) Standart\n 2) Hovering\n 3) SKV\n");

							while (!std::regex_match(ch1, regex("[1-3]")))//повторяем ввод пока не будет цифра от 1 до 4
								ch1 = getch();//считываем буфер ввода

							switch (ch1[0])
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

							if (hovering)
							{
								system("cls");
								printf(" Enter range (in seconds): [start] [end]\n ");
								cin >> timeStart;
								cin >> timeEnd;
							}
							else if (skv)
							{
								soundFFT.p_model_stop = 1;
								system("cls");
								printf(" TEST:\n 1) 0 - 260\n 2) 261 - 691\n 3) 692 - 992\n 4) [custom time]\n");

								while (!std::regex_match(ch, regex("[1-4]")))//повторяем ввод пока не будет цифра от 1 до 4
									ch = getch();//считываем буфер ввода

								switch (ch[0])
								{
								case '1':
									offsetTest = 0;
									timeEnd = 260;
									break;
								case '2':
									offsetTest = 261;
									timeEnd = 691;
									break;
								case '3':
									offsetTest = 692;
									timeEnd = 992;
									break;
								case '4':
									system("cls");
									printf(" Enter range (in seconds): [start] [end]\n ");
									cin >> offsetTest;
									cin >> timeEnd;
									break;
								}
							}
							else
							{
								soundFFT.p_model_stop = 1;
								system("cls");
								printf(" TEST:\n 1) 0 - 100\n 2) 101 - 441\n 3) 442 - 852\n 4) 853 - 1283\n 5) 1284 - 1484\n 6) [custom time]\n");

								while (!std::regex_match(ch, regex("[1-6]")))//повторяем ввод пока не будет цифра от 1 до 4
									ch = getch();//считываем буфер ввода

								switch (ch[0])
								{
								case '1':
									offsetTest = 0;
									timeEnd = 100;
									break;
								case '2':
									offsetTest = 101;
									timeEnd = 441;
									break;
								case '3':
									offsetTest = 442;
									timeEnd = 852;
									break;
								case '4':
									offsetTest = 853;
									timeEnd = 1283;
									break;
								case '5':
									offsetTest = 1284;
									timeEnd = 1484;
									break;
								case '6':
									system("cls");
									printf(" Enter range (in seconds): [start] [end]\n ");
									cin >> offsetTest;
									cin >> timeEnd;
									break;
								}
							}

							timeStart = offsetTest;
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							timeReset = 1;
							system("cls");
						}
						if (helicopter.modelName == "mi_26")
						{
							string ch1;
							system("cls");
							printf(" Choose type:\n 1) Standart\n 2) Hovering\n 3) SKV\n");

							while (!std::regex_match(ch1, regex("[1-3]")))//повторяем ввод пока не будет цифра от 1 до 4
								ch1 = getch();//считываем буфер ввода

							switch (ch1[0])
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

							if (hovering)
							{
								system("cls");
								printf(" Enter range (in seconds): [start] [end]\n ");
								cin >> timeStart;
								cin >> timeEnd;


							}
							else if (skv)
							{
								soundFFT.p_model_stop = 1;
								system("cls");
								printf(" TEST:\n 1) 0 - 260\n 2) 261 - 691\n 3) 692 - 992\n 4) [custom time]\n");

								while (!std::regex_match(ch, regex("[1-4]")))//повторяем ввод пока не будет цифра от 1 до 4
									ch = getch();//считываем буфер ввода

								switch (ch[0])
								{
								case '1':
									offsetTest = 0;
									timeEnd = 260;
									break;
								case '2':
									offsetTest = 261;
									timeEnd = 691;
									break;
								case '3':
									offsetTest = 692;
									timeEnd = 992;
									break;
								case '4':
									system("cls");
									printf(" Enter range (in seconds): [start] [end]\n ");
									cin >> offsetTest;
									cin >> timeEnd;
									break;
								}
							}
							else
							{
								soundFFT.p_model_stop = 1;
								system("cls");

								cout << " TEST:\n 1) 21 - 649\n 2) 650 - 760\n 3) 761 - 906\n 4) 907 - 1062\n";
								cout << " 5) 1063 - 1383\n 6) 1384 - 1914\n 7) 1915 - 2175\n 8) 2176 - 2366\n";
								cout << " 9) 2367 - 2647\n 10) 2648 - 3018\n 11) 3019 - 3179\n 12) 3180 - 3300\n 13) 3301 - 3641\n";
								cout << " 0) [custom time]\n";
								int num;
								cout << " Enter test number: ";
								cin >> num;

								switch (num)
								{
								case 1:
									offsetTest = 21;
									timeEnd = 649;
									break;
								case 2:
									offsetTest = 650;
									timeEnd = 760;
									break;
								case 3:
									offsetTest = 761;
									timeEnd = 906;
									break;
								case 4:
									offsetTest = 907;
									timeEnd = 1062;
									break;
								case 5:
									offsetTest = 1063;
									timeEnd = 1383;
									break;
								case 6:
									offsetTest = 1384;
									timeEnd = 1914;
									break;
								case 7:
									offsetTest = 1915;
									timeEnd = 2175;
									break;
								case 8:
									offsetTest = 2176;
									timeEnd = 2366;
									break;
								case 9:
									offsetTest = 2367;
									timeEnd = 2647;
									break;
								case 10:
									offsetTest = 2648;
									timeEnd = 3018;
									break;
								case 11:
									offsetTest = 3019;
									timeEnd = 3179;
									break;
								case 12:
									offsetTest = 3180;
									timeEnd = 3300;
									break;
								case 13:
									offsetTest = 3301;
									timeEnd = 3641;
									break;
								case 0:
									system("cls");
									printf(" Enter range (in seconds): [start] [end]\n ");
									cin >> offsetTest;
									cin >> timeEnd;
									break;
								}
							}

							timeStart = offsetTest;
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							timeReset = 1;
							system("cls");
						}
						if (helicopter.modelName == "ka_29")
						{
							string ch1;
							system("cls");
							printf(" Choose type:\n 1) Standart\n 2) Hovering\n");

							while (!std::regex_match(ch1, regex("[1-2]")))//повторяем ввод пока не будет цифра от 1 до 4
								ch1 = getch();//считываем буфер ввода

							switch (ch1[0])
							{
							case '1':
								hovering = 0;
								break;
							case '2':
								hovering = 1;
								break;
							}

							if (hovering)
							{

							}
							else
							{
								soundFFT.p_model_stop = 1;
								system("cls");
								printf(" TEST:\n 1) 0 - 75\n 2) 77 - 429\n 3) 431 - 701\n 4) 703 - 858\n 5) 860 - 920\n 6) 922 - 1387\n 7) 1389 - 1919\n 8) [custom time]\n");

								while (!std::regex_match(ch, regex("[1-8]")))//повторяем ввод пока не будет цифра от 1 до 4
									ch = getch();//считываем буфер ввода

								switch (ch[0])
								{
								case '1':
									offsetTest = 0;
									timeEnd = 75;
									break;
								case '2':
									offsetTest = 77;
									timeEnd = 429;
									break;
								case '3':
									offsetTest = 431;
									timeEnd = 701;
									break;
								case '4':
									offsetTest = 703;
									timeEnd = 858;
									break;
								case '5':
									offsetTest = 860;
									timeEnd = 920;
									break;
								case '6':
									offsetTest = 922;
									timeEnd = 1387;
									break;
								case '7':
									offsetTest = 1389;
									timeEnd = 1919;
									break;
								case '8':
									system("cls");
									printf(" Enter range (in seconds): [start] [end]\n ");
									cin >> offsetTest;
									cin >> timeEnd;
									break;
								}
							}

							timeStart = offsetTest;
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							timeReset = 1;
							system("cls");
						}
						if (helicopter.modelName == "ka_27")
						{
							soundFFT.p_model_stop = 1;
							system("cls");
							printf(" Choose type:\n 1) Standart\n 2) Hovering\n");

							while (!std::regex_match(ch, regex("[1-2]")))//повторяем ввод пока не будет цифра от 1 до 4
								ch = getch();//считываем буфер ввода

							switch (ch[0])
							{
							case '1':
								hovering = 0;
								break;
							case '2':
								hovering = 1;
								break;
							}

							system("cls");
							printf(" Enter range (in seconds): [start] [end]\n ");
							cin >> timeStart;
							cin >> timeEnd;

							offsetTest = timeStart;
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							delta = 0;
							timeReset = 1;
							system("cls");

						}
						vectload = 0;
					}

					if (hovering)
					{
						filename[0] = "test/" + helicopter.modelName + "/Hovering/eng1.txt";
						filename[1] = "test/" + helicopter.modelName + "/Hovering/eng2.txt";
						filename[2] = "test/" + helicopter.modelName + "/Hovering/red.txt";
						filename[3] = "test/" + helicopter.modelName + "/Hovering/h.txt";
						filename[4] = "test/" + helicopter.modelName + "/Hovering/tangaz.txt";
						filename[5] = "test/" + helicopter.modelName + "/Hovering/step.txt";
						filename[6] = "test/" + helicopter.modelName + "/Hovering/v.txt";
					}
					else if (skv)
					{
						filename[0] = "test/" + helicopter.modelName + "/SKV/eng1.txt";
						filename[1] = "test/" + helicopter.modelName + "/SKV/eng2.txt";
						filename[2] = "test/" + helicopter.modelName + "/SKV/red.txt";
						filename[3] = "test/" + helicopter.modelName + "/SKV/h.txt";
						filename[4] = "test/" + helicopter.modelName + "/SKV/tangaz.txt";
						filename[5] = "test/" + helicopter.modelName + "/SKV/step.txt";
						filename[6] = "test/" + helicopter.modelName + "/SKV/v.txt";
					}
					else
					{
						filename[0] = "test/" + helicopter.modelName + "/Standart/eng1.txt";
						filename[1] = "test/" + helicopter.modelName + "/Standart/eng2.txt";
						filename[2] = "test/" + helicopter.modelName + "/Standart/red.txt";
						filename[3] = "test/" + helicopter.modelName + "/Standart/h.txt";
						filename[4] = "test/" + helicopter.modelName + "/Standart/tangaz.txt";
						filename[5] = "test/" + helicopter.modelName + "/Standart/step.txt";
						filename[6] = "test/" + helicopter.modelName + "/Standart/v.txt";
					}

					if (vectload == 0)
					{
						//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)
						for (size_t i = 0; i < 7; i++)
						{
							ifstream base(filename[i]);
							while (!base.eof())
							{
								string str;
								double t = 0;
								double v = 0;
								getline(base, str);
								sscanf(str.c_str(), "%lf %lf", &t, &v);
								vectorPar[i].push_back({ t,v });
							}
							base.close();
						}
						vectload = 1;
					}

					//Признак работы теста
					soundFFT.p_model_stop = 0;

					offsetTest += delta;
					soundFFT.eng1_obor = getParameterFromVector(vectorPar[0], offsetTest);//дв1
					soundFFT.eng2_obor = getParameterFromVector(vectorPar[1], offsetTest);//дв2
					soundFFT.reduktor_gl_obor = getParameterFromVector(vectorPar[2], offsetTest);//редуктор
					soundFFT.styk_hv = getParameterFromVector(vectorPar[3], offsetTest);//высота
					soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
					soundFFT.osadki = getParameterFromVector(vectorPar[4], offsetTest);//тангаж
					soundFFT.ny = getParameterFromVector(vectorPar[5], offsetTest);//шаг
					soundFFT.v = getParameterFromVector(vectorPar[6], offsetTest);//скорость
					soundFFT.p_vu3 = 1;//Cвист винта

					//Тест закончился
					if (soundFFT.time + timeStart > timeEnd)
					{
						//Признак работы теста
						soundFFT.p_model_stop = 1;
						system("cls");
						cout << "Test ended..." << endl;
						cout << "Continue? [y/n]" << endl;
						while (!std::regex_match(ch, regex("[yn]")))//повторяем ввод пока не будет цифра от 1 до 4
							ch = getch();//считываем буфер ввода
						switch (ch[0])
						{
						case 'y':
							timeReset = 0;
							timeEnd = 0;
							offsetTest = 0;
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
				printf(" Time = %8.3f OffsetTest = %6.3f H = %6.3f VX = %6.3f STEP = %6.3f Rd = %6.3f E1 = %6.3f E2 = %6.3f\t\t\t\t\t\r", soundFFT.time, offsetTest, soundFFT.styk_hv, soundFFT.v, soundFFT.ny, soundFFT.reduktor_gl_obor, soundFFT.eng1_obor, soundFFT.eng2_obor);
			}
			else
			{
				cout << " Paused...\t\t\t\t\r";
			}
		}
		else
		{
			printf(" Time = %8.3f OffsetTest = %6.3f H = %6.3f VX = %6.3f STEP = %6.3f Rd = %6.3f E1 = %6.3f E2 = %6.3f | For Standalone using push [^]\r", soundFFT.time, offsetTest, soundFFT.styk_hv, soundFFT.v, soundFFT.ny, soundFFT.reduktor_gl_obor, soundFFT.eng1_obor, soundFFT.eng2_obor);
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
			soundFFT.p_vsu_zap = 1;
			soundFFT.p_vsu_ostanov = 0;
			break;
		case 'r':		//ВСУ останов
			soundFFT.p_vsu_ostanov = 1;
			soundFFT.p_vsu_zap = 0;
			soundFFT.p_vsu_hp = 0;
			break;
		case 'd':		//ВСУ ХП
			//soundFFT.p_vsu_zap = 0;
			soundFFT.p_vsu_hp = !soundFFT.p_vsu_hp;
			//soundFFT.p_vsu_ostanov = 0;
			break;
		case 'y':		//Дв1 запуск
			soundFFT.p_eng1_zap = 1;
			soundFFT.p_eng1_hp = 0;
			soundFFT.p_eng1_ostanov = 0;
			break;
		case 't':		//Дв1 hp
			soundFFT.p_eng1_hp = !soundFFT.p_eng1_hp;
			//soundFFT.p_eng1_hp = 1;
			//soundFFT.p_eng1_zap = 0;
			//soundFFT.p_eng1_ostanov = 0;
			break;
		case 'u':		//Дв1 останов
			soundFFT.p_eng1_ostanov = 1;
			soundFFT.p_eng1_zap = 0;
			soundFFT.p_eng1_hp = 0;
			break;
		case 'g':		//Дв2 hp
			soundFFT.p_eng2_hp = !soundFFT.p_eng2_hp;
			//soundFFT.p_eng2_hp = 1;
			//soundFFT.p_eng2_zap = 0;
			//soundFFT.p_eng2_ostanov = 0;
			break;
		case 'h':		//Дв2 запуск
			soundFFT.p_eng2_zap = 1;
			soundFFT.p_eng2_hp = 0;
			soundFFT.p_eng2_ostanov = 0;
			break;
		case 'j':		//Дв2 останов
			soundFFT.p_eng2_ostanov = 1;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng2_hp = 0;
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
		case 'B':
			soundFFT.rez_10 = !soundFFT.rez_10;//Хлопки винта 
			break;
		case 'K':
			soundFFT.rez_9 = !soundFFT.rez_9;//КО-50(обогреватель)
			break;
		case 'o':
			soundFFT.p_kran_perekr_1 = !soundFFT.p_kran_perekr_1;//Левый кран
			break;
		case 'p':
			soundFFT.p_kran_perekr_2 = !soundFFT.p_kran_perekr_2;//Правый кран
			break;
		case 's':
			soundFFT.v += 3.;//Увеличить скорость
			soundFFT.v = (soundFFT.v > 100.) ? 100. : soundFFT.v;
			break;
		case 'x':
			soundFFT.v -= 3.;//Уменьшить скорость
			soundFFT.v = (soundFFT.v < 0.) ? 0. : soundFFT.v;
			break;
		case 'k':
			soundFFT.p_reduktor_gl_crash = !soundFFT.p_reduktor_gl_crash;//Неисправность главного редуктора
			break;
		case ';':
			soundFFT.p_nar_s8 = !soundFFT.p_nar_s8;//НАР 8
			break;
		case 'l':
			soundFFT.p_nar_c13 = !soundFFT.p_nar_c13;//НАР 13
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
			soundFFT.rez_2 = !soundFFT.rez_2;//Аккумулятор
			break;
		case 'Q':
			soundFFT.p_trans_36_osn = !soundFFT.p_trans_36_osn;//36В
			break;
		case 'W':
			soundFFT.p_po500 = !soundFFT.p_po500;//115В
			break;
		case 'N':
			soundFFT.rez_3 = !soundFFT.rez_3;//НИП
			break;
		case 'T':
			soundFFT.rez_4 = !soundFFT.rez_4;//Насос расходного бака
			break;
		case '{':
			soundFFT.p_kran_poj_l = !soundFFT.p_kran_poj_l;//ПОЖ кран л
			break;
		case '}':
			soundFFT.p_kran_poj_r = !soundFFT.p_kran_poj_r;//ПОЖ кран п
			break;
		case '9':
			soundFFT.p_skv_on = !soundFFT.p_skv_on;//СКВ
			soundFFT.rez_8 = !soundFFT.rez_8;//СКВ
			break;
		case '-':
			soundFFT.rez_7 = !soundFFT.rez_7;//бибиби
			break;
		case '^':
			standAlone = !standAlone;
			break;
		case '7':
			soundFFT.rez_5 = !soundFFT.rez_5;//хз 3
			break;
		case '8':
			soundFFT.rez_6 = !soundFFT.rez_6;//хз 2
			break;
		case 'V':
			soundFFT.p_nasos = !soundFFT.p_nasos;//Насосная станция
			break;
		case 'i':
			soundFFT.rez_1 = !soundFFT.rez_1;//потребитель
			break;
		case 'I':
			soundFFT.p_kran_kolcev = !soundFFT.p_kran_kolcev;//потребитель
			break;
		case '$':
			soundFFT.p_ur_igla = !soundFFT.p_ur_igla;//потребитель
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
			soundFFT.p_eng1_ostanov = 0;
			soundFFT.p_eng2_ostanov = 0;
			soundFFT.p_eng1_lkorr = 1;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 1;
			soundFFT.p_eng1_rkorr = 0;
			soundFFT.p_eng2_rkorr = 0;
			statusEng1 = "NULL";
			statusEng2 = "NULL";
			statusRed = "NULL";
			break;
		case '<'://режим мг - редуктора на 1 дв (shift + ,)
			soundFFT.reduktor_gl_obor = helicopter.red_obor_mg1;
			soundFFT.eng1_obor = helicopter.eng_obor_mg;
			soundFFT.eng2_obor = 0;
			soundFFT.p_eng1_zap = 1;
			soundFFT.p_eng2_zap = 0;
			soundFFT.p_eng1_ostanov = 0;
			soundFFT.p_eng2_ostanov = 0;
			soundFFT.p_eng1_lkorr = 1;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 1;
			soundFFT.p_eng1_rkorr = 0;
			soundFFT.p_eng2_rkorr = 0;
			statusEng1 = "NULL";
			statusEng2 = "NULL";
			statusRed = "NULL";
			break;
		case '>'://режим мг - редуктора на 2 дв(shift + .)
			soundFFT.reduktor_gl_obor = helicopter.red_obor_mg2;
			soundFFT.eng1_obor = helicopter.eng_obor_mg;
			soundFFT.eng2_obor = helicopter.eng_obor_mg;
			soundFFT.p_eng1_zap = 1;
			soundFFT.p_eng2_zap = 1;
			soundFFT.p_eng1_ostanov = 0;
			soundFFT.p_eng2_ostanov = 0;
			soundFFT.p_eng1_lkorr = 1;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 1;
			soundFFT.p_eng1_rkorr = 0;
			soundFFT.p_eng2_rkorr = 0;
			statusEng1 = "NULL";
			statusEng2 = "NULL";
			statusRed = "NULL";
			break;
		case '?'://режим автомат - редуктора на 2 дв(shift + /)
			soundFFT.reduktor_gl_obor = helicopter.red_obor_avt;
			soundFFT.eng1_obor = helicopter.eng_obor_avt;
			soundFFT.eng2_obor = helicopter.eng_obor_avt;
			soundFFT.p_eng1_zap = 1;
			soundFFT.p_eng2_zap = 1;
			soundFFT.p_eng1_ostanov = 0;
			soundFFT.p_eng2_ostanov = 0;
			soundFFT.p_eng1_lkorr = 0;//Правая - левая коррекция
			soundFFT.p_eng2_lkorr = 0;
			soundFFT.p_eng1_rkorr = 1;
			soundFFT.p_eng2_rkorr = 1;
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