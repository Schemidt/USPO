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

void delayMs(double ms);

void kbHit();

double getTimeMs();

double lineInterpolation(double x0, double fx0, double x1, double fx1, double x);

double squareInterpolation(double x0, double fx0, double x1, double fx1, double x2, double fx2, double x);

float getOffset(string filename, float parameter);

double getParameterFromFile(string filename, double offset);

double getParameterFromVector(vector<double> value, vector<double> time, double offset);

int binSer(vector<double> value, vector<double> time, double offset);

SOUNDFFT soundFFT;
RED red;
Helicopter helicopter;

float test = 0;
bool avtOn = 0;
float delta = 0;
float offsetTest = 0;
float timeEnd = 0;
float timeStart = 0;
float pause = 0;
float timeReset = 0;

string statusEng1;
string statusEng2;
string statusRed;

float turn = 0;
float turnMg1 = 0;
float turnMg2 = 0;
float turnAvt = 0;
float turnMgEng1 = 0;
float turnAvtEng1 = 0;
float turnMgEng2 = 0;
float turnAvtEng2 = 0;
float turnRevRedAnsat = 0;
float offsetMgEng1 = 0;
float offsetAvtEng1 = 0;
float offsetMgEng2 = 0;
float offsetAvtEng2 = 0;
float offsetMg1 = 0;
float offsetMg2 = 0;
float offsetAvt = 0;
float offsetAnsatRev = 0;

int main(int argc, char* argv[])
{
	if (argc > 1)// если передаем аргументы, то argc будет больше 1(в зависимости от кол-ва аргументов)
	{
		if (argv[1] == mi_8_mtv5.modelName) { helicopter = mi_8_mtv5; }
		else if (argv[1] == mi_8_amtsh.modelName) { helicopter = mi_8_amtsh; }
		else if (argv[1] == mi_26.modelName) { helicopter = mi_26; }
		else if (argv[1] == mi_28.modelName) { helicopter = mi_28; }
		else if (argv[1] == ka_226.modelName) { helicopter = ka_226; }
		else if (argv[1] == ka_27.modelName) { helicopter = ka_27; }
		else if (argv[1] == ka_29.modelName) { helicopter = ka_29; }
		else if (argv[1] == ansat.modelName) { helicopter = ansat; }
		else
		{
			cout << " Unknown argument" << endl;
			helicopter = ka_29;
		}
	}
	else
	{
		helicopter = ka_29;
	}
	//else
	//{
	//	string ch;
	//	string Type;
	//	printf(" Types of Armed Forces:\n 1) VVS\n 2) VMF\n");

	//	while (!std::regex_match(ch, regex("[1-2]")))//повторяем ввод пока не будет цифра от 1 до 4
	//		ch = getch();//считываем буфер ввода

	//	switch (ch[0])
	//	{
	//	case '1':
	//		Type = "VVS";
	//		break;
	//	case '2':
	//		Type = "VMF";
	//		break;
	//	}
	//	//Type = "VVS";
	//	system("cls");
	//	if (Type == "VVS")
	//	{
	//		ch = "NULL";
	//		printf(" Choose Helicopter:\n 1) Mi-8 MTV-5\n 2) Mi-8 AMTSH\n 3) Mi-26\n 4) Mi-28\n 5) Ka-226\n 6) ANSAT\n");

	//		while (!std::regex_match(ch, regex("[1-6]")))//повторяем ввод пока не будет цифра от 1 до 4
	//			ch = getch();//считываем буфер ввода

	//		switch (ch[0])
	//		{
	//		case '1':
	//			helicopter = mi_8_mtv5;
	//			break;
	//		case '2':
	//			helicopter = mi_8_amtsh;
	//			break;
	//		case '3':
	//			helicopter = mi_26;
	//			break;
	//		case '4':
	//			helicopter = mi_28;
	//			break;
	//		case '5':
	//			helicopter = ka_226;
	//			break;
	//		case '6':
	//			helicopter = ansat;
	//			break;
	//		}
	//	}
	//	else if (Type == "VMF")
	//	{
	//		ch = "NULL";
	//		printf(" Choose Helicopter:\n 1) Ka-27M\n 2) Ka-29\n");

	//		while (!std::regex_match(ch, regex("[1-2]")))//повторяем ввод пока не будет цифра от 1 до 4
	//			ch = getch();//считываем буфер ввода

	//		switch (ch[0])
	//		{
	//		case '1':
	//			helicopter = ka_27;
	//			break;
	//		case '2':
	//			helicopter = ka_29;
	//			break;
	//		}
	//	}

	//}
	system("cls");
	std::cout << " Using " << helicopter.modelName << std::endl;
	helicopter.setPath(helicopter.modelName + "/");

	//test = 1;
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
	float currentTime = 0;
	float output = 0;

	vector <double> timeTest, eng1Test, eng2Test, redTest, highTest, velocityTest, tangazTest, stepTest;
	remove("test.txt");
	while (1)
	{
		delta = rt.timeS - currentTime;
		currentTime = rt.timeS;
		

		if (!rt.pExchOK)
		{		// Анализ  
			kbHit();
		}
		if (1)
		{
			if (!pause) //Программа не поставлена на паузу
			{
				
				//Блок штатной работы программы
				if (!test)
				{
					avtOn = soundFFT.p_eng2_rkorr & soundFFT.p_eng1_rkorr;

					if (helicopter.modelName == ansat.modelName)
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
						soundFFT.vsu_obor = (soundFFT.vsu_obor >(VSU_MAX_TURN * 0.35)) ? (VSU_MAX_TURN * 0.35) : soundFFT.vsu_obor;
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
					//
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
					//Возвращение в исходное состояние
					if (soundFFT.eng1_obor == 0)
					{
						soundFFT.p_eng1_ostanov = 0;
					}
					if (soundFFT.eng2_obor == 0)
					{
						soundFFT.p_eng2_ostanov = 0;
					}
				}
				else //тестовые циклограммы полеты для некоторых вертолетов
				{
					string ch;
					if (helicopter.modelName == "mi_8_mtv5" || helicopter.modelName == "mi_8_amtsh")
					{
						if (timeReset == 0)
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

						offsetTest += delta;
						soundFFT.eng2_obor = getParameterFromFile("test/mi_8_amtsh/Standart/eng2_p.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
						soundFFT.eng1_obor = getParameterFromFile("test/mi_8_amtsh/Standart/eng1_p.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
						soundFFT.reduktor_gl_obor = getParameterFromFile("test/mi_8_amtsh/Standart/red_p.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
						soundFFT.styk_hv = getParameterFromFile("test/mi_8_amtsh/Standart/h_p.txt", offsetTest);//
						soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
						soundFFT.osadki = getParameterFromFile("test/mi_8_amtsh/Standart/tangaz_p.txt", offsetTest);
						soundFFT.ny = getParameterFromFile("test/mi_8_amtsh/Standart/step_p.txt", offsetTest);//
						soundFFT.v = getParameterFromFile("test/mi_8_amtsh/Standart/v_p.txt", offsetTest);//
						//Признак работы теста
						soundFFT.p_model_stop = 0;
					}
					if (helicopter.modelName == "mi_28")
					{
						//Сброс параметров в начале теста
						if (timeReset == 0)
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
						if (hovering)
						{
							//Передача данных теста
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromFile("test/mi_28/Hovering/eng1.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.eng2_obor = getParameterFromFile("test/mi_28/Hovering/eng2.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.reduktor_gl_obor = getParameterFromFile("test/mi_28/Hovering/red.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.styk_hv = getParameterFromFile("test/mi_28/Hovering/h.txt", offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromFile("test/mi_28/Hovering/tangaz.txt", offsetTest);
							soundFFT.ny = getParameterFromFile("test/mi_28/Hovering/step.txt", offsetTest);//
							soundFFT.v = getParameterFromFile("test/mi_28/Hovering/v.txt", offsetTest);//
							soundFFT.p_vu3 = 1;
							soundFFT.p_model_stop = 0;//Признак работы теста
						}
						else if (skv)
						{
							//Передача данных теста
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromFile("test/mi_28/SKV/eng1_8s.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.eng2_obor = getParameterFromFile("test/mi_28/SKV/eng2_8s.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.reduktor_gl_obor = getParameterFromFile("test/mi_28/SKV/red_8s.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.styk_hv = getParameterFromFile("test/mi_28/SKV/h_8s.txt", offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromFile("test/mi_28/SKV/tangaz_8s.txt", offsetTest);
							soundFFT.ny = getParameterFromFile("test/mi_28/SKV/step_8s.txt", offsetTest);//
							soundFFT.v = getParameterFromFile("test/mi_28/SKV/v_8s.txt", offsetTest);//
							soundFFT.p_vu3 = 1;									  
							soundFFT.p_model_stop = 0;//Признак работы теста
						}
						else
						{
							//Передача данных теста
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromFile("test/mi_28/Standart/eng1_8.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.eng2_obor = getParameterFromFile("test/mi_28/Standart/eng2_8.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.reduktor_gl_obor = getParameterFromFile("test/mi_28/Standart/red_8.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.styk_hv = getParameterFromFile("test/mi_28/Standart/h_8.txt", offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromFile("test/mi_28/Standart/tangaz_8.txt", offsetTest);
							soundFFT.ny = getParameterFromFile("test/mi_28/Standart/step_8.txt", offsetTest);//
							soundFFT.v = getParameterFromFile("test/mi_28/Standart/v_8.txt", offsetTest);//
							soundFFT.p_vu3 = 1;
							soundFFT.p_model_stop = 0;//Признак работы теста
						}
					}
					if (helicopter.modelName == "ka_29")
					{
						//Сброс параметров в начале теста
						if (timeReset == 0)
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
								system("cls");
								printf(" Enter range (in seconds): [start] [end]\n ");
								cin >> timeStart;
								cin >> timeEnd;

								//ДВ1
								ifstream base("test/ka_27/hovering/eng1_7h.txt");
								while (!base.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									eng1Test.push_back(v);
								}
								base.close();

								//ДВ2
								ifstream base1("test/ka_27/hovering/eng2_7h.txt");
								while (!base1.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base1, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									eng2Test.push_back(v);
								}
								base1.close();

								//Ред
								ifstream base2("test/ka_27/hovering/red_7h.txt");
								while (!base2.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base2, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									redTest.push_back(v);
								}
								base2.close();

								//Скорость
								ifstream base3("test/ka_27/hovering/v_7h.txt");
								while (!base3.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base3, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									velocityTest.push_back(v);
								}
								base3.close();

								//Высота
								ifstream base4("test/ka_27/hovering/h_7h.txt");
								while (!base4.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base4, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									highTest.push_back(v);
								}
								base4.close();

								//Время
								ifstream base5("test/ka_27/hovering/h_7h.txt");
								while (!base5.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base5, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									timeTest.push_back(t);
								}
								base5.close();

								//Шаг
								ifstream base7("test/ka_27/hovering/step_7h.txt");
								while (!base7.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base7, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									stepTest.push_back(v);
								}
								base7.close();
								//Тангаж
								ifstream base8("test/ka_27/hovering/tangaz_7h.txt");
								while (!base8.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base8, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									tangazTest.push_back(v);
								}
								base8.close();
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
						if (hovering)
						{
							//Передача данных теста
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromVector(eng1Test, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.eng2_obor = getParameterFromVector(eng2Test, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.reduktor_gl_obor = getParameterFromVector(redTest, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.styk_hv = getParameterFromVector(highTest, timeTest, offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromVector(tangazTest, timeTest, offsetTest);//тангаж
							soundFFT.ny = getParameterFromVector(stepTest, timeTest, offsetTest);//
							soundFFT.v = getParameterFromVector(velocityTest, timeTest, offsetTest);//									  
							soundFFT.p_model_stop = 0;//Признак работы теста
						}
						else
						{

							//Передача данных теста
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromFile("test/ka_29/Standart/eng1_k.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.eng2_obor = getParameterFromFile("test/ka_29/Standart/eng2_k.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.reduktor_gl_obor = getParameterFromFile("test/Standart/red_k.txt", offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.styk_hv = getParameterFromFile("test/ka_29/Standart/h_k.txt", offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromFile("test/ka_29/Standart/tangaz_k.txt", offsetTest);
							soundFFT.ny = getParameterFromFile("test/ka_29/Standart/step_k.txt", offsetTest);//
							soundFFT.v = getParameterFromFile("test/ka_29/Standart/v_k.txt", offsetTest);//
							//Признак работы теста
							soundFFT.p_model_stop = 0;
						}
					}
					if (helicopter.modelName == "ka_27")
					{
						//Сброс параметров в начале теста
						if (timeReset == 0)
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
							
							if (hovering)
							{
								//ДВ1
								ifstream base("test/ka_27/hovering/eng1_7h.txt");
								while (!base.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									eng1Test.push_back(v);
								}
								base.close();

								//ДВ2
								ifstream base1("test/ka_27/hovering/eng2_7h.txt");
								while (!base1.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base1, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									eng2Test.push_back(v);
								}
								base1.close();

								//Ред
								ifstream base2("test/ka_27/hovering/red_7h.txt");
								while (!base2.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base2, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									redTest.push_back(v);
								}
								base2.close();

								//Скорость
								ifstream base3("test/ka_27/hovering/v_7h.txt");
								while (!base3.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base3, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									velocityTest.push_back(v);
								}
								base3.close();

								//Высота
								ifstream base4("test/ka_27/hovering/h_7h.txt");
								while (!base4.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base4, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									highTest.push_back(v);
								}
								base4.close();

								//Время
								ifstream base5("test/ka_27/hovering/h_7h.txt");
								while (!base5.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base5, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									timeTest.push_back(t);
								}
								base5.close();

								//Шаг
								ifstream base7("test/ka_27/hovering/step_7h.txt");
								while (!base7.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base7, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									stepTest.push_back(v);
								}
								base7.close();
								//Тангаж
								ifstream base8("test/ka_27/hovering/tangaz_7h.txt");
								while (!base8.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base8, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									tangazTest.push_back(v);
								}
								base8.close();
							}
							else
							{
								//ДВ1
								ifstream base("test/ka_27/Standart/eng1_7.txt");
								while (!base.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									eng1Test.push_back(v);
								}
								base.close();

								//ДВ2
								ifstream base1("test/ka_27/Standart/eng2_7.txt");
								while (!base1.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base1, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									eng2Test.push_back(v);
								}
								base1.close();

								//Ред
								ifstream base2("test/ka_27/Standart/red_7.txt");
								while (!base2.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base2, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									redTest.push_back(v);
								}
								base2.close();

								//Скорость
								ifstream base3("test/ka_27/Standart/v_7.txt");
								while (!base3.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base3, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									velocityTest.push_back(v);
								}
								base3.close();

								//Высота
								ifstream base4("test/ka_27/Standart/h_7.txt");
								while (!base4.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base4, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									highTest.push_back(v);
								}
								base4.close();

								//Тангаж
								ifstream base5("test/ka_27/Standart/tangaz_7.txt");
								while (!base5.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base5, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									tangazTest.push_back(v);
								}
								base5.close();

								//Шаг
								ifstream base7("test/ka_27/Standart/step_7.txt");
								while (!base7.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base7, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									stepTest.push_back(v);
								}
								base7.close();

								//Время
								ifstream base6("test/ka_27/Standart/h_7.txt");
								while (!base6.eof())
								{
									string str;
									float t = 0;
									float v = 0;
									getline(base6, str);
									sscanf(str.c_str(), "%f %f", &t, &v);
									timeTest.push_back(t);
								}
								base6.close();
							}
							offsetTest = timeStart;
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							delta = 0;
							timeReset = 1;
							system("cls");
						}

						if (hovering)
						{
							//Передача данных теста
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromVector(eng1Test,timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.eng2_obor = getParameterFromVector(eng2Test, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.reduktor_gl_obor = getParameterFromVector(redTest, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.styk_hv = getParameterFromVector(highTest, timeTest, offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromVector(tangazTest, timeTest, offsetTest);//тангаж
							soundFFT.ny = getParameterFromVector(stepTest, timeTest, offsetTest);//
							soundFFT.v = getParameterFromVector(velocityTest, timeTest, offsetTest);//									  
							soundFFT.p_model_stop = 0;//Признак работы теста
						}
						else
						{
							//Передача данных теста
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromVector(eng1Test, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.eng2_obor = getParameterFromVector(eng2Test, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.reduktor_gl_obor = getParameterFromVector(redTest, timeTest, offsetTest);//функция выбирающая обороты дв относительно времени от начала разгона
							soundFFT.styk_hv = getParameterFromVector(highTest, timeTest, offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromVector(tangazTest, timeTest, offsetTest);//тангаж
							soundFFT.ny = getParameterFromVector(stepTest, timeTest, offsetTest);//шаг
							soundFFT.v = getParameterFromVector(velocityTest, timeTest, offsetTest) * 0.28;//										  
							soundFFT.p_model_stop = 0;//Признак работы теста
							
						}

						output += delta;
						if (output>=0.01)
						{
							FILE* test = fopen("test.txt", "at");
							fprintf(test, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", soundFFT.eng1_obor, soundFFT.eng2_obor, soundFFT.reduktor_gl_obor, soundFFT.styk_hv, soundFFT.osadki, soundFFT.ny, soundFFT.v, soundFFT.time);
							fclose(test);
							output = 0;
						}
					}
					//Тест закончился
					if (soundFFT.time + timeStart > timeEnd)
					{
						eng1Test.clear();
						eng2Test.clear();
						redTest.clear();
						tangazTest.clear();
						highTest.clear();
						velocityTest.clear();
						timeTest.clear();
						stepTest.clear();
						//Признак работы теста
						soundFFT.p_model_stop = 1;
						system("cls");
						cout << "Test ended..."<< endl;
						cout << "Continue? [y/n]" << endl;
						while (!std::regex_match(ch, regex("[yn]")))//повторяем ввод пока не будет цифра от 1 до 4
							ch = getch();//считываем буфер ввода
						switch (ch[0])
						{
						case 'y':
							timeReset = 0;
							break;
						case 'n':
							return 1;
							break;
						}
					}
				}
				soundFFT.time = currentTime;
				printf(" Time = %8.3f OffsetTest = %6.3f H = %6.3f VX = %6.3f STEP = %6.3f Rd = %6.3f E1 = %6.3f E2 = %6.3f\t\t\t\t\t\r", soundFFT.time, offsetTest, soundFFT.styk_hv, soundFFT.v, soundFFT.ny, soundFFT.reduktor_gl_obor, soundFFT.eng1_obor, soundFFT.eng2_obor);

			}
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
	if(_kbhit()){
		c = _getch_nolock();

		switch(c)
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
				soundFFT.v = (soundFFT.v > 100.)? 100.:soundFFT.v;
				break;
			case 'x':
				soundFFT.v -= 3.;//Уменьшить скорость
				soundFFT.v = (soundFFT.v < 0.)? 0.:soundFFT.v;
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
				soundFFT.p_nasos_podk_1= !soundFFT.p_nasos_podk_1;//Топливный насос левый
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

double lineInterpolation(double x0, double fx0, double x1, double fx1, double x)
{
	double fx, a0, a1, a2;
	if (x0<x1 && x>x1)
	{
		return fx1;
	}
	if (x0<x1 && x<x0)
	{
		return fx0;
	}
	if (x0>x1 && x<x1)
	{
		return fx1;
	}
	if (x0>x1 && x>x0)
	{
		return fx0;
	}

	return	fx = fx0 + ((fx1 - fx0) / (x1 - x0))*(x - x0);
}

double squareInterpolation(double x0, double fx0, double x1, double fx1, double x2, double fx2, double x)
{
	double fx, a0, a1, a2;
	if (x0<x2 && x>x2)
	{
		return fx2;
	}
	if (x0<x2 && x<x0)
	{
		return fx0;
	}
	if (x0>x2 && x<x2)
	{
		return fx2;
	}
	if (x0>x2 && x>x0)
	{
		return fx0;
	}

	//если квадратичная интерполяция не работает - берем линейную
	if (x1 == x0 | x2 == x1)
	{
		return	lineInterpolation(x0, fx0, x1, fx1, x);

	}
	else
	{
		a2 = ((fx2 - fx0) / ((x2 - x0)*(x2 - x1))) - ((fx1 - fx0) / ((x1 - x0)*(x2 - x1)));
		a1 = ((fx1 - fx0) / (x1 - x0)) - (a2*(x1 + x0));
		a0 = fx0 - a1 * x0 - a2 * x0*x0;
		return fx = a0 + a1 * x + a2*x*x;

	}

}

double getParameterFromFile(string filename, double offset)
{
	double turn = 0;
	double t = 0;
	double v = 0;
	int i = 0;
	vector <double> time, value;

	//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)
	string str;
	ifstream base(filename);
	while (!base.eof())
	{
		getline(base, str);
		sscanf(str.c_str(), "%lf %lf", &t, &v);
		time.push_back(t);
		value.push_back(v);
	}
	base.close();

	return turn = getParameterFromVector(value, time, offset);
}

float getPitch(float offset, string filename, float parameter)
{
	float new_pitch;
	float turn = 0;
	float t = 0;
	float v = 0;
	int i = 0;
	vector <float> time, value;

	//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)
	string str;
	ifstream base(filename);
	while (!base.eof())
	{
		getline(base, str);
		sscanf(str.c_str(), "%f %f", &t, &v);
		time.push_back(t);
		value.push_back(v);
	}
	base.close();

	float x, x0, x1, x2, fx, fx0, fx1, fx2, a0, a1, a2;
	int n = time.size();

	for (i = 0; i < n; i++)
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
					x = offset; x0 = time[i]; fx0 = value[i]; x1 = time[i + 1]; fx1 = value[i + 1]; x2 = time[i + 2]; fx2 = value[i + 2];
				}
				if (i + 1 == n)
				{
					x = offset; x0 = time[i - 2]; fx0 = value[i - 2]; x1 = time[i - 1]; fx1 = value[i - 1]; x2 = time[i]; fx2 = value[i];
				}
			}
			else
			{
				x = offset; x0 = time[i - 1]; fx0 = value[i - 1]; x1 = time[i]; fx1 = value[i]; x2 = time[i + 1]; fx2 = value[i + 1];
			}
			//если квадратичная интерполяция не работает - берем линейную
			if (x1 == x0 | x2 == x1)
			{
				turn = lineInterpolation(x0, fx0, x1, fx1, x);
			}
			else
			{
				turn = squareInterpolation(x0, fx0, x1, fx1, x2, fx2, x);
			}
		}
	}
	if (turn <= 0)
		new_pitch = 1;
	else
		new_pitch = parameter / turn;	//вычисляем результирующий Pitch на основе отношениz настоящего уровня оборотов к базовому (получен при записи аудио-файла)

	return new_pitch;
}

float getOffset(string filename, float parameter)
{
	float new_offset = 0;
	float turn = 0;
	int i = 0;

	float t = 0;
	float v = 0;
	vector <float> time, value;

	//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)
	string str;
	ifstream base(filename);
	while (!base.eof())
	{
		getline(base, str);
		sscanf(str.c_str(), "%f %f", &t, &v);
		time.push_back(t);
		value.push_back(v);
	}
	base.close();
	int n = time.size();

	if (parameter < 0)
		turn = 0;
	else
		turn = parameter;

	float x, x0, x1, x2, fx, fx0, fx1, fx2, a0, a1, a2;

	if (value[0] <= value[n - 1])
	{
		for (i = 0; i < n; i++)
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
						x = turn; x0 = value[i - 1]; fx0 = time[i - 1]; x1 = value[i]; fx1 = time[i]; x2 = value[i + 1]; fx2 = time[i + 1];
					}
					if (i + 1 == n)
					{
						x = turn; x0 = value[i - 2]; fx0 = time[i - 2]; x1 = value[i - 1]; fx1 = time[i - 1]; x2 = value[i]; fx2 = time[i];
					}
				}
				else
				{
					x = turn; x0 = value[i]; fx0 = time[i]; x1 = value[i + 1]; fx1 = time[i + 1]; x2 = value[i + 2]; fx2 = time[i + 2];
				}

				new_offset = squareInterpolation(x0, fx0, x1, fx1, x2, fx2, x);
			}

		}

	}
	else
	{
		for (i = 0; i < n; i++)
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
						x = turn; x0 = value[i - 1]; fx0 = time[i - 1]; x1 = value[i]; fx1 = time[i]; x2 = value[i + 1]; fx2 = time[i + 1];
					}
					if (i + 1 == n)
					{
						x = turn; x0 = value[i - 2]; fx0 = time[i - 2]; x1 = value[i - 1]; fx1 = time[i - 1]; x2 = value[i]; fx2 = time[i];
					}
				}
				else
				{
					x = turn; x0 = value[i]; fx0 = time[i]; x1 = value[i + 1]; fx1 = time[i + 1]; x2 = value[i + 2]; fx2 = time[i + 2];
				}

				new_offset = squareInterpolation(x0, fx0, x1, fx1, x2, fx2, x);
			}

		}
	}

	if (new_offset <= 0)
		new_offset = 0;

	return new_offset;

}

double getParameterFromVector(vector<double> value, vector<double> time, double offset)
{
	double turn = 0;
	int n = time.size();
	double x, x0, x1, x2, fx, fx0, fx1, fx2, a0, a1, a2;

	if (offset < time[0])
	{
		return turn = value[0];//достаем обороты из базы
	}
	else if (offset > time[n - 1])//отметка не совпала с базой
	{
		return turn = value[n - 1];//достаем обороты из базы
	}
	else
	{
		n = binSer(value,time,offset);
	}
	//Выбираем 3 точки (вариант -1 0 +1)
	if (n - 1 == -1)
	{
		x = offset; x0 = time[n]; fx0 = value[n]; x1 = time[n + 1]; fx1 = value[n + 1]; x2 = time[n + 2]; fx2 = value[n + 2];
	}
	else if (n + 1 == time.size())
	{
		x = offset; x0 = time[n - 2]; fx0 = value[n - 2]; x1 = time[n - 1]; fx1 = value[n - 1]; x2 = time[n]; fx2 = value[n];
	}
	else 
	{
		x = offset; x0 = time[n - 1]; fx0 = value[n - 1]; x1 = time[n]; fx1 = value[n]; x2 = time[n + 1]; fx2 = value[n + 1];
	}
	////Выбираем 3 точки (вариант 0 +1 +2)
	//if (n + 1 == time.size())
	//{
	//	x = offset; x0 = time[n - 2]; fx0 = value[n - 2]; x1 = time[n - 1]; fx1 = value[n - 1]; x2 = time[n]; fx2 = value[n];
	//}
	//else if (n + 2 == time.size())
	//{
	//	x = offset; x0 = time[n - 1]; fx0 = value[n - 1]; x1 = time[n]; fx1 = value[n]; x2 = time[n + 1]; fx2 = value[n + 1];
	//}
	//else 
	//{
	//	x = offset; x0 = time[n]; fx0 = value[n]; x1 = time[n + 1]; fx1 = value[n + 1]; x2 = time[n + 2]; fx2 = value[n + 2];
	//}
	
	//если квадратичная интерполяция не работает - берем линейную
	if (x1 == x0 | x2 == x1)
	{
		turn = lineInterpolation(x0, fx0, x1, fx1, x);
	}
	else
	{
		turn = squareInterpolation(x0, fx0, x1, fx1, x2, fx2, x);
	}

	return turn;
}
//Функция жрущая файл с любым количеством столбцев, записывает их в переданные параметры
template<typename... T>//WIP
float getParameterFromFile(string filename, float offset,const T*... args)
{
	float t = 0;
	float v = 0;
	vector <float> time, value;

	//данные в базе должны храниться в строках парами, по паре в каждой строке (не больше)
	string str;
	ifstream base(filename);
	while (!base.eof())
	{
		getline(base, str);
		sscanf(str.c_str(), "%f %f", &t, &v);
		time.push_back(t);
		value.push_back(v);
	}
	base.close();

	for (auto it : std::initializer_list<float> args)
	{

	}

	return 1;
}

int binSer(vector<double> value, vector<double> time, double offset)
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