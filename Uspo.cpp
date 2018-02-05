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

float lineInterpolation(float x0, float fx0, float x1, float fx1, float x);

float squareInterpolation(float x0, float fx0, float x1, float fx1, float x2, float fx2, float x);

float getOffset(string filename, float parameter);

float getParameterFromFile(string filename, float offset);

float getParameterFromVector(vector<float> value, vector<float> time, float offset);

SOUNDFFT soundFFT;
RED red;
Helicopter helicopter;

float test = 0;
bool avtOn = 0;
float delta = 0;
float offsetTest = 0;
float timeEnd = 0;
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
	if (argc > 1)// ���� �������� ���������, �� argc ����� ������ 1(� ����������� �� ���-�� ����������)
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
			return 0;
		}
	}
	//else
	//{
	//	string ch;
	//	string Type;
	//	printf(" Types of Armed Forces:\n 1) VVS\n 2) VMF\n");

	//	while (!std::regex_match(ch, regex("[1-2]")))//��������� ���� ���� �� ����� ����� �� 1 �� 4
	//		ch = getch();//��������� ����� �����

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

	//		while (!std::regex_match(ch, regex("[1-6]")))//��������� ���� ���� �� ����� ����� �� 1 �� 4
	//			ch = getch();//��������� ����� �����

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

	//		while (!std::regex_match(ch, regex("[1-2]")))//��������� ���� ���� �� ����� ����� �� 1 �� 4
	//			ch = getch();//��������� ����� �����

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
	helicopter = ka_27;
	system("cls");
	std::cout << "Using " << helicopter.modelName << std::endl;
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

	if (!shaInit())				// ������������� ����� ������ 
		return 0;
	InitRealTime(1);
	bool hovering = 0;
	float currentTime = 0;

	vector <float> timeTest, eng1Test, eng2Test, redTest, highTest, velocityTest, tangazTest, stepTest;
	while (1)
	{
		delta = rt.timeS - currentTime;
		currentTime = rt.timeS;

		if (!rt.pExchOK)
		{		// ������  
			kbHit();
		}
		if (1)
		{
			if (!pause) //��������� �� ���������� �� �����
			{
				
				//���� ������� ������ ���������
				if (!test)
				{
					avtOn = soundFFT.p_eng2_rkorr & soundFFT.p_eng1_rkorr;

					if (helicopter.modelName == ansat.modelName)
					{
						bool case1 = soundFFT.eng1_obor >= ANSAT_ENG_REV_TURN & soundFFT.eng2_obor >= ANSAT_ENG_REV_TURN;//������ ��� �����
						bool case2 = soundFFT.eng1_obor < ANSAT_ENG_REV_TURN & soundFFT.eng2_obor < ANSAT_ENG_REV_TURN;//���������� ������
						bool case3 = (soundFFT.eng1_obor < ANSAT_ENG_REV_TURN & soundFFT.eng2_obor >= ANSAT_ENG_REV_TURN) | (soundFFT.eng1_obor >= ANSAT_ENG_REV_TURN & soundFFT.eng2_obor < ANSAT_ENG_REV_TURN);//���������� ������

																																																				 //��������� 1
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
						//��������� 2
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

						//��������
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

						//��������� 1
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
						//��������� 2
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

						//��������
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
								turnMg2 = getParameterFromFile(helicopter.fullName["red_on_mg"], offsetMg2) * turnMg1 / helicopter.red_obor_mg1;
								offsetMg2 += delta;
								turnAvt = getParameterFromFile(helicopter.fullName["red_mg_avt"], offsetAvt) * turnMg1 / helicopter.red_obor_mg1 * turnMg2 / helicopter.red_obor_mg2;
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
										turnMg2 = getParameterFromFile(helicopter.fullName["red_on_mg"], offsetMg2) * turnMg1 / helicopter.red_obor_mg1;
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

					//�������� ��������� ���
					if (soundFFT.p_vsu_hp & !soundFFT.p_vsu_zap)
					{
						if (soundFFT.vsu_obor < (VSU_MAX_TURN * 0.35))
							soundFFT.vsu_obor += (VSU_MAX_TURN * 0.35) / 8. * (delta);
						soundFFT.vsu_obor = (soundFFT.vsu_obor >(VSU_MAX_TURN * 0.35)) ? (VSU_MAX_TURN * 0.35) : soundFFT.vsu_obor;
					}
					//�������� ��������� ��� ����
					if (!soundFFT.p_vsu_hp & !soundFFT.p_vsu_zap)
					{

						soundFFT.vsu_obor -= (VSU_MAX_TURN * 0.35) / 2. * (delta);
						soundFFT.vsu_obor = (soundFFT.vsu_obor < 0) ? 0 : soundFFT.vsu_obor;
					}
					//������ ���
					if (soundFFT.p_vsu_zap)
					{
						soundFFT.p_vsu_ostanov = 0;
						if (soundFFT.vsu_obor < VSU_MAX_TURN)
							soundFFT.vsu_obor += VSU_MAX_TURN / 10. * (delta);
						soundFFT.vsu_obor = (soundFFT.vsu_obor > VSU_MAX_TURN) ? VSU_MAX_TURN : soundFFT.vsu_obor;
					}
					//��������� ���
					if (soundFFT.p_vsu_ostanov && soundFFT.vsu_obor != 0)
					{
						soundFFT.vsu_obor -= VSU_MAX_TURN / 12. * (delta);
						//������� ��� �� ������ ������ ���� 0
						soundFFT.vsu_obor = (soundFFT.vsu_obor < 0) ? 0 : soundFFT.vsu_obor;
					}
					//
					if (soundFFT.vsu_obor == 0)
						soundFFT.p_vsu_ostanov = 0;


					//�������� ��������� ��������� 1
					if (soundFFT.p_eng1_hp && soundFFT.eng1_obor < 20 && !soundFFT.p_eng1_ostanov && !soundFFT.p_eng1_zap)
					{
						statusEng1 = "eng_hp";
						soundFFT.eng1_obor += (helicopter.eng_obor_mg * 0.3) / 22.*(delta);
					}
					//�������� ��������� ��������� 1 ����
					if (!soundFFT.p_eng1_hp && soundFFT.eng1_obor > 0 && !soundFFT.p_eng1_ostanov && !soundFFT.p_eng1_zap)
					{
						statusEng1 = "eng_hp";
						soundFFT.eng1_obor -= (helicopter.eng_obor_mg * 0.3) / 8.*(delta);
						soundFFT.eng1_obor = (soundFFT.eng1_obor < 0) ? 0 : soundFFT.eng1_obor;
					}
					//�������� ��������� ��������� 2
					if (soundFFT.p_eng2_hp && soundFFT.eng2_obor < 20 && !soundFFT.p_eng2_ostanov && !soundFFT.p_eng2_zap)
					{
						statusEng2 = "eng_hp";
						soundFFT.eng2_obor += (helicopter.eng_obor_mg * 0.3) / 22.*(delta);
					}
					//�������� ��������� ��������� 2 ����
					if (!soundFFT.p_eng2_hp && soundFFT.eng2_obor > 0 && !soundFFT.p_eng2_ostanov && !soundFFT.p_eng2_zap)
					{
						statusEng2 = "eng_hp";
						soundFFT.eng2_obor -= (helicopter.eng_obor_mg * 0.3) / 8.*(delta);
						soundFFT.eng2_obor = (soundFFT.eng2_obor < 0) ? 0 : soundFFT.eng2_obor;
					}
					//����������� � �������� ���������
					if (soundFFT.eng1_obor == 0)
					{
						soundFFT.p_eng1_ostanov = 0;
					}
					if (soundFFT.eng2_obor == 0)
					{
						soundFFT.p_eng2_ostanov = 0;
					}
				}
				else //�������� ����������� ������ ��� ��������� ����������
				{
					string ch;
					if (helicopter.modelName == "mi_8_mtv5" || helicopter.modelName == "mi_8_amtsh")
					{
						if (timeReset == 0)
						{
							soundFFT.p_model_stop = 1;
							system("cls");
							printf(" TEST:\n 1) 0 - 200\n 2) 203 - 292\n 3) 295 - 434\n 4) 437 - 586\n 5) 589 - 688\n 6) 691 - 1100\n 7) 1103 - 1322\n 8) 1325 - 1589\n");

							while (!std::regex_match(ch, regex("[1-8]")))//��������� ���� ���� �� ����� ����� �� 1 �� 4
								ch = getch();//��������� ����� �����

							switch (ch[0])
							{
							case '1':
								offsetTest = 0;
								timeEnd = 200;
								break;
							case '2':
								offsetTest = 203;
								timeEnd = 292;
								break;
							case '3':
								offsetTest = 295;
								timeEnd = 434;
								break;
							case '4':
								offsetTest = 437;
								timeEnd = 586;
								break;
							case '5':
								offsetTest = 589;
								timeEnd = 688;
								break;
							case '6':
								offsetTest = 691;
								timeEnd = 1100;
								break;
							case '7':
								offsetTest = 1103;
								timeEnd = 1322;
								break;
							case '8':
								offsetTest = 1325;
								timeEnd = 1589;
								break;
							}
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							timeReset = 1;
							system("cls");
						}

						offsetTest += delta;
						soundFFT.eng2_obor = getParameterFromFile("test/eng2_k.txt", offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
						soundFFT.eng1_obor = getParameterFromFile("test/eng1_k.txt", offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
						soundFFT.reduktor_gl_obor = getParameterFromFile("test/red_k.txt", offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
						soundFFT.styk_hv = getParameterFromFile("test/h_k.txt", offsetTest);//
						soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
						soundFFT.osadki = getParameterFromFile("test/tangaz_k.txt", offsetTest);
						soundFFT.ny = getParameterFromFile("test/step_k.txt", offsetTest);//
						soundFFT.v = getParameterFromFile("test/v_k.txt", offsetTest) * 0.2777;//
						//������� ������ �����
						soundFFT.p_model_stop = 0;
					}
					if (helicopter.modelName == "ka_29")
					{
						//����� ���������� � ������ �����
						if (timeReset == 0)
						{
							soundFFT.p_model_stop = 1;
							system("cls");
							printf(" TEST:\n 1) 0 - 75\n 2) 77 - 429\n 3) 431 - 701\n 4) 703 - 858\n 5) 860 - 920\n 6) 922 - 1387\n 7) 1389 - 1919\n");

							while (!std::regex_match(ch, regex("[1-8]")))//��������� ���� ���� �� ����� ����� �� 1 �� 4
								ch = getch();//��������� ����� �����

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
							}
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							timeReset = 1;
							system("cls");
						}
						
						//�������� ������ �����
						offsetTest += delta;
						soundFFT.eng1_obor = getParameterFromFile("test/eng1_k.txt", offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
						soundFFT.eng2_obor = getParameterFromFile("test/eng2_k.txt", offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
						soundFFT.reduktor_gl_obor = getParameterFromFile("test/red_k.txt", offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
						soundFFT.styk_hv = getParameterFromFile("test/h_k.txt", offsetTest);//
						soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
						soundFFT.osadki = getParameterFromFile("test/tangaz_k.txt", offsetTest);
						soundFFT.ny = getParameterFromFile("test/step_k.txt", offsetTest);//
						soundFFT.v = getParameterFromFile("test/v_k.txt", offsetTest);//
						//������� ������ �����
						soundFFT.p_model_stop = 0;
					}
					if (helicopter.modelName == "ka_27")
					{
						//����� ���������� � ������ �����
						if (timeReset == 0)
						{
							soundFFT.p_model_stop = 1;
							system("cls");
							printf(" Choose type:\n 1) Standart\n 2) Hovering\n");

							while (!std::regex_match(ch, regex("[1-2]")))//��������� ���� ���� �� ����� ����� �� 1 �� 4
								ch = getch();//��������� ����� �����

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
							cin >> offsetTest;
							cin >> timeEnd;
							
							if (hovering)
							{
								//��1
								ifstream base("test/eng1_7h.txt");
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

								//��2
								ifstream base1("test/eng2_7h.txt");
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

								//���
								ifstream base2("test/red_7h.txt");
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

								//��������
								ifstream base3("test/v_7h.txt");
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

								//������
								ifstream base4("test/h_7h.txt");
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

								//�����
								ifstream base5("test/h_7h.txt");
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
							}
							else
							{
								//��1
								ifstream base("test/eng1_7.txt");
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

								//��2
								ifstream base1("test/eng2_7.txt");
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

								//���
								ifstream base2("test/red_7.txt");
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

								//��������
								ifstream base3("test/v_7.txt");
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

								//������
								ifstream base4("test/h_7.txt");
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

								//������
								ifstream base5("test/tangaz_7.txt");
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

								//���
								ifstream base7("test/step_7.txt");
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

								//�����
								ifstream base6("test/h_7.txt");
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
							soundFFT.time = 0;
							rt.timeS = 0;
							currentTime = 0;
							delta = 0;
							timeReset = 1;
							system("cls");
						}

						if (hovering)
						{
							//�������� ������ �����
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromVector(eng1Test,timeTest, offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
							soundFFT.eng2_obor = getParameterFromVector(eng2Test, timeTest, offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
							soundFFT.reduktor_gl_obor = getParameterFromVector(redTest, timeTest, offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
							soundFFT.styk_hv = getParameterFromVector(highTest, timeTest, offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = 0;
							soundFFT.ny = getParameterFromVector(stepTest, timeTest, offsetTest);//
							soundFFT.v = getParameterFromVector(velocityTest, timeTest, offsetTest) * 0.28;//									  
							soundFFT.p_model_stop = 0;//������� ������ �����
						}
						else
						{
							//�������� ������ �����
							offsetTest += delta;
							soundFFT.eng1_obor = getParameterFromVector(eng1Test, timeTest, offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
							soundFFT.eng2_obor = getParameterFromVector(eng2Test, timeTest, offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
							soundFFT.reduktor_gl_obor = getParameterFromVector(redTest, timeTest, offsetTest);//������� ���������� ������� �� ������������ ������� �� ������ �������
							soundFFT.styk_hv = getParameterFromVector(highTest, timeTest, offsetTest);//
							soundFFT.styk_hv = (soundFFT.styk_hv < 0) ? 0 : soundFFT.styk_hv;
							soundFFT.osadki = getParameterFromVector(tangazTest, timeTest, offsetTest);//������
							soundFFT.ny = getParameterFromVector(stepTest, timeTest, offsetTest);//���
							soundFFT.v = getParameterFromVector(velocityTest, timeTest, offsetTest) * 0.28;//										  
							soundFFT.p_model_stop = 0;//������� ������ �����
						}
					}
					//���� ����������
					if (offsetTest > timeEnd)
					{
						eng1Test.clear();
						eng2Test.clear();
						redTest.clear();
						tangazTest.clear();
						highTest.clear();
						velocityTest.clear();
						timeTest.clear();
						stepTest.clear();
						//������� ������ �����
						soundFFT.p_model_stop = 1;
						system("cls");
						cout << "Test ended..."<< endl;
						cout << "Continue? [y/n]" << endl;
						while (!std::regex_match(ch, regex("[yn]")))//��������� ���� ���� �� ����� ����� �� 1 �� 4
							ch = getch();//��������� ����� �����
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
			}
		}
		if (rt.pExchOK) 
		{
			rt.pExchOK = 0;
		}
		printf("Time = %10.3f VSU = %6.3f H = %6.3f VX = %6.3f STEP = %6.3f VY = %6.3f Rd = %6.3f E1 = %6.3f E2 = %6.3f\r", soundFFT.time, soundFFT.vsu_obor, soundFFT.styk_hv, soundFFT.v, soundFFT.ny, soundFFT.vy, soundFFT.reduktor_gl_obor, soundFFT.eng1_obor, soundFFT.eng2_obor);
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
				soundFFT.p_crash = !soundFFT.p_crash;//������������ � ������������
				break;		
			case '1':		
				soundFFT.obj_nos = !soundFFT.obj_nos;//������� ����� �������� ������ �����
				soundFFT.obj_hv = !soundFFT.obj_hv;//������� ������ �������� ������ �����
				soundFFT.obj_l = !soundFFT.obj_l;//������� ����� ������ �����
				soundFFT.obj_r = !soundFFT.obj_r;//������� ������ ������ �����
				break;		
			case '2':
				soundFFT.p_eng1_pomp = !soundFFT.p_eng1_pomp;//������ 1�� ���������
				break;		
			case '3':
				soundFFT.p_eng2_pomp = !soundFFT.p_eng2_pomp;//������ 2�� ���������
				break;		
			case '4':
				soundFFT.p_ur_ataka = !soundFFT.p_ur_ataka;//������� ���������� ��
				break;		
			case '5':
				soundFFT.p_spo_upk = !soundFFT.p_spo_upk;//������� ���������� ��� ���
				break;		
			case 'q':		
				soundFFT.p_pts = !soundFFT.p_pts;//���
				break;	
			case 'J':
				soundFFT.p_vu1 = !soundFFT.p_vu1;//��
				break;
			case 'w':		
				//
				break;		
			case 'e':		//��� ������
				soundFFT.p_vsu_hp = 0;
				soundFFT.p_vsu_zap = 1;
				soundFFT.p_vsu_ostanov = 0;
				break;		
			case 'r':		//��� �������
				soundFFT.p_vsu_ostanov = 1;
				soundFFT.p_vsu_zap = 0;
				soundFFT.p_vsu_hp = 0;
				break;		
			case 'd':		//��� ��
				//soundFFT.p_vsu_zap = 0;
				soundFFT.p_vsu_hp = !soundFFT.p_vsu_hp;
				//soundFFT.p_vsu_ostanov = 0;
				break;		
			case 'y':		//��1 ������
				soundFFT.p_eng1_zap = 1;
				soundFFT.p_eng1_hp = 0;
				soundFFT.p_eng1_ostanov = 0;
				break;
			case 't':		//��1 hp
				soundFFT.p_eng1_hp = !soundFFT.p_eng1_hp;
				//soundFFT.p_eng1_hp = 1;
				//soundFFT.p_eng1_zap = 0;
				//soundFFT.p_eng1_ostanov = 0;
				break;
			case 'u':		//��1 �������
				soundFFT.p_eng1_ostanov = 1;
				soundFFT.p_eng1_zap = 0;
				soundFFT.p_eng1_hp = 0;
				break;
			case 'g':		//��2 hp
				soundFFT.p_eng2_hp = !soundFFT.p_eng2_hp;
				//soundFFT.p_eng2_hp = 1;
				//soundFFT.p_eng2_zap = 0;
				//soundFFT.p_eng2_ostanov = 0;
				break;
			case 'h':		//��2 ������
				soundFFT.p_eng2_zap = 1;
				soundFFT.p_eng2_hp = 0;
				soundFFT.p_eng2_ostanov = 0;
				break;
			case 'j':		//��2 �������
				soundFFT.p_eng2_ostanov = 1;
				soundFFT.p_eng2_zap = 0;
				soundFFT.p_eng2_hp = 0;
				break;
			case 'a':
				soundFFT.eng1_obor += .5;//������ ������������� �������� ���������� (�������� �� ��������)
				soundFFT.eng2_obor += .5;
				break;
			case 'z':
				soundFFT.eng1_obor -= .5;//������ ������������� �������� ���������� (�������� �� ��������)
				soundFFT.eng2_obor -= .5;
				break;
			case 'c':
				soundFFT.master_gain -= .1f;//��������� ���������
				break;
			case 'v':
				soundFFT.master_gain += .1f;//��������� ���������
				break;
			case 'n':
				soundFFT.master_gain = 0;//������ ����
				break;
			case 'm':
				soundFFT.master_gain = 1;//���� �� ��������
				break;
			case 'b':
				soundFFT.tormoz_vint = !soundFFT.tormoz_vint;//������ �����
				break;
			case 'B':
				soundFFT.rez_10 = !soundFFT.rez_10;//������ ����� 
				break;
			case 'K':
				soundFFT.rez_9 = !soundFFT.rez_9;//��-50(������������)
				break;
			case 'o':
				soundFFT.p_kran_perekr_1 = !soundFFT.p_kran_perekr_1;//����� ����
				break;
			case 'p':
				soundFFT.p_kran_perekr_2 = !soundFFT.p_kran_perekr_2;//������ ����
				break;
			case 's':
				soundFFT.v += 3.;//��������� ��������
				soundFFT.v = (soundFFT.v > 100.)? 100.:soundFFT.v;
				break;
			case 'x':
				soundFFT.v -= 3.;//��������� ��������
				soundFFT.v = (soundFFT.v < 0.)? 0.:soundFFT.v;
				break;
			case 'k':
				soundFFT.p_reduktor_gl_crash = !soundFFT.p_reduktor_gl_crash;//������������� �������� ���������
				break;
			case ';':
				soundFFT.p_nar_s8 = !soundFFT.p_nar_s8;//��� 8
				break;
			case 'l':
				soundFFT.p_nar_c13 = !soundFFT.p_nar_c13;//��� 13
				break;
			case '[':
				soundFFT.p_spo_ppu = !soundFFT.p_spo_ppu;//��� ���
				break;
			case ']':
				soundFFT.p_tormoz = !soundFFT.p_tormoz;//������� ������ �����
				break;
			case 'f':
				soundFFT.p_rocket_hit = !soundFFT.p_rocket_hit;//������� ��������� �������
				break;
			case '6':
				soundFFT.p_eng1_lkorr = !soundFFT.p_eng1_lkorr;//������ - ����� ���������
				soundFFT.p_eng2_lkorr = !soundFFT.p_eng2_lkorr;
				soundFFT.p_eng1_rkorr = !soundFFT.p_eng1_lkorr;
				soundFFT.p_eng2_rkorr = !soundFFT.p_eng2_lkorr;
				break;
			case '&':
				soundFFT.p_nasos_podk_1= !soundFFT.p_nasos_podk_1;//��������� ����� �����
				break;
			case '*':
				soundFFT.p_nasos_podk_2 = !soundFFT.p_nasos_podk_2;//��������� ����� ������
				break;
			case 'Z':
				soundFFT.p_rain = !soundFFT.p_rain;//�����
				break;
			case '/':
				soundFFT.rez_2 = !soundFFT.rez_2;//�����������
				break;
			case 'Q':
				soundFFT.p_trans_36_osn = !soundFFT.p_trans_36_osn;//36�
				break;
			case 'W':
				soundFFT.p_po500 = !soundFFT.p_po500;//115�
				break;
			case 'N':
				soundFFT.rez_3 = !soundFFT.rez_3;//���
				break;
			case 'T':
				soundFFT.rez_4 = !soundFFT.rez_4;//����� ���������� ����
				break;
			case '{':
				soundFFT.p_kran_poj_l = !soundFFT.p_kran_poj_l;//��� ���� �
				break;
			case '}':
				soundFFT.p_kran_poj_r = !soundFFT.p_kran_poj_r;//��� ���� �
				break;
			case '9':
				soundFFT.p_skv_on = !soundFFT.p_skv_on;//���
				soundFFT.rez_8 = !soundFFT.rez_8;//���
				break;
			case '-':
				soundFFT.rez_7 = !soundFFT.rez_7;//������
				break;
			case '7':
				soundFFT.rez_5 = !soundFFT.rez_5;//�� 3
				break;
			case '8':
				soundFFT.rez_6 = !soundFFT.rez_6;//�� 2
				break;
			case 'V':
				soundFFT.p_nasos = !soundFFT.p_nasos;//�������� �������
				break;
			case 'i':
				soundFFT.rez_1 = !soundFFT.rez_1;//�����������
				break;
			case 'I':
				soundFFT.p_kran_kolcev = !soundFFT.p_kran_kolcev;//�����������
				break;
			case '$':
				soundFFT.p_ur_igla = !soundFFT.p_ur_igla;//�����������
				break;
			case 'F':
				test = !test;//�����
				break;
			case 'S'://����� (shift + s)
				pause = !pause;
				/*if (pause)
					PauseRealTime();
				else
					RewindRealTime();*/
				break;
			case 'M'://��������� � ������ (shift + m)
				soundFFT.reduktor_gl_obor = 0;
				soundFFT.eng1_obor = 0;
				soundFFT.eng2_obor = 0;
				soundFFT.p_eng1_zap = 0;
				soundFFT.p_eng2_zap = 0;
				soundFFT.p_eng1_ostanov = 0;
				soundFFT.p_eng2_ostanov = 0;
				soundFFT.p_eng1_lkorr = 1;//������ - ����� ���������
				soundFFT.p_eng2_lkorr = 1;
				soundFFT.p_eng1_rkorr = 0;
				soundFFT.p_eng2_rkorr = 0;
				statusEng1 = "NULL";
				statusEng2 = "NULL";
				statusRed = "NULL";
				break;
			case '<'://����� �� - ��������� �� 1 �� (shift + ,)
				soundFFT.reduktor_gl_obor = helicopter.red_obor_mg1;
				soundFFT.eng1_obor = helicopter.eng_obor_mg;
				soundFFT.eng2_obor = 0;
				soundFFT.p_eng1_zap = 1;
				soundFFT.p_eng2_zap = 0;
				soundFFT.p_eng1_ostanov = 0;
				soundFFT.p_eng2_ostanov = 0;
				soundFFT.p_eng1_lkorr = 1;//������ - ����� ���������
				soundFFT.p_eng2_lkorr = 1;
				soundFFT.p_eng1_rkorr = 0;
				soundFFT.p_eng2_rkorr = 0;
				statusEng1 = "NULL";
				statusEng2 = "NULL";
				statusRed = "NULL";
				break;
			case '>'://����� �� - ��������� �� 2 ��(shift + .)
				soundFFT.reduktor_gl_obor = helicopter.red_obor_mg2;
				soundFFT.eng1_obor = helicopter.eng_obor_mg;
				soundFFT.eng2_obor = helicopter.eng_obor_mg;
				soundFFT.p_eng1_zap = 1;
				soundFFT.p_eng2_zap = 1;
				soundFFT.p_eng1_ostanov = 0;
				soundFFT.p_eng2_ostanov = 0;
				soundFFT.p_eng1_lkorr = 1;//������ - ����� ���������
				soundFFT.p_eng2_lkorr = 1;
				soundFFT.p_eng1_rkorr = 0;
				soundFFT.p_eng2_rkorr = 0;
				statusEng1 = "NULL";
				statusEng2 = "NULL";
				statusRed = "NULL";
				break;
			case '?'://����� ������� - ��������� �� 2 ��(shift + /)
				soundFFT.reduktor_gl_obor = helicopter.red_obor_avt;
				soundFFT.eng1_obor = helicopter.eng_obor_avt;
				soundFFT.eng2_obor = helicopter.eng_obor_avt;
				soundFFT.p_eng1_zap = 1;
				soundFFT.p_eng2_zap = 1;
				soundFFT.p_eng1_ostanov = 0;
				soundFFT.p_eng2_ostanov = 0;
				soundFFT.p_eng1_lkorr = 0;//������ - ����� ���������
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

float lineInterpolation(float x0, float fx0, float x1, float fx1, float x)
{
	float fx, a0, a1, a2;
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

float squareInterpolation(float x0, float fx0, float x1, float fx1, float x2, float fx2, float x)
{
	float fx, a0, a1, a2;
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

	//���� ������������ ������������ �� �������� - ����� ��������
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

float getParameterFromFile(string filename, float offset)
{
	float turn = 0;
	float t = 0;
	float v = 0;
	int i = 0;
	vector <float> time, value;

	//������ � ���� ������ ��������� � ������� ������, �� ���� � ������ ������ (�� ������)
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

	//������ � ���� ������ ��������� � ������� ������, �� ���� � ������ ������ (�� ������)
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
			turn = value[0];//������� ������� �� ����
			break;
		}
		if (offset == time[i])//�������� ������� ������� ������� � �������� �� ��
		{
			turn = value[i];//������� ������� �� ����
			break;
		}
		if (offset > time[n - 1])//������� �� ������� � �����
		{
			turn = value[n - 1];//������� ������� �� ����
			break;
		}
		if (offset > time[i] && offset < time[i + 1])//������� �� ������� � �����
		{

			//������������ ������������
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
			//���� ������������ ������������ �� �������� - ����� ��������
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
		new_pitch = parameter / turn;	//��������� �������������� Pitch �� ������ ��������z ���������� ������ �������� � �������� (������� ��� ������ �����-�����)

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

	//������ � ���� ������ ��������� � ������� ������, �� ���� � ������ ������ (�� ������)
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
				new_offset = time[0];//������� ������� �� ����
				break;
			}
			if (turn == value[i])//�������� ������� ������� ������� � �������� �� ��
			{
				new_offset = time[i];//������� ������� �� ����
				break;
			}
			if (turn > value[n - 1])//������� �� ������� � �����
			{
				new_offset = time[n - 1];//������� ������� �� ����
				break;
			}
			if (turn > value[i] && turn < value[i + 1])//������� �� ������� � �����
			{

				//������������ ������������
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
				new_offset = time[0];//������� ������� �� ����
				break;
			}
			if (turn == value[i])//�������� ������� ������� ������� � �������� �� ��
			{
				new_offset = time[i];//������� ������� �� ����
				break;
			}
			if (turn < value[n - 1])//������� �� ������� � �����
			{
				new_offset = time[n - 1];//������� ������� �� ����
				break;
			}
			if (turn < value[i] && turn > value[i + 1])//������� �� ������� � �����
			{

				//������������ ������������
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

float getParameterFromVector(vector<float> value, vector<float> time, float offset)
{
	float turn = 0;
	float x, x0, x1, x2, fx, fx0, fx1, fx2, a0, a1, a2;
	int n = time.size();

	for (int i = 0; i < n; i++)
	{
		if (offset < time[0])
		{
			turn = value[0];//������� ������� �� ����
			break;
		}
		if (offset == time[i])//�������� ������� ������� ������� � �������� �� ��
		{
			turn = value[i];//������� ������� �� ����
			break;
		}
		if (offset > time[n - 1])//������� �� ������� � �����
		{
			turn = value[n - 1];//������� ������� �� ����
			break;
		}
		if (offset > time[i] && offset < time[i + 1])//������� �� ������� � �����
		{

			//������������ ������������
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
			//���� ������������ ������������ �� �������� - ����� ��������
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
	return turn;
}
//������� ������ ���� � ����� ����������� ��������, ���������� �� � ���������� ���������
template<typename... T>//WIP
float getParameterFromFile(string filename, float offset,const T*... args)
{
	float t = 0;
	float v = 0;
	vector <float> time, value;

	//������ � ���� ������ ��������� � ������� ������, �� ���� � ������ ������ (�� ������)
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