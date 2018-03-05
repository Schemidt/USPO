#include "string.h"
#include "map"
using namespace std;
#pragma once

//��������� ��� ���������� ����� ����������
class Helicopter
{
public:
	string modelName;

	double eng_obor_mg; //������� ��������� �� ����� ����
	double eng_obor_avt; //������� ��������� �� ������ �������
	double red_obor_mg1; //������� ��������� � 1�� ���������� �� ����� ����
	double red_obor_mg2; //������� ��������� � 2�� ����������� �� ����� ����
	double red_obor_avt; //������� ��������� �� ������ �������


	map<string, string> shortName;
	map<string, string> fullName;

	void setPath(string pathToFile)
	{
		//TXT
		//���������
		shortName["eng_on"] = "eng_on.txt"; //!<��� ����� � ���������� �������� ������� ��������� �� ������ ��
		shortName["eng_off"] = "eng_off.txt"; //!<��� ����� � ���������� �������� ��������� ��������� 
		shortName["eng_mg_avt"] = "eng_mg_avt.txt"; //!<��� ����� � ���������� �������� ������� ��������� �� ������ �������
		shortName["eng_avt_mg"] = "eng_avt_mg.txt"; //!<��� ����� c ���������� �������� ��������� �� ������ ������ ����
		shortName["ansatFirstEng"] = "eng1_mg.txt"; //!<
		shortName["ansatSecondEng"] = "eng2_mg.txt"; //!<
		//��������
		shortName["red_on"] = "red_on.txt"; //!<��� ����� c ���������� �������� ������� ��������� �� ������ ������ ���� �� 1�� ���������
		shortName["red_on_mg"] = "red_on_mg.txt"; //!<��� ����� c ���������� �������� ������� ��������� �� ������ ������ ���� �� 2� ����������
		shortName["red_mg_avt"] = "red_mg_avt.txt"; //!<��� ����� c ���������� �������� ������� ��������� �� ������ �������
		shortName["red_avt_mg"] = "red_avt_mg.txt"; //!<��� ����� c ���������� �������� ���������� ��������� �� ������ ��
		shortName["red_off"] = "red_off.txt"; //!<��� ����� c ���������� �������� ��������� ��������� ���� ������ ��
		shortName["ansatRed"] = "red_mg.txt"; //!<
			
		fullName = shortName;
		std::map<std::string, string> ::iterator num, num1;
		for (num = shortName.begin(), num1 = fullName.begin(); num != shortName.end(); num++, num1++)
		{
			(*num1).second = pathToFile + (*num).second;
		}
	}
};

extern Helicopter mi_8_mtv5{ "mi_8_mtv5",75, 88, 50, 63 , 96};
extern Helicopter mi_8_amtsh{ "mi_8_amtsh",75, 88, 50, 63, 96};
extern Helicopter mi_26{  "mi_26",69, 84, 37, 47, 85};
extern Helicopter mi_28{"mi_28",73, 86, 48, 60, 93};
extern Helicopter ka_226{ "ka_226",61, 79, 49, 62, 99};
extern Helicopter ansat{ "ansat",65, 80, 65, 65, 100};
extern Helicopter ka_27{ "ka_27",75, 85, 44, 60, 90};
extern Helicopter ka_29{ "ka_29",75, 85, 44, 60, 90};