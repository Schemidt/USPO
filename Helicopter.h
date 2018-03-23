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
	void setParam(string model);
};
