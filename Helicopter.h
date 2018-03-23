#include "string.h"
#include "map"
using namespace std;
#pragma once

//Структура для добавления новых вертолетов
class Helicopter
{
public:
	string modelName;

	double eng_obor_mg; //обороты двигателя на малом газу
	double eng_obor_avt; //обороты двигателя на режиме автомат
	double red_obor_mg1; //обороты редуктора с 1им двигателем на малом газу
	double red_obor_mg2; //обороты редуктора с 2мя двигателями на малом газу
	double red_obor_avt; //обороты редуктора на режиме автомат


	map<string, string> shortName;
	map<string, string> fullName;

	void setPath(string pathToFile)
	{
		//TXT
		//Двигатель
		shortName["eng_on"] = "eng_on.txt"; //!<имя файла с переходной функцией разгона двигателя до режима МГ
		shortName["eng_off"] = "eng_off.txt"; //!<имя файла с переходной функцией остановки двигателя 
		shortName["eng_mg_avt"] = "eng_mg_avt.txt"; //!<имя файла с переходной функцией разгона двигателя до режима автомат
		shortName["eng_avt_mg"] = "eng_avt_mg.txt"; //!<имя файла c переходной функцией остановки до режима малого газа
		shortName["ansatFirstEng"] = "eng1_mg.txt"; //!<
		shortName["ansatSecondEng"] = "eng2_mg.txt"; //!<
		//Редуктор
		shortName["red_on"] = "red_on.txt"; //!<имя файла c переходной функцией разгона редуктора до режима малого газа на 1ом двигателе
		shortName["red_on_mg"] = "red_on_mg.txt"; //!<имя файла c переходной функцией разгона редуктора до режима малого газа на 2х двигателях
		shortName["red_mg_avt"] = "red_mg_avt.txt"; //!<имя файла c переходной функцией разгона редуктора до режима автомат
		shortName["red_avt_mg"] = "red_avt_mg.txt"; //!<имя файла c переходной функцией замедления редуктора до режима мг
		shortName["red_off"] = "red_off.txt"; //!<имя файла c переходной функцией остановки редуктора ниже режима мг
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
