#include "string.h"
#include "map"
using namespace std;
#pragma once

//Структура для добавления новых вертолетов
class Helicopter
{
public:
	string modelName;

	double engTurnoverMg; //обороты двигателя на малом газу
	double engTurnoverAvt; //обороты двигателя на режиме автомат
	double redTurnoverMg1; //обороты редуктора с 1им двигателем на малом газу
	double redTurnoverMg2; //обороты редуктора с 2мя двигателями на малом газу
	double redTurnoverAvt; //обороты редуктора на режиме автомат

	double vsuTimeOn;
	double vsuTimeOff;
	double vsuHptimeOn;
	double vsuHPtimeOff;

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
		shortName["ansat1EngOff"] = "eng1_off.txt"; //!<
		//Редуктор
		shortName["red_on"] = "red_on.txt"; //!<имя файла c переходной функцией разгона редуктора до режима малого газа на 1ом двигателе
		shortName["red_on_wfe"] = "red_on_wfe.txt"; //!<имя файла c переходной функцией разгона редуктора до режима малого газа на 1ом двигателе
		shortName["red_on_mg"] = "red_on_mg.txt"; //!<имя файла c переходной функцией разгона редуктора до режима малого газа на 2х двигателях
		shortName["red_mg_avt"] = "red_mg_avt.txt"; //!<имя файла c переходной функцией разгона редуктора до режима автомат
		shortName["red_avt_mg"] = "red_avt_mg.txt"; //!<имя файла c переходной функцией замедления редуктора до режима мг
		shortName["red_off"] = "red_off.txt"; //!<имя файла c переходной функцией остановки редуктора ниже режима мг
		shortName["ansatRed"] = "red_mg.txt"; //!<

		fullName = shortName;

		map<string, string> ::iterator num;
		for (num = fullName.begin(); num != fullName.end(); num++)
		{
			(*num).second = pathToFile + (*num).second;
		}
	}
	void setParam(string model);
};
