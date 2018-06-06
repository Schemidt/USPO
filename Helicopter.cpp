#include "helicopter.h"
#include "string.h"
#include "iostream"

void Helicopter::setParam(string model)
{

	modelName = model;
	if (model == "mi_8_mtv5")
	{
		//Константы
		engTurnoverMg = 75;
		engTurnoverAvt = 88;
		redTurnoverMg1 = 50;
		redTurnoverMg2 = 63;
		redTurnoverAvt = 96;

		vsuTimeOn = 10;
		vsuTimeOff = 12;
		vsuHptimeOn = 8;
		vsuHPtimeOff = 2;
	}
	else if (model == "mi_8_amtsh")
	{
		//Константы		
		engTurnoverMg = 75;
		engTurnoverAvt = 88;
		redTurnoverMg1 = 50;
		redTurnoverMg2 = 63;
		redTurnoverAvt = 96;

		vsuTimeOn = 28;
		vsuTimeOff = 10;
		vsuHptimeOn = 8;
		vsuHPtimeOff = 2;

	}
	else if (model == "mi_26")
	{
		//Константы		

		engTurnoverMg = 69;
		engTurnoverAvt = 84;
		redTurnoverMg1 = 37;
		redTurnoverMg2 = 47;
		redTurnoverAvt = 85;

		vsuTimeOn = 29;
		vsuTimeOff = 42;
		vsuHptimeOn = 6;
		vsuHPtimeOff = 19;

	}
	else if (model == "mi_28")
	{
		//Константы		

		engTurnoverMg = 73;
		engTurnoverAvt = 86;
		redTurnoverMg1 = 48;
		redTurnoverMg2 = 60;
		redTurnoverAvt = 93;

		vsuTimeOn = 10;
		vsuTimeOff = 12;
		vsuHptimeOn = 9;
		vsuHPtimeOff = 2;

	}
	else if (model == "ka_226")
	{
		//Константы		

		engTurnoverMg = 61;
		engTurnoverAvt = 79;
		redTurnoverMg1 = 49;
		redTurnoverMg2 = 62;
		redTurnoverAvt = 99;

		vsuTimeOn = 0;
		vsuTimeOff = 0;
		vsuHptimeOn = 0;
		vsuHPtimeOff = 0;

	}
	else if (model == "ansat")
	{
		//Константы		

		engTurnoverMg = 65;
		engTurnoverAvt = 80;
		redTurnoverMg1 = 65;
		redTurnoverMg2 = 65;
		redTurnoverAvt = 100;

		vsuTimeOn = 0;
		vsuTimeOff = 0;
		vsuHptimeOn = 0;
		vsuHPtimeOff = 0;

	}
	else if (model == "ka_27")
	{
		//Константы		

		engTurnoverMg = 75;
		engTurnoverAvt = 85;
		redTurnoverMg1 = 44;
		redTurnoverMg2 = 60;
		redTurnoverAvt = 90;

		vsuTimeOn = 10;
		vsuTimeOff = 12;
		vsuHptimeOn = 9;
		vsuHPtimeOff = 2;

	}
	else if (model == "ka_29")
	{
		//Константы		

		engTurnoverMg = 75;
		engTurnoverAvt = 85;
		redTurnoverMg1 = 44;
		redTurnoverMg2 = 60;
		redTurnoverAvt = 90;

		vsuTimeOn = 5;
		vsuTimeOff = 11;
		vsuHptimeOn = 5;
		vsuHPtimeOff = 5;

	}
	else
	{
		cout << " Unknown argument" << endl;
		throw 0;
	}

}
