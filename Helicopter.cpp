#include "helicopter.h"
#include "string.h"
#include "iostream"

void Helicopter::setParam(string model)
{

	modelName = model;
	if (model == "mi_8_mtv5")
	{
		//���������
		engTurnoverMg = 75;
		engTurnoverAvt = 88;
		redTurnoverMg1 = 50;
		redTurnoverMg2 = 63;
		redTurnoverAvt = 96;
	}
	else if (model == "mi_8_amtsh")
	{
		//���������		

		engTurnoverMg = 75;
		engTurnoverAvt = 88;
		redTurnoverMg1 = 50;
		redTurnoverMg2 = 63;
		redTurnoverAvt = 96;

	}
	else if (model == "mi_26")
	{
		//���������		

		engTurnoverMg = 69;
		engTurnoverAvt = 84;
		redTurnoverMg1 = 37;
		redTurnoverMg2 = 47;
		redTurnoverAvt = 85;

	}
	else if (model == "mi_28")
	{
		//���������		

		engTurnoverMg = 73;
		engTurnoverAvt = 86;
		redTurnoverMg1 = 48;
		redTurnoverMg2 = 60;
		redTurnoverAvt = 93;

	}
	else if (model == "ka_226")
	{
		//���������		

		engTurnoverMg = 61;
		engTurnoverAvt = 79;
		redTurnoverMg1 = 49;
		redTurnoverMg2 = 62;
		redTurnoverAvt = 99;

	}
	else if (model == "ansat")
	{
		//���������		

		engTurnoverMg = 65;
		engTurnoverAvt = 80;
		redTurnoverMg1 = 65;
		redTurnoverMg2 = 65;
		redTurnoverAvt = 100;

	}
	else if (model == "ka_27")
	{
		//���������		

		engTurnoverMg = 75;
		engTurnoverAvt = 85;
		redTurnoverMg1 = 44;
		redTurnoverMg2 = 60;
		redTurnoverAvt = 90;

	}
	else if (model == "ka_29")
	{
		//���������		

		engTurnoverMg = 75;
		engTurnoverAvt = 85;
		redTurnoverMg1 = 44;
		redTurnoverMg2 = 60;
		redTurnoverAvt = 90;

	}
	else
	{
		cout << " Unknown argument" << endl;
		throw 0;
	}

}
