#include "helicopter.h"
#include "string.h"
#include "map"
#include "iostream"

void Helicopter::setParam(string model)
{
	
		modelName = model;
		if (model == "mi_8_mtv5")
		{
			//Константы
			eng_obor_mg = 75;
			eng_obor_avt = 88;
			red_obor_mg1 = 50;
			red_obor_mg2 = 63;
			red_obor_avt = 96;
		}
		else if (model == "mi_8_amtsh")
		{
			//Константы		
			
			eng_obor_mg = 75;
			eng_obor_avt = 88;
			red_obor_mg1 = 50;
			red_obor_mg2 = 63;
			red_obor_avt = 96;
			
		}
		else if (model == "mi_26")
		{
			//Константы		
			
			eng_obor_mg = 69;
			eng_obor_avt = 84;
			red_obor_mg1 = 37;
			red_obor_mg2 = 47;
			red_obor_avt = 85;
			
		}
		else if (model == "mi_28")
		{
			//Константы		
			
			eng_obor_mg = 73;
			eng_obor_avt = 86;
			red_obor_mg1 = 48;
			red_obor_mg2 = 60;
			red_obor_avt = 93;
			
		}
		else if (model == "ka_226")
		{
			//Константы		
			
			eng_obor_mg = 61;
			eng_obor_avt = 79;
			red_obor_mg1 = 49;
			red_obor_mg2 = 62;
			red_obor_avt = 99;
			
		}
		else if (model == "ansat")
		{
			//Константы		
			
			eng_obor_mg = 65;
			eng_obor_avt = 80;
			red_obor_mg1 = 65;
			red_obor_mg2 = 65;
			red_obor_avt = 100;
			
		}
		else if (model == "ka_27")
		{
			//Константы		
			
			eng_obor_mg = 75;
			eng_obor_avt = 85;
			red_obor_mg1 = 44;
			red_obor_mg2 = 60;
			red_obor_avt = 90;
			
		}
		else if (model == "ka_29")
		{
			//Константы		
			
			eng_obor_mg = 75;
			eng_obor_avt = 85;
			red_obor_mg1 = 44;
			red_obor_mg2 = 60;
			red_obor_avt = 90;
			
		}
		else
		{
			cout << " Unknown argument" << endl;
			throw 0;
		}
	
}
