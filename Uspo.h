#include "stdio.h"
#include "tchar.h"
#include "fstream"
#include "regex"
#include "iostream"
#include "string.h"

using namespace std;

#pragma once

#ifndef __SERVER_H__
#define __SERVER_H__


const int GUID_STR_SIZE = 38;  // размер GUID (без '\0')
const int SUBHEADER_SIZE = 7;  // размер подзаголовка (без '\0')

#pragma pack ( push, 1 )
struct SOUNDFFT {
	char headerGUID[GUID_STR_SIZE];
	unsigned short crc16;
	char subHeader[SUBHEADER_SIZE];

	float time;                 // Время текущее, с
	float master_gain;          // Общая громкость, 0..1
	float v;                    // Скорость приборная, м/с
	float v_atm_x;              // Продольная составляющая скорости относительно атмосферы в ССК, м/с
	float v_atm_y;              // Вертикальная составляющая скорости относительно атмосферы в ССК, м/с
	float v_atm_z;              // Поперечная составляющая скорости относительно атмосферы в ССК, м/с
	float v_surf_x;             // Продольная составляющая скорости относительно поверхности в ССК, м/с
	float v_surf_y;             // Вертикальная составляющая скорости относительно поверхности в ССК, м/с
	float v_surf_z;             // Поперечная составляющая скорости относительно поверхности в ССК, м/с
	float ny;                   // Перегрузка Ny
	float vy;                   // Вертикальная скорость Vy, м/с
	float al;                   // Угол атаки, гр
	bool p_skv_on;              // Признак работы СКВ, 0-1
	float obj_nos;              // Обжатие носовой (левой передней) стойки шасси, 0..1
	float obj_hv;               // Обжатие хвостовой (правой передней) стойки шасси, 0..1
	float obj_l;                // Обжатие левой стойки шасси, 0..1
	float obj_r;                // Обжатие правой стойки шасси, 0..1
	float h_nos;                // Высота под носовой (левой передней) стойкой шасси, м
	float h_hv;                 // Высота под хвостовой (правой передней) стойкой шасси, м
	float h_l;                  // Высота под левой стойкой шасси, м
	float h_r;                  // Высота под правой стойкой шасси, м
	bool p_tormoz_press;        // Признак нажатого тормоза колес (давление в системе), 0-1
	float tormoz;               // Степень торможения колес, 0..1
	float tormoz_vint;          // Степень зажатия тормоза винта, 0..1
	bool p_rain;                // Признак дождя, 0-1
	bool p_snow;                // Признак снега, 0-1
	float osadki;               // Интенсивность осадков, 0-100
	bool p_ops;                 // Признак работы привода ОПС, 0-1
	bool p_stekl;               // Признак работы стеклоочистителей, 0-1
	bool p_nasos;               // Насосная станция, 0-1
	float styk_nos;             // Интенсивность удара при проходе стыка плит ВПП и РД носовой (левой передней)стойкой , 0..1
	float styk_hv;              // Интенсивность удара при проходе стыка плит ВПП и РД хвостовой (правой передней) стойкой , 0..1
	float styk_l;               // Интенсивность удара при проходе стыка плит ВПП и РД левой стойкой , 0..1
	float styk_r;               // Интенсивность удара при проходе стыка плит ВПП и РД правой стойкой , 0..1
	bool p_na_vpp;              // Признак движения на ВПП и РД
	char p_nar_s8;              // Признак применения НАР С-8, 1-левый борт,2-правый борт,3-оба борта
	char p_nar_s13;             // Признак применения НАР С-13, 1-левый борт,2-правый борт,3-оба борта
	char p_spo_guv;             // Признак применения СПО ГУВ, 1-левый борт,2-правый борт,3-оба борта
	char p_spo_upk;             // Признак применения СПО УПК, 1-левый борт,2-правый борт,3-оба борта
	char p_spo_ppu;             // Признак применения СПО ППУ, 1-левый борт,2-правый борт,3-оба борта (центр)
	char p_ur_ataka;            // Признак применения УР Атака, 1-левый борт,2-правый борт,3-оба борта
	char p_ur_igla;             // Признак применения УР Игла, 1-левый борт,2-правый борт,3-оба борта
	bool p_crash;               // Признак столкновения с препятствием, 0-1
	bool p_model_stop;          // Признак выключения Модели, 0-1
	bool p_eng1_zap;            // ДВ1 признак запуска, 0-1
	bool p_eng1_ostanov;        // ДВ1 признак останова, 0-1
	bool p_eng1_hp;             // ДВ1 признак холодная прокрутка, 0-1
	bool p_eng1_pomp;           // ДВ1 признак помпаж, 0-1
	float eng1_obor;            // ДВ1 обороты, 0..100
	bool p_eng1_lkorr;          // ДВ1 режим левой коррекции (режим МГ)
	bool p_eng1_rkorr;          // ДВ1 режим правой коррекции (режим АР, АВТОМАТ, ПОЛЕТ)
	bool p_eng2_zap;            // ДВ2 признак запуска, 0-1
	bool p_eng2_ostanov;        // ДВ2 признак останова, 0-1
	bool p_eng2_hp;             // ДВ2 признак холодная прокрутка, 0-1
	bool p_eng2_pomp;           // ДВ2 признак помпаж, 0-1
	float eng2_obor;            // ДВ2 обороты, 0..100
	bool p_eng2_lkorr;          // ДВ2 режим левой коррекции (режим МГ)
	bool p_eng2_rkorr;          // ДВ2 режим правой коррекции (режим АР, АВТОМАТ, ПОЛЕТ)
	bool p_vsu_zap;             // ВСУ признак запуска, 0-1
	bool p_vsu_ostanov;         // ВСУ признак останова, 0-1
	bool p_vsu_hp;              // ВСУ признак холодная прокрутка, 0-1
	float vsu_obor;             // ВСУ обороты, 0..100
	bool p_vsu_nasos;           // ВСУ признак работы насоса, 0-1
	bool p_vsu_zhaluzi;         // ВСУ признак работы жалюзей, 0-1
	float vint_nes_obor;        // Винт несущий обороты, 0..100
	float vint_rul_obor;        // Винт рулевой обороты, 0..100
	float reduktor_gl_obor;     // Редуктор главный обороты, 0..100
	float reduktor_pr_obor;     // Редуктор промежуточный обороты, 0..100
	float reduktor_ug_l_obor;   // Редуктор угловой левый обороты, 0..100
	float reduktor_ug_r_obor;   // Редуктор угловой правый обороты, 0..100
	float reduktor_hv_obor;     // Редуктор хвостовой обороты, 0..100
	bool p_reduktor_gl_crash;   // Редуктор главный признак неисправности, 0-1
	bool p_reduktor_pr_crash;   // Редуктор промежуточный признак неисправности, 0-1
	bool p_reduktor_ug_l_crash; // Редуктор угловой левый признак неисправности, 0-1
	bool p_reduktor_ug_r_crash; // Редуктор угловой правый признак неисправности, 0-1
	bool p_reduktor_hv_crash;   // Редуктор хвостовой признак неисправности, 0-1
	bool p_pts;                 // ПТС (преобразователь 36В) признак работы, 0-1
	bool p_po500;               // ПО-500 (преобразователь 115В – 1, преобразователь 115В – 2) признак работы, 0-1
	bool p_trans_36_osn;        // трансформатор 36В основной (первый) признак работы, 0-1
	bool p_trans_36_rez;        // трансформатор 36В резервный (второй) признак работы, 0-1
	bool p_vu1;                 // ВУ1 признак работы, 0-1
	bool p_vu2;                 // ВУ2 признак работы, 0-1
	bool p_vu3;                 // ВУ3 признак работы, 0-1
	bool p_vu4;                 // ВУ4 признак работы, 0-1
	bool p_ppo;                 // ППО (привод постоянных оборотов) признак работы, 0-1
	char p_kran_perekr_1;       // Перекрывной кран 1 (бак 9) признак движения, 1 - открытие, -1 - закрытие
	char p_kran_perekr_2;       // Перекрывной кран 2 (бак 10) признак движения, 1 - открытие, -1 - закрытие
	char p_kran_perekr_vsu;     // Перекрывной кран ВСУ признак движения, 1 - открытие, -1 - закрытие
	char p_kran_kolcev;         // Кран колцевания признак движения, 1 - открытие, -1 - закрытие
	bool p_nasos_podk_1;        // Подкачивающий насос 1 (топливный насос левый) признак работы, 0-1
	bool p_nasos_podk_2;        // Перекачивающий насос 2 (топливный насос правый) признак работы, 0-1
	char p_kran_poj_l;          // Пожарный кран левый признак движения, 1 - открытие, -1 - закрытие
	char p_kran_poj_r;          // Пожарный кран правый признак движения, 1 - открытие, -1 - закрытие
	bool p_nasos_bak5_1;        // Насосы бака №5 1ый признак работы, 0-1
	bool p_nasos_bak5_2;        // Насосы бака №5 2ой признак работы, 0-1
	bool p_nasos_bak4_1;        // Насосы бака №4 1ый признак работы, 0-1
	bool p_nasos_bak4_2;        // Насосы бака №4 2ой признак работы, 0-1
	bool p_nasos_perek;         // Насос перекачки признак работы, 0-1
	bool p_rocket_hit;          // Признак попадания ракетой, 0-1
	bool p_bullet_hit;          // Признак попадания из пулемета, 0-1
	float hight;                 // Высота радио, м
	float tangaz;               // Тангаж фюз, градусы
	float step;                 // Шаг винта, градусы
	bool accumulator;           // Аккумулятор
	bool ground_power_supply;   // НИП (работа)
	bool dis_tank_pump;         // Насос расходного бака
	bool undefined;             // Неопределенный (Ми-28)
	bool zoomer;                // Зуммер (Ми-28)
	bool stove;                 // КО-50
	bool trim;                  // признак нажатия триммера
	bool frict;                 // признак нажатия фрикциона
};
#pragma pack ( pop )

extern SOUNDFFT soundFFT;

#endif

/*!
\brief Класс "точка"
\details Определяет класс точки
*/
class point {
public:
	double x = 0;//Абсцисса
	double y = 0;//Ордината

	point()
	{

	}

	point(double xi, double yi)
	{
		x = xi;
		y = yi;
	}

	//!<Меняет x и y местами
	point swap()
	{
		double _x;
		double _y;

		_x = y;
		_y = x;
		y = _y;
		x = _x;

		return *this;
	}

	point& operator =(const point &copy);
};

class testChunk
{
public:
	
	double start;
	double end;
};

void delayMs(double ms);

void kbHit();

double getTimeMs();

double interpolation(point p1, point p2, double x);

double interpolation(point p1, point p2, point p3, double x);

double getOffset(string filename, double parameter);

double getParameterFromFile(string filename, double offset);

double getParameterFromVector(vector<point> &value, double offset);

//
vector <point> getVectorFromFile(string filename);

int binSer(vector<point> &points, double offset);
