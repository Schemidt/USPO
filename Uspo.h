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


const int GUID_STR_SIZE = 38;  // ������ GUID (��� '\0')
const int SUBHEADER_SIZE = 7;  // ������ ������������ (��� '\0')

#pragma pack ( push, 1 )
struct SOUNDFFT {
	char headerGUID[GUID_STR_SIZE];
	unsigned short crc16;
	char subHeader[SUBHEADER_SIZE];

	float time;                 // ����� �������, �
	float master_gain;          // ����� ���������, 0..1
	float v;                    // �������� ���������, �/�
	float v_atm_x;              // ���������� ������������ �������� ������������ ��������� � ���, �/�
	float v_atm_y;              // ������������ ������������ �������� ������������ ��������� � ���, �/�
	float v_atm_z;              // ���������� ������������ �������� ������������ ��������� � ���, �/�
	float v_surf_x;             // ���������� ������������ �������� ������������ ����������� � ���, �/�
	float v_surf_y;             // ������������ ������������ �������� ������������ ����������� � ���, �/�
	float v_surf_z;             // ���������� ������������ �������� ������������ ����������� � ���, �/�
	float ny;                   // ���������� Ny
	float vy;                   // ������������ �������� Vy, �/�
	float al;                   // ���� �����, ��
	bool p_skv_on;              // ������� ������ ���, 0-1
	float obj_nos;              // ������� ������� (����� ��������) ������ �����, 0..1
	float obj_hv;               // ������� ��������� (������ ��������) ������ �����, 0..1
	float obj_l;                // ������� ����� ������ �����, 0..1
	float obj_r;                // ������� ������ ������ �����, 0..1
	float h_nos;                // ������ ��� ������� (����� ��������) ������� �����, �
	float h_hv;                 // ������ ��� ��������� (������ ��������) ������� �����, �
	float h_l;                  // ������ ��� ����� ������� �����, �
	float h_r;                  // ������ ��� ������ ������� �����, �
	bool p_tormoz_press;        // ������� �������� ������� ����� (�������� � �������), 0-1
	float tormoz;               // ������� ���������� �����, 0..1
	float tormoz_vint;          // ������� ������� ������� �����, 0..1
	bool p_rain;                // ������� �����, 0-1
	bool p_snow;                // ������� �����, 0-1
	float osadki;               // ������������� �������, 0-100
	bool p_ops;                 // ������� ������ ������� ���, 0-1
	bool p_stekl;               // ������� ������ �����������������, 0-1
	bool p_nasos;               // �������� �������, 0-1
	float styk_nos;             // ������������� ����� ��� ������� ����� ���� ��� � �� ������� (����� ��������)������� , 0..1
	float styk_hv;              // ������������� ����� ��� ������� ����� ���� ��� � �� ��������� (������ ��������) ������� , 0..1
	float styk_l;               // ������������� ����� ��� ������� ����� ���� ��� � �� ����� ������� , 0..1
	float styk_r;               // ������������� ����� ��� ������� ����� ���� ��� � �� ������ ������� , 0..1
	bool p_na_vpp;              // ������� �������� �� ��� � ��
	char p_nar_s8;              // ������� ���������� ��� �-8, 1-����� ����,2-������ ����,3-��� �����
	char p_nar_s13;             // ������� ���������� ��� �-13, 1-����� ����,2-������ ����,3-��� �����
	char p_spo_guv;             // ������� ���������� ��� ���, 1-����� ����,2-������ ����,3-��� �����
	char p_spo_upk;             // ������� ���������� ��� ���, 1-����� ����,2-������ ����,3-��� �����
	char p_spo_ppu;             // ������� ���������� ��� ���, 1-����� ����,2-������ ����,3-��� ����� (�����)
	char p_ur_ataka;            // ������� ���������� �� �����, 1-����� ����,2-������ ����,3-��� �����
	char p_ur_igla;             // ������� ���������� �� ����, 1-����� ����,2-������ ����,3-��� �����
	bool p_crash;               // ������� ������������ � ������������, 0-1
	bool p_model_stop;          // ������� ���������� ������, 0-1
	bool p_eng1_zap;            // ��1 ������� �������, 0-1
	bool p_eng1_ostanov;        // ��1 ������� ��������, 0-1
	bool p_eng1_hp;             // ��1 ������� �������� ���������, 0-1
	bool p_eng1_pomp;           // ��1 ������� ������, 0-1
	float eng1_obor;            // ��1 �������, 0..100
	bool p_eng1_lkorr;          // ��1 ����� ����� ��������� (����� ��)
	bool p_eng1_rkorr;          // ��1 ����� ������ ��������� (����� ��, �������, �����)
	bool p_eng2_zap;            // ��2 ������� �������, 0-1
	bool p_eng2_ostanov;        // ��2 ������� ��������, 0-1
	bool p_eng2_hp;             // ��2 ������� �������� ���������, 0-1
	bool p_eng2_pomp;           // ��2 ������� ������, 0-1
	float eng2_obor;            // ��2 �������, 0..100
	bool p_eng2_lkorr;          // ��2 ����� ����� ��������� (����� ��)
	bool p_eng2_rkorr;          // ��2 ����� ������ ��������� (����� ��, �������, �����)
	bool p_vsu_zap;             // ��� ������� �������, 0-1
	bool p_vsu_ostanov;         // ��� ������� ��������, 0-1
	bool p_vsu_hp;              // ��� ������� �������� ���������, 0-1
	float vsu_obor;             // ��� �������, 0..100
	bool p_vsu_nasos;           // ��� ������� ������ ������, 0-1
	bool p_vsu_zhaluzi;         // ��� ������� ������ �������, 0-1
	float vint_nes_obor;        // ���� ������� �������, 0..100
	float vint_rul_obor;        // ���� ������� �������, 0..100
	float reduktor_gl_obor;     // �������� ������� �������, 0..100
	float reduktor_pr_obor;     // �������� ������������� �������, 0..100
	float reduktor_ug_l_obor;   // �������� ������� ����� �������, 0..100
	float reduktor_ug_r_obor;   // �������� ������� ������ �������, 0..100
	float reduktor_hv_obor;     // �������� ��������� �������, 0..100
	bool p_reduktor_gl_crash;   // �������� ������� ������� �������������, 0-1
	bool p_reduktor_pr_crash;   // �������� ������������� ������� �������������, 0-1
	bool p_reduktor_ug_l_crash; // �������� ������� ����� ������� �������������, 0-1
	bool p_reduktor_ug_r_crash; // �������� ������� ������ ������� �������������, 0-1
	bool p_reduktor_hv_crash;   // �������� ��������� ������� �������������, 0-1
	bool p_pts;                 // ��� (��������������� 36�) ������� ������, 0-1
	bool p_po500;               // ��-500 (��������������� 115� � 1, ��������������� 115� � 2) ������� ������, 0-1
	bool p_trans_36_osn;        // ������������� 36� �������� (������) ������� ������, 0-1
	bool p_trans_36_rez;        // ������������� 36� ��������� (������) ������� ������, 0-1
	bool p_vu1;                 // ��1 ������� ������, 0-1
	bool p_vu2;                 // ��2 ������� ������, 0-1
	bool p_vu3;                 // ��3 ������� ������, 0-1
	bool p_vu4;                 // ��4 ������� ������, 0-1
	bool p_ppo;                 // ��� (������ ���������� ��������) ������� ������, 0-1
	char p_kran_perekr_1;       // ����������� ���� 1 (��� 9) ������� ��������, 1 - ��������, -1 - ��������
	char p_kran_perekr_2;       // ����������� ���� 2 (��� 10) ������� ��������, 1 - ��������, -1 - ��������
	char p_kran_perekr_vsu;     // ����������� ���� ��� ������� ��������, 1 - ��������, -1 - ��������
	char p_kran_kolcev;         // ���� ���������� ������� ��������, 1 - ��������, -1 - ��������
	bool p_nasos_podk_1;        // ������������� ����� 1 (��������� ����� �����) ������� ������, 0-1
	bool p_nasos_podk_2;        // �������������� ����� 2 (��������� ����� ������) ������� ������, 0-1
	char p_kran_poj_l;          // �������� ���� ����� ������� ��������, 1 - ��������, -1 - ��������
	char p_kran_poj_r;          // �������� ���� ������ ������� ��������, 1 - ��������, -1 - ��������
	bool p_nasos_bak5_1;        // ������ ���� �5 1�� ������� ������, 0-1
	bool p_nasos_bak5_2;        // ������ ���� �5 2�� ������� ������, 0-1
	bool p_nasos_bak4_1;        // ������ ���� �4 1�� ������� ������, 0-1
	bool p_nasos_bak4_2;        // ������ ���� �4 2�� ������� ������, 0-1
	bool p_nasos_perek;         // ����� ��������� ������� ������, 0-1
	bool p_rocket_hit;          // ������� ��������� �������, 0-1
	bool p_bullet_hit;          // ������� ��������� �� ��������, 0-1
	float hight;                 // ������ �����, �
	float tangaz;               // ������ ���, �������
	float step;                 // ��� �����, �������
	bool accumulator;           // �����������
	bool ground_power_supply;   // ��� (������)
	bool dis_tank_pump;         // ����� ���������� ����
	bool undefined;             // �������������� (��-28)
	bool zoomer;                // ������ (��-28)
	bool stove;                 // ��-50
	bool trim;                  // ������� ������� ��������
	bool frict;                 // ������� ������� ���������
};
#pragma pack ( pop )

extern SOUNDFFT soundFFT;

#endif

/*!
\brief ����� "�����"
\details ���������� ����� �����
*/
class point {
public:
	double x = 0;//��������
	double y = 0;//��������

	point()
	{

	}

	point(double xi, double yi)
	{
		x = xi;
		y = yi;
	}

	//!<������ x � y �������
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
