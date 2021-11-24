#include "stdafx.h"
#include "Conf.h"
#include <string>

//�ɱ���ص�
float Conf::unit_cost = 1.0;
//ʱ����ص�
float Conf::unit_time = 1.0;
//�׸�������ص�
float Conf::decay_rate_travel = 0.014;		//ÿ��λ�������ʻʱ��
float Conf::decay_rate_load = 0.049;			//ÿ��λ�������ʻʱ��
float Conf::unit_cargoCost = 10;				//��λ����ɱ�
												//��س���ص�
float Conf::thermal_load_travel = 0.4;		//ÿ��λ�������ʻʱ��
float Conf::thermal_load_load = 2.4;			//ÿ��λ�������ʻʱ��
float Conf::unit_kcalCost = 4;				//��λkcal�ɱ�


//����������ص�			
int Conf::cost_type = 0;				
int Conf::descent_strategy = 0;		

//�г���ص�
int Conf::MAX_COLUMN_IN_RMP = 50000;		//�гص��������ޣ�������10^7��
int Conf::MAX_PASS_NUM = 10;
int Conf::MAX_PSP_SIZE = 3;					
int Conf::MAX_TEMPORARY_inPSP = 10000;		
int Conf::MAX_M3_CPLEX = 100;                 
int Conf::MAX_SHOW = 2000;                  
int Conf::MAX_NODES = 35;
int Conf::MAX_VECTOR = 15;
int Conf::EACH_LONG_NUM = 30;			     

//������������ص�
int Conf::MAX_PATH_CONTAINER = 100000;		//PSP��label�ص��������ޣ�������10^6��
int Conf::MAX_PATH_NUM = 20000;				//PSP��ÿ��customer��Ԥ��label����������Ϊֻ������������ôռ��λ�ã�������2*10^5��
bool Conf::Not_DynamicBound = false;
int Conf::MAX_NEIGHBOURHOOD_NUM = 5;		//ngset�Ĵ�С������Ϊ1��һ��Ϊ1/5/10/15/20
int Conf::CYCLE_NUM = 2;
float Conf::Exact_threshold = 0.01;			//�Ƚ���Ҫ��0.01�����������о͹�����
float Conf::Remain_proIni=0.1;				//����ʽDP�����������ʼ��ģ��0-1֮��

//��֧��ص�
int Conf::MAX_BRANCH_NODES = 10000;
int Conf::MAX_ADD = 1;						//ÿһ֦�����Լ����������MAX_ADD=1ʱ���Ǿ�ȷ��֧������������ǿ�����ȷ�֧��Ϊ�õ�һ���Ͻ磩
float Conf::MIN_GAP = 0.005;
float Conf::PRE_BOUND = 99999;
float Conf::heuristic_BOUND = 99999;

//��cut��ص�
int Conf::MAX_ADD_SDC = 10;

int Conf::MAX_SR_SET = 4;
float Conf::MAX_SR_DISTANCE = 200;
float Conf::MIN_SR_THRESHOLD = 0.1;
int Conf::MAX_ADD_SR = 30;
int Conf::MAX_SR_NUM = 150;
float Conf::SR_MULTIPY_3 = 0.5;
float Conf::SR_MULTIPY_4 = 0.6666667;
float Conf::SR_MULTIPY_5 = 0.3333333;

int Conf::MAX_ROBUST_DEPTH = 2;

int Conf::MAX_KPATH_NUM = 100;
int Conf::MAX_ADD_KPATH = 20;
int Conf::MAX_SUBSET_KPATH = 10;
float Conf::MIN_KPATH_THRESHOLD = 0.05;

int Conf::MAX_RCC_NUM = 100;
int Conf::MAX_ADD_RCC = 20;
int Conf::MAX_SUBSET_RCC = 25;
float Conf::MIN_RCC_THRESHOLD = 0.05;
float Conf::MAX_RCC_RHS = 2.0;
