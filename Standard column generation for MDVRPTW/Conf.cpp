#include "stdafx.h"
#include "Conf.h"
#include <string>

//成本相关的
float Conf::unit_cost = 1.0;
//时间相关的
float Conf::unit_time = 1.0;
//易腐货物相关的
float Conf::decay_rate_travel = 0.014;		//每单位距离的行驶时间
float Conf::decay_rate_load = 0.049;			//每单位距离的行驶时间
float Conf::unit_cargoCost = 10;				//单位货物成本
												//冷藏车相关的
float Conf::thermal_load_travel = 0.4;		//每单位距离的行驶时间
float Conf::thermal_load_load = 2.4;			//每单位距离的行驶时间
float Conf::unit_kcalCost = 4;				//单位kcal成本


//问题类型相关的			
int Conf::cost_type = 0;				
int Conf::descent_strategy = 0;		

//列池相关的
int Conf::MAX_COLUMN_IN_RMP = 50000;		//列池的容量上限，不超过10^7个
int Conf::MAX_PASS_NUM = 10;
int Conf::MAX_PSP_SIZE = 3;					
int Conf::MAX_TEMPORARY_inPSP = 10000;		
int Conf::MAX_M3_CPLEX = 100;                 
int Conf::MAX_SHOW = 2000;                  
int Conf::MAX_NODES = 35;
int Conf::MAX_VECTOR = 15;
int Conf::EACH_LONG_NUM = 30;			     

//定价子问题相关的
int Conf::MAX_PATH_CONTAINER = 100000;		//PSP中label池的容量上限，不超过10^6个
int Conf::MAX_PATH_NUM = 20000;				//PSP中每个customer上预留label的数量，因为只是索引，不怎么占用位置，不超过2*10^5个
bool Conf::Not_DynamicBound = false;
int Conf::MAX_NEIGHBOURHOOD_NUM = 5;		//ngset的大小，至少为1，一般为1/5/10/15/20
int Conf::CYCLE_NUM = 2;
float Conf::Exact_threshold = 0.01;			//比较重要，0.01在理论最优中就够用了
float Conf::Remain_proIni=0.1;				//启发式DP的消减网络初始规模，0-1之间

//分支相关的
int Conf::MAX_BRANCH_NODES = 10000;
int Conf::MAX_ADD = 1;						//每一枝上面的约束数量，当MAX_ADD=1时，是精确分支，其他情况都是快速深度分支（为得到一个上界）
float Conf::MIN_GAP = 0.005;
float Conf::PRE_BOUND = 99999;
float Conf::heuristic_BOUND = 99999;

//与cut相关的
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
