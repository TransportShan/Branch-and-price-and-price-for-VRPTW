#ifndef PROBLEM_H_
#define PROBLEM_H_

#include "Vehicle.h"
#include "Customer.h"
#include "Columns.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

class Problem
{
public:
	Problem();
	virtual ~Problem();
	void ReadfromCSV(const string & file_name);
	void Bulid_matrix(void);
	void Bulid_feasibleArc(void);		//根据时间窗判断哪两个点之间不会存在弧，不存在为1，否则为0
	void Bulid_ngset(void);
	void Bulid_network(void);
	void Sort_customer(void);
	void Set_dssr_ngset(void);

	//辅助函数
	float Calculate_distance_byCustomerId(int startid, int endid);
	float Calculate_travelcost(int startnode, int endnode);
	float Calculate_traveltime(int startnode, int endnode);

	void Generate_cycle(int path_lenght, int*path, int startposi);				//顺次寻找重复的customer形成一个cycle
	void Generate_cycle_back(int path_lenght, int*path, int endposi);			//倒序寻找重复的customer形成一个cycle
	bool Add_ngset_byCycle(void);												//向Allnode中的dssr_ngSet添加新的元素
	bool Add_Angngset_byCycle(void);											//向Allnode中的Ang_ngSet添加新的元素
	bool belong_toset(int node, long *set);										//检查是node是否在集合set中
	bool check_samesubset(long* S1, long* S2);									//检查S1是否与S2相同，如果相同返回true，
	void Insert_toPassnode(int insert_modifynode, long *temp_passnode);			//将insert_modifynode插入到temp_passnode并形成二进制

	bool Check_Augng_Feasible(int cyc_posi, Columns obj_col);							//检查路径obj_col上第cyc_posi个位置反向寻找的第一个cycle是否为Augng_Feasible，true则为Augng_Feasible；false为Augng_infeasible
public:
	int ColumnPool_num;		//列池的数量
	//常规
	int Vehicle_Num;		//VRPTW问题每个场站中的最多可用车辆数
	int Customer_Num;		//VRPTW问题中客户数量
	int Depot_Num;			//VRPTW问题中场站个数

	Customer *Allnode;		//网络中的所有节点，包含客户和场站，场站依次排序在客户的后面
	Customer *Cus;			//客户
	Customer *Depot;		//场站
	Vehicle Veh;			//包含车辆类

	//真实矩阵
	int *Cus_ascend_startTW;			//对Allnode排序，按照startTW的升序排序，记录Allnode中的序号，大小为[Customer_Num]
	float **Distance_matrix;			//真实的距离矩阵，从0到Customer_Num-1为Allnode点，从Customer_Num到Customer_Num+Depot_Num-1为Depot点，没有路径为MAXNUM，大小为[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num]
	float *Max_arrivaltime;				//到达每个场站的最晚可行时间，也可以看做是最长的行驶时间，大小为[Depot_Num]
	//计算系数后的矩阵
	float **Cost_network;				//行驶成本网络，只包括真实成本，在PSP中还有一个对偶成本网络，用于叠加在该网络上，若两点之间没有路径相连，那么距离为MAXNUM，大小为[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num]
	float **Time_network;				//行驶时间网络，若两点之间没有路径相连，那么时间为MAXNUM，大小为[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num] 
	//
	long *powerlist;
	int passnode_length;
	//CPA框架下拓展ngset的中间变量（找到的环）
	int *cycle_node;					//记录RMP当前解一个路径上一个cycle包含的Allnode序号，大小为[Customer_Num+1]
	int cycle_node_num;					//cycle_node的大小
};

#endif