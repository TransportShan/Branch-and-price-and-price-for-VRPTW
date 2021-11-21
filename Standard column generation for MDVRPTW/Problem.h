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
	void Bulid_feasibleArc(void);		//����ʱ�䴰�ж���������֮�䲻����ڻ���������Ϊ1������Ϊ0
	void Bulid_ngset(void);
	void Bulid_network(void);
	void Sort_customer(void);
	void Set_dssr_ngset(void);

	//��������
	float Calculate_distance_byCustomerId(int startid, int endid);
	float Calculate_travelcost(int startnode, int endnode);
	float Calculate_traveltime(int startnode, int endnode);

	void Generate_cycle(int path_lenght, int*path, int startposi);				//˳��Ѱ���ظ���customer�γ�һ��cycle
	void Generate_cycle_back(int path_lenght, int*path, int endposi);			//����Ѱ���ظ���customer�γ�һ��cycle
	bool Add_ngset_byCycle(void);												//��Allnode�е�dssr_ngSet����µ�Ԫ��
	bool Add_Angngset_byCycle(void);											//��Allnode�е�Ang_ngSet����µ�Ԫ��
	bool belong_toset(int node, long *set);										//�����node�Ƿ��ڼ���set��
	bool check_samesubset(long* S1, long* S2);									//���S1�Ƿ���S2��ͬ�������ͬ����true��
	void Insert_toPassnode(int insert_modifynode, long *temp_passnode);			//��insert_modifynode���뵽temp_passnode���γɶ�����

	bool Check_Augng_Feasible(int cyc_posi, Columns obj_col);							//���·��obj_col�ϵ�cyc_posi��λ�÷���Ѱ�ҵĵ�һ��cycle�Ƿ�ΪAugng_Feasible��true��ΪAugng_Feasible��falseΪAugng_infeasible
public:
	int ColumnPool_num;		//�гص�����
	//����
	int Vehicle_Num;		//VRPTW����ÿ����վ�е������ó�����
	int Customer_Num;		//VRPTW�����пͻ�����
	int Depot_Num;			//VRPTW�����г�վ����

	Customer *Allnode;		//�����е����нڵ㣬�����ͻ��ͳ�վ����վ���������ڿͻ��ĺ���
	Customer *Cus;			//�ͻ�
	Customer *Depot;		//��վ
	Vehicle Veh;			//����������

	//��ʵ����
	int *Cus_ascend_startTW;			//��Allnode���򣬰���startTW���������򣬼�¼Allnode�е���ţ���СΪ[Customer_Num]
	float **Distance_matrix;			//��ʵ�ľ�����󣬴�0��Customer_Num-1ΪAllnode�㣬��Customer_Num��Customer_Num+Depot_Num-1ΪDepot�㣬û��·��ΪMAXNUM����СΪ[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num]
	float *Max_arrivaltime;				//����ÿ����վ���������ʱ�䣬Ҳ���Կ����������ʻʱ�䣬��СΪ[Depot_Num]
	//����ϵ����ľ���
	float **Cost_network;				//��ʻ�ɱ����磬ֻ������ʵ�ɱ�����PSP�л���һ����ż�ɱ����磬���ڵ����ڸ������ϣ�������֮��û��·����������ô����ΪMAXNUM����СΪ[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num]
	float **Time_network;				//��ʻʱ�����磬������֮��û��·����������ôʱ��ΪMAXNUM����СΪ[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num] 
	//
	long *powerlist;
	int passnode_length;
	//CPA�������չngset���м�������ҵ��Ļ���
	int *cycle_node;					//��¼RMP��ǰ��һ��·����һ��cycle������Allnode��ţ���СΪ[Customer_Num+1]
	int cycle_node_num;					//cycle_node�Ĵ�С
};

#endif