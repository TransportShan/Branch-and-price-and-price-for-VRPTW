#include "stdafx.h"
#include "Cut.h"

//Repetition *Rep_customers;				//用来存储RMP当前解每个客户被经过的次数，大小为[p.Customer_Num]

bool compare_Rep(Repetition &a1, Repetition &a2)
{
	return a1.Indicator > a2.Indicator;  //会产生降序排列，若改为<,则会产生升序； 
}

SDC::SDC()
{
}


SDC::~SDC()
{
}

void SDC::Ini_SDC(Problem & p)
{
	SDC_nodes = new int[p.Customer_Num];
	SDC_indicator = new int[p.Customer_Num];
	SDC_cons_index = new int[p.Customer_Num];

	Rep_customers = new Repetition[p.Customer_Num];
	Rep_customers_copy = new Repetition[p.Customer_Num];

	for (int i = 0; i < p.Customer_Num; i++)
	{
		SDC_nodes[i] = -1;
		SDC_indicator[i] = 0;
	}
	processed_num = 0;
	added_num = 0;

	ngset_change = false;
	ngset_addNum = new int[p.Customer_Num];

	for (int i = 0; i < p.Customer_Num; i++)
	{
		ngset_addNum[i] = 0;
	}
}

int SDC::Find_SDC(Utility & Results, Problem & p)
{
	int temp_num = 0;

#if CustomerToSDC == 1
	//赋值给副本Rep_customers_copy
	Cpoy_Rep_customers(p);
#endif

	//对Cuts_sdc.Rep_customers按照Indicator降序排序
	sort(Rep_customers, Rep_customers + p.Customer_Num, compare_Rep);
	//找出最多前Conf::MAX_ADD_SDC个重复经过的客户
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//要么已经找不到重复经过的客户
		if (Rep_customers[i].Indicator<MINDOUBLE)
		{
			break;
		}
		else
		{
#if CustomerToSDC == 0
			temp_num = temp_num + 1;
#else
			if (0 == Rep_customers_copy[Rep_customers[i].Customer_id].Prohibit_ornot)
			{
				temp_num = temp_num + 1;
				//客户i的ngSet周围客户被禁止加入SDC
				for (int j = 0; j < p.Allnode[Rep_customers[i].Customer_id].ngSet_num; j++)
				{
					Rep_customers_copy[p.Allnode[Rep_customers[i].Customer_id].ngSet[j]].Prohibit_ornot = 1;
				}
	}
#endif
}
		//要么已经达到上限个数
		if (temp_num >= Conf::MAX_ADD_SDC)
		{
			break;
		}
	}

	//更新Utility
	if (temp_num > 0)
	{
		ngset_change = true;
		Results.SDC_eachitr.push_back({});
		Results.SDC_eachitr[Results.SDC_eachitr.size() - 1].reserve(100);
	}

	//更新SDC_nodes
	added_num = 0;
	for (int i = 0; i < temp_num; i++)
	{
		//检查Rep_customers[i].Customer_id是否已经在SDC_nodes中
		if (0 == SDC_indicator[Rep_customers[i].Customer_id])
		{
			//添加新的SDC_nodes
			SDC_nodes[processed_num + added_num] = Rep_customers[i].Customer_id;
			SDC_indicator[Rep_customers[i].Customer_id] = 1;
			added_num = added_num + 1;
		}
		else
		{
			Results.SDC_eachitr[Results.SDC_eachitr.size() - 1].push_back(Rep_customers[i].Customer_id);
		}
		ngset_addNum[Rep_customers[i].Customer_id] = 1;
	}

	//再添加所有新的节点上SDC
	for (int i = 0; i < added_num; i++)
	{
		Results.SDC_eachitr[Results.SDC_eachitr.size() - 1].push_back(SDC_nodes[processed_num + i]);
	}

	return temp_num;
}

void SDC::Reset_RepCus(Problem &p)
{
	for (int i = 0; i < p.Customer_Num; i++)
	{
		Rep_customers[i].Customer_id = i;
#if SDCindicator == 0
		Rep_customers[i].Indicator = 0;
#else
		Rep_customers[i].Indicator = 1;
#endif

#if CustomerToSDC == 1
		Rep_customers[i].Prohibit_ornot = 0;
#endif
}
}

void SDC::Cpoy_Rep_customers(Problem & p)
{
	for (int i = 0; i < p.Customer_Num; i++)
	{
		Rep_customers_copy[i].Customer_id = Rep_customers[i].Customer_id;
		Rep_customers_copy[i].Prohibit_ornot = Rep_customers[i].Prohibit_ornot;
	}
}

KPATH::KPATH()
{
}


KPATH::~KPATH()
{
}

void KPATH::Ini_KPATH(Problem & p)
{
	Kpath_subset = new int*[Conf::MAX_KPATH_NUM];
	Kpath_subset_len = new int[Conf::MAX_KPATH_NUM];
	Kpath_indicator = new int*[Conf::MAX_KPATH_NUM];
	Kpath_cons_index = new int[Conf::MAX_KPATH_NUM];

	for (int i = 0; i < Conf::MAX_KPATH_NUM; i++)
	{
		Kpath_subset[i]=new int[Conf::MAX_SUBSET_KPATH];
		for (int j = 0; j < Conf::MAX_SUBSET_KPATH; j++)
		{
			Kpath_subset[i][j] = -1;
		}
		Kpath_subset_len[i] = 0;
		Kpath_indicator[i] = new int[p.Customer_Num+p.Depot_Num];
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			Kpath_indicator[i][j] = 0;
		}
		Kpath_cons_index[i] = -1;
	}

	processed_num = 0;
	added_num = 0;
}

bool KPATH::Already_inKpath(int * obj_subset, int obj_subset_num)
{
	bool check=false;
	for (int i = 0; i < processed_num+added_num; i++)
	{
		check = true;
		for (int j = 0; j < obj_subset_num; j++)
		{
			if (0== Kpath_indicator[i][obj_subset[j]])
			{
				check = false;
				break;
			}
		}
		if (check == true)
		{
			break;
		}
	}
	return check;
}

void KPATH::Add_Kpath(int * obj_subset, int obj_subset_num)
{
	for (int i = 0; i < obj_subset_num; i++)
	{
		Kpath_subset[processed_num + added_num][i] = obj_subset[i];
		Kpath_indicator[processed_num + added_num][obj_subset[i]] = 1;
	}
	Kpath_subset_len[processed_num + added_num] = obj_subset_num;
	added_num++;
}


Repetition::Repetition()
{
}

Repetition::~Repetition()
{
}

RCC::RCC()
{
}

RCC::~RCC()
{
}

void RCC::Ini_RCC(Problem & p)
{
	RCC_subset = new int*[Conf::MAX_RCC_NUM];
	RCC_subset_len = new int[Conf::MAX_RCC_NUM];
	RCC_RHS = new int[Conf::MAX_RCC_NUM];
	RCC_indicator = new int*[Conf::MAX_RCC_NUM];
	RCC_cons_index = new int[Conf::MAX_RCC_NUM];
	RCC_subset_passnode = new long*[Conf::MAX_RCC_NUM];

	for (int i = 0; i < Conf::MAX_RCC_NUM; i++)
	{
		RCC_subset[i] = new int[Conf::MAX_SUBSET_RCC];
		for (int j = 0; j < Conf::MAX_SUBSET_RCC; j++)
		{
			RCC_subset[i][j] = -1;
		}
		RCC_subset_passnode[i] = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		for (int j = 0; j < int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1; j++)
		{
			RCC_subset_passnode[i][j] = 0;
		}
		RCC_subset_len[i] = 0;
		RCC_RHS[i] = 0;
		RCC_indicator[i] = new int[p.Customer_Num + p.Depot_Num];
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			RCC_indicator[i][j] = 0;
		}
		RCC_cons_index[i] = -1;
	}

	processed_num = 0;
	added_num = 0;
}

bool RCC::Already_inRCC(long* obj_subset_passnode, Problem &p)
{
	for (int i = 0; i <processed_num + added_num; i++)
	{
		if (true==p.check_samesubset(obj_subset_passnode, RCC_subset_passnode[i]))
		{
			return true;
		}
	}
	return false;
}

void RCC::Add_RCC(int obj_RHS, int * obj_subset, int obj_subset_num, long* obj_subset_passnode, Problem &p)
{
	RCC_RHS[processed_num + added_num] = obj_RHS;
	for (int i = 0; i < obj_subset_num; i++)
	{
		RCC_subset[processed_num + added_num][i] = obj_subset[i];
		RCC_indicator[processed_num + added_num][obj_subset[i]] = 1;
	}
	for (int i = 0; i < p.passnode_length; i++)
	{
		RCC_subset_passnode[processed_num + added_num][i] = obj_subset_passnode[i];
	}
	RCC_subset_len[processed_num + added_num] = obj_subset_num;
	added_num++;
}

SRC::SRC()
{
}


SRC::~SRC()
{
}

void SRC::Ini_SRC(Problem & p)
{
	SRC_subset = new int*[Conf::MAX_SR_NUM];
	SRC_subset_len = new int[Conf::MAX_SR_NUM];
	SRC_RHS = new int[Conf::MAX_SR_NUM];
	SRC_subset_indicator = new int*[Conf::MAX_SR_NUM];
	SRC_cons_index = new int[Conf::MAX_SR_NUM];
	SRC_LimitVertexSet= new int*[Conf::MAX_SR_NUM];
	SRC_LimitVertexSet_num= new int[Conf::MAX_SR_NUM];
	SRC_LimitVertexSet_indicator = new int*[Conf::MAX_SR_NUM];
	SRC_LimitArcSet_indicator = new int**[Conf::MAX_SR_NUM];

	for (int i = 0; i < Conf::MAX_SR_NUM; i++)
	{
		SRC_subset[i] = new int[Conf::MAX_SR_SET];
		for (int j = 0; j < Conf::MAX_SR_SET; j++)
		{
			SRC_subset[i][j] = -1;
		}
		SRC_subset_len[i] = 0;
		SRC_RHS[i] = 0;
		SRC_subset_indicator[i] = new int[p.Customer_Num + p.Depot_Num];
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			SRC_subset_indicator[i][j] = 0;
		}
		SRC_cons_index[i] = -1;
		SRC_LimitVertexSet[i] = new int[p.Customer_Num];
		SRC_LimitVertexSet_num[i] = 0;
		SRC_LimitVertexSet_indicator[i] = new int[p.Customer_Num + p.Depot_Num];
		SRC_LimitArcSet_indicator[i] = new int*[p.Customer_Num + p.Depot_Num];
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			SRC_LimitVertexSet_indicator[i][j] = 0;
			SRC_LimitArcSet_indicator[i][j] = new int[p.Customer_Num + p.Depot_Num];
			for (int k = 0; k < p.Customer_Num + p.Depot_Num; k++)
			{
				SRC_LimitArcSet_indicator[i][j][k] = 0;
			}
		}
	}

	processed_num = 0;
	added_num = 0;

	violation.resize(Conf::MAX_ADD_SR + 1);
	subset_route.resize(Conf::MAX_ADD_SR + 1);
	temp_2D.resize(Conf::MAX_SR_SET + 100);
	for (int i = 0; i < Conf::MAX_ADD_SR + 1; i++)
	{
		subset_route[i].resize(Conf::MAX_SR_SET + 100);
	}
}

float SRC::add_state(int SRC_length)
{
	if (3 == SRC_length)
	{
		return Conf::SR_MULTIPY_3;
	}
	else if (4 == SRC_length)
	{
		return Conf::SR_MULTIPY_4;
	}
	else if (5 == SRC_length)
	{
		return Conf::SR_MULTIPY_5;
	}
	else
	{
		cout << "ERROR：SRC不等式错误" << endl;
		cin.get();
	}
}

int SRC::Get_RHS(int SRC_length)
{
	if (3 == SRC_length)
	{
		return floor(3 * Conf::SR_MULTIPY_3);
	}
	else if (4 == SRC_length)
	{
		return floor(4 * Conf::SR_MULTIPY_4);
	}
	else if (5 == SRC_length)
	{
		return floor(5 * Conf::SR_MULTIPY_5);
	}
	else
	{
		cout << "ERROR：SRC不等式错误" << endl;
		cin.get();
	}
}

int SRC::Get_insert_no(float cur_violation)
{
	int inposi = violation_num;
	//按照violation降序找到插入位置
	for (int i = 0; i<violation_num; i++)
	{
		if (cur_violation>violation[i])
		{
			inposi = i;
			break;
		}
	}
	if (inposi >= Conf::MAX_ADD_SR)
	{
		return -1;
	}
	return inposi;
}

void SRC::Insert_VioandRoute(int posi, float cur_violation)
{
	//插入前删除末尾节点
	violation.pop_back();
	subset_route.pop_back();
	//插入在violation和subset_route的第posi个位置
	violation.insert(violation.begin() + posi, cur_violation);
	subset_route.insert(subset_route.begin() + posi, temp_2D);

	if (violation_num<Conf::MAX_ADD_SR)
	{
		violation_num = violation_num + 1;
	}
}

bool SRC::Check_SameSubset(int S1, int S2)
{
	if (SRC_subset_len[S1] == SRC_subset_len[S2])
	{
		return false;
	}
	else
	{
		for (int i = 0; i < SRC_subset_len[S1]; i++)
		{
			if (SRC_subset_indicator[S1][SRC_subset[S1][i]]!= SRC_subset_indicator[S2][SRC_subset[S1][i]])
			{
				return false;
			}
		}
	}

	return true;
}
