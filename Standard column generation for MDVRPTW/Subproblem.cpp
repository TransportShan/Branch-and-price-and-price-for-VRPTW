#include "stdafx.h"
#include "Subproblem.h"
#include "Conf.h"

int *temp_Index;
long *temppass;			
int passnode_length;	
long *passnodelist;	
vector<int> elimi_nodes;
Path *ForwardPath_container;				//用来存储PSP中找到的所有前向路径，大小为[MAX_PATH_CONTAINER]
int ForwardPath_container_num;				//跟踪ForwardPath_container已经使用的数量
Path *BackwardPath_container;				//用来存储PSP中找到的所有前向路径，大小为[MAX_PATH_CONTAINER]
int BackwardPath_container_num;				//跟踪BackwardPath_container已经使用的数量

bool compare_for(int a1, int a2)
{
	return ForwardPath_container[a1].RC_value < ForwardPath_container[a2].RC_value;  //会产生升序排列，若改为>,则会产生降序； 
}
bool compare_back(int a1, int a2)
{
	return BackwardPath_container[a1].RC_value < BackwardPath_container[a2].RC_value;  //会产生升序排列，若改为>,则会产生降序； 
}
Path::Path()
{
}
Path::~Path()
{
}
void Path::get_ComplementarySet_toFeasibleExtensionsbyNgpath(long * SetA, long * SetB)
{
	for (int i = 0; i < passnode_length; i++)
	{
		feasible_extensions_byngpath[i] = SetA[i] - (SetA[i] & SetB[i]);
	}
}
bool Path::get_IntersectionSet_toFeasibleExtensionsbyNgpath(long * SetA)
{
	for (int i = 0; i < passnode_length; i++)
	{
		feasible_extensions_byngpath[i] = feasible_extensions_byngpath[i] & SetA[i];
	}
	for (int i = 0; i < passnode_length; i++)
	{
		if (feasible_extensions_byngpath[i] != 0)
		{
			return false;
		}
	}
	return true;//为空
}

void Path::Copy_Customer(Path & fromlabel)
{
	for (int i = 0; i<fromlabel.customeList_num; i++)
	{
		customeList[i] = fromlabel.customeList[i];
	}
	customeList_num = fromlabel.customeList_num;

}
void Path::Copy_SDCnode(Path & fromlabel, SDC &Cuts_sdc)
{
#if SDCTYPE == 0
	for (int i = 0; i<fromlabel.SDCnode_num; i++)
	{
		SDCnode[i] = fromlabel.SDCnode[i];
	}
	SDCnode_num = fromlabel.SDCnode_num;
#endif
}
void Path::update_passnode_kcycle(int cycle_num,int next_modifynode)
{
	if (cycle_num>2)
	{
		cout<< "ERROR:没考虑3-cycle情况 " << endl;
		cin.get();
	}

#if Frameworks == 0
	if (true == elementaty)
	{
		//先判断插入点next_modifynode是否构成环
		int lineid, rowid;
		rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
		lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
		if (0 == (passnode[rowid] & passnodelist[lineid]))
		{
			//不构成环
			passnode[rowid] = passnode[rowid] + passnodelist[lineid];
			elementaty = true;
		}
		else
		{
			//形成环
			elementaty = false;
		}
	}
#else
	//先判断插入点next_modifynode是否构成环
	int lineid, rowid;
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
	if (0 == (passnode[rowid] & passnodelist[lineid]))
	{
		//不构成环,加入不重复点
		passnode[rowid] = passnode[rowid] + passnodelist[lineid];
		elementaty = elementaty & true;
	}
	else
	{
		//形成环，则不用加入重复经过节点
		elementaty = false;
	}
#endif

}
void Path::copy_passnode_ngpath(Path & fromlabel)
{
	for (int i = 0; i < passnode_length; i++)
	{
		passnode_ngpath[i] = fromlabel.passnode_ngpath[i];
		feasible_extensions_byngpath[i] = 0;
	}
	ngpath_num = 0;
}
void Path::copy_passnode(Path & fromlabel)
{
	elementaty = fromlabel.elementaty;
#if Frameworks == 0
	if (true == fromlabel.elementaty)//只有当前label还初等时，才继续记录passnode
	{
		for (int i = 0; i<passnode_length; i++)
		{
			passnode[i] = fromlabel.passnode[i];
		}
	}
#else
	for (int i = 0; i<passnode_length; i++)
	{
		passnode[i] = fromlabel.passnode[i];
	}
#endif

}
void Path::update_passnode_ngpath(long * nextcustomer_neighbourhood_passnode, int next_modifynode, Path & fromlabel)
{
	int lineid, rowid;
	int temp_num = 0;
	//首先是交集
	//passnode_ngpath
	for (int i = 0; i < passnode_length; i++)
	{
		passnode_ngpath[i] = passnode_ngpath[i] & nextcustomer_neighbourhood_passnode[i];
	}
	//ngpath
	for (int i = 0; i < fromlabel.ngpath_num; i++)
	{
		if (true == belong_toset(fromlabel.ngpath[i], nextcustomer_neighbourhood_passnode))
		{
			ngpath[ngpath_num] = fromlabel.ngpath[i];
			ngpath_num = ngpath_num +1;
		}

	}
	//最后并上下一个点
	//passnode_ngpath
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
	passnode_ngpath[rowid] += passnodelist[lineid];
	//ngpath
	ngpath[ngpath_num] = next_modifynode;
	ngpath_num = ngpath_num + 1;	
}
bool Path::belong_toset(int node, long * set)
{
	int lineid, rowid;
	rowid = int((node + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = node - rowid*Conf::EACH_LONG_NUM;
	if (0 == (set[rowid] & passnodelist[lineid]))
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Path::update_elementary_passnode(int next_modifynode)
{
	//先判断插入点next_modifynode是否构成环
	int lineid, rowid;
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
	if (0 == (passnode[rowid] & passnodelist[lineid]))
	{
		//不构成环
		passnode[rowid] = passnode[rowid] + passnodelist[lineid];
		return true;
	}
	else
	{
		//形成环
		return false;
	}
}

void Path::copy_passnode_2cycle(long * frompassnode_ngpath)
{
	for (int i = 0; i<passnode_length; i++)
	{
		passnode_2cycle[i] = frompassnode_ngpath[i];
	}
}

void Path::Union_2cycle(int obj_modifynode)
{
	int lineid, rowid;
	rowid = int((obj_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = obj_modifynode - rowid*Conf::EACH_LONG_NUM;
	passnode_2cycle[rowid] = passnode_2cycle[rowid] | passnodelist[lineid];	//并集，只要有一个存在就可以了
}

void Path::copy_passnode_simple(Path & fromlabel)
{
	elementaty = fromlabel.elementaty;
	for (int i = 0; i<passnode_length; i++)
	{
		passnode[i] = fromlabel.passnode[i];
	}
}

void Path::update_passnode_kcycle_simple(int cycle_num, int next_modifynode)
{
	//先判断插入点next_modifynode是否构成环
	int lineid, rowid;
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
	if (0 == (passnode[rowid] & passnodelist[lineid]))
	{
		//不构成环,加入不重复点
		passnode[rowid] = passnode[rowid] + passnodelist[lineid];
		elementaty = elementaty & true;
	}
	else
	{
		//形成环，则不用加入重复经过节点
		elementaty = false;
	}
}

NodeBucket::NodeBucket()
{
}
NodeBucket::~NodeBucket()
{
}
Subproblem::Subproblem()
{
	all_label_num = 0;
	init_label_num = 0;
}

Subproblem::~Subproblem()
{
}

void Subproblem::Claim_PSP(Problem &p)
{
#if EXACTEXTEND == 0
	restricted_extend = true;
#else
	restricted_extend = false;
#endif
	//网络
	ForfeasiExten_Index = new int*[p.Customer_Num + 1];
	ForfeasiExten_Index_num = new int[p.Customer_Num + 1];
	BackfeasiExten_Index = new int*[p.Customer_Num + 1];
	BackfeasiExten_Index_num = new int[p.Customer_Num + 1];

	for (int i = 0; i < p.Customer_Num + 1; i++)
	{
		ForfeasiExten_Index[i] = new int[p.Customer_Num + 1];
		BackfeasiExten_Index[i] = new int[p.Customer_Num + 1];
	}

	Mapping_Reduced = new int[p.Customer_Num];
	Mapping_fromRN_toMN = new int[p.Customer_Num];
	for (int i = 0; i < p.Customer_Num; i++)
	{
		Mapping_Reduced[i] = -1;
		Mapping_fromRN_toMN[i] = -1;
	}
	//前向容器
	ForwardPath_container = new Path[Conf::MAX_PATH_CONTAINER];
	for (int i = 0; i<Conf::MAX_PATH_CONTAINER; i++)
	{
		ForwardPath_container[i].customeList = new int[Conf::MAX_NODES];
		ForwardPath_container[i].passnode = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		ForwardPath_container[i].passnode_ngpath = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		ForwardPath_container[i].ngpath = new int[Conf::MAX_NODES];
		ForwardPath_container[i].feasible_extensions_byngpath = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		ForwardPath_container[i].passnode_2cycle = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];

#if Frameworks == 1
		ForwardPath_container[i].SDCnode.resize(Conf::MAX_VECTOR);
		ForwardPath_container[i].passnode_SDCnode = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
#endif
#if SRCUT == 1
		ForwardPath_container[i].state_src= new float[Conf::MAX_SR_NUM];
#endif
	}
	Delelted_Forwardindex = new int[Conf::MAX_PATH_CONTAINER];

	//反向容器
	BackwardPath_container = new Path[Conf::MAX_PATH_CONTAINER];
	for (int i = 0; i<Conf::MAX_PATH_CONTAINER; i++)
	{
		BackwardPath_container[i].customeList = new int[Conf::MAX_NODES];
		BackwardPath_container[i].passnode = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		BackwardPath_container[i].passnode_ngpath = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		BackwardPath_container[i].ngpath = new int[Conf::MAX_NODES];
		BackwardPath_container[i].feasible_extensions_byngpath = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		BackwardPath_container[i].passnode_2cycle = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];

#if Frameworks == 1
		BackwardPath_container[i].SDCnode.resize(Conf::MAX_VECTOR);
		BackwardPath_container[i].passnode_SDCnode = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
#endif
#if SRCUT == 1
		BackwardPath_container[i].state_src = new float[Conf::MAX_SR_NUM];
#endif
	}
	Delelted_Backwardindex = new int[Conf::MAX_PATH_CONTAINER];

	//解池容器
	FoundPath_container = new Columns[p.Customer_Num + 1];
	for (int i = 0; i<p.Customer_Num + 1; i++)
	{
		FoundPath_container[i].nodes = new int[Conf::MAX_NODES];
		for (int j = 0; j < Conf::MAX_NODES; j++)
		{
			FoundPath_container[i].nodes[j] = -1;
		}
		FoundPath_container[i].nodesNum = 0;
	}

	Delelted_Foundindex = new int[p.Customer_Num + 1];

	//DSSR相关
	Shortest_path.customeList = new int[Conf::MAX_NODES];
	Shortest_path.passnode = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
	Shortest_path.passnode_ngpath = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
	Shortest_path.ngpath = new int[Conf::MAX_NODES];
	Shortest_path.feasible_extensions_byngpath = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
	Shortest_path.passnode_2cycle = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
#if SRCUT == 1
	Shortest_path.state_src = new float[Conf::MAX_SR_NUM];
#endif
	cycle_Modifynode = new int[p.Customer_Num + 1];


	//初始化bucket
	ForwardBucket = new NodeBucket[p.Customer_Num+1];
	BackwardBucket = new NodeBucket[p.Customer_Num + 1];
	for (int i = 0; i<p.Customer_Num + 1; i++)
	{
		ForwardBucket[i].Path_index = new int[Conf::MAX_PATH_NUM];
		BackwardBucket[i].Path_index = new int[Conf::MAX_PATH_NUM];
		//别忘了Completion_bounds
		ForwardBucket[i].Completion_bounds = new float[int(p.Depot[current_depot].endTW)];
		BackwardBucket[i].Completion_bounds = new float[int(p.Depot[current_depot].endTW)];
		for(int j = 0; j < int(p.Depot[current_depot].endTW); j++)
		{
			ForwardBucket[i].Completion_bounds[j] = MINNUM;
			BackwardBucket[i].Completion_bounds[j] = MINNUM;
		}
	}
	Ini_compleBound_bucket(p);

	//初始化ngset
	neighbourhood = new int*[p.Customer_Num];
	neighbourhood_num = new int[p.Customer_Num];
	neighbourhood_passnode = new long*[p.Customer_Num];
	ng_memory_passnode = new long*[p.Customer_Num];
	Aug_neighbourhood_passnode = new long*[p.Customer_Num];
	for (int i = 0; i < p.Customer_Num; i++)
	{
		neighbourhood[i] = new int[Conf::MAX_NODES];
		neighbourhood_passnode[i] = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		ng_memory_passnode[i] = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		Aug_neighbourhood_passnode[i] = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
	}
	//初始化常数
	passnodelist = new long[Conf::EACH_LONG_NUM];
	passnodelist[0] = 1;
	for (int i = 1; i < Conf::EACH_LONG_NUM; i++)
	{
		passnodelist[i] = passnodelist[i - 1] * 2;
	}
	temppass = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
	temp_Index = new int[p.Customer_Num + 1];
	
	//初始化vector
	elimi_nodes.resize(Conf::MAX_NODES);
	temp_dual = new float[p.Customer_Num];
}

void Subproblem::Get_dual_info(int obj_depot, float remain_arc, BranchABound & BB, RMP & lp, Problem & p, SDC &Cuts_sdc)
{
	current_depot = obj_depot;
	arc_remain_proportion = remain_arc;
	fix_cost = lp.Vehicle_dual[current_depot];

	//初始化一大堆参数
	all_label_num = 0;
	init_label_num = 0;
	Modifiednetwork_Nodes_num = 0;
	reduce_cost = MAXNUM;

	for (int i = 0; i < p.Customer_Num + 1; i++)
	{
		ForfeasiExten_Index_num[i] = 0;
		BackfeasiExten_Index_num[i] = 0;
	}
	for (int i = 0; i < p.Customer_Num; i++)
	{
		Mapping_fromRN_toMN[i] = -1;
	}
	if (true== exact_ornot)
	{
		for (int i = 0; i < p.Customer_Num; i++)
		{
			for (int j = ForwardBucket[i].Bucket_min; j <  ForwardBucket[i].Bucket_max+1; j++)
			{
				ForwardBucket[i].Completion_bounds[j] = MINNUM;
			}
			for (int j = BackwardBucket[i].Bucket_min; j < BackwardBucket[i].Bucket_max + 1; j++)
			{
				BackwardBucket[i].Completion_bounds[j] = MINNUM;
			}
		}
	}

	Label_clear(p);
	//重构映射关系和FeasiExten_Index
	Build_Mapping(BB,lp, p);
	Feasible_Extension(BB,lp, p);
	//Reset_Maxtime(BB, lp, p);
}

void Subproblem::Label_clear(Problem &p)
{
	//for (int i = 0; i < ForwardPath_container_num; i++)
	//{
	//	ForwardPath_container[i].exist_state = 0;
	//}
	//for (int i = 0; i < BackwardPath_container_num; i++)
	//{
	//	BackwardPath_container[i].exist_state = 0;
	//}

	ForwardPath_container_num = 0;
	Delelted_Forwardindex_num = 0;
	BackwardPath_container_num = 0;
	Delelted_Backwardindex_num = 0;
	FoundPath_container_num = 0;
	Delelted_Foundindex_num = 0;


	for (int i = 0; i < p.Customer_Num + 1; i++)
	{
		ForwardBucket[i].Path_Num = 0;
		BackwardBucket[i].Path_Num = 0;
	}
}

void Subproblem::Feasible_Extension(BranchABound & BB, RMP & lp, Problem & p)
{
	int curnode = BB.cur_node;
	bool vaild_node;

#if FEASIBLE == 0
	//更新FeasiExten_Index
	for (int i = 0; i < p.Customer_Num; i++)
	{
		for (int j = 0; j < p.Customer_Num; j++)
		{
			vaild_node = Check_validCus(j, BB, lp);

			if (true== vaild_node)
			{
				if (BB.branch[curnode].CostNetwork_Branch[i][j] <MAXNUM - 1)
				{
					ForfeasiExten_Index[i][ForfeasiExten_Index_num[i]] = j;
					ForfeasiExten_Index_num[i] = ForfeasiExten_Index_num[i] + 1;
				}
				if (BB.branch[curnode].CostNetwork_Branch[j][i] <MAXNUM - 1)
				{
					BackfeasiExten_Index[i][BackfeasiExten_Index_num[i]] = j;
					BackfeasiExten_Index_num[i] = BackfeasiExten_Index_num[i] + 1;
				}
			}
		}
	}
#elif FEASIBLE == 1
	//更新FeasiExten_Index
	for (int i = 0; i < p.Customer_Num; i++)
	{
		for (int j = 0; j < p.Customer_Num; j++)
		{
			vaild_node = Check_validCus(j, BB, lp);

			if (true== vaild_node)
			{
				if (BB.branch[curnode].CostNetwork_Branch[i][j] <MAXNUM - 1 && 1 == p.Allnode[i].for_feasible_extensions[j])
				{
					ForfeasiExten_Index[i][ForfeasiExten_Index_num[i]] = j;
					ForfeasiExten_Index_num[i] = ForfeasiExten_Index_num[i] + 1;
				}
				if (BB.branch[curnode].CostNetwork_Branch[j][i] <MAXNUM - 1 && 1 == p.Allnode[i].back_feasible_extensions[j])
				{
					BackfeasiExten_Index[i][BackfeasiExten_Index_num[i]] = j;
					BackfeasiExten_Index_num[i] = BackfeasiExten_Index_num[i] + 1;
				}
			}
		}
	}
#endif
	//更新从场站出发能够到达的customer
	for (int i = 0; i < p.Customer_Num; i++)
	{
		vaild_node = Check_validCus(i, BB, lp);

		if (true == vaild_node)
		{
			if (BB.branch[curnode].CostNetwork_Branch[p.Customer_Num+current_depot][i] < MAXNUM - 1)
			{
				ForfeasiExten_Index[p.Customer_Num][ForfeasiExten_Index_num[p.Customer_Num]] = i;
				ForfeasiExten_Index_num[p.Customer_Num] = ForfeasiExten_Index_num[p.Customer_Num] + 1;
			}
			if (BB.branch[curnode].CostNetwork_Branch[i][p.Customer_Num + current_depot] <MAXNUM - 1)
			{
				BackfeasiExten_Index[p.Customer_Num][BackfeasiExten_Index_num[p.Customer_Num]] = i;
				BackfeasiExten_Index_num[p.Customer_Num] = BackfeasiExten_Index_num[p.Customer_Num] + 1;
			}
		}
	}

	//根据arc_remain_proportion更新FeasiExten_Index
	if (arc_remain_proportion<(1 - MINDOUBLE))
	{
		int temp_posi, i, j, k;
		int ordered_num;	//已经排序好的数量
		int savenum;
		//按照成本+RC值升序排序
		//前向
		for (i = 0; i < p.Customer_Num; i++)
		{
			if (ForfeasiExten_Index_num[i] <= 25)continue; //PSP网络中一个节点的发出弧至少有25条
			savenum = max(25, int(arc_remain_proportion*ForfeasiExten_Index_num[i])); 
			for (j = 0; j <ForfeasiExten_Index_num[i]; j++)
			{
				temp_Index[j] = ForfeasiExten_Index[i][j];
			}
			//开始排序
			ForfeasiExten_Index[i][0] = temp_Index[0];
			ordered_num = 1;
			for (j = 1; j <ForfeasiExten_Index_num[i]; j++)
			{
				for (k = ordered_num; k >= 1; k--)
				{
					if (
						(BB.branch[curnode].CostNetwork_Branch[i][temp_Index[j]] - lp.Customer_dual[temp_Index[j]])
						<(BB.branch[curnode].CostNetwork_Branch[i][ForfeasiExten_Index[i][k - 1]] - lp.Customer_dual[ForfeasiExten_Index[i][k - 1]])
						)
					{
						continue;
					}
					else
					{
						break;
					}
				}
				//
				temp_posi = k;
				if (temp_posi<savenum)
				{
					if (ordered_num<savenum)
					{
						ordered_num = ordered_num + 1;
					}
					//前面的依次向后串
					for (k = ordered_num - 1; k > temp_posi; k--)
					{
						ForfeasiExten_Index[i][k] = ForfeasiExten_Index[i][k - 1];
					}
					//再赋值
					ForfeasiExten_Index[i][temp_posi] = temp_Index[j];
				}
			}
			ForfeasiExten_Index_num[i] = savenum;
		}
		//反向
		for (i = 0; i < p.Customer_Num; i++)
		{
			if (BackfeasiExten_Index_num[i] <= 1)continue;
			savenum = int(arc_remain_proportion*BackfeasiExten_Index_num[i]);
			for (j = 0; j <BackfeasiExten_Index_num[i]; j++)
			{
				temp_Index[j] = BackfeasiExten_Index[i][j];
			}
			//开始排序
			BackfeasiExten_Index[i][0] = temp_Index[0];
			ordered_num = 1;
			for (j = 1; j <BackfeasiExten_Index_num[i]; j++)
			{
				for (k = ordered_num; k >= 1; k--)
				{
					if (
						(BB.branch[curnode].CostNetwork_Branch[temp_Index[j]][i] - lp.Customer_dual[temp_Index[j]])
						<(BB.branch[curnode].CostNetwork_Branch[BackfeasiExten_Index[i][k - 1]][i] - lp.Customer_dual[BackfeasiExten_Index[i][k - 1]])
						)
					{
						continue;
					}
					else
					{
						break;
					}
				}
				//
				temp_posi = k;
				if (temp_posi<savenum)
				{
					if (ordered_num<savenum)
					{
						ordered_num = ordered_num + 1;
					}
					//前面的依次向后串
					for (k = ordered_num - 1; k > temp_posi; k--)
					{
						BackfeasiExten_Index[i][k] = BackfeasiExten_Index[i][k - 1];
					}
					//再赋值
					BackfeasiExten_Index[i][temp_posi] = temp_Index[j];
				}
			}
			BackfeasiExten_Index_num[i] = savenum;
		}
	}
}

void Subproblem::Build_Mapping(BranchABound & BB, RMP & lp, Problem & p)
{
	bool vaild_node;

	for (int i = 0; i < p.Customer_Num; i++)
	{
		vaild_node = Check_validCus(i,  BB, lp);

		if (true== vaild_node)
		{
			Mapping_Reduced[Modifiednetwork_Nodes_num] = i;
			Mapping_fromRN_toMN[i] = Modifiednetwork_Nodes_num;
			Modifiednetwork_Nodes_num = Modifiednetwork_Nodes_num + 1;
		}
	}
	passnode_length = int((float)(Modifiednetwork_Nodes_num - MINDOUBLE) / (float)Conf::EACH_LONG_NUM) + 1;
}

void Subproblem::Reset_Maxtime(BranchABound & BB, RMP & lp, Problem & p)
{
	float temp=-1;
	float temp_time;
	int temp_customer;

	for (int j = 0; j<Modifiednetwork_Nodes_num; j++)
	{
		temp_customer = Mapping_Reduced[j];

		temp_time = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot];		//时间可能需要转换
		if (temp_time<MAXNUM - 1)
		{
			if ((p.Allnode[temp_customer].endTW + p.Allnode[temp_customer].servicetime + temp_time)>temp)
			{
				temp = p.Allnode[temp_customer].endTW + p.Allnode[temp_customer].servicetime + temp_time;
			}
		}
	}
	p.Max_arrivaltime[current_depot] = temp;
}

bool Subproblem::Check_validCus(int check_node, BranchABound & BB, RMP & lp)
{
	bool temp_check = false;

#if Frameworks == 0
	if (lp.Customer_dual[check_node] > 10 * MINDOUBLE || 0 == BB.branch[BB.cur_node].valid_Cus[check_node])
	{
		temp_check = true;
	}
#else
	if (lp.Customer_dual[check_node] > 10 * MINDOUBLE || lp.SDC_dual[check_node] > 10 * MINDOUBLE)
	{
		temp_check = true;
	}
	else if (0 == BB.branch[BB.cur_node].valid_Cus[check_node])
	{
		temp_check = true;
	}
#endif
	
	return temp_check;
}

void Subproblem::Ini_compleBound_bucket(Problem & p)
{
	for (int i = 0; i<p.Customer_Num; i++)
	{
		//前向
		ForwardBucket[i].Bucket_max = int(floor(p.Max_arrivaltime[current_depot] - p.Allnode[i].startTW - p.Allnode[i].servicetime));
		ForwardBucket[i].Bucket_min = int(floor(max(p.Time_network[i][p.Customer_Num+ current_depot], p.Max_arrivaltime[current_depot]- p.Allnode[i].endTW - p.Allnode[i].servicetime)));
		//反向
		BackwardBucket[i].Bucket_max = int(floor(p.Allnode[i].endTW));
		BackwardBucket[i].Bucket_min = int(floor(p.Allnode[i].startTW));
	}

}

void Subproblem::Initial_Hpoints(Problem &p)
{
	float bound = 0.48;

	if (true == Conf::Not_DynamicBound)
	{
		//以lastest_departuretime还是以surplus_time为关键资源，初始化过程相同
		Hpoint_Back = 0;
		Hpoint_For = p.Max_arrivaltime[current_depot];
	}
	else
	{
		//以lastest_departuretime还是以surplus_time为关键资源，初始化过程相同
		Hpoint_Back = (1 - bound)*p.Max_arrivaltime[current_depot];
		Hpoint_For = bound*p.Max_arrivaltime[current_depot];
	}
}

void Subproblem::Ini_ngset(BranchABound & BB, RMP & lp, Problem & p)
{
	int temp_posi,i,j,k;
	int ordered_num;	//neighbourhood中已经排序好的数量
	bool check;
	//根据对偶变量，找到i节点上离得最近的NEIGHBOURHOOD_NUM个节点,其中i节点一定在neighbourhood中(升序)
	ng_clear();

	//确定ng的大小
	int temp_ngnum;
	if (Conf::MAX_NEIGHBOURHOOD_NUM>Modifiednetwork_Nodes_num)
	{
		temp_ngnum = Modifiednetwork_Nodes_num;
	}
	else
	{
		temp_ngnum = Conf::MAX_NEIGHBOURHOOD_NUM;
	}

#if NGINI == 0
	int cur_customer, add_customer;
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		cur_customer = Mapping_Reduced[i];
		for (j = 0; j < Modifiednetwork_Nodes_num; j++)
		{
			add_customer = Mapping_Reduced[j];
			if (true == belong_toset(add_customer, p.Allnode[cur_customer].negSet_passnode))
			{
				neighbourhood[i][neighbourhood_num[i]] = j;
				neighbourhood_num[i] = neighbourhood_num[i] + 1;
			}
		}
	}
#elif NGINI == 1
	//生成每个customer的ngset，pr06一次运行0.001s
	//按照RC值升序排序
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		neighbourhood[i][0] = 0;
		ordered_num = 1;
		for (j = 1; j <Modifiednetwork_Nodes_num; j++)
		{
			for (k = ordered_num; k >= 1; k--)
			{
				if (
					(BB.branch[BB.cur_node].CostNetwork_Branch[Mapping_Reduced[i]][Mapping_Reduced[j]] - lp.Customer_dual[Mapping_Reduced[j]])
					<(BB.branch[BB.cur_node].CostNetwork_Branch[Mapping_Reduced[i]][Mapping_Reduced[neighbourhood[i][k - 1]]] - lp.Customer_dual[Mapping_Reduced[neighbourhood[i][k - 1]]])
					)
				{
					continue;
				}
				else
				{
					break;
				}
}
			//
			temp_posi = k;
			if (temp_posi<temp_ngnum)
			{
				if (ordered_num<temp_ngnum)
				{
					ordered_num = ordered_num + 1;
				}
				//前面的依次向后串
				for (k = ordered_num - 1; k > temp_posi; k--)
				{
					neighbourhood[i][k] = neighbourhood[i][k - 1];
				}
				//再赋值
				neighbourhood[i][temp_posi] = j;
			}
			}
		neighbourhood_num[i] = ordered_num;
		}
#endif

	//检查i节点是否在i节点的邻域中，如果不在则加入末尾位置
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		check = true;
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			if (i == neighbourhood[i][j])
			{
				check = false;
				break;
			}
		}
		if (true == check)//i节点不在i节点的邻域中
		{
			neighbourhood[i][neighbourhood_num[i] - 1] = i;
		}
	}
	//转为二进制，存在neighbourhood_passnode中
	int lineid, rowid, nodeno;
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			nodeno = neighbourhood[i][j];
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			neighbourhood_passnode[i][rowid] += passnodelist[lineid];
		}

	}
	//转换neighbourhood得到ng_memory_passnode
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			nodeno = i;
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			ng_memory_passnode[neighbourhood[i][j]][rowid] += passnodelist[lineid];
		}
	}
}

void Subproblem::ng_clear(void)
{
	for (int i = 0; i <Modifiednetwork_Nodes_num; i++)
	{
		for (int j = 0; j < passnode_length;j++)
		{
			neighbourhood_passnode[i][j] = 0;
			ng_memory_passnode[i][j] = 0;
		}
		neighbourhood_num[i] = 0;
	}

}

void Subproblem::Initial_lables(BranchABound & BB, RMP & lp, Problem & p, SRC & Cuts_src, SDC &Cuts_sdc)
{
	int i,j;
	int temp_back_posi, temp_for_posi;
	int temp_customer,modify_Node;		//node_id
	int incre_deleted;

	//在拓展网络中，所有label按照node_id存储，所以有p.Customer_Num个Bucket
	//在每个customer中存储的label都已经拓展到出发点
	//对每个Mapping_Reduced中的点
	/////////////////////////////////////反向标签////////////////////////////////////////////////////
	for (i=0;i<BackfeasiExten_Index_num[p.Customer_Num];i++)
	{
		//客户
		temp_customer = BackfeasiExten_Index[p.Customer_Num][i];
#if EXACTEXTEND==0
		if (true== restricted_extend && (BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] - lp.Customer_dual[temp_customer])>-MINDOUBLE)continue;
#endif
		if (BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num+current_depot]>MAXNUM - 1) continue;
		//拓展网络对应点
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//确定插入BackwardPath_container的位置
		temp_back_posi = Seek_containerIndex(BACKWARD);
		//以下在BackwardPath_container中temp_back_posi位置构建副本
		BackwardPath_container[temp_back_posi].exist_state = 0;
		BackwardPath_container[temp_back_posi].extend_state = 1;
		//拓展到出发点:从进入点到出发点
		BackwardPath_container[temp_back_posi].customeList[0] = temp_customer;
		BackwardPath_container[temp_back_posi].customeList_num = 1;
		//更新资源信息
		BackwardPath_container[temp_back_posi].availablecapacity = p.Veh.Veh_Capacity - p.Allnode[temp_customer].demand;
		BackwardPath_container[temp_back_posi].accu_duration =BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot]+p.Allnode[temp_customer].servicetime;//时间可能需要转换
		BackwardPath_container[temp_back_posi].consumed_time = max(BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot], p.Max_arrivaltime[current_depot] - p.Allnode[temp_customer].endTW - p.Allnode[temp_customer].servicetime);//时间可能需要转换
		BackwardPath_container[temp_back_posi].surplus_time = p.Max_arrivaltime[current_depot] - BackwardPath_container[temp_back_posi].consumed_time;
		if (BackwardPath_container[temp_back_posi].surplus_time<Hpoint_For)
		{
			BackwardPath_container[temp_back_posi].extend_state = 0;
		}
		//拓展时，路径的RC值直接包含temp_customer所有的node
#if KPATHCUT +RCCUT==0
		BackwardPath_container[temp_back_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] - lp.Customer_dual[temp_customer] - fix_cost;
#else
		BackwardPath_container[temp_back_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] - lp.RobustCut_dual[temp_customer][p.Customer_Num + current_depot] - lp.Customer_dual[temp_customer] - fix_cost;
#endif
		
		BackwardPath_container[temp_back_posi].augment_ornot = false;

		//重置经过的节点
		BackwardPath_container[temp_back_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			BackwardPath_container[temp_back_posi].passnode[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_2cycle[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_ngpath[j] = 0;
			BackwardPath_container[temp_back_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary有关的
		BackwardPath_container[temp_back_posi].elementaty = true;
		BackwardPath_container[temp_back_posi].prenode = -1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode);
		//ngpath有关的
		BackwardPath_container[temp_back_posi].ngpath[BackwardPath_container[temp_back_posi].ngpath_num] = modify_Node;
		BackwardPath_container[temp_back_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode_ngpath);

		//SDC使用的label resource
#if Frameworks == 1
		for (j = 0; j < passnode_length; j++)
		{
			BackwardPath_container[temp_back_posi].passnode_SDCnode[j] = 0;
		}
		if (1==Cuts_sdc.SDC_indicator[temp_customer])
		{
			//插入在SDCnode的第一个位置
			BackwardPath_container[temp_back_posi].SDCnode[0] = temp_customer;
			BackwardPath_container[temp_back_posi].SDCnode_num = 1;
			Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode_SDCnode);
			BackwardPath_container[temp_back_posi].RC_value = BackwardPath_container[temp_back_posi].RC_value - lp.SDC_dual[temp_customer];
		}
		else
		{
			BackwardPath_container[temp_back_posi].SDCnode_num = 0;
		}
#endif

#if SRCUT == 1
		//初始化state
		//首先把state_src重置为0
		for (j = 0; j < Cuts_src.processed_num; j++)
		{
			BackwardPath_container[temp_back_posi].state_src[j] = 0;
			//判断节点temp_customer是否在第j个Cuts_src的subset中
			if (1==Cuts_src.SRC_subset_indicator[j][temp_customer])
			{
				BackwardPath_container[temp_back_posi].state_src[j] = Cuts_src.add_state(Cuts_src.SRC_subset_len[j]);
			}
		}
#endif
		incre_deleted = Delelted_Backwardindex_num;	//初始化
		//最后来判断是否支配
		if (false == domination_2cycle_ngpath_cuts(BACKWARD, temp_customer, lp,BackwardPath_container[temp_back_posi], Cuts_src))
		{
			incre_deleted = Delelted_Backwardindex_num - incre_deleted;
			//如果不被支配，则加入BackwardBucket[temp_customer]中
			//首先在BackwardPath_container中确认添加
			BackwardPath_container[temp_back_posi].exist_state = 1;
			//然后更新BackwardPath_container和Delelted_Backwardindex中的序号
			Add_container(BACKWARD, temp_back_posi, incre_deleted);
			//最后添加到BackwardBucket[temp_customer]中
			BackwardBucket[temp_customer].Path_index[BackwardBucket[temp_customer].Path_Num] = temp_back_posi;
			BackwardBucket[temp_customer].Path_Num = BackwardBucket[temp_customer].Path_Num + 1;
			//Bac_unprocessed_label_num = Bac_unprocessed_label_num + 1;
			//Bac_generated_label_num = Bac_generated_label_num + 1;
		}
	}
	/////////////////////////////////////前向标签////////////////////////////////////////////////////
	for (i = 0; i < ForfeasiExten_Index_num[p.Customer_Num]; i++)
	{
		//客户
		temp_customer = ForfeasiExten_Index[p.Customer_Num][i];
#if EXACTEXTEND==0
		if (true == restricted_extend && (BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - lp.Customer_dual[temp_customer])>-MINDOUBLE)continue;
#endif
		if (BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer]>MAXNUM - 1) continue;
		//拓展网络对应点
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//确定插入ForwardPath_container的位置
		temp_for_posi = Seek_containerIndex(FORWARD);
		//在ForwardPath_container中temp_for_posi位置构建副本
		ForwardPath_container[temp_for_posi].exist_state = 0;
		ForwardPath_container[temp_for_posi].extend_state = 1;
		//拓展到出发点:从进入点到出发点
		ForwardPath_container[temp_for_posi].customeList[0] = temp_customer;
		ForwardPath_container[temp_for_posi].customeList_num = 1;
		//资源更新
		ForwardPath_container[temp_for_posi].usedcapacity = p.Allnode[temp_customer].demand;
		ForwardPath_container[temp_for_posi].accu_duration =BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] + p.Allnode[temp_customer].servicetime;  //时间可能需要转换
		ForwardPath_container[temp_for_posi].arrivaltime = max(BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer], p.Allnode[temp_customer].startTW); //时间可能需要转换


		if (ForwardPath_container[temp_for_posi].arrivaltime > Hpoint_For)
		{
			ForwardPath_container[temp_for_posi].extend_state = 0;
		}
		//拓展时，路径的RC值直接包含temp_customer所有的node
#if KPATHCUT +RCCUT==0
		ForwardPath_container[temp_for_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - lp.Customer_dual[temp_customer] - fix_cost;
#else
		ForwardPath_container[temp_for_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - lp.RobustCut_dual[p.Customer_Num + current_depot][temp_customer] - lp.Customer_dual[temp_customer] - fix_cost;
#endif
		ForwardPath_container[temp_for_posi].augment_ornot = false;

		//重置经过的节点
		ForwardPath_container[temp_for_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			ForwardPath_container[temp_for_posi].passnode[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_2cycle[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_ngpath[j] = 0;
			ForwardPath_container[temp_for_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary有关的
		ForwardPath_container[temp_for_posi].elementaty = true;
		ForwardPath_container[temp_for_posi].prenode = -1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode);
		//ngpath有关的
		ForwardPath_container[temp_for_posi].ngpath[ForwardPath_container[temp_for_posi].ngpath_num] = modify_Node;
		ForwardPath_container[temp_for_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode_ngpath);

		//SDC使用的label resource
#if Frameworks == 1
		for (j = 0; j < passnode_length; j++)
		{
			ForwardPath_container[temp_for_posi].passnode_SDCnode[j] = 0;
		}
		if (1 == Cuts_sdc.SDC_indicator[temp_customer])
		{
			//插入在SDCnode的第一个位置
			ForwardPath_container[temp_for_posi].SDCnode[0] = temp_customer;
			ForwardPath_container[temp_for_posi].SDCnode_num = 1;
			Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode_SDCnode);
			ForwardPath_container[temp_for_posi].RC_value = ForwardPath_container[temp_for_posi].RC_value - lp.SDC_dual[temp_customer];
		}
		else
		{
			ForwardPath_container[temp_for_posi].SDCnode_num = 0;
		}
#endif

#if SRCUT == 1
		//初始化state
		//首先把state_src重置为0
		for (j = 0; j < Cuts_src.processed_num; j++)
		{
			ForwardPath_container[temp_for_posi].state_src[j] = 0;
			//判断节点temp_customer是否在第j个Cuts_src的subset中
			if (1 == Cuts_src.SRC_subset_indicator[j][temp_customer])
			{
				ForwardPath_container[temp_for_posi].state_src[j] = Cuts_src.add_state(Cuts_src.SRC_subset_len[j]);
			}
		}
#endif
		incre_deleted = Delelted_Forwardindex_num;	//初始化
		//最后来判断是否支配
		if (false == domination_2cycle_ngpath_cuts(FORWARD, temp_customer, lp, ForwardPath_container[temp_for_posi], Cuts_src))
		{
			incre_deleted = Delelted_Forwardindex_num - incre_deleted;
			//如果不被支配，则加入ForwardBucket[temp_customer]中
			//首先在ForwardPath_container中确认添加
			ForwardPath_container[temp_for_posi].exist_state = 1;
			//然后更新ForwardPath_container和Delelted_Forwardindex中的序号
			Add_container(FORWARD, temp_for_posi, incre_deleted);
			//最后添加到ForwardBucket[temp_customer]中
			ForwardBucket[temp_customer].Path_index[ForwardBucket[temp_customer].Path_Num] = temp_for_posi;
			ForwardBucket[temp_customer].Path_Num = ForwardBucket[temp_customer].Path_Num + 1;
			//Bac_unprocessed_label_num = Bac_unprocessed_label_num + 1;
			//Bac_generated_label_num = Bac_generated_label_num + 1;
		}
	}
}

int Subproblem::Seek_containerIndex(int direction)
{
	int temp_posi;
	if (BACKWARD== direction)
	{
		if (Delelted_Backwardindex_num>0)
		{
			//Delelted_Backwardindex倒序添加
			temp_posi = Delelted_Backwardindex[Delelted_Backwardindex_num - 1];
		}
		else
		{
			//BackwardPath_container顺序添加
			temp_posi = BackwardPath_container_num;
		}
	}
	else
	{
		if (Delelted_Forwardindex_num>0)
		{
			temp_posi = Delelted_Forwardindex[Delelted_Forwardindex_num - 1];
		}
		else
		{
			temp_posi = ForwardPath_container_num;
		}
	}
	return temp_posi;
}

int Subproblem::Get_FoundIndex(void)
{
	int temp_posi;
	if (Delelted_Foundindex_num>0)
	{
		//Delelted_Foundindex_num倒序添加
		temp_posi = Delelted_Foundindex[Delelted_Foundindex_num - 1];
	}
	else
	{
		//FoundPath_container顺序添加
		temp_posi = FoundPath_container_num;
	}
	return temp_posi;
}

void Subproblem::Add_container(int direction, int order,int incre)
{
	if (BACKWARD == direction)
	{
		if (order >= BackwardPath_container_num)
		{
			BackwardPath_container_num = BackwardPath_container_num + 1;
		}
		else
		{
			if (0 == incre)
			{
				Delelted_Backwardindex_num = Delelted_Backwardindex_num - 1;
			}
			else
			{
				Delelted_Backwardindex[Delelted_Backwardindex_num - incre - 1] = Delelted_Backwardindex[Delelted_Backwardindex_num - 1] ;
				Delelted_Backwardindex_num = Delelted_Backwardindex_num - 1;
			}
		}
	}
	else
	{
		if (order >= ForwardPath_container_num)
		{
			ForwardPath_container_num = ForwardPath_container_num + 1;
		}
		else
		{
			if (0== incre)
			{
				Delelted_Forwardindex_num = Delelted_Forwardindex_num - 1;
			}
			else
			{
				Delelted_Forwardindex[Delelted_Forwardindex_num - incre - 1] = Delelted_Forwardindex[Delelted_Forwardindex_num - 1];
				Delelted_Forwardindex_num = Delelted_Forwardindex_num - 1;
			}
		}
	}
}

void Subproblem::Add_found(int order)
{
	if (order >= FoundPath_container_num)
	{
		FoundPath_container_num = FoundPath_container_num + 1;
	}
	else
	{
		Delelted_Foundindex_num = Delelted_Foundindex_num - 1;
	}
}

void Subproblem::Delete_container(int direction, int container_index)
{
	if (BACKWARD == direction)
	{
		Delelted_Backwardindex[Delelted_Backwardindex_num] = container_index;
		Delelted_Backwardindex_num = Delelted_Backwardindex_num + 1;
	}
	else
	{
		Delelted_Forwardindex[Delelted_Forwardindex_num] = container_index;
		Delelted_Forwardindex_num = Delelted_Forwardindex_num + 1;
	}
}

void Subproblem::Delete_pathindex(int direction, int customer, int pathindex)
{
	if (BACKWARD == direction)
	{
		//首先把objbucket.Path_index最后一个元素与objbucket.Path_index[pathindex]交换
		BackwardBucket[customer].Path_index[pathindex] = BackwardBucket[customer].Path_index[BackwardBucket[customer].Path_Num-1];
		BackwardBucket[customer].Path_Num = BackwardBucket[customer].Path_Num - 1;
	}
	else
	{
		ForwardBucket[customer].Path_index[pathindex] = ForwardBucket[customer].Path_index[ForwardBucket[customer].Path_Num - 1];
		ForwardBucket[customer].Path_Num = ForwardBucket[customer].Path_Num - 1;
	}
}

void Subproblem::Insert_toPassnode(int insert_modifynode, long * temp_passnode)
{
	int lineid, rowid;
	rowid = int((insert_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = insert_modifynode - rowid*Conf::EACH_LONG_NUM;
	temp_passnode[rowid] += passnodelist[lineid];
}

void Subproblem::remove_fromPassnode(int remove_modifynode, long * temp_passnode)
{
	int lineid, rowid;
	rowid = int((remove_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = remove_modifynode - rowid*Conf::EACH_LONG_NUM;
	temp_passnode[rowid] = temp_passnode[rowid] - passnodelist[lineid];
}

bool Subproblem::if_union_domination(bool iniornot, long * newroute)
{
	if (true == iniornot)
	{
		//初始化模板
		for (int i = 0; i < passnode_length; i++)
		{
			temppass[i] = newroute[i];
		}
		return false;
	}
	else
	{
		for (int i = 0; i < passnode_length; i++)
		{
			temppass[i] = temppass[i] & newroute[i];
			if (temppass[i] != 0)
			{
				return false;
			}
		}
	}
	return true;
}

bool Subproblem::domination_2cycle_ngpath_cuts(int direction, int nextcustomer, RMP & lp, Path & nextV,  SRC & Cuts_src)
{
	int i, k;
	float RC_increment = 0;							//若L1支配L2，则L1的RC值至少要比L2的RC值少了RC_increment
													//EPO框架下，RC_increment为0
													//CPA框架下，RC_increment可能为一个正数
	int node_num=0,store_prenode = -1;
	int next_Labelindex;							//container中的下一个label的序号
	bool controled = false;							//新线路是否被支配，true被支配，false不被支配
	bool ini_ornot = true;							//首先要初始化模板
	bool complete_donimated_byNgpath = false;

	if (direction == FORWARD)//正向
	{
		///////////////////////////////////////////////////////////////////先判断新线路nextV是否被ForwardBucket[nextcustomer]中某条线路支配////////////////////////////////////////////////////////////////////////////
		//比较的是终点为nextcustomer的所有路径
		for(k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到ForwardBucket[nextcustomer]每个label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			//路径的支配关系为：设P1路径的经过的最后cycle_eli_num个节点集合为S1，P2路径经过的最后cycle_eli_num个节点集合为S2
			//S1对应的资源消耗量为R1(包括成本)，S2对应的资源消耗量为R2（包括成本）
			//情况1：S1是S2的子集，如果R1<=R2，P1支配P2
			//情况2：如果存在任意一条路径P3（终点与P1和P2相同），满足：1) S2的补集是（S1的补集并上S3的补集）的子集；2）R1<=R2和R3<=R2 ,那么P2可以被支配

			//考虑的路径的最后节点一定为nextcustomer
			//首先看，ForwardPath_container[next_Labelindex]是否为nextV的子集
			//if (ForwardPath_container[next_Labelindex].nodenum>nextV.nodenum)continue;		//如果ForwardPath_container[next_Labelindex]的节点数比新线路大，继续循环
			//首先找到支配nextV的ForwardPath_container[next_Labelindex],支配条件使用最松的SPPRC准则
			if (ForwardPath_container[next_Labelindex].usedcapacity>nextV.usedcapacity)continue;
			if (ForwardPath_container[next_Labelindex].arrivaltime>nextV.arrivaltime/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (ForwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif
			RC_increment = Calculate_RCincrement(ForwardPath_container[next_Labelindex], nextV, lp);	//计算RC_increment
			if (ForwardPath_container[next_Labelindex].RC_value+ RC_increment>nextV.RC_value + MINDOUBLE)continue;
#if SRCUT == 1
			if (ForwardPath_container[next_Labelindex].RC_value- Calculate_SRCdual_Gap(ForwardPath_container[next_Labelindex], nextV, lp, Cuts_src)>nextV.RC_value + MINDOUBLE)continue;
#endif

			//满足以上条件，说明ForwardPath_container[next_Labelindex]至少可以部分支配nextV
#if NGDOI == 0		//ng完全支配
			//ForwardPath_container[next_Labelindex]的ng-path是nextV的ng-path的子集，则ForwardPath_container[next_Labelindex]支配nextV
			//ng-path
			//判断是否完全支配
			if (true == check_subset(ForwardPath_container[next_Labelindex].passnode_ngpath, nextV.passnode_ngpath))
			{
				controled = true;				//如果上述条件都满足则新线路被支配
				return controled;
			}
			//首先，找到ForwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集
			for (i = 0; i < ForwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//找到ForwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集个每个元素
				if (false == belong_toset(ForwardPath_container[next_Labelindex].ngpath[i], nextV.passnode_ngpath))
				{
					if (false == nextV.augment_ornot)
					{
						nextV.get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[ForwardPath_container[next_Labelindex].ngpath[i]], nextV.passnode_ngpath);
						nextV.augment_ornot = true;
					}
					else
					{
						if (true == nextV.get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[ForwardPath_container[next_Labelindex].ngpath[i]]))
						{
							//多个部分支配，组成完全支配
							controled = true;				//如果上述条件都满足则新线路被支配
							return controled;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng支配
			complete_donimated_byNgpath = true;
			//ForwardPath_container[next_Labelindex]的ng-path是nextV的ng-path的子集，则ForwardPath_container[next_Labelindex]支配nextV
			//ng-path
			//首先，找到ForwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集
			for (i = 0; i < ForwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//找到ForwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集个每个元素
				if (false == belong_toset(ForwardPath_container[next_Labelindex].ngpath[i], nextV.passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == nextV.augment_ornot)
					{
						nextV.get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[ForwardPath_container[next_Labelindex].ngpath[i]], nextV.passnode_ngpath);
						nextV.augment_ornot = true;
					}
					else
					{
						if (true == nextV.get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[ForwardPath_container[next_Labelindex].ngpath[i]]))
						{
							//多个部分支配，组成完全支配
							controled = true;				//如果上述条件都满足则新线路被支配
							return controled;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//如果上述条件都满足则新线路被支配
				return controled;
			}
#endif
			if (false== exact_ornot)
			{
#if TCYCLE == 1		//精确的2cycle支配规则
				if (true == check_subset(ForwardPath_container[next_Labelindex].passnode_2cycle, nextV.passnode_2cycle))
				{
					//先判断ForwardPath_container[next_Labelindex].passnode_2cycle是否为和nextV.passnode_2cycle的子集
					//如果相同，则nextV被支配
					controled = true;				//如果上述条件都满足则新线路被支配
					return controled;
				}
				else
				{
					//再判断是否存在其他任意个路径且都支配nextV的路径
					//如果存在，则nextV被支配
					if (false == if_union_domination(ini_ornot, ForwardPath_container[next_Labelindex].passnode_2cycle))
					{
						//先初始化模板
						ini_ornot = false;
						continue;
					}
					else
					{
						controled = true;				//如果上述条件都满足则新线路被支配
						return controled;
					}
				}
#elif TCYCLE == 2	//启发式的2cycle支配规则
				if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					//先判断ForwardPath_container[next_Labelindex]和nextV倒数第二个节点是否相同
					//如果相同，则nextV被支配
					controled = true;
					return controled;
				}
				else
				{
					//ForwardBucket[nextcustomer]中至少存在两个ForwardPath_container[next_Labelindex]与nextV倒数第二个节点不同的
					//这两个ForwardPath_container[next_Labelindex]的prenode也不相同
					if (ForwardPath_container[next_Labelindex].prenode != store_prenode)
					{
						if (1 == node_num)
						{
							//如果已经找到满足条件的两个ForwardPath_container[next_Labelindex]，则nextV被支配
							controled = true;
							return controled;
						}
						else
						{
							//记录找到的ForwardPath_container[next_Labelindex]
							store_prenode = ForwardPath_container[next_Labelindex].prenode;
							node_num = node_num + 1;
						}
					}
				}
#endif
			}
		}
		////////////////////////////////////////////////////////再判断ForwardBucket[nextcustomer]的某条线路是否被新线路nextV支配////////////////////////////////////////////////////////
		for (k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到ForwardBucket[nextcustomer]每个label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			if (0 == ForwardPath_container[next_Labelindex].exist_state)
			{
				continue;//如果ForwardPath_container[next_Labelindex]已经被支配掉了，就没有必要去比了
			}
			//首先找到被nextV支配的ForwardPath_container[next_Labelindex]，使用SPPRC支配准则
			//如果nextV不支配ForwardPath_container[next_Labelindex]的跳过
			if (nextV.usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
			if (nextV.arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
			RC_increment = Calculate_RCincrement(nextV, ForwardPath_container[next_Labelindex], lp);	//计算RC_increment
			if (nextV.RC_value+ RC_increment>ForwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#if SRCUT == 1
			if (nextV.RC_value - Calculate_SRCdual_Gap(ForwardPath_container[next_Labelindex], nextV, lp, Cuts_src)>ForwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#endif

			//满足以上条件，说明nextV至少可以部分支配ForwardPath_container[next_Labelindex]
#if NGDOI == 0		//ng完全支配
			//ng-path
			//判断是否完全支配
			if (true == check_subset(nextV.passnode_ngpath, ForwardPath_container[next_Labelindex].passnode_ngpath))
			{
				//同下，先处理container,再处理链表
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
				continue;
			}
			//因为ForwardPath_container[next_Labelindex].feasible_extensions_byngpath已经被更新过，所以只需要检查nextV对ForwardPath_container[next_Labelindex]的局部支配关系
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//找到nextV.passnode_ngpath对ForwardPath_container[next_Labelindex].passnode_ngpath的补集个每个元素
				if (false == belong_toset(nextV.ngpath[i], ForwardPath_container[next_Labelindex].passnode_ngpath))
				{
					if (false == ForwardPath_container[next_Labelindex].augment_ornot)
					{
						ForwardPath_container[next_Labelindex].get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]], ForwardPath_container[next_Labelindex].passnode_ngpath);
						ForwardPath_container[next_Labelindex].augment_ornot = true;
					}
					else
					{
						if (true == ForwardPath_container[next_Labelindex].get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]]))
						{
							//多个部分支配，导致ForwardPath_container[next_Labelindex]被完全支配
							//删除ForwardBucket[nextcustomer].Path_index[k]序号;
							Delete_pathindex(direction, nextcustomer, k);
							//删除ForwardPath_container中元素，并记录在Delelted_Forwardindex中
							Delete_container(direction, next_Labelindex);
							//先标注ForwardPath_container中exist_state状态
							ForwardPath_container[next_Labelindex].exist_state = 0;
							//注意迭代器
							k = k - 1;
							break;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng支配
			complete_donimated_byNgpath = true;
			//ng-path
			//因为ForwardPath_container[next_Labelindex].feasible_extensions_byngpath已经被更新过，所以只需要检查nextV对ForwardPath_container[next_Labelindex]的局部支配关系
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//找到nextV.passnode_ngpath对ForwardPath_container[next_Labelindex].passnode_ngpath的补集个每个元素
				if (false == belong_toset(nextV.ngpath[i], ForwardPath_container[next_Labelindex].passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == ForwardPath_container[next_Labelindex].augment_ornot)
					{
						ForwardPath_container[next_Labelindex].get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]], ForwardPath_container[next_Labelindex].passnode_ngpath);
						ForwardPath_container[next_Labelindex].augment_ornot = true;
					}
					else
					{
						if (true == ForwardPath_container[next_Labelindex].get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]]))
						{
							//多个部分支配，导致ForwardPath_container[next_Labelindex]被完全支配
							//删除ForwardBucket[nextcustomer].Path_index[k]序号;
							Delete_pathindex(direction, nextcustomer, k);
							//删除ForwardPath_container中元素，并记录在Delelted_Forwardindex中
							Delete_container(direction, next_Labelindex);
							//先标注ForwardPath_container中exist_state状态
							ForwardPath_container[next_Labelindex].exist_state = 0;
							//注意迭代器
							k = k - 1;
							break;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				//同上，先处理container,再处理链表
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}
#endif

			if (0 == ForwardPath_container[next_Labelindex].exist_state)continue;

			if (false==exact_ornot)
			{
#if TCYCLE == 1		//精确的2cycle支配规则
				if (true == check_subset(nextV.passnode_2cycle, ForwardPath_container[next_Labelindex].passnode_2cycle))
				{
					//先判断nextV.passnode_2cycle是否为ForwardPath_container[next_Labelindex].passnode_2cycle的子集
					//如果相同，则ForwardPath_container[next_Labelindex]被支配
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					ForwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					if_union_domination(true, nextV.passnode_2cycle);
					//首先知道：nextV已经SPPRC支配ForwardPath_container[next_Labelindex]
					//再判断是否存在其他任意个路径与nextV配合支配ForwardPath_container[next_Labelindex]
					//如果存在，则ForwardPath_container[next_Labelindex]被支配
					for (i = 0; i < ForwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = ForwardBucket[nextcustomer].Path_index[i];
						if (0 == ForwardPath_container[temp_label].exist_state)
						{
							continue;//如果ForwardPath_container[temp_label]已经被支配掉了，就没有必要去比了
						}
						//找到支配ForwardPath_container[next_Labelindex]的ForwardPath_container[temp_label]
						if (ForwardPath_container[temp_label].usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
						if (ForwardPath_container[temp_label].arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
						if (ForwardPath_container[temp_label].accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp);	//计算RC_increment
						if (ForwardPath_container[temp_label].RC_value+ RC_increment>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (ForwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp, Cuts_src)>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//ForwardPath_container[temp_label]也SPPRC支配ForwardPath_container[next_Labelindex]
						if (true == if_union_domination(false, ForwardPath_container[temp_label].passnode_2cycle))
						{
							Delete_pathindex(direction, nextcustomer, k);
							Delete_container(direction, next_Labelindex);
							ForwardPath_container[next_Labelindex].exist_state = 0;
							k = k - 1;
							break;
						}
					}
				}
#elif TCYCLE == 2	//启发式的2cycle支配规则
				//如果被支配：
				//先判断nextV倒数第二节点是否和ForwardPath_container[next_Labelindex]的倒数第二个节点相同
				if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					ForwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					//首先知道：nextV已经SPPRC支配ForwardPath_container[next_Labelindex]
					//再判断是否存在其他任意个路径与nextV配合支配ForwardPath_container[next_Labelindex]
					//如果存在，则ForwardPath_container[next_Labelindex]被支配
					for (i = 0; i < ForwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = ForwardBucket[nextcustomer].Path_index[i];
						if (0 == ForwardPath_container[temp_label].exist_state)
						{
							continue;//如果ForwardPath_container[temp_label]已经被支配掉了，就没有必要去比了
						}
						//找到支配ForwardPath_container[next_Labelindex]的ForwardPath_container[temp_label]
						if (ForwardPath_container[temp_label].usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
						if (ForwardPath_container[temp_label].arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
						if (ForwardPath_container[temp_label].accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp);	//计算RC_increment
						if (ForwardPath_container[temp_label].RC_value+ RC_increment>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (ForwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp, Cuts_src)>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//ForwardPath_container[temp_label]也SPPRC支配ForwardPath_container[next_Labelindex]
						//ForwardPath_container[temp_label]与nextV的倒数第二个节点不同
						if (nextV.prenode != ForwardPath_container[temp_label].prenode)
						{
							Delete_pathindex(direction, nextcustomer, k);
							Delete_container(direction, next_Labelindex);
							ForwardPath_container[next_Labelindex].exist_state = 0;
							k = k - 1;
							break;
						}
					}
				}
#endif
			}
		}
	}
	else//逆向
	{
		///////////////////////////////////////////////////////////////////先判断新线路nextV是否被某条线路支配////////////////////////////////////////////////////////////////////////////
		//比较的是终点为nextcustomer的所有路径
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到BackwardBucket[nextcustomer]每个label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			//原理同正向
			//首先找到支配nextV的BackwardPath_container[next_Labelindex],支配条件使用最松的SPPRC准则
			if (BackwardPath_container[next_Labelindex].availablecapacity <nextV.availablecapacity)continue;	//如果curS解的第k条线路的剩余空间比新线路小，继续循环
			if (BackwardPath_container[next_Labelindex].surplus_time<nextV.surplus_time/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (BackwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif	
			RC_increment = Calculate_RCincrement(BackwardPath_container[next_Labelindex], nextV, lp);	//计算RC_increment
			if (BackwardPath_container[next_Labelindex].RC_value+ RC_increment>nextV.RC_value + MINDOUBLE)continue;//如果curS解的第k条线路的目标值比新线路不好，继续循环
#if SRCUT == 1
			if (BackwardPath_container[next_Labelindex].RC_value - Calculate_SRCdual_Gap(BackwardPath_container[next_Labelindex], nextV, lp, Cuts_src)>nextV.RC_value + MINDOUBLE)continue;
#endif			

			//满足以上条件，说明BackwardPath_container[next_Labelindex]至少可以部分支配nextV

#if NGDOI == 0		//ng完全支配
			//ng-path
			//判断是否完全支配
			if (true == check_subset(BackwardPath_container[next_Labelindex].passnode_ngpath, nextV.passnode_ngpath))
			{
				controled = true;				//如果上述条件都满足则新线路被支配
				return controled;
			}
			//首先，找到BackwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集
			for (i = 0; i < BackwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//找到BackwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集个每个元素
				if (false == belong_toset(BackwardPath_container[next_Labelindex].ngpath[i], nextV.passnode_ngpath))
				{
					if (false == nextV.augment_ornot)
					{
						nextV.get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[BackwardPath_container[next_Labelindex].ngpath[i]], nextV.passnode_ngpath);
						nextV.augment_ornot = true;
					}
					else
					{
						if (true == nextV.get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[BackwardPath_container[next_Labelindex].ngpath[i]]))
						{
							//多个部分支配，组成完全支配
							controled = true;				//如果上述条件都满足则新线路被支配
							return controled;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng支配
			complete_donimated_byNgpath = true;
			//ng-path
			//首先，找到BackwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集
			for (i = 0; i < BackwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//找到BackwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集个每个元素
				if (false == belong_toset(BackwardPath_container[next_Labelindex].ngpath[i], nextV.passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == nextV.augment_ornot)
					{
						nextV.get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[BackwardPath_container[next_Labelindex].ngpath[i]], nextV.passnode_ngpath);
						nextV.augment_ornot = true;
					}
					else
					{
						if (true == nextV.get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[BackwardPath_container[next_Labelindex].ngpath[i]]))
						{
							//多个部分支配，组成完全支配
							controled = true;				//如果上述条件都满足则新线路被支配
							return controled;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//如果上述条件都满足则新线路被支配
				return controled;
			}
#endif

			if (false==exact_ornot)
			{
#if TCYCLE == 1		//精确的2cycle支配规则
				if (true == check_subset(BackwardPath_container[next_Labelindex].passnode_2cycle, nextV.passnode_2cycle))
				{
					//先判断BackwardPath_container[next_Labelindex].passnode_2cycle是否为和nextV.passnode_2cycle的子集
					//如果相同，则nextV被支配
					controled = true;				//如果上述条件都满足则新线路被支配
					return controled;
				}
				else
				{
					//再判断是否存在其他任意个路径且都支配nextV的路径
					//如果存在，则nextV被支配
					if (false == if_union_domination(ini_ornot, BackwardPath_container[next_Labelindex].passnode_2cycle))
					{
						//先初始化模板
						ini_ornot = false;
						continue;
					}
					else
					{
						controled = true;				//如果上述条件都满足则新线路被支配
						return controled;
					}
				}

#elif TCYCLE == 2	//启发式的2cycle支配规则
				if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					//先判断BackwardPath_container[next_Labelindex]和nextV倒数第二个节点是否相同
					//如果相同，则nextV被支配
					controled = true;
					return controled;
				}
				else
				{
					//BackwardBucket[nextcustomer]中至少存在两个BackwardPath_container[next_Labelindex]与nextV倒数第二个节点不同的
					//这两个BackwardPath_container[next_Labelindex]的prenode也不相同
					if (BackwardPath_container[next_Labelindex].prenode != store_prenode)
					{
						if (1 == node_num)
						{
							//如果已经找到满足条件的两个BackwardPath_container[next_Labelindex]，则nextV被支配
							controled = true;
							return controled;
						}
						else
						{
							//记录找到的BackwardPath_container[next_Labelindex]
							store_prenode = BackwardPath_container[next_Labelindex].prenode;
							node_num = node_num + 1;
						}
					}
				}
#endif
			}
		}
		////////////////////////////////////////////////////////再判断BackwardPath_container[temp_nextcus]的某条线路是否被新线路nextV支配////////////////////////////////////////////////////////
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到BackwardBucket[nextcustomer]每个label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			if (0 == BackwardPath_container[next_Labelindex].exist_state)continue;//如果BackwardPath_container[next_Labelindex]已经被支配掉了，就没有必要去比了
			//首先找到被nextV支配的BackwardPath_container[next_Labelindex]，使用SPPRC支配准则
			//如果nextV不支配BackwardPath_container[next_Labelindex]的跳过
			if (nextV.availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
			if (nextV.surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif	
			RC_increment = Calculate_RCincrement(nextV, BackwardPath_container[next_Labelindex], lp);	//计算RC_increment
			if (nextV.RC_value+ RC_increment>BackwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#if SRCUT == 1
			if (nextV.RC_value - Calculate_SRCdual_Gap(nextV, BackwardPath_container[next_Labelindex], lp, Cuts_src)>BackwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#endif	

			//满足以上条件，说明nextV至少可以部分支配BackwardPath_container[next_Labelindex]
#if NGDOI == 0		//ng完全支配
			//判断是否完全支配
			//ng-path
			if (true == check_subset(nextV.passnode_ngpath, BackwardPath_container[next_Labelindex].passnode_ngpath))
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				BackwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
				continue;
			}
			//因为BackwardPath_container[next_Labelindex].feasible_extensions_byngpath已经被更新过，所以只需要检查nextV对BackwardPath_container[next_Labelindex]的局部支配关系
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//找到nextV.passnode_ngpath对BackwardPath_container[next_Labelindex].passnode_ngpath的补集个每个元素
				if (false == belong_toset(nextV.ngpath[i], BackwardPath_container[next_Labelindex].passnode_ngpath))
				{
					if (false == BackwardPath_container[next_Labelindex].augment_ornot)
					{
						BackwardPath_container[next_Labelindex].get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]], BackwardPath_container[next_Labelindex].passnode_ngpath);
						BackwardPath_container[next_Labelindex].augment_ornot = true;
					}
					else
					{
						if (true == BackwardPath_container[next_Labelindex].get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]]))
						{
							//多个部分支配，导致BackwardPath_container[next_Labelindex]被完全支配
							//删除BackwardBucket[nextcustomer].Path_index[k]序号;
							Delete_pathindex(direction, nextcustomer, k);
							//删除BackwardPath_container中元素，并记录在Delelted_Backwardindex中
							Delete_container(direction, next_Labelindex);
							//先标注BackwardPath_container中exist_state状态
							BackwardPath_container[next_Labelindex].exist_state = 0;
							//再删除链表中元素,注意这里必须考虑迭代器的生存域
							k = k - 1;
							break;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng支配
			complete_donimated_byNgpath = true;
			//ng-path
			//if (false == nextV.check_subset(nextV.passnode_ngpath, BackwardPath_container[next_Labelindex].passnode_ngpath))continue;
			//因为BackwardPath_container[next_Labelindex].feasible_extensions_byngpath已经被更新过，所以只需要检查nextV对BackwardPath_container[next_Labelindex]的局部支配关系
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//找到nextV.passnode_ngpath对BackwardPath_container[next_Labelindex].passnode_ngpath的补集个每个元素
				if (false == belong_toset(nextV.ngpath[i], BackwardPath_container[next_Labelindex].passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == BackwardPath_container[next_Labelindex].augment_ornot)
					{
						BackwardPath_container[next_Labelindex].get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]], BackwardPath_container[next_Labelindex].passnode_ngpath);
						BackwardPath_container[next_Labelindex].augment_ornot = true;
					}
					else
					{
						if (true == BackwardPath_container[next_Labelindex].get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]]))
						{
							//多个部分支配，导致BackwardPath_container[next_Labelindex]被完全支配
							//删除BackwardBucket[nextcustomer].Path_index[k]序号;
							Delete_pathindex(direction, nextcustomer, k);
							//删除BackwardPath_container中元素，并记录在Delelted_Backwardindex中
							Delete_container(direction, next_Labelindex);
							//先标注BackwardPath_container中exist_state状态
							BackwardPath_container[next_Labelindex].exist_state = 0;
							//再删除链表中元素,注意这里必须考虑迭代器的生存域
							k = k - 1;
							break;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				BackwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}
#endif

			if (0 == BackwardPath_container[next_Labelindex].exist_state)continue;

			if(false==exact_ornot)
			{
#if TCYCLE == 1		//精确的2cycle支配规则
				if (true == check_subset(nextV.passnode_2cycle, BackwardPath_container[next_Labelindex].passnode_2cycle))
				{
					//先判断nextV.passnode_2cycle是否为BackwardPath_container[next_Labelindex].passnode_2cycle的子集
					//如果相同，则BackwardPath_container[next_Labelindex]被支配
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					BackwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					if_union_domination(true, nextV.passnode_2cycle);
					//首先知道：nextV已经SPPRC支配BackwardPath_container[next_Labelindex]
					//再判断是否存在其他任意个路径与nextV配合支配BackwardPath_container[next_Labelindex]
					//如果存在，则BackwardPath_container[next_Labelindex]被支配
					for (i = 0; i < BackwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = BackwardBucket[nextcustomer].Path_index[i];
						if (0 == BackwardPath_container[temp_label].exist_state)
						{
							continue;//如果BackwardPath_container[temp_label]已经被支配掉了，就没有必要去比了
						}
						//找到支配BackwardPath_container[next_Labelindex]的BackwardPath_container[temp_label]
						if (BackwardPath_container[temp_label].usedcapacity>BackwardPath_container[next_Labelindex].usedcapacity)continue;
						if (BackwardPath_container[temp_label].arrivaltime>BackwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
						if (BackwardPath_container[temp_label].accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp);	//计算RC_increment
						if (BackwardPath_container[temp_label].RC_value+ RC_increment>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (BackwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp, Cuts_src)>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//BackwardPath_container[temp_label]也SPPRC支配BackwardPath_container[next_Labelindex]
						if (true == if_union_domination(false, BackwardPath_container[temp_label].passnode_2cycle))
						{
							Delete_pathindex(direction, nextcustomer, k);
							Delete_container(direction, next_Labelindex);
							BackwardPath_container[next_Labelindex].exist_state = 0;
							k = k - 1;
							break;
						}
					}
				}
#elif TCYCLE == 2	//启发式的2cycle支配规则
				//如果被支配：
				//先判断nextV倒数第二节点是否和BackwardPath_container[next_Labelindex]的倒数第二个节点相同
				if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					BackwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					//首先知道：nextV已经SPPRC支配BackwardPath_container[next_Labelindex]
					//再判断是否存在其他任意个路径与nextV配合支配BackwardPath_container[next_Labelindex]
					//如果存在，则nextV被支配
					for (i = 0; i < BackwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = BackwardBucket[nextcustomer].Path_index[i];
						if (0 == BackwardPath_container[temp_label].exist_state)continue;//如果BackwardPath_container[temp_label]已经被支配掉了，就没有必要去比了
						 //找到支配BackwardPath_container[next_Labelindex]的BackwardPath_container[temp_label]
						if (BackwardPath_container[temp_label].availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
						if (BackwardPath_container[temp_label].surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
						if (BackwardPath_container[temp_label].accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp);	//计算RC_increment
						if (BackwardPath_container[temp_label].RC_value+ RC_increment>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (BackwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp, Cuts_src)>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//BackwardPath_container[temp_label]也SPPRC支配BackwardPath_container[next_Labelindex]
						//BackwardPath_container[temp_label]与nextV的倒数第二个节点不同
						if (nextV.prenode != BackwardPath_container[temp_label].prenode)
						{
							Delete_pathindex(direction, nextcustomer, k);
							Delete_container(direction, next_Labelindex);
							BackwardPath_container[next_Labelindex].exist_state = 0;
							k = k - 1;
							break;
						}
					}
				}
#endif
			}
		}
	}
	//没有找到释放迭代器listitor的函数，作为局部变量没有new应该没有问题吧
	return controled;
}

bool Subproblem::belong_toset(int node, long *set)
{
	int lineid, rowid;
	rowid = int((node + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = node - rowid*Conf::EACH_LONG_NUM;
	if (0 == (set[rowid] & passnodelist[lineid]))
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Subproblem::check_subset(long * S1, long * S2)
{
	for (int i = 0; i < passnode_length; i++)
	{
		if (S1[i] != (S1[i] & S2[i]))
		{
			return false;
		}
	}
	return true;
}


bool Subproblem::DP_Labeling(int direction, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src,  SDC &Cuts_sdc)
{
	bool exact_ornot=true;
	int solved_customers = 0;
	int Extend_Customer = 0;
	int temp_itor=0;
	
	//这里是对每个customer上的label都进行拓展
	//终止条件是没有customer上存在可以拓展的label了
	do
	{
		solved_customers = 0;
		for (temp_itor=0; temp_itor<p.Customer_Num; temp_itor++)
		{
			//确定循环顺序,极限是确定从哪个label拓展
			//我们这里只确定从哪个customer开始拓展
			Extend_Customer = Search_Nextcustomer(temp_itor, p);
			//对Extend_Customer上的所有label进行拓展
			if (true== ExtendLable_onCustomer(direction, Extend_Customer, BB,lp, p, Cuts_src,Cuts_sdc))
			{
				solved_customers = solved_customers + 1;
			}
		}		
	} while (solved_customers<p.Customer_Num);

	return exact_ornot;
}

int Subproblem::Search_Nextcustomer(int itor, Problem & p)
{
	int temp_ExtenCustomer = 0;

#if SEARCH == 0
	//策略1：按照customer顺序搜索
	temp_ExtenCustomer = itor;
	//
#elif SEARCH == 1	
	//策略2：按照开始服务时间的升序找到customer
	temp_ExtenCustomer = p.Cus_ascend_startTW[itor];
	//
#else
	cout << "Error: SEARCH assignment error." << endl;
	cin.get();
	return false;
#endif

	return temp_ExtenCustomer;
}

bool Subproblem::ExtendLable_onCustomer(int direction, int cur_customer, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc)
{
	bool solved_ornot = true;
	int next_customer;
	int curLabel, nextLabel;
	int incre_deleted;

	//拓展网络中的label拓展有特殊规则：
	//由于支配只在cutomer上发生，同一个customer下的所有label都被存储在出发点
	//拓展过程也是：从出发点拓展，直到另一个出发点停下，并检验dominance rule
	if (direction == BACKWARD)
	{
		//比较的cur_customer（终点）上的所有路径
		for (int i = 0; i<BackwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = BackwardBucket[cur_customer].Path_index[i];		//BackwardPath_container中的序号
			//第一，BackwardPath_container[curLabel]一定是可以拓展的
			if (0 == BackwardPath_container[curLabel].extend_state)continue;
			//遍历每个可行拓展
			for (int k = 0; k<BackfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//获取新的label的位置
				next_customer = BackfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Backwardindex_num;	//初始化
				if (true == Extend_Onestep(direction, cur_customer, BackwardPath_container[curLabel], next_customer, BackwardPath_container[nextLabel], BB,lp, p, Cuts_src, Cuts_sdc))
				{
					//若拓展成功则需要判断支配条件
					if (false == domination_2cycle_ngpath_cuts(direction, next_customer, lp, BackwardPath_container[nextLabel], Cuts_src))
					{
						//Delelted_Backwardindex_num增量
						incre_deleted = Delelted_Backwardindex_num - incre_deleted;
						//如果不被支配，则加入BackwardBucket[next_customer]中
						//首先在BackwardPath_container中确认添加
						BackwardPath_container[nextLabel].exist_state = 1;
						//然后更新BackwardPath_container和Delelted_Backwardindex中的序号
						Add_container(direction, nextLabel, incre_deleted);
						//最后添加到BackwardBucket[next_customer]中
						BackwardBucket[next_customer].Path_index[BackwardBucket[next_customer].Path_Num] = nextLabel;
						BackwardBucket[next_customer].Path_Num = BackwardBucket[next_customer].Path_Num + 1;
						//说明这点可以拓展
						solved_ornot = false;
					}
				}
			}
			//BackwardPath_container[curLabel]已经拓展完毕，转为永久标签
			//不可再拓展
			BackwardPath_container[curLabel].extend_state = 0;
		}
	}
	else
	{
		//比较的cur_customer（终点）上的所有路径
		for (int i = 0; i<ForwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = ForwardBucket[cur_customer].Path_index[i];		//ForwardPath_container中的序号
			//第一，ForwardPath_container[curLabel]一定是可以拓展的
			if (0 == ForwardPath_container[curLabel].extend_state)continue;
			//遍历每个可行拓展
			for (int k = 0; k<ForfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//获取新的label的位置
				next_customer = ForfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Forwardindex_num;	//初始化
				if (true == Extend_Onestep(direction, cur_customer, ForwardPath_container[curLabel], next_customer, ForwardPath_container[nextLabel], BB,lp, p, Cuts_src, Cuts_sdc))
				{
					//若拓展成功则需要判断支配条件
					if (false == domination_2cycle_ngpath_cuts(direction, next_customer, lp, ForwardPath_container[nextLabel], Cuts_src))
					{
						//Delelted_Forwardindex_num增量
						incre_deleted = Delelted_Forwardindex_num - incre_deleted;
						//如果不被支配，则加入ForwardBucket[next_customer]中
						//首先在ForwardPath_container中确认添加
						ForwardPath_container[nextLabel].exist_state = 1;
						//然后更新ForwardPath_container和Delelted_Forwardindex中的序号
						Add_container(direction, nextLabel, incre_deleted);
						//最后添加到ForwardBucket[next_customer]中
						ForwardBucket[next_customer].Path_index[ForwardBucket[next_customer].Path_Num] = nextLabel;
						ForwardBucket[next_customer].Path_Num = ForwardBucket[next_customer].Path_Num + 1;
						//说明这点可以拓展
						solved_ornot = false;
					}
				}
			}
			//ForwardPath_container[curLabel]已经拓展完毕，转为永久标签
			//不可再拓展
			ForwardPath_container[curLabel].extend_state = 0;
		}
	}
	return solved_ornot;
}

bool Subproblem::Extend_Onestep(int direction, int currentCus, Path & curlabel,  int nextCus, Path & nextlabel, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc)
{
	int next_modifyNode;
	float SDC_modifiedRC = 0;
	float SRC_modifiedRC = 0;

	if (direction == BACKWARD)
	{
		if (BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus]>MAXNUM - 1)return false;
		next_modifyNode = Mapping_fromRN_toMN[nextCus];

		//首先判断是否形成1-cycle？
		if (currentCus == nextCus)return false;

		//启发式拓展
#if EXACTEXTEND ==0
		if (true == restricted_extend && (/*curlabel.RC_value+*/BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus] - lp.Customer_dual[nextCus]) >-MINDOUBLE)return false;
#endif	

		if (false == exact_ornot)
		{
			//检验是否会形成k-cycle
#if TCYCLE >0
			if (true == if_2cycle(curlabel, next_modifyNode))return false;
#endif	
		}
		else
		{
#if Frameworks == 0
			//completion bound
#if KPATHCUT +RCCUT==0
			if (curlabel.RC_value + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus]
				+ BackwardBucket[nextCus].Completion_bounds[int(ceil(p.Max_arrivaltime[current_depot] - curlabel.consumed_time - p.Allnode[currentCus].servicetime - BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus] - p.Allnode[nextCus].servicetime))]
				+ fix_cost > -Conf::Exact_threshold)
#else
			if (curlabel.RC_value + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus] - lp.RobustCut_dual[nextCus][currentCus]
				+ BackwardBucket[nextCus].Completion_bounds[int(ceil(p.Max_arrivaltime[current_depot] - curlabel.consumed_time - p.Allnode[currentCus].servicetime - BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus] - p.Allnode[nextCus].servicetime))]
				+ fix_cost > -Conf::Exact_threshold)
#endif
			{
				return false;
			}
#endif
		}

		//然后看联合ng支配
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return false;
		}
		//检验关键资源是否超过关键点,以及资源满足
		if (false== satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return false;
		//检验是否会形成ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return false;

		//更新SDC资源
#if Frameworks == 1
		//复制有效SDCnode给nextlabel
		nextlabel.Copy_SDCnode(curlabel, Cuts_sdc);
#if SDCTYPE == 0
		//首先看nextCus是否属于Cuts_sdc.SDC_nodes
		if (1 == Cuts_sdc.SDC_indicator[nextCus])
		{
			//再看nextCus是否已经被curlabel经过
			if (false == belong_toset(next_modifyNode, curlabel.passnode))
			{
				//把nextCus插入nextlabel的SDCnode中
				SDC_modifiedRC = Insert_toSDC(nextCus, nextlabel, lp, Cuts_sdc);
			}
		}
#elif SDCTYPE == 1
		//接下来判断nextCus是否需要插入nextlabel.SDCnode中
		if (Cuts_sdc.processed_num>0)
		{
			if (false == Copy_Augngset_SDC(nextCus, nextlabel, curlabel))
			{
				//Augng-infeasible
				//不计入temp_modify，但是SDCnode中仍然保留customer_no
				SDC_modifiedRC = 0;
			}
			else
			{
				//若nextCus都不在Cuts_sdc.SDC_nodes中，则不需要插入（不操作）
				if (1 == Cuts_sdc.SDC_indicator[nextCus])
				{
					SDC_modifiedRC = Insert_toSDC(nextCus, nextlabel, lp);
				}
				else
				{
					SDC_modifiedRC = 0;
	}
}
		}
#endif
#endif

#if SRCUT == 1
		//对当前所有有效的Cuts_src
		for (int i = 0; i < Cuts_src.processed_num; i++)
		{
			//首先继承curlabel上的state_src
			nextlabel.state_src[i] = curlabel.state_src[i];
			//然后更新state_src
			//第一，判断弧[nextCus][currentCus]是否在Cuts_src.SRC_LimitArcSet_indicator[i]中
			if (0== Cuts_src.SRC_LimitArcSet_indicator[i][nextCus][currentCus])
			{
				nextlabel.state_src[i] = 0;
			}
			//第二，判断nextCus是否在Cuts_src.SRC_subset_indicator[i]中
			if (1== Cuts_src.SRC_subset_indicator[i][nextCus])
			{
				nextlabel.state_src[i] = nextlabel.state_src[i] + Cuts_src.add_state(Cuts_src.SRC_subset_len[i]);
				if (nextlabel.state_src[i]>=1)
				{
					SRC_modifiedRC = -lp.SRC_dual[i];
					nextlabel.state_src[i] = nextlabel.state_src[i] - 1;
				}
			}
		}
#endif

		//路径至少可以拓展到nextlabel
		//先初始化
		nextlabel.augment_ornot = false;
		
		//有关资源的更新都在函数satisfying_resource_constrains中

		//更新customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num]= nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//更新与ngpath有关属性
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//更新与kcycle有关的
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];
		if (false==exact_ornot)
		{
#if TCYCLE == 1
			//先把passnode_ngpath复制给passnode_2cycle
			nextlabel.copy_passnode_2cycle(nextlabel.passnode_ngpath);
			//然后把next_modifyNode从passnode_2cycle中去除（不包含倒数第一个节点）
			remove_fromPassnode(next_modifyNode, nextlabel.passnode_2cycle);
			//最后并上倒数第二个节点（prenode）
			nextlabel.Union_2cycle(nextlabel.prenode);
#endif
		}

		//更新与elementary有关的
		nextlabel.copy_passnode(curlabel);
		nextlabel.update_passnode_kcycle(Conf::CYCLE_NUM, next_modifyNode);

		//最后修改成本(reduced cost)
		nextlabel.RC_value = curlabel.RC_value+ Add_RC(direction, currentCus, curlabel, nextCus,BB,lp,p)+ SDC_modifiedRC+ SRC_modifiedRC;

		return true;
	}
	else
	{
		if (BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus]>MAXNUM - 1)return false;
		next_modifyNode = Mapping_fromRN_toMN[nextCus];

		//接着判断是否形成1-cycle？
		if (currentCus == nextCus)return false;
		
		//启发式拓展
#if EXACTEXTEND ==0
		if (true == restricted_extend && (/*curlabel.RC_value +*/ BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus] - lp.Customer_dual[nextCus]) >-MINDOUBLE)return false;
#endif

		if (false == exact_ornot)
		{
			//检验是否会形成k-cycle
#if TCYCLE >0
			if (true == if_2cycle(curlabel, next_modifyNode))return false;
#endif	
		}
		else
		{
#if Frameworks == 0
			//completion bound
#if KPATHCUT +RCCUT==0
			if (curlabel.RC_value + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus]
				+ ForwardBucket[nextCus].Completion_bounds[int(ceil(p.Max_arrivaltime[current_depot] - curlabel.arrivaltime - p.Allnode[currentCus].servicetime - BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus] - p.Allnode[nextCus].servicetime))]
				+ fix_cost > -Conf::Exact_threshold)
#else
			if (curlabel.RC_value + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus] - lp.RobustCut_dual[currentCus][nextCus]
				+ ForwardBucket[nextCus].Completion_bounds[int(ceil(p.Max_arrivaltime[current_depot] - curlabel.arrivaltime - p.Allnode[currentCus].servicetime - BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus] - p.Allnode[nextCus].servicetime))]
				+ fix_cost > -Conf::Exact_threshold)
#endif

			{
				return false;
			}
#endif	
		}

		//然后看联合ng支配
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return false;
		}
		//检验关键资源是否超过关键点,以及资源满足
		if (false == satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return false;
		//检验是否会形成ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return false;
		
		//更新SDC资源
#if Frameworks == 1
		//复制有效SDCnode给nextlabel
		nextlabel.Copy_SDCnode(curlabel, Cuts_sdc);
#if SDCTYPE == 0
		//首先看nextCus是否属于Cuts_sdc.SDC_nodes
		if (1 == Cuts_sdc.SDC_indicator[nextCus])
		{
			//再看nextCus是否已经被curlabel经过
			if (false == belong_toset(next_modifyNode, curlabel.passnode))
			{
				//把nextCus插入nextlabel的SDCnode中
				SDC_modifiedRC = Insert_toSDC(nextCus, nextlabel, lp, Cuts_sdc);
			}
		}
#elif SDCTYPE == 1
		//接下来判断nextCus是否需要插入nextlabel.SDCnode中
		if (Cuts_sdc.processed_num>0)
		{
			if (false == Copy_Augngset_SDC(nextCus, nextlabel, curlabel))
			{
				//Augng-infeasible
				//不计入temp_modify，但是SDCnode中仍然保留customer_no
				SDC_modifiedRC = 0;
			}
			else
			{
				//若nextCus都不在Cuts_sdc.SDC_nodes中，则不需要插入（不操作）
				if (1 == Cuts_sdc.SDC_indicator[nextCus])
				{
					SDC_modifiedRC = Insert_toSDC(nextCus, nextlabel, lp);
				}
				else
				{
					SDC_modifiedRC = 0;
	}
			}
		}
#endif
#endif

#if SRCUT == 1
		//对当前所有有效的Cuts_src
		for (int i = 0; i < Cuts_src.processed_num; i++)
		{
			//首先继承curlabel上的state_src
			nextlabel.state_src[i] = curlabel.state_src[i];
			//然后更新state_src
			//第一，判断弧[currentCus][nextCus]是否在Cuts_src.SRC_LimitArcSet_indicator[i]中
			if (0== Cuts_src.SRC_LimitArcSet_indicator[i][currentCus][nextCus])
			{
				nextlabel.state_src[i] = 0;
			}
			//第二，判断nextCus是否在Cuts_src.SRC_subset_indicator[i]中
			if (1== Cuts_src.SRC_subset_indicator[i][nextCus])
			{
				nextlabel.state_src[i] = nextlabel.state_src[i] + Cuts_src.add_state(Cuts_src.SRC_subset_len[i]);
				if (nextlabel.state_src[i]>=1)
				{
					SRC_modifiedRC = -lp.SRC_dual[i];
					nextlabel.state_src[i] = nextlabel.state_src[i] - 1;
				}
			}
		}
#endif	
		//路径至少可以拓展到nextlabel
		//先初始化
		nextlabel.augment_ornot = false;
	
		//有关资源的更新都在函数satisfying_resource_constrains中

		//更新customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num] = nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//更新与ngpath有关属性
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//更新与kcycle有关的
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];
		if (false==exact_ornot)
		{
#if TCYCLE == 1
			//先把passnode_ngpath复制给passnode_2cycle
			nextlabel.copy_passnode_2cycle(nextlabel.passnode_ngpath);
			//然后把next_modifyNode从passnode_2cycle中去除（不包含倒数第一个节点）
			remove_fromPassnode(next_modifyNode, nextlabel.passnode_2cycle);
			//最后并上倒数第二个节点（prenode）
			nextlabel.Union_2cycle(nextlabel.prenode);
#endif
		}

		//更新与elementary有关的
		nextlabel.copy_passnode(curlabel);
		nextlabel.update_passnode_kcycle(Conf::CYCLE_NUM, next_modifyNode);

		//最后修改成本(reduced cost)
		nextlabel.RC_value = curlabel.RC_value + Add_RC(direction, currentCus, curlabel, nextCus, BB,lp,p) + SDC_modifiedRC+ SRC_modifiedRC;

		return true;
	}
}

bool Subproblem::satisfying_resource_constrains(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, Problem & p)
{
	if (direction == BACKWARD)
	{
		//载重量
		nextlabel.availablecapacity = curlabel.availablecapacity - p.Allnode[nextCus].demand;
		if (nextlabel.availablecapacity<0)return false;
		//duration
		nextlabel.accu_duration = curlabel.accu_duration + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus]+ p.Allnode[nextCus].servicetime;		//时间可能需要转换
#if DURATIONORNOT == 1
		if (nextlabel.accu_duration>p.Veh.Duration)return false;
#endif
		//时间窗
		nextlabel.consumed_time = max((curlabel.consumed_time + p.Allnode[currentCus].servicetime + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus]), //时间可能需要转换
			(p.Max_arrivaltime[current_depot] - p.Allnode[nextCus].endTW - p.Allnode[nextCus].servicetime));
		if (nextlabel.consumed_time>(p.Max_arrivaltime[current_depot] - p.Allnode[nextCus].startTW - p.Allnode[nextCus].servicetime))return false;
		nextlabel.surplus_time = p.Max_arrivaltime[current_depot] - nextlabel.consumed_time;
		//资源定界
		if (nextlabel.surplus_time >= Hpoint_For)
		{
			//还可以拓展
			nextlabel.extend_state = 1;
			return true;
		}
		else
		{
			//这是最后一次拓展
			nextlabel.extend_state = 0;
			return true;
		}
	}
	else
	{
		//载重量
		nextlabel.usedcapacity = curlabel.usedcapacity + p.Allnode[nextCus].demand;
		if (nextlabel.usedcapacity>p.Veh.Veh_Capacity)return false;
		//duration
		nextlabel.accu_duration = curlabel.accu_duration + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus]+p.Allnode[nextCus].servicetime;  //时间可能需要转换
#if DURATIONORNOT == 1
		if (nextlabel.accu_duration>p.Veh.Duration)return false;
#endif
		//时间窗
		nextlabel.arrivaltime = max(curlabel.arrivaltime + p.Allnode[currentCus].servicetime + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus], p.Allnode[nextCus].startTW);  //时间可能需要转换
		if (nextlabel.arrivaltime <= p.Allnode[nextCus].endTW)
		{
			//资源定界
			if (nextlabel.arrivaltime <= Hpoint_For)
			{
				//还可以拓展
				nextlabel.extend_state = 1;
				return true;
			}
			else
			{
				//这是最后一次拓展
				nextlabel.extend_state = 0;
				return true;
			}
		}
		else
		{
			return false;
		}
	}
}

bool Subproblem::if_2cycle(Path & curlabel, int next_modifynode)
{
	if (next_modifynode == curlabel.prenode)
	{
		return true;
	}
	return false;
}

bool Subproblem::if_ng_path(int next_modifynode, Path & curlabel)
{
	int lineid, rowid;
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;

	if (0== (curlabel.passnode_ngpath[rowid] & passnodelist[lineid]))
	{
		return false;
	}
	else
	{
		return true;//形成ng-path
	}
}

float Subproblem::Add_RC(int direction, int currentCus, Path & curlabel, int nextCus, BranchABound & BB, RMP & lp, Problem & p)
{
	float nextRC = 0;

	if (0 == Conf::cost_type)
	{
		if (BACKWARD == direction)
		{
#if KPATHCUT +RCCUT==0
			nextRC = -lp.Customer_dual[nextCus] + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus];
#else
			nextRC = -lp.Customer_dual[nextCus] - lp.RobustCut_dual[nextCus][currentCus] + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus];
#endif
			//if(tempdisDis>0)return (float)MAX_INT;
		}
		else if (FORWARD == direction)
		{
#if KPATHCUT +RCCUT==0
			nextRC = -lp.Customer_dual[nextCus] + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus];
#else
			nextRC = -lp.Customer_dual[nextCus] - lp.RobustCut_dual[currentCus][nextCus] + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus];
#endif
			//if(tempdisDis>0)return (float)MAX_INT;
		}
	}
	else
	{
		cout << "ERROR：未指定cost类型" << endl;
		cin.get();
	}
	return nextRC;
}

bool Subproblem::combinebidirection_kcycle_ngpath_cuts_stable(BranchABound & BB, RMP & lp, Problem & p, SRC & Cuts_src)
{
	int i,j,tempk = 0;
	int combine_route_length = 0;
	int for_Label, back_Label;
	float temp_RC, RC_modify, redundant_ST;
	Shortest_path.RC_value = MAXNUM;
	reduce_cost = MAXNUM;

#if FOUND == 0

#elif FOUND == 1	
	for (int i = 0; i<p.Customer_Num+1; i++)
	{
		FoundPath_container[i].Reducedcost = MAXNUM;
	}
	FoundPath_container_num = p.Customer_Num;
#elif FOUND == 2	

#elif FOUND == 3
	FoundPath_container[0].Reducedcost = MAXNUM;
#endif

	//srand((unsigned)time(0));  //随机数初始化。为了产生随机数。//可以注释

	//排序,如果时间太长就注释
	//使ForwardBucket或者BackwardBucket中的label的RC值升序
#if ORDERLABEL == 1
	for (int i = 0; i<p.Customer_Num; i++)
	{
		if (ForwardBucket[i].Path_Num>0)
		{
			sort(ForwardBucket[i].Path_index, ForwardBucket[i].Path_index+ ForwardBucket[i].Path_Num, compare_for);
		}
		if (BackwardBucket[i].Path_Num>0)
		{
			sort(BackwardBucket[i].Path_index, BackwardBucket[i].Path_index+ BackwardBucket[i].Path_Num, compare_back);
		}
	}
#endif

	//前向遍历每个customer
	for (int for_customer=0; for_customer<p.Customer_Num; for_customer++)
	{
		//遍历for_customer上的所有前向label
		for (i=0;i<ForwardBucket[for_customer].Path_Num;i++)
		{
			for_Label = ForwardBucket[for_customer].Path_index[i]; //前向label的序号

			for (int back_customer = 0; back_customer < p.Customer_Num; back_customer++)
			{
				//前向label与反向label的最后一个点相同时没有必要合并
				//因为总存在另一对合并的label
				if(for_customer== back_customer)continue;
				//遍历back_customer上的所有前向label
				for(j=0;j<BackwardBucket[back_customer].Path_Num;j++)
				{
					back_Label = BackwardBucket[back_customer].Path_index[j]; //反向label的序号

					//EPO框架下同时跟踪两个值：
					//第一个是RC最小的label，不论是否初等
					//第二个是所有RC<0的初等label

					//CPA框架下只跟踪一个值：
					//RC最小的label，不论是否初等

					//计算ForwardPath_container[for_Label]和BackwardPath_container[back_Label]合并后的RC值
					//EPO框架下要去除一个fix_cost
					//CPA框架下要去除一个fix_cost和前后label中SDCnode中重复点对应的SDC_dual
					RC_modify= Calculate_Joinmodify(ForwardPath_container[for_Label], BackwardPath_container[back_Label],lp);
#if KPATHCUT +RCCUT==0
					temp_RC = ForwardPath_container[for_Label].RC_value + BackwardPath_container[back_Label].RC_value
						+ BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer] + RC_modify;
#else
					temp_RC = ForwardPath_container[for_Label].RC_value + BackwardPath_container[back_Label].RC_value
						+ BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer] - lp.RobustCut_dual[for_customer][back_customer] + RC_modify;
#endif


#if SRCUT == 1
					//遍历每一个Cuts_src
					for (int k = 0; k < Cuts_src.processed_num; k++)
					{
						//检验弧[for_customer][back_customer]是否在Cuts_src.SRC_LimitArcSet_indicator[k]中
						if (1== Cuts_src.SRC_LimitArcSet_indicator[k][for_customer][back_customer])
						{
							if (ForwardPath_container[for_Label].state_src[k]+ BackwardPath_container[back_Label].state_src[k]>=1)
							{
								temp_RC = temp_RC - lp.SRC_dual[k];
							}
						}
					}
#endif

					//先判断资源可行性
					if (temp_RC >= -Conf::Exact_threshold || temp_RC>reduce_cost)continue;
					if (ForwardPath_container[for_Label].usedcapacity >BackwardPath_container[back_Label].availablecapacity)continue;	 //检测是否满足需求约束
					if ((ForwardPath_container[for_Label].arrivaltime + p.Allnode[for_customer].servicetime +BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer]
						+ p.Allnode[back_customer].servicetime + BackwardPath_container[back_Label].consumed_time)>p.Max_arrivaltime[current_depot])continue;	//检测是否满足时间窗约束
#if DURATIONORNOT == 1
					if (for_customer== back_customer)
					{
						redundant_ST = p.Allnode[for_customer].servicetime;
					}
					else
					{
						redundant_ST = 0;
					}
					if ((ForwardPath_container[for_Label].accu_duration + BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer]
						+ BackwardPath_container[back_Label].accu_duration - redundant_ST) >p.Veh.Duration)continue;	 //检测是否满足duration约束
#endif
					
					//如果合并label满足所有约束
#if Frameworks == 0
					//先处理FoundPath_container
					if (true == ForwardPath_container[for_Label].elementaty &&		//首先前后向label是否存在环
						true == BackwardPath_container[back_Label].elementaty)
					{
						//合成后的label不能存在环
						if (false == check_route_samenode(ForwardPath_container[for_Label].passnode, BackwardPath_container[back_Label].passnode))
						{
							if (true == Update_FoundPath(temp_RC, for_Label, ForwardPath_container[for_Label], back_Label, BackwardPath_container[back_Label], p))
							{
								//找到足够多的elementary的路径
								return true;
							}
						}
					}
					else
					{
						//DSSR-NG框架下拓展的是前向或者后向有环的label
						//但该有环的label仍不能存在2-cycle或者是ng-infeasible的
						if (ForwardPath_container[for_Label].prenode> 0 && Mapping_Reduced[ForwardPath_container[for_Label].prenode] == back_customer)continue;			//前向路径不能与后向路径末位点形成2-cycle
						if (BackwardPath_container[back_Label].prenode> 0 && Mapping_Reduced[BackwardPath_container[back_Label].prenode] == for_customer)continue;		//前向路径不能与后向路径末位点形成2-cycle
						if (true == check_route_samenode(ForwardPath_container[for_Label].passnode_ngpath, BackwardPath_container[back_Label].passnode_ngpath))continue;				//前向和后向label的ngpath不能具有相同的点
						//再处理Shortest_path
						if (temp_RC < Shortest_path.RC_value)
						{
							Shortest_path.RC_value = temp_RC;
							Shortest_forlabel = for_Label;
							Shortest_backlabel = back_Label;
						}
					}
#else
					//CPA框架下合并路径时，
					//首先，不能形成2-cycle
					if (ForwardPath_container[for_Label].prenode> 0 && Mapping_Reduced[ForwardPath_container[for_Label].prenode] == back_customer)continue;			//前向路径不能与后向路径末位点形成2-cycle
					if (BackwardPath_container[back_Label].prenode> 0 && Mapping_Reduced[BackwardPath_container[back_Label].prenode] == for_customer)continue;		//前向路径不能与后向路径末位点形成2-cycle
					//其次，合并后必须是ng-feasible的
					if (true == check_route_samenode(ForwardPath_container[for_Label].passnode_ngpath, BackwardPath_container[back_Label].passnode_ngpath))continue;				//前向和后向label的ngpath不能具有相同的点

					if (true == Update_FoundPath(temp_RC, for_Label, ForwardPath_container[for_Label], back_Label, BackwardPath_container[back_Label], p))
					{
						return true;
					}
#endif
				}
			}
		}
	}
	//最后判断在FoundPath_container是否找到elementary的路径
#if Frameworks == 0
	if (reduce_cost<-Conf::Exact_threshold)
	{
		//存在一个RC<0的初等路径
		return true;
	}
	else
	{
		//查找非初等路径是否RC<0
		reduce_cost = Shortest_path.RC_value;
		if (Shortest_path.RC_value <-Conf::Exact_threshold)
		{
			return false;
		}
		else
		{
			return true;
		}
		return true;
	}
#else
	return true;
#endif
}

bool Subproblem::check_route_samenode(long * pass1, long * pass2)
{
	long test;
	for (int i = 0; i <passnode_length; i++)
	{
		test = pass1[i] & pass2[i];
		if (test>0)return true;
	}
	return false;
}

bool Subproblem::Update_FoundPath(float tempRC,int forLabel_index, Path forlabel, int backLabel_index, Path backlabel, Problem &p)
{
	int i, j;
	int foundIndex;
	//当elementary的路径的RC值足够大时，应该寻找理论最短路了
	if (tempRC>-Conf::Exact_threshold)
	{
		return false;
	}

#if FOUND == 0
	//获取FoundPath_container序号
	foundIndex = Get_FoundIndex();
	//合并路径
	FoundPath_container[foundIndex].nodesNum = forlabel.customeList_num + backlabel.customeList_num +2; //这里没有算起终点
	FoundPath_container[foundIndex].nodes[0] = p.Customer_Num+current_depot;
	//前向
	for (int i = 0; i < forlabel.customeList_num; i++)
		FoundPath_container[foundIndex].nodes[1+i] = forlabel.customeList[i];
	//反向
	i = forlabel.customeList_num;
	for (j = backlabel.customeList_num - 1; j >= 0; j--)
	{
		FoundPath_container[foundIndex].nodes[1+i] = backlabel.customeList[j];
		i++;
	}
	FoundPath_container[foundIndex].nodes[FoundPath_container[foundIndex].nodesNum -1] = p.Customer_Num+ current_depot;
	//存下RC值
	FoundPath_container[foundIndex].Reducedcost = tempRC;
	//添加进入FoundPath_container
	Add_found(foundIndex);
	if (tempRC<reduce_cost)
	{
		reduce_cost = tempRC;
	}
	if (Conf::MAX_PSP_SIZE == FoundPath_container_num)
	{
		return true;
	}
	else
	{
		return false;
	}

#elif FOUND == 1	
	//记录合并的起点
	foundIndex = forlabel.customeList[forlabel.customeList_num-1];
	//在每个合并的起点记录
	if (tempRC<-Conf::Exact_threshold && tempRC<FoundPath_container[foundIndex].Reducedcost- MINDOUBLE)
	{
		//存下RC值
		FoundPath_container[foundIndex].Reducedcost = tempRC;
		FoundPath_container[foundIndex].pre_label=forLabel_index;
		FoundPath_container[foundIndex].succ_label=backLabel_index;
		if (tempRC<reduce_cost)
		{
			reduce_cost = tempRC;
		}
		//添加进入FoundPath_container
		return false;
	}
	return false;
#elif FOUND == 2	

#elif FOUND == 3	
	if (tempRC<FoundPath_container[0].Reducedcost)
	{
		foundIndex = 0;
		//存下RC值
		FoundPath_container[foundIndex].Reducedcost = tempRC;
		FoundPath_container[foundIndex].pre_label = forLabel_index;
		FoundPath_container[foundIndex].succ_label = backLabel_index;
		//添加进入FoundPath_container
		FoundPath_container_num =1;
		reduce_cost = FoundPath_container[foundIndex].Reducedcost;
		return false;
	}
	return false;
#else
	cout << "Error: FOUND assignment error." << endl;
	cin.get();
	return false;
#endif
}

void Subproblem::Update_ngset(Problem & p)
{
	//前向label有环
	if (false== ForwardPath_container[Shortest_forlabel].elementaty)
	{
		Extend_ngset(FORWARD, p);
	}
	//反向label有环
	if (false == BackwardPath_container[Shortest_backlabel].elementaty)
	{
		Extend_ngset(BACKWARD, p);
	}
}

void Subproblem::Update_compleBound(Problem & p)
{
	int bucket_index,label_index;
	//首先更新每个customer上每个bucket上的bound
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//前向搜索，找反向label
		for (int j = 0; j < BackwardBucket[i].Path_Num; j++)
		{
			label_index = BackwardBucket[i].Path_index[j];
			bucket_index = int(ceil(BackwardPath_container[label_index].consumed_time));
			if (BackwardPath_container[label_index].RC_value<ForwardBucket[i].Completion_bounds[bucket_index] || ForwardBucket[i].Completion_bounds[bucket_index]<MINNUM+1)
			{
				ForwardBucket[i].Completion_bounds[bucket_index] = BackwardPath_container[label_index].RC_value;
			}
		}
		//反向搜索，找前向label
		for (int j = 0; j < ForwardBucket[i].Path_Num; j++)
		{
			label_index = ForwardBucket[i].Path_index[j];
			bucket_index = int(ceil(ForwardPath_container[label_index].arrivaltime)) ;
			if (ForwardPath_container[label_index].RC_value<BackwardBucket[i].Completion_bounds[bucket_index] || BackwardBucket[i].Completion_bounds[bucket_index]<MINNUM + 1)
			{
				BackwardBucket[i].Completion_bounds[bucket_index] = ForwardPath_container[label_index].RC_value;
			}
		}
	}
	//然后再逆序排序每个customer上bucket上的bound
#if COMPLETION == 0
	for (int i = 0; i < p.Customer_Num; i++)
	{
		for (int j = ForwardBucket[i].Bucket_min + 1; j < ForwardBucket[i].Bucket_max; j++)
		{
			if (ForwardBucket[i].Completion_bounds[j - 1]>MINNUM + 1)
			{
				if (ForwardBucket[i].Completion_bounds[j - 1]<ForwardBucket[i].Completion_bounds[j] || ForwardBucket[i].Completion_bounds[j]<MINNUM + 1)
				{
					ForwardBucket[i].Completion_bounds[j] = ForwardBucket[i].Completion_bounds[j - 1];
				}
			}
		}
		for (int j = BackwardBucket[i].Bucket_min + 1; j < BackwardBucket[i].Bucket_max; j++)
		{
			if (BackwardBucket[i].Completion_bounds[j - 1]>MINNUM + 1)
			{
				if (BackwardBucket[i].Completion_bounds[j - 1]<BackwardBucket[i].Completion_bounds[j] || BackwardBucket[i].Completion_bounds[j]<MINNUM + 1)
				{
					BackwardBucket[i].Completion_bounds[j] = BackwardBucket[i].Completion_bounds[j - 1];
				}
			}
		}
	}
#elif COMPLETION == 1	
	float temp_min;
	for (int i = 0; i < p.Customer_Num; i++)
	{
		temp_min = MAXNUM;
		for (int j = ForwardBucket[i].Bucket_min + 1; j < ForwardBucket[i].Bucket_max; j++)
		{
			if (ForwardBucket[i].Completion_bounds[j - 1] > MINNUM + 1)
			{
				if (ForwardBucket[i].Completion_bounds[j - 1]<temp_min)
				{
					temp_min = ForwardBucket[i].Completion_bounds[j - 1];
					if (temp_min<ForwardBucket[i].Completion_bounds[j])
					{
						ForwardBucket[i].Completion_bounds[j] = temp_min;
					}
					else if (ForwardBucket[i].Completion_bounds[j]> MINNUM + 1)
					{
						temp_min = ForwardBucket[i].Completion_bounds[j];
					}
				}
				else
				{
					ForwardBucket[i].Completion_bounds[j-1] = temp_min;
				}
			}

		}
		temp_min = MAXNUM;
		for (int j = BackwardBucket[i].Bucket_min + 1; j < BackwardBucket[i].Bucket_max; j++)
		{
			if (BackwardBucket[i].Completion_bounds[j - 1] > MINNUM + 1)
			{
				if (BackwardBucket[i].Completion_bounds[j - 1]<temp_min)
				{
					temp_min = BackwardBucket[i].Completion_bounds[j - 1];
					if (temp_min<BackwardBucket[i].Completion_bounds[j])
					{
						BackwardBucket[i].Completion_bounds[j] = temp_min;
					}
					else if (BackwardBucket[i].Completion_bounds[j]> MINNUM + 1)
					{
						temp_min = BackwardBucket[i].Completion_bounds[j];
					}
				}
				else
				{
					BackwardBucket[i].Completion_bounds[j - 1] = temp_min;
				}
			}
		}
	}
#endif
}

void Subproblem::Generate_cycle(int direction, int endposi, Problem & p)
{
	int obj_modifyNode,pre_modifyNode;

	if (BACKWARD == direction)
	{
		obj_modifyNode = Mapping_fromRN_toMN[BackwardPath_container[Shortest_backlabel].customeList[endposi]];
		cycle_Modifynode[0] = obj_modifyNode;
		cycle_Modifynode_num = 1;

		for (int i = endposi - 1; i >= 0; i--)
		{
			pre_modifyNode = Mapping_fromRN_toMN[BackwardPath_container[Shortest_backlabel].customeList[i]];
			cycle_Modifynode[cycle_Modifynode_num] = pre_modifyNode;
			cycle_Modifynode_num = cycle_Modifynode_num + 1;
			if (pre_modifyNode == obj_modifyNode)
			{
				break;
			}
		}
	
	}
	else
	{
		obj_modifyNode = Mapping_fromRN_toMN[ForwardPath_container[Shortest_forlabel].customeList[endposi]];
		cycle_Modifynode[0] = obj_modifyNode;
		cycle_Modifynode_num = 1;

		for (int i = endposi - 1; i>=0; i--)
		{
			pre_modifyNode = Mapping_fromRN_toMN[ForwardPath_container[Shortest_forlabel].customeList[i]];
			cycle_Modifynode[cycle_Modifynode_num] = pre_modifyNode;
			cycle_Modifynode_num = cycle_Modifynode_num + 1;
			if (pre_modifyNode == obj_modifyNode)
			{
				break;
			}
		}
	}
}

void Subproblem::Add_ngset_byCycle(Problem & p)
{
	int add_modifyNode = cycle_Modifynode[0];
	int check_modifyNode;

	for (int i=1;i<cycle_Modifynode_num-1;i++)
	{
		check_modifyNode = cycle_Modifynode[i];
		//首先判断check_modifyNode的ngset中是否已经存在add_modifyNode
		//如果add_modifyNode已经在check_modifyNode的ngset中,就不用额外操作了
		if (false==belong_toset(add_modifyNode, neighbourhood_passnode[check_modifyNode]))
		{
			//否则就把向check_modifyNode的ngset的最后位置加入add_modifyNode
			//neighbourhood
			neighbourhood[check_modifyNode][neighbourhood_num[check_modifyNode]] = add_modifyNode;
			neighbourhood_num[check_modifyNode] = neighbourhood_num[check_modifyNode] +1;
			//neighbourhood_passnode
			Insert_toPassnode(add_modifyNode, neighbourhood_passnode[check_modifyNode]);
			//ng_memory_passnode
			Insert_toPassnode(check_modifyNode, ng_memory_passnode[add_modifyNode]);

#if DSSRNG==1	
			//存储dssr-ngset的更新
			int add_customer = Mapping_Reduced[add_modifyNode];
			int check_customer= Mapping_Reduced[check_modifyNode];
			if (false== belong_toset(add_customer, p.Allnode[check_customer].dssr_negSet_passnode))
			{
				p.Allnode[check_customer].dssr_ngSet[p.Allnode[check_customer].dssr_ngSet_num] = add_customer;
				p.Allnode[check_customer].dssr_ngSet_num = p.Allnode[check_customer].dssr_ngSet_num + 1;
				Insert_toPassnode(add_customer, p.Allnode[check_customer].dssr_negSet_passnode);
				Insert_toPassnode(check_customer, p.Allnode[add_customer].dssr_ngMem_passnode);
			}
#endif
		}
	}
}

void Subproblem::Extend_ngset(int direction, Problem & p)
{
	int next_modifyNode;
	//先重置Shortest_path.passnode
	for (int i = 0; i < passnode_length; i++)
	{
		Shortest_path.passnode[i] = 0;
	}

	if (BACKWARD == direction)
	{
		//从label的开始位置，顺序判断是否存在环
		//寻找BackwardPath_container[Shortest_backlabel]中的所有环
		for (int i = 0; i<BackwardPath_container[Shortest_backlabel].customeList_num; i++)
		{
			//寻找一个modifyNode(可能由BackwardPath_container[Shortest_backlabel]中若干个node代表)
			next_modifyNode = Mapping_fromRN_toMN[BackwardPath_container[Shortest_backlabel].customeList[i]];
			//更新Shortest_path的passnode
			if (false == Shortest_path.update_elementary_passnode(next_modifyNode))
			{
				//形成环，则该cycle的重复点为next_modifyNode
				//从位置i反向探索直到找到第一个相同next_modifyNode的node，形成cycle
				Generate_cycle(direction, i, p);
				//对每个cycle中的节点都更新ngset
				Add_ngset_byCycle(p);
			}
		}
	}
	else
	{
		//从label的开始位置，顺序判断是否存在环
		//寻找ForwardPath_container[Shortest_forlabel]中的所有环
		for (int i = 0; i<ForwardPath_container[Shortest_forlabel].customeList_num; i++)
		{
			//寻找一个modifyNode(可能由ForwardPath_container[Shortest_forlabel]中若干个node代表)
			next_modifyNode = Mapping_fromRN_toMN[ForwardPath_container[Shortest_forlabel].customeList[i]];
			//更新Shortest_path的passnode
			if (false == Shortest_path.update_elementary_passnode(next_modifyNode))
			{
				//形成环，则该cycle的重复点为next_modifyNode
				//从位置i反向探索直到找到第一个相同next_modifyNode的node，形成cycle
				Generate_cycle(direction,i, p);
				//对每个cycle中的节点都更新ngset
				Add_ngset_byCycle(p);
			}
		}
	}
}

void Subproblem::Label_update(Problem & p)
{
	int label_index;
	//DSSR-ng框架下，label需要更新2类信息：
	//是否被删除：exist_state；是否能拓展：extend_state

	//elementary为true的label留下
	//如果满足Hpoints则extend_state=1，否则extend_state=0
	//无论extend_state为0或者1，只要留下的label的ngset都不变，feasible_extensions_byngpath和augment_ornot不变
	//前向
	for (int k = 0; k < p.Customer_Num; k++)
	{
		for (int i = 0; i < ForwardBucket[k].Path_Num; i++)
		{
			label_index = ForwardBucket[k].Path_index[i];
			if (0 == ForwardPath_container[label_index].exist_state) continue;
			if (true == ForwardPath_container[label_index].elementaty)
			{
				//留下
				if (ForwardPath_container[label_index].arrivaltime <= Hpoint_For)
				{
					ForwardPath_container[label_index].extend_state = 1;
					//更新ngpath，ngpath_num和passnode_ngpath
					generate_ngpath_Byroute(ForwardPath_container[label_index]);
					//更新feasible_extensions_byngpath和augment_ornot
					for (int j = 0; j < passnode_length; j++)
					{
						ForwardPath_container[label_index].feasible_extensions_byngpath[j] = 0;
					}
					ForwardPath_container[label_index].augment_ornot = false;
				}
			}
			else
			{
#if NEWNG == 0
				//不满足的需要删除
				Delete_pathindex(FORWARD, k, i);
				Delete_container(FORWARD, label_index);
				ForwardPath_container[label_index].exist_state = 0;
				i = i - 1;
#elif NEWNG == 1	
				//剩下那些非elementary
				//检验ngpath是否满足，若满足则留下：
				if (true == Check_ngpath_Byroute(ForwardPath_container[label_index]))
				{
					//根据Hpoints判断extend_state
					if (ForwardPath_container[label_index].arrivaltime <= Hpoint_For)
					{
						ForwardPath_container[label_index].extend_state = 1;
						//更新feasible_extensions_byngpath和augment_ornot
						for (int j = 0; j < passnode_length; j++)
						{
							ForwardPath_container[label_index].feasible_extensions_byngpath[j] = 0;
						}
						ForwardPath_container[label_index].augment_ornot = false;
					}
				}
				else
				{
					//不满足的需要删除
					Delete_pathindex(FORWARD, k, i);
					Delete_container(FORWARD, label_index);
					ForwardPath_container[label_index].exist_state = 0;
					i = i - 1;
				}
#endif

			}
		}
	}
	//反向
	for (int k = 0; k < p.Customer_Num; k++)
	{
		for (int i = 0; i < BackwardBucket[k].Path_Num; i++)
		{
			label_index = BackwardBucket[k].Path_index[i];
			if (0 == BackwardPath_container[label_index].exist_state) continue;
			if (true == BackwardPath_container[label_index].elementaty)
			{
				if (BackwardPath_container[label_index].surplus_time >= Hpoint_For)
				{
					BackwardPath_container[label_index].extend_state = 1;
					//更新ngpath，ngpath_num和passnode_ngpath
					generate_ngpath_Byroute(BackwardPath_container[label_index]);
					//更新feasible_extensions_byngpath和augment_ornot
					for (int j = 0; j < passnode_length; j++)
					{
						BackwardPath_container[label_index].feasible_extensions_byngpath[j] = 0;
					}
					BackwardPath_container[label_index].augment_ornot = false;
				}
			}
			else
			{
#if NEWNG == 0
				Delete_pathindex(BACKWARD, k, i);
				Delete_container(BACKWARD, label_index);
				BackwardPath_container[label_index].exist_state = 0;
				i = i - 1;
#elif NEWNG == 1	
				//剩下那些非elementary
				//检验ngpath是否满足，若满足则留下：
				if (true == Check_ngpath_Byroute(BackwardPath_container[label_index]))
				{
					//根据Hpoints判断extend_state
					if (BackwardPath_container[label_index].surplus_time >= Hpoint_For)
					{
						BackwardPath_container[label_index].extend_state = 1;
						//更新feasible_extensions_byngpath和augment_ornot
						for (int j = 0; j < passnode_length; j++)
						{
							BackwardPath_container[label_index].feasible_extensions_byngpath[j] = 0;
						}
						BackwardPath_container[label_index].augment_ornot = false;
					}
				}
				else
				{
					//不满足的需要删除
					Delete_pathindex(BACKWARD, k, i);
					Delete_container(BACKWARD, label_index);
					BackwardPath_container[label_index].exist_state = 0;
					i = i - 1;
				}
#endif
			}
		}
	}
}

void Subproblem::generate_ngpath_Byroute(Path & objlabel)
{
	int next_modifynode;
	//passnode_ngpath
	//重置passnode_ngpath
	for (int i = 0; i < passnode_length; i++)
	{
		objlabel.passnode_ngpath[i] = 0;
	}
	//第一个customer
	Insert_toPassnode(Mapping_fromRN_toMN[objlabel.customeList[0]], objlabel.passnode_ngpath);
	//之后的customer
	for (int i = 1; i < objlabel.customeList_num; i++)
	{
		next_modifynode = Mapping_fromRN_toMN[objlabel.customeList[i]];
		for (int j = 0; j < passnode_length; j++)
		{
			objlabel.passnode_ngpath[j] = objlabel.passnode_ngpath[j] & neighbourhood_passnode[next_modifynode][j];
		}
		//最后并上下一个点
		Insert_toPassnode(next_modifynode, objlabel.passnode_ngpath);
	}
	//ngpath
	//重置ngpath
	objlabel.ngpath_num = 0;
	for (int i = 0; i < objlabel.customeList_num; i++)
	{
		if (true == belong_toset(Mapping_fromRN_toMN[objlabel.customeList[i]], objlabel.passnode_ngpath))
		{
			objlabel.ngpath[objlabel.ngpath_num] = Mapping_fromRN_toMN[objlabel.customeList[i]];
			objlabel.ngpath_num = objlabel.ngpath_num + 1;
		}
	}
}

bool Subproblem::Check_ngpath_Byroute(Path & objlabel)
{
	int next_modifynode;
	//passnode_ngpath
	//重置passnode_ngpath
	for (int i = 0; i < passnode_length; i++)
	{
		objlabel.passnode_ngpath[i] = 0;
	}
	//第一个customer
	Insert_toPassnode(Mapping_fromRN_toMN[objlabel.customeList[0]], objlabel.passnode_ngpath);
	//之后的customer
	for (int i = 1; i < objlabel.customeList_num; i++)
	{
		next_modifynode = Mapping_fromRN_toMN[objlabel.customeList[i]];
		if (true == belong_toset(next_modifynode, objlabel.passnode_ngpath))
		{
			return false;
		}
		for (int j = 0; j < passnode_length; j++)
		{
			objlabel.passnode_ngpath[j] = objlabel.passnode_ngpath[j] & neighbourhood_passnode[next_modifynode][j];
		}
		//最后并上下一个点
		Insert_toPassnode(next_modifynode, objlabel.passnode_ngpath);
	}
	//ngpath
	//重置ngpath
	objlabel.ngpath_num = 0;
	for (int i = 0; i < objlabel.customeList_num; i++)
	{
		if (true == belong_toset(Mapping_fromRN_toMN[objlabel.customeList[i]], objlabel.passnode_ngpath))
		{
			objlabel.ngpath[objlabel.ngpath_num] = Mapping_fromRN_toMN[objlabel.customeList[i]];
			objlabel.ngpath_num = objlabel.ngpath_num + 1;
		}
	}
	return true;
}

void Subproblem::Construct_FoundPath(Problem & p)
{
	int forLabel_index, backLabel_index;
	int k;

#if FOUND == 1
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//能够形成初等路径的节点
		if (FoundPath_container[i].Reducedcost+ MINDOUBLE<MAXNUM)
		{
			forLabel_index = FoundPath_container[i].pre_label;
			backLabel_index = FoundPath_container[i].succ_label;
			//合并路径
			FoundPath_container[i].nodesNum = ForwardPath_container[forLabel_index].customeList_num + BackwardPath_container[backLabel_index].customeList_num + 2; //这里没有算起终点
			FoundPath_container[i].nodes[0] = p.Customer_Num+current_depot;
			//前向
			for (int j = 0; j < ForwardPath_container[forLabel_index].customeList_num; j++)
				FoundPath_container[i].nodes[1 + j] = ForwardPath_container[forLabel_index].customeList[j];
			//反向
			k = ForwardPath_container[forLabel_index].customeList_num;
			for (int j = BackwardPath_container[backLabel_index].customeList_num - 1; j >= 0; j--)
			{
				FoundPath_container[i].nodes[1 + k] = BackwardPath_container[backLabel_index].customeList[j];
				k++;
			}
			FoundPath_container[i].nodes[FoundPath_container[i].nodesNum - 1] = p.Customer_Num +current_depot;
		}
	}
#elif FOUND == 3
	forLabel_index = FoundPath_container[0].pre_label;
	backLabel_index = FoundPath_container[0].succ_label;
	//合并路径
	FoundPath_container[0].nodesNum = ForwardPath_container[forLabel_index].customeList_num + BackwardPath_container[backLabel_index].customeList_num + 2; //这里没有算起终点
	FoundPath_container[0].nodes[0] = p.Customer_Num + current_depot;;
	//前向
	for (int i = 0; i < forlabel.customeList_num; i++)
		FoundPath_container[0].nodes[1 + i] = ForwardPath_container[forLabel_index].customeList[i];
	//反向
	k = ForwardPath_container[forLabel_index].customeList_num;
	for (int j = BackwardPath_container[backLabel_index].customeList_num - 1; j >= 0; j--)
	{
		FoundPath_container[0].nodes[1 + k] = BackwardPath_container[backLabel_index].customeList[j];
		k++;
	}
	FoundPath_container[0].nodes[FoundPath_container[0].nodesNum - 1] = p.Customer_Num + current_depot;
#endif

}

void Subproblem::Delete_duplicate(Problem & p)
{
	//检查标准一：RC值不同
	for (int i=0;i<p.Customer_Num;i++)
	{
		if (FoundPath_container[i].Reducedcost>-Conf::Exact_threshold)continue;

		for (int j = i+1; j<p.Customer_Num; j++)
		{
			if (fabs(FoundPath_container[i].Reducedcost - FoundPath_container[j].Reducedcost)<-0.01*reduce_cost)
			{
				//重复的路径
				if (FoundPath_container[i].Reducedcost>=FoundPath_container[j].Reducedcost)
				{
					FoundPath_container[i].Reducedcost = MAXNUM;
					break;
				}
				else
				{
					FoundPath_container[j].Reducedcost = MAXNUM;
				}

			}
		}
	}
}

void Subproblem::Set_dssr_ngset(Problem & p)
{
	//首先重置子问题内部的neighbourhood
	ng_clear();
	int i,j,cur_customer;

	//找到dssr_ngSet中出现在Mapping_Reduced中的ngset
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		cur_customer = Mapping_Reduced[i];
		for (j=0;j<p.Allnode[cur_customer].dssr_ngSet_num;j++)
		{
			if (Mapping_fromRN_toMN[p.Allnode[cur_customer].dssr_ngSet[j]]>=0)
			{
				neighbourhood[i][neighbourhood_num[i]] = Mapping_fromRN_toMN[p.Allnode[cur_customer].dssr_ngSet[j]];
				neighbourhood_num[i] = neighbourhood_num[i] + 1;
			}
		}
	}
	//转为二进制，存在neighbourhood_passnode中
	int lineid, rowid, nodeno;
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			nodeno = neighbourhood[i][j];
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			neighbourhood_passnode[i][rowid] += passnodelist[lineid];
		}

	}
	//转换neighbourhood得到ng_memory_passnode
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			nodeno = i;
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			ng_memory_passnode[neighbourhood[i][j]][rowid] += passnodelist[lineid];
		}
	}
}

bool Subproblem::Copy_Augngset_SDC(int next_cus, Path & nextlabel, Path & curlabel)
{
	int temp_modify = Mapping_fromRN_toMN[next_cus];

	if (true == belong_toset(temp_modify, curlabel.passnode_SDCnode))
	{
		for (int i = 0; i < passnode_length; i++)
		{
			nextlabel.passnode_SDCnode[i] = curlabel.passnode_SDCnode[i];
		}
		for (int i = 0; i < curlabel.SDCnode_num; i++)
		{
			nextlabel.SDCnode[i] = curlabel.SDCnode[i];
		}
		nextlabel.SDCnode_num = curlabel.SDCnode_num;
		return false;
	}
	else
	{
		//passnode_ngpath
		for (int i = 0; i < passnode_length; i++)
		{
			nextlabel.passnode_SDCnode[i] = curlabel.passnode_SDCnode[i] & Aug_neighbourhood_passnode[temp_modify][i];
		}
		//SDCnode
		nextlabel.SDCnode_num = 0;
		for (int i = 0; i < curlabel.SDCnode_num; i++)
		{
			if (true == belong_toset(Mapping_fromRN_toMN[curlabel.SDCnode[i]], Aug_neighbourhood_passnode[temp_modify]))
			{
				nextlabel.SDCnode[nextlabel.SDCnode_num] = curlabel.SDCnode[i];
				nextlabel.SDCnode_num = nextlabel.SDCnode_num + 1;
			}
		}
		return true;
	}
}

bool Subproblem::In_SDCcycle(int customer_no, Path & check_label)
{
	bool check = false;

	for (int i = 0; i < check_label.SDCnode_num; i++)
	{
		if (customer_no == check_label.SDCnode[i])
		{
			check = true;
			break;
		}
	}

	return check;
}

float Subproblem::Insert_toSDC(int customer_no, Path &objlabel, RMP & lp)
{
	float temp_modify = 0;
	int inposi = objlabel.SDCnode_num;
	//按照从小到大找到插入位置
	for (int i = 0; i<objlabel.SDCnode_num; i++)
	{
		if (customer_no<objlabel.SDCnode[i])
		{
			inposi = i;
			break;
		}
	}
	if (inposi == Conf::MAX_NODES - 1)
	{
		cout << "ERROR: SDCnode超过界限" << endl;
		cin.get();
	}
	//插入前删除末尾节点
	objlabel.SDCnode.pop_back();
	//插入在SDCnode的第inposi个位置
	objlabel.SDCnode.insert(objlabel.SDCnode.begin() + inposi, customer_no);
	objlabel.SDCnode_num = objlabel.SDCnode_num + 1;

	temp_modify = - lp.SDC_dual[customer_no];
	return  temp_modify;
}

void Subproblem::Insert_intoVector(int insert_posi, int customer_no, Path & objlabel)
{
	//倒序赋值
	for (int i = objlabel.SDCnode_num; i>insert_posi; i--)
	{
		objlabel.SDCnode[i] = objlabel.SDCnode[i - 1];
	}
	//插入点
	objlabel.SDCnode[insert_posi] = customer_no;
}

float Subproblem::Calculate_RCincrement(Path & better_label, Path & dominated_label, RMP & lp)
{
	float temp_RCincrement = 0;

#if Frameworks == 1
	//找到better_label.SDCnode与dominated_label.SDCnode的差集
	if (0==better_label.SDCnode_num)
	{
		temp_RCincrement = 0;
	}
	else if(0==dominated_label.SDCnode_num)
	{
		for (int i = 0; i < better_label.SDCnode_num; i++)
		{
			temp_RCincrement = temp_RCincrement + lp.SDC_dual[better_label.SDCnode[i]];
		}
	}
	else
	{
		elimi_nodes.resize(0);
		std::set_difference(better_label.SDCnode.begin(), better_label.SDCnode.begin()+ better_label.SDCnode_num,
			dominated_label.SDCnode.begin(), dominated_label.SDCnode.begin()+ dominated_label.SDCnode_num,
			std::inserter(elimi_nodes, elimi_nodes.begin()));
		for (int i = 0; i < elimi_nodes.size(); i++)
		{
			temp_RCincrement = temp_RCincrement + lp.SDC_dual[elimi_nodes[i]];
		}
	}
#endif

	return temp_RCincrement;
}

float Subproblem::Calculate_Joinmodify(Path & forlabel, Path & backlabel, RMP & lp)
{
	float temp_RCmodify=0;

#if Frameworks == 1
	//找到forlabel.SDCnode和backlabel.SDCnode的重复点
	if (0==forlabel.SDCnode_num)
	{
		temp_RCmodify = 0;
	}
	else if (0 == backlabel.SDCnode_num)
	{
		temp_RCmodify = 0;
	}
	else
	{
		elimi_nodes.resize(0);
		std::set_intersection(forlabel.SDCnode.begin(), forlabel.SDCnode.begin()+ forlabel.SDCnode_num,
			backlabel.SDCnode.begin(), backlabel.SDCnode.begin()+ backlabel.SDCnode_num,
			std::inserter(elimi_nodes, elimi_nodes.begin()));
		for (int i = 0; i < elimi_nodes.size(); i++)
		{
			temp_RCmodify = temp_RCmodify + lp.SDC_dual[elimi_nodes[i]];
		}
	}
#endif

	temp_RCmodify = temp_RCmodify + fix_cost;
	return temp_RCmodify;
}

float Subproblem::Calculate_SRCdual_Gap(Path & better_label, Path & dominated_label, RMP & lp,SRC & Cuts_src)
{
	float diff = 0;

	for (int i = 0; i < Cuts_src.processed_num; i++)
	{
		//在第i个Cuts_src对应的state中，better_label的state_src要严格大于dominated_label的state_src，则累积一次对偶变量
		if (better_label.state_src[i]>dominated_label.state_src[i]+ MINDOUBLE)
		{
			diff = diff + lp.SRC_dual[i];
		}
	}

	return diff;
}

bool Subproblem::DP_Ng_Cuts_Dssr_Stable(BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc)
{
	//////////////////////////////////////////////////////////////////初始化开始////////////////////////////////////////////////////////////////////////////////////////////////////
	//cout << "读入数据";
	long start = clock();
	//预处理技术：
	//性质1：如果节点关联的reduced cost为正，那么该节点一定不会出现在reduced cost最小的路径（最优解）上。
	//但是，如果考虑到RMP的兼容性，reduced cost为正的节点应该保留，而最短路问题变为满足兼容性（要求经过指定的节点）条件下，reduced cost最小。（这个方法需要预留冗余空间）
	//性质2：假设存在一条弧，满足：该弧的权重+弧的尾节点的reduced cost为正时。那么该弧仍有可能在最短路上。
	//然而，这条弧有可能在某个state(一条路径)中被支配；
	//或者，如果与该弧的尾点相关的所有三边系统都满足：经过该节点的路径会被支配，那么网络中该弧的尾点可以被消减。（这个方法满足条件严格，故先预留冗余空间）
	//再或者，如果该弧构成的所有三边系统都满足：走该弧的路径会被支配，那么网络中该弧可以被消减。（这个方法满足条件较前一个较松，故先预留冗余空间）

	//对当前基求逆
	//Get_inverseBasis(temp_op, routeset);
	Initial_Hpoints(p);			//给分界点上下界赋值
	//预处理：生成当前PSP下的ngset
#if DSSRNG+AUGMENT >0	//在EPO-dssr或者CPA-ng框架下都需要对ngset进行拓展
	Set_dssr_ngset(p);
#else
	Ini_ngset(BB, lp, p);
#endif
		
	//////////////////////////////////////////////////////////////////DSSR框架////////////////////////////////////////////////////////////////////////////////////////////////////
	int iter_count = 0;
	do
	{
		//生成初始列，生成过程中调用支配
		Initial_lables(BB,lp, p, Cuts_src, Cuts_sdc);  //生成所有关联reduced cost
		//labeling过程，主要策略为Search strategy和Dominance rule
		DP_Labeling(FORWARD, BB,lp, p, Cuts_src, Cuts_sdc);	//正向拓展
		DP_Labeling(BACKWARD, BB,lp, p, Cuts_src, Cuts_sdc);	//反向拓展
		////////////////////////////////////////////////////////合并解，合并正反方向解/////////////////////////////////////////////////////////////////////////////////////////////
		//在DSSR-NG框架下的合并对ngset的拓展目的只是为了使前向或者后向的label不出现环(elementary)
		//DSSR-NG框架下ngset拓展条件为：前向或者后向label有环，且合并后RC值小于0
		//DSSR-NG的终止条件为：前向和后向label无环，且合并后为elementary，且最小的RC值大于等于0
		if (true == combinebidirection_kcycle_ngpath_cuts_stable(BB, lp, p, Cuts_src))
		{
			//要么找到了RC<-Conf::Exact_threshold的elementary路径
			//要么前后向无环的label能够形成的elementary路径的RC都>-Conf::Exact_threshold
			break;
		}
		else
		{
			//先更新Completion Bound
			//因为只要求下界，因此不用在乎ngset更新后label是否可行
			if (true==exact_ornot)
			{
				Update_compleBound(p);
			}
			//根据非elementary的路径，对ngset进行拓展
			//先找到Shortest_path中的环，更新ngset
			Update_ngset(p);
			//其次，检索ForwardPath_container和BackwardPath_container中非elementary的label，
			//第一，删除更新ngset后不可行的label
			//第二，留下的label可以再拓展：即设置extend_state=1
			Label_update(p);
			//Label_clear(p);
		}
		iter_count++;
	} while (1);
	////////////////////////////////////////////////////////////////////结束//////////////////////////////////////////////////////////////////////////////////////////////////////
	//首先构建FoundPath_container
	Construct_FoundPath(p);
#if FOUND==1
	//再对FoundPath_container进行处理:删除重复的
	Delete_duplicate(p);
#endif

	long end = clock();
	//PSP指标
	PSP_time = float(end - start) / 1000;
	all_label_num = ForwardPath_container_num + BackwardPath_container_num;
	cout << "子问题花费：" << PSP_time << "秒    ||     ";
	cout <<"RC值：" << reduce_cost << endl;
	//
	if (reduce_cost<-Conf::Exact_threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Subproblem::TSPTW_exist(int start_depot,int *subset, int subset_num, BranchABound & BB, RMP & lp, Problem & p)
{
	bool check = false;
	//先构建环境
	temp_passnodevalue = 0;
	for (int i = 0; i < subset_num; i++)
	{
		temp_passnodevalue += passnodelist[i];
	}
	temp_Max_arrivaltime = p.Max_arrivaltime[current_depot];
	for (int i = 0; i < p.Customer_Num; i++)
	{
		temp_dual[i] = -999;
	}
	for (int i = 0; i < subset_num; i++)
	{
		temp_dual[subset[i]] = 999;
	}
	current_depot = start_depot;
	arc_remain_proportion = 1;
	fix_cost = 0;
	//初始化一大堆参数
	all_label_num = 0;
	init_label_num = 0;
	Modifiednetwork_Nodes_num = 0;
	reduce_cost = MAXNUM;
	for (int i = 0; i < p.Customer_Num + 1; i++)
	{
		ForfeasiExten_Index_num[i] = 0;
		BackfeasiExten_Index_num[i] = 0;
	}
	for (int i = 0; i < p.Customer_Num; i++)
	{
		Mapping_fromRN_toMN[i] = -1;
	}
	Label_clear(p);
	//重构映射关系和FeasiExten_Index
	Build_Mapping_simple(BB, lp, p);
	Feasible_Extension_simple(BB, lp, p);
	Reset_Maxtime(BB, lp, p);
	//开始labeling
	Initial_Hpoints(p);			//给分界点上下界赋值
	//预处理：生成当前PSP下的ngset
	Ini_ngset_simple(BB, lp, p);
	//生成初始列，生成过程中调用支配
	Initial_lables_simple(BB, lp, p);  //生成所有关联reduced cost
	//labeling过程，主要策略为Search strategy和Dominance rule
	if (false == DP_Labeling_simple(FORWARD, BB, lp, p))		//正向拓展
	{
		p.Max_arrivaltime[current_depot] = temp_Max_arrivaltime;
		return true;
	}
	if (false == DP_Labeling_simple(BACKWARD, BB, lp, p))		//反向拓展
	{
		p.Max_arrivaltime[current_depot] = temp_Max_arrivaltime;
		return true;
	}
	////////////////////////////////////////////////////////合并解，合并正反方向解/////////////////////////////////////////////////////////////////////////////////////////////
	//终止条件为：只要出现一个包含subset所有点的路径，就返回true
	check=combinebidirection_kcycle_ngpath_cuts_stable_simple(BB, lp, p);

	p.Max_arrivaltime[current_depot] = temp_Max_arrivaltime;
	return check;
}

void Subproblem::Build_Mapping_simple(BranchABound & BB, RMP & lp, Problem & p)
{
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (temp_dual[i] > 10 * MINDOUBLE)
		{
			Mapping_Reduced[Modifiednetwork_Nodes_num] = i;
			Mapping_fromRN_toMN[i] = Modifiednetwork_Nodes_num;
			Modifiednetwork_Nodes_num = Modifiednetwork_Nodes_num + 1;
		}
	}
	passnode_length = int((float)(Modifiednetwork_Nodes_num - MINDOUBLE) / (float)Conf::EACH_LONG_NUM) + 1;
}

void Subproblem::Feasible_Extension_simple(BranchABound & BB, RMP & lp, Problem & p)
{
	int curnode = BB.cur_node;

	//更新FeasiExten_Index
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (temp_dual[i] > 10 * MINDOUBLE)
		{
			for (int j = 0; j < p.Customer_Num; j++)
			{
				if (temp_dual[j] > 10 * MINDOUBLE)
				{
					if (BB.branch[curnode].CostNetwork_Branch[i][j] <MAXNUM - 1 && 1 == p.Allnode[i].for_feasible_extensions[j])
					{
						ForfeasiExten_Index[i][ForfeasiExten_Index_num[i]] = j;
						ForfeasiExten_Index_num[i] = ForfeasiExten_Index_num[i] + 1;
					}
					if (BB.branch[curnode].CostNetwork_Branch[j][i] <MAXNUM - 1 && 1 == p.Allnode[i].back_feasible_extensions[j])
					{
						BackfeasiExten_Index[i][BackfeasiExten_Index_num[i]] = j;
						BackfeasiExten_Index_num[i] = BackfeasiExten_Index_num[i] + 1;
					}
				}
			}
		}
	}
	//更新从场站出发能够到达的customer
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (temp_dual[i] > 10 * MINDOUBLE)
		{
			if (BB.branch[curnode].CostNetwork_Branch[p.Customer_Num + current_depot][i] < MAXNUM - 1)
			{
				ForfeasiExten_Index[p.Customer_Num][ForfeasiExten_Index_num[p.Customer_Num]] = i;
				ForfeasiExten_Index_num[p.Customer_Num] = ForfeasiExten_Index_num[p.Customer_Num] + 1;
			}
			if (BB.branch[curnode].CostNetwork_Branch[i][p.Customer_Num + current_depot] <MAXNUM - 1)
			{
				BackfeasiExten_Index[p.Customer_Num][BackfeasiExten_Index_num[p.Customer_Num]] = i;
				BackfeasiExten_Index_num[p.Customer_Num] = BackfeasiExten_Index_num[p.Customer_Num] + 1;
			}
		}
	}
}

void Subproblem::Ini_ngset_simple(BranchABound & BB, RMP & lp, Problem & p)
{
	int temp_posi, i, j, k;
	int ordered_num;	//neighbourhood中已经排序好的数量
	bool check;
	//根据对偶变量，找到i节点上离得最近的NEIGHBOURHOOD_NUM个节点,其中i节点一定在neighbourhood中(升序)
	ng_clear();

	//确定ng的大小
	int temp_ngnum=fmin(5, Modifiednetwork_Nodes_num);
	//生成每个customer的ngset，pr06一次运行0.001s
	//按照RC值升序排序
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		neighbourhood[i][0] = 0;
		ordered_num = 1;
		for (j = 1; j <Modifiednetwork_Nodes_num; j++)
		{
			for (k = ordered_num; k >= 1; k--)
			{
				if (
					(BB.branch[BB.cur_node].CostNetwork_Branch[Mapping_Reduced[i]][Mapping_Reduced[j]] - temp_dual[Mapping_Reduced[j]])
					<(BB.branch[BB.cur_node].CostNetwork_Branch[Mapping_Reduced[i]][Mapping_Reduced[neighbourhood[i][k - 1]]] - temp_dual[Mapping_Reduced[neighbourhood[i][k - 1]]])
					)
				{
					continue;
				}
				else
				{
					break;
				}
			}
			//
			temp_posi = k;
			if (temp_posi<temp_ngnum)
			{
				if (ordered_num<temp_ngnum)
				{
					ordered_num = ordered_num + 1;
				}
				//前面的依次向后串
				for (k = ordered_num - 1; k > temp_posi; k--)
				{
					neighbourhood[i][k] = neighbourhood[i][k - 1];
				}
				//再赋值
				neighbourhood[i][temp_posi] = j;
			}
		}
		neighbourhood_num[i] = ordered_num;
	}

	//检查i节点是否在i节点的邻域中，如果不在则加入末尾位置
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		check = true;
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			if (i == neighbourhood[i][j])
			{
				check = false;
				break;
			}
		}
		if (true == check)//i节点不在i节点的邻域中
		{
			neighbourhood[i][neighbourhood_num[i] - 1] = i;
		}
	}
	//转为二进制，存在neighbourhood_passnode中
	int lineid, rowid, nodeno;
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			nodeno = neighbourhood[i][j];
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			neighbourhood_passnode[i][rowid] += passnodelist[lineid];
		}

	}
	//转换neighbourhood得到ng_memory_passnode
	for (i = 0; i < Modifiednetwork_Nodes_num; i++)
	{
		for (j = 0; j < neighbourhood_num[i]; j++)
		{
			nodeno = i;
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			ng_memory_passnode[neighbourhood[i][j]][rowid] += passnodelist[lineid];
		}
	}
}

void Subproblem::Initial_lables_simple(BranchABound & BB, RMP & lp, Problem & p)
{
	int i, j;
	int temp_back_posi, temp_for_posi;
	int temp_customer, modify_Node;		//node_id
	int incre_deleted;

	//在拓展网络中，所有label按照node_id存储，所以有p.Customer_Num个Bucket
	//在每个customer中存储的label都已经拓展到出发点
	//对每个Mapping_Reduced中的点
	/////////////////////////////////////反向标签////////////////////////////////////////////////////
	for (i = 0; i<BackfeasiExten_Index_num[p.Customer_Num]; i++)
	{
		//客户
		temp_customer = BackfeasiExten_Index[p.Customer_Num][i];
		if (BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot]>MAXNUM - 1) continue;
		//拓展网络对应点
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//确定插入BackwardPath_container的位置
		temp_back_posi = Seek_containerIndex(BACKWARD);
		//以下在BackwardPath_container中temp_back_posi位置构建副本
		BackwardPath_container[temp_back_posi].exist_state = 0;
		BackwardPath_container[temp_back_posi].extend_state = 1;
		//拓展到出发点:从进入点到出发点
		BackwardPath_container[temp_back_posi].customeList[0] = temp_customer;
		BackwardPath_container[temp_back_posi].customeList_num = 1;
		//更新资源信息
		BackwardPath_container[temp_back_posi].availablecapacity = p.Veh.Veh_Capacity - p.Allnode[temp_customer].demand;
		BackwardPath_container[temp_back_posi].accu_duration = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] + p.Allnode[temp_customer].servicetime;//时间可能需要转换
		BackwardPath_container[temp_back_posi].consumed_time = max(BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot], p.Max_arrivaltime[current_depot] - p.Allnode[temp_customer].endTW - p.Allnode[temp_customer].servicetime);//时间可能需要转换
		BackwardPath_container[temp_back_posi].surplus_time = p.Max_arrivaltime[current_depot] - BackwardPath_container[temp_back_posi].consumed_time;
		if (BackwardPath_container[temp_back_posi].surplus_time<Hpoint_For)
		{
			BackwardPath_container[temp_back_posi].extend_state = 0;
		}
		//拓展时，路径的RC值直接包含temp_customer所有的node
		BackwardPath_container[temp_back_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot]  - temp_dual[temp_customer] - fix_cost;
		BackwardPath_container[temp_back_posi].augment_ornot = false;

		//重置经过的节点
		BackwardPath_container[temp_back_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			BackwardPath_container[temp_back_posi].passnode[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_2cycle[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_ngpath[j] = 0;
			BackwardPath_container[temp_back_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary有关的
		BackwardPath_container[temp_back_posi].elementaty = true;
		BackwardPath_container[temp_back_posi].prenode = -1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode);
		//ngpath有关的
		BackwardPath_container[temp_back_posi].ngpath[BackwardPath_container[temp_back_posi].ngpath_num] = modify_Node;
		BackwardPath_container[temp_back_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode_ngpath);

		incre_deleted = Delelted_Backwardindex_num;	//初始化
		//最后来判断是否支配
		if (false == domination_2cycle_ngpath_cuts_simple(BACKWARD, temp_customer, lp, BackwardPath_container[temp_back_posi]))
		{
			incre_deleted = Delelted_Backwardindex_num - incre_deleted;
			//如果不被支配，则加入BackwardBucket[temp_customer]中
			//首先在BackwardPath_container中确认添加
			BackwardPath_container[temp_back_posi].exist_state = 1;
			//然后更新BackwardPath_container和Delelted_Backwardindex中的序号
			Add_container(BACKWARD, temp_back_posi, incre_deleted);
			//最后添加到BackwardBucket[temp_customer]中
			BackwardBucket[temp_customer].Path_index[BackwardBucket[temp_customer].Path_Num] = temp_back_posi;
			BackwardBucket[temp_customer].Path_Num = BackwardBucket[temp_customer].Path_Num + 1;
		}
	}
	/////////////////////////////////////前向标签////////////////////////////////////////////////////
	for (i = 0; i < ForfeasiExten_Index_num[p.Customer_Num]; i++)
	{
		//客户
		temp_customer = ForfeasiExten_Index[p.Customer_Num][i];
		if (BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer]>MAXNUM - 1) continue;
		//拓展网络对应点
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//确定插入ForwardPath_container的位置
		temp_for_posi = Seek_containerIndex(FORWARD);
		//在ForwardPath_container中temp_for_posi位置构建副本
		ForwardPath_container[temp_for_posi].exist_state = 0;
		ForwardPath_container[temp_for_posi].extend_state = 1;
		//拓展到出发点:从进入点到出发点
		ForwardPath_container[temp_for_posi].customeList[0] = temp_customer;
		ForwardPath_container[temp_for_posi].customeList_num = 1;
		//资源更新
		ForwardPath_container[temp_for_posi].usedcapacity = p.Allnode[temp_customer].demand;
		ForwardPath_container[temp_for_posi].accu_duration = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] + p.Allnode[temp_customer].servicetime;  //时间可能需要转换
		ForwardPath_container[temp_for_posi].arrivaltime = max(BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer], p.Allnode[temp_customer].startTW); //时间可能需要转换

		if (ForwardPath_container[temp_for_posi].arrivaltime > Hpoint_For)
		{
			ForwardPath_container[temp_for_posi].extend_state = 0;
		}
		//拓展时，路径的RC值直接包含temp_customer所有的node
		ForwardPath_container[temp_for_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - temp_dual[temp_customer] - fix_cost;
		ForwardPath_container[temp_for_posi].augment_ornot = false;

		//重置经过的节点
		ForwardPath_container[temp_for_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			ForwardPath_container[temp_for_posi].passnode[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_2cycle[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_ngpath[j] = 0;
			ForwardPath_container[temp_for_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary有关的
		ForwardPath_container[temp_for_posi].elementaty = true;
		ForwardPath_container[temp_for_posi].prenode = -1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode);
		//ngpath有关的
		ForwardPath_container[temp_for_posi].ngpath[ForwardPath_container[temp_for_posi].ngpath_num] = modify_Node;
		ForwardPath_container[temp_for_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode_ngpath);

		incre_deleted = Delelted_Forwardindex_num;	//初始化
		//最后来判断是否支配
		if (false == domination_2cycle_ngpath_cuts_simple(FORWARD, temp_customer, lp, ForwardPath_container[temp_for_posi]))
		{
			incre_deleted = Delelted_Forwardindex_num - incre_deleted;
			//如果不被支配，则加入ForwardBucket[temp_customer]中
			//首先在ForwardPath_container中确认添加
			ForwardPath_container[temp_for_posi].exist_state = 1;
			//然后更新ForwardPath_container和Delelted_Forwardindex中的序号
			Add_container(FORWARD, temp_for_posi, incre_deleted);
			//最后添加到ForwardBucket[temp_customer]中
			ForwardBucket[temp_customer].Path_index[ForwardBucket[temp_customer].Path_Num] = temp_for_posi;
			ForwardBucket[temp_customer].Path_Num = ForwardBucket[temp_customer].Path_Num + 1;
		}
	}
}

bool Subproblem::DP_Labeling_simple(int direction, BranchABound & BB, RMP & lp, Problem & p)
{
	bool exact_ornot = true;
	int solved_customers = 0;
	int Extend_Customer = 0;
	int temp_itor = 0;
	int temp_check;

	//这里是对每个customer上的label都进行拓展
	//终止条件是没有customer上存在可以拓展的label了
	do
	{
		solved_customers = 0;
		for (temp_itor = 0; temp_itor<p.Customer_Num; temp_itor++)
		{
			//确定循环顺序,极限是确定从哪个label拓展
			//我们这里只确定从哪个customer开始拓展
			Extend_Customer = Search_Nextcustomer(temp_itor, p);
			//对Extend_Customer上的所有label进行拓展
			temp_check = ExtendLable_onCustomer_simple(direction, Extend_Customer, BB, lp, p);
			if (1 == temp_check)
			{
				solved_customers = solved_customers + 1;
			}
			else if (2 == temp_check)
			{
				return false;
			}
		}
	} while (solved_customers<p.Customer_Num);

	return exact_ornot;
}

int Subproblem::ExtendLable_onCustomer_simple(int direction, int cur_customer, BranchABound & BB, RMP & lp, Problem & p)
{
	bool solved_ornot = 1;
	int next_customer;
	int curLabel, nextLabel;
	int incre_deleted;
	int temp_check;

	//拓展网络中的label拓展有特殊规则：
	//由于支配只在cutomer上发生，同一个customer下的所有label都被存储在出发点
	//拓展过程也是：从出发点拓展，直到另一个出发点停下，并检验dominance rule
	if (direction == BACKWARD)
	{
		//比较的cur_customer（终点）上的所有路径
		for (int i = 0; i<BackwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = BackwardBucket[cur_customer].Path_index[i];		//BackwardPath_container中的序号
																		//第一，BackwardPath_container[curLabel]一定是可以拓展的
			if (0 == BackwardPath_container[curLabel].extend_state)continue;
			//遍历每个可行拓展
			for (int k = 0; k<BackfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//获取新的label的位置
				next_customer = BackfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Backwardindex_num;	//初始化
				temp_check = Extend_Onestep_simple(direction, cur_customer, BackwardPath_container[curLabel], next_customer, BackwardPath_container[nextLabel], BB, lp, p);
				if (1 == temp_check)
				{
					//若拓展成功则需要判断支配条件
					if (false == domination_2cycle_ngpath_cuts_simple(direction, next_customer, lp, BackwardPath_container[nextLabel]))
					{
						//Delelted_Backwardindex_num增量
						incre_deleted = Delelted_Backwardindex_num - incre_deleted;
						//如果不被支配，则加入BackwardBucket[next_customer]中
						//首先在BackwardPath_container中确认添加
						BackwardPath_container[nextLabel].exist_state = 1;
						//然后更新BackwardPath_container和Delelted_Backwardindex中的序号
						Add_container(direction, nextLabel, incre_deleted);
						//最后添加到BackwardBucket[next_customer]中
						BackwardBucket[next_customer].Path_index[BackwardBucket[next_customer].Path_Num] = nextLabel;
						BackwardBucket[next_customer].Path_Num = BackwardBucket[next_customer].Path_Num + 1;
						//说明这点可以拓展
						solved_ornot = 0;
					}
				}
				else if (2 == temp_check)
				{
					return 2;
				}
			}
			//BackwardPath_container[curLabel]已经拓展完毕，转为永久标签
			//不可再拓展
			BackwardPath_container[curLabel].extend_state = 0;
		}
	}
	else
	{
		//比较的cur_customer（终点）上的所有路径
		for (int i = 0; i<ForwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = ForwardBucket[cur_customer].Path_index[i];		//ForwardPath_container中的序号
																		//第一，ForwardPath_container[curLabel]一定是可以拓展的
			if (0 == ForwardPath_container[curLabel].extend_state)continue;
			//遍历每个可行拓展
			for (int k = 0; k<ForfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//获取新的label的位置
				next_customer = ForfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Forwardindex_num;	//初始化
				temp_check = Extend_Onestep_simple(direction, cur_customer, ForwardPath_container[curLabel], next_customer, ForwardPath_container[nextLabel], BB, lp, p);
				if (1 == temp_check)
				{
					//若拓展成功则需要判断支配条件
					if (false == domination_2cycle_ngpath_cuts_simple(direction, next_customer, lp, ForwardPath_container[nextLabel]))
					{
						//Delelted_Forwardindex_num增量
						incre_deleted = Delelted_Forwardindex_num - incre_deleted;
						//如果不被支配，则加入ForwardBucket[next_customer]中
						//首先在ForwardPath_container中确认添加
						ForwardPath_container[nextLabel].exist_state = 1;
						//然后更新ForwardPath_container和Delelted_Forwardindex中的序号
						Add_container(direction, nextLabel, incre_deleted);
						//最后添加到ForwardBucket[next_customer]中
						ForwardBucket[next_customer].Path_index[ForwardBucket[next_customer].Path_Num] = nextLabel;
						ForwardBucket[next_customer].Path_Num = ForwardBucket[next_customer].Path_Num + 1;
						//说明这点可以拓展
						solved_ornot = 0;
					}
				}
				else if(2== temp_check)
				{
					return 2;
				}
			}
			//ForwardPath_container[curLabel]已经拓展完毕，转为永久标签
			//不可再拓展
			ForwardPath_container[curLabel].extend_state = 0;
		}
	}
	return solved_ornot;
}

int Subproblem::Extend_Onestep_simple(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, RMP & lp, Problem & p)
{
	int next_modifyNode;

	if (direction == BACKWARD)
	{
		if (BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus]>MAXNUM - 1)return 0;
		next_modifyNode = Mapping_fromRN_toMN[nextCus];

		//首先判断是否形成1-cycle？
		if (currentCus == nextCus)return 0;

		//检验是否会形成k-cycle
		if (true == if_2cycle(curlabel, next_modifyNode))return 0;

		//然后看联合ng支配
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return 0;
		}
		//检验关键资源是否超过关键点,以及资源满足
		if (false == satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return 0;
		//检验是否会形成ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return 0;

		//路径至少可以拓展到nextlabel
		//先初始化
		nextlabel.augment_ornot = false;

		//有关资源的更新都在函数satisfying_resource_constrains中

		//更新customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num] = nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//更新与ngpath有关属性
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//更新与kcycle有关的
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];

		//更新与elementary有关的
		nextlabel.copy_passnode_simple(curlabel);
		nextlabel.update_passnode_kcycle_simple(Conf::CYCLE_NUM, next_modifyNode);

		//最后修改成本(reduced cost)
		nextlabel.RC_value = curlabel.RC_value - temp_dual[nextCus] + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus];

		if (temp_passnodevalue == nextlabel.passnode[0])
		{
			return 2;
		}
		return 1;
	}
	else
	{
		if (BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus]>MAXNUM - 1)return 0;
		next_modifyNode = Mapping_fromRN_toMN[nextCus];
		//接着判断是否形成1-cycle？
		if (currentCus == nextCus)return 0;

		//检验是否会形成k-cycle
		if (true == if_2cycle(curlabel, next_modifyNode))return 0;

		//然后看联合ng支配
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return 0;
		}
		//检验关键资源是否超过关键点,以及资源满足
		if (false == satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return 0;
		//检验是否会形成ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return 0;

		//路径至少可以拓展到nextlabel
		//先初始化
		nextlabel.augment_ornot = false;

		//有关资源的更新都在函数satisfying_resource_constrains中

		//更新customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num] = nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//更新与ngpath有关属性
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//更新与kcycle有关的
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];

		//更新与elementary有关的
		nextlabel.copy_passnode_simple(curlabel);
		nextlabel.update_passnode_kcycle_simple(Conf::CYCLE_NUM, next_modifyNode);

		//最后修改成本(reduced cost)
		nextlabel.RC_value = curlabel.RC_value - temp_dual[nextCus] + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus];

		if (temp_passnodevalue == nextlabel.passnode[0])
		{
			return 2;
		}
		return 1;
	}
}

bool Subproblem::domination_2cycle_ngpath_cuts_simple(int direction, int nextcustomer, RMP & lp, Path & nextV)
{
	int i, k;
	int node_num = 0, store_prenode = -1;
	int next_Labelindex;							//container中的下一个label的序号
	bool controled = false;							//新线路是否被支配，true被支配，false不被支配
	bool ini_ornot = true;							//首先要初始化模板
	bool complete_donimated_byNgpath = false;

	if (direction == FORWARD)//正向
	{
		///////////////////////////////////////////////////////////////////先判断新线路nextV是否被ForwardBucket[nextcustomer]中某条线路支配////////////////////////////////////////////////////////////////////////////
		//比较的是终点为nextcustomer的所有路径
		for (k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到ForwardBucket[nextcustomer]每个label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			//路径的支配关系为：设P1路径的经过的最后cycle_eli_num个节点集合为S1，P2路径经过的最后cycle_eli_num个节点集合为S2
			//S1对应的资源消耗量为R1(包括成本)，S2对应的资源消耗量为R2（包括成本）
			//情况1：S1是S2的子集，如果R1<=R2，P1支配P2
			//情况2：如果存在任意一条路径P3（终点与P1和P2相同），满足：1) S2的补集是（S1的补集并上S3的补集）的子集；2）R1<=R2和R3<=R2 ,那么P2可以被支配

			//考虑的路径的最后节点一定为nextcustomer
			//首先看，ForwardPath_container[next_Labelindex]是否为nextV的子集
			//if (ForwardPath_container[next_Labelindex].nodenum>nextV.nodenum)continue;		//如果ForwardPath_container[next_Labelindex]的节点数比新线路大，继续循环
			//首先找到支配nextV的ForwardPath_container[next_Labelindex],支配条件使用最松的SPPRC准则
			if (ForwardPath_container[next_Labelindex].usedcapacity>nextV.usedcapacity)continue;
			if (ForwardPath_container[next_Labelindex].arrivaltime>nextV.arrivaltime/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (ForwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif
			if (ForwardPath_container[next_Labelindex].RC_value >nextV.RC_value + MINDOUBLE)continue;

			//满足以上条件，说明ForwardPath_container[next_Labelindex]至少可以部分支配nextV
			//multi-ng支配
			complete_donimated_byNgpath = true;
			//ForwardPath_container[next_Labelindex]的ng-path是nextV的ng-path的子集，则ForwardPath_container[next_Labelindex]支配nextV
			//ng-path
			//首先，找到ForwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集
			for (i = 0; i < ForwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//找到ForwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集个每个元素
				if (false == belong_toset(ForwardPath_container[next_Labelindex].ngpath[i], nextV.passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == nextV.augment_ornot)
					{
						nextV.get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[ForwardPath_container[next_Labelindex].ngpath[i]], nextV.passnode_ngpath);
						nextV.augment_ornot = true;
					}
					else
					{
						if (true == nextV.get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[ForwardPath_container[next_Labelindex].ngpath[i]]))
						{
							//多个部分支配，组成完全支配
							controled = true;				//如果上述条件都满足则新线路被支配
							return controled;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//如果上述条件都满足则新线路被支配
				return controled;
			}
			//启发式的2cycle支配规则
			if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				//先判断ForwardPath_container[next_Labelindex]和nextV倒数第二个节点是否相同
				//如果相同，则nextV被支配
				controled = true;
				return controled;
			}
			else
			{
				//ForwardBucket[nextcustomer]中至少存在两个ForwardPath_container[next_Labelindex]与nextV倒数第二个节点不同的
				//这两个ForwardPath_container[next_Labelindex]的prenode也不相同
				if (ForwardPath_container[next_Labelindex].prenode != store_prenode)
				{
					if (1 == node_num)
					{
						//如果已经找到满足条件的两个ForwardPath_container[next_Labelindex]，则nextV被支配
						controled = true;
						return controled;
					}
					else
					{
						//记录找到的ForwardPath_container[next_Labelindex]
						store_prenode = ForwardPath_container[next_Labelindex].prenode;
						node_num = node_num + 1;
					}
				}
			}
		}
		////////////////////////////////////////////////////////再判断ForwardBucket[nextcustomer]的某条线路是否被新线路nextV支配////////////////////////////////////////////////////////
		for (k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到ForwardBucket[nextcustomer]每个label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			if (0 == ForwardPath_container[next_Labelindex].exist_state)
			{
				continue;//如果ForwardPath_container[next_Labelindex]已经被支配掉了，就没有必要去比了
			}
			//首先找到被nextV支配的ForwardPath_container[next_Labelindex]，使用SPPRC支配准则
			//如果nextV不支配ForwardPath_container[next_Labelindex]的跳过
			if (nextV.usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
			if (nextV.arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
			if (nextV.RC_value>ForwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;

			//满足以上条件，说明nextV至少可以部分支配ForwardPath_container[next_Labelindex]
			//multi-ng支配
			complete_donimated_byNgpath = true;
			//ng-path
			//因为ForwardPath_container[next_Labelindex].feasible_extensions_byngpath已经被更新过，所以只需要检查nextV对ForwardPath_container[next_Labelindex]的局部支配关系
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//找到nextV.passnode_ngpath对ForwardPath_container[next_Labelindex].passnode_ngpath的补集个每个元素
				if (false == belong_toset(nextV.ngpath[i], ForwardPath_container[next_Labelindex].passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == ForwardPath_container[next_Labelindex].augment_ornot)
					{
						ForwardPath_container[next_Labelindex].get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]], ForwardPath_container[next_Labelindex].passnode_ngpath);
						ForwardPath_container[next_Labelindex].augment_ornot = true;
					}
					else
					{
						if (true == ForwardPath_container[next_Labelindex].get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]]))
						{
							//多个部分支配，导致ForwardPath_container[next_Labelindex]被完全支配
							//删除ForwardBucket[nextcustomer].Path_index[k]序号;
							Delete_pathindex(direction, nextcustomer, k);
							//删除ForwardPath_container中元素，并记录在Delelted_Forwardindex中
							Delete_container(direction, next_Labelindex);
							//先标注ForwardPath_container中exist_state状态
							ForwardPath_container[next_Labelindex].exist_state = 0;
							//注意迭代器
							k = k - 1;
							break;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				//同上，先处理container,再处理链表
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}

			if (0 == ForwardPath_container[next_Labelindex].exist_state)continue;

			//启发式的2cycle支配规则
			//如果被支配：
			//先判断nextV倒数第二节点是否和ForwardPath_container[next_Labelindex]的倒数第二个节点相同
			if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}
			else
			{
				//首先知道：nextV已经SPPRC支配ForwardPath_container[next_Labelindex]
				//再判断是否存在其他任意个路径与nextV配合支配ForwardPath_container[next_Labelindex]
				//如果存在，则ForwardPath_container[next_Labelindex]被支配
				for (i = 0; i < ForwardBucket[nextcustomer].Path_Num; i++)
				{
					int temp_label = ForwardBucket[nextcustomer].Path_index[i];
					if (0 == ForwardPath_container[temp_label].exist_state)
					{
						continue;//如果ForwardPath_container[temp_label]已经被支配掉了，就没有必要去比了
					}
					//找到支配ForwardPath_container[next_Labelindex]的ForwardPath_container[temp_label]
					if (ForwardPath_container[temp_label].usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
					if (ForwardPath_container[temp_label].arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
					if (ForwardPath_container[temp_label].accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
					if (ForwardPath_container[temp_label].RC_value>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;

					//ForwardPath_container[temp_label]也SPPRC支配ForwardPath_container[next_Labelindex]
					//ForwardPath_container[temp_label]与nextV的倒数第二个节点不同
					if (nextV.prenode != ForwardPath_container[temp_label].prenode)
					{
						Delete_pathindex(direction, nextcustomer, k);
						Delete_container(direction, next_Labelindex);
						ForwardPath_container[next_Labelindex].exist_state = 0;
						k = k - 1;
						break;
					}
				}
			}
		}
	}
	else//逆向
	{
		///////////////////////////////////////////////////////////////////先判断新线路nextV是否被某条线路支配////////////////////////////////////////////////////////////////////////////
		//比较的是终点为nextcustomer的所有路径
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到BackwardBucket[nextcustomer]每个label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			//原理同正向
			//首先找到支配nextV的BackwardPath_container[next_Labelindex],支配条件使用最松的SPPRC准则
			if (BackwardPath_container[next_Labelindex].availablecapacity <nextV.availablecapacity)continue;	//如果curS解的第k条线路的剩余空间比新线路小，继续循环
			if (BackwardPath_container[next_Labelindex].surplus_time<nextV.surplus_time/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (BackwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif	
			if (BackwardPath_container[next_Labelindex].RC_value >nextV.RC_value + MINDOUBLE)continue;//如果curS解的第k条线路的目标值比新线路不好，继续循环
			

			//满足以上条件，说明BackwardPath_container[next_Labelindex]至少可以部分支配nextV
			//multi-ng支配
			complete_donimated_byNgpath = true;
			//ng-path
			//首先，找到BackwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集
			for (i = 0; i < BackwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//找到BackwardPath_container[next_Labelindex].passnode_ngpath对nextV.passnode_ngpath的补集个每个元素
				if (false == belong_toset(BackwardPath_container[next_Labelindex].ngpath[i], nextV.passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == nextV.augment_ornot)
					{
						nextV.get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[BackwardPath_container[next_Labelindex].ngpath[i]], nextV.passnode_ngpath);
						nextV.augment_ornot = true;
					}
					else
					{
						if (true == nextV.get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[BackwardPath_container[next_Labelindex].ngpath[i]]))
						{
							//多个部分支配，组成完全支配
							controled = true;				//如果上述条件都满足则新线路被支配
							return controled;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//如果上述条件都满足则新线路被支配
				return controled;
			}

			//启发式的2cycle支配规则
			if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				//先判断BackwardPath_container[next_Labelindex]和nextV倒数第二个节点是否相同
				//如果相同，则nextV被支配
				controled = true;
				return controled;
			}
			else
			{
				//BackwardBucket[nextcustomer]中至少存在两个BackwardPath_container[next_Labelindex]与nextV倒数第二个节点不同的
				//这两个BackwardPath_container[next_Labelindex]的prenode也不相同
				if (BackwardPath_container[next_Labelindex].prenode != store_prenode)
				{
					if (1 == node_num)
					{
						//如果已经找到满足条件的两个BackwardPath_container[next_Labelindex]，则nextV被支配
						controled = true;
						return controled;
					}
					else
					{
						//记录找到的BackwardPath_container[next_Labelindex]
						store_prenode = BackwardPath_container[next_Labelindex].prenode;
						node_num = node_num + 1;
					}
				}
			}
		}
		////////////////////////////////////////////////////////再判断BackwardPath_container[temp_nextcus]的某条线路是否被新线路nextV支配////////////////////////////////////////////////////////
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//找到BackwardBucket[nextcustomer]每个label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			if (0 == BackwardPath_container[next_Labelindex].exist_state)continue;//如果BackwardPath_container[next_Labelindex]已经被支配掉了，就没有必要去比了
																				  //首先找到被nextV支配的BackwardPath_container[next_Labelindex]，使用SPPRC支配准则
																				  //如果nextV不支配BackwardPath_container[next_Labelindex]的跳过
			if (nextV.availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
			if (nextV.surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif	
			if (nextV.RC_value >BackwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;


			//满足以上条件，说明nextV至少可以部分支配BackwardPath_container[next_Labelindex]
			//multi-ng支配
			complete_donimated_byNgpath = true;
			//ng-path
			//if (false == nextV.check_subset(nextV.passnode_ngpath, BackwardPath_container[next_Labelindex].passnode_ngpath))continue;
			//因为BackwardPath_container[next_Labelindex].feasible_extensions_byngpath已经被更新过，所以只需要检查nextV对BackwardPath_container[next_Labelindex]的局部支配关系
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//找到nextV.passnode_ngpath对BackwardPath_container[next_Labelindex].passnode_ngpath的补集个每个元素
				if (false == belong_toset(nextV.ngpath[i], BackwardPath_container[next_Labelindex].passnode_ngpath))
				{
					complete_donimated_byNgpath = false;
					if (false == BackwardPath_container[next_Labelindex].augment_ornot)
					{
						BackwardPath_container[next_Labelindex].get_ComplementarySet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]], BackwardPath_container[next_Labelindex].passnode_ngpath);
						BackwardPath_container[next_Labelindex].augment_ornot = true;
					}
					else
					{
						if (true == BackwardPath_container[next_Labelindex].get_IntersectionSet_toFeasibleExtensionsbyNgpath(ng_memory_passnode[nextV.ngpath[i]]))
						{
							//多个部分支配，导致BackwardPath_container[next_Labelindex]被完全支配
							//删除BackwardBucket[nextcustomer].Path_index[k]序号;
							Delete_pathindex(direction, nextcustomer, k);
							//删除BackwardPath_container中元素，并记录在Delelted_Backwardindex中
							Delete_container(direction, next_Labelindex);
							//先标注BackwardPath_container中exist_state状态
							BackwardPath_container[next_Labelindex].exist_state = 0;
							//再删除链表中元素,注意这里必须考虑迭代器的生存域
							k = k - 1;
							break;
						}
					}
				}
			}
			//判断是否完全支配
			if (true == complete_donimated_byNgpath)
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				BackwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}

			if (0 == BackwardPath_container[next_Labelindex].exist_state)continue;

			//启发式的2cycle支配规则
			//如果被支配：
			//先判断nextV倒数第二节点是否和BackwardPath_container[next_Labelindex]的倒数第二个节点相同
			if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				BackwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}
			else
			{
				//首先知道：nextV已经SPPRC支配BackwardPath_container[next_Labelindex]
				//再判断是否存在其他任意个路径与nextV配合支配BackwardPath_container[next_Labelindex]
				//如果存在，则nextV被支配
				for (i = 0; i < BackwardBucket[nextcustomer].Path_Num; i++)
				{
					int temp_label = BackwardBucket[nextcustomer].Path_index[i];
					if (0 == BackwardPath_container[temp_label].exist_state)continue;//如果BackwardPath_container[temp_label]已经被支配掉了，就没有必要去比了
																						//找到支配BackwardPath_container[next_Labelindex]的BackwardPath_container[temp_label]
					if (BackwardPath_container[temp_label].availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
					if (BackwardPath_container[temp_label].surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
					if (BackwardPath_container[temp_label].accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif
					if (BackwardPath_container[temp_label].RC_value >BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;

					//BackwardPath_container[temp_label]也SPPRC支配BackwardPath_container[next_Labelindex]
					//BackwardPath_container[temp_label]与nextV的倒数第二个节点不同
					if (nextV.prenode != BackwardPath_container[temp_label].prenode)
					{
						Delete_pathindex(direction, nextcustomer, k);
						Delete_container(direction, next_Labelindex);
						BackwardPath_container[next_Labelindex].exist_state = 0;
						k = k - 1;
						break;
					}
				}
			}
		}
	}
	//没有找到释放迭代器listitor的函数，作为局部变量没有new应该没有问题吧
	return controled;
}

bool Subproblem::combinebidirection_kcycle_ngpath_cuts_stable_simple(BranchABound & BB, RMP & lp, Problem & p)
{
	int i, j;
	int for_Label, back_Label;
	float redundant_ST;
	bool check = false;

	//前向遍历每个customer
	for (int for_customer = 0; for_customer<p.Customer_Num; for_customer++)
	{
		//遍历for_customer上的所有前向label
		for (i = 0; i<ForwardBucket[for_customer].Path_Num; i++)
		{
			for_Label = ForwardBucket[for_customer].Path_index[i]; //前向label的序号

			for (int back_customer = 0; back_customer < p.Customer_Num; back_customer++)
			{
				//前向label与反向label的最后一个点相同时没有必要合并
				//因为总存在另一对合并的label
				if (for_customer == back_customer)continue;
				//遍历back_customer上的所有前向label
				for (j = 0; j<BackwardBucket[back_customer].Path_Num; j++)
				{
					back_Label = BackwardBucket[back_customer].Path_index[j]; //反向label的序号

					//先判断资源可行性
					if (ForwardPath_container[for_Label].usedcapacity >BackwardPath_container[back_Label].availablecapacity)continue;	 //检测是否满足需求约束
					if ((ForwardPath_container[for_Label].arrivaltime + p.Allnode[for_customer].servicetime + BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer]
						+ p.Allnode[back_customer].servicetime + BackwardPath_container[back_Label].consumed_time)>p.Max_arrivaltime[current_depot])continue;	//检测是否满足时间窗约束
#if DURATIONORNOT == 1
					if (for_customer == back_customer)
					{
						redundant_ST = p.Allnode[for_customer].servicetime;
					}
					else
					{
						redundant_ST = 0;
					}
					if ((ForwardPath_container[for_Label].accu_duration + BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer]
						+ BackwardPath_container[back_Label].accu_duration - redundant_ST) >p.Veh.Duration)continue;	 //检测是否满足duration约束
#endif
					//如果合并label满足所有约束
					if (temp_passnodevalue ==(ForwardPath_container[for_Label].passnode[0]+ BackwardPath_container[back_Label].passnode[0]))
					{
						check = true;
						return check;
					}
				}
			}
		}
	}
	return check;
}

