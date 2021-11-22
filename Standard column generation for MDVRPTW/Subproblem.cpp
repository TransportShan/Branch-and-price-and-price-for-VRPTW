#include "stdafx.h"
#include "Subproblem.h"
#include "Conf.h"

int *temp_Index;
long *temppass;			
int passnode_length;	
long *passnodelist;	
vector<int> elimi_nodes;
Path *ForwardPath_container;				//�����洢PSP���ҵ�������ǰ��·������СΪ[MAX_PATH_CONTAINER]
int ForwardPath_container_num;				//����ForwardPath_container�Ѿ�ʹ�õ�����
Path *BackwardPath_container;				//�����洢PSP���ҵ�������ǰ��·������СΪ[MAX_PATH_CONTAINER]
int BackwardPath_container_num;				//����BackwardPath_container�Ѿ�ʹ�õ�����

bool compare_for(int a1, int a2)
{
	return ForwardPath_container[a1].RC_value < ForwardPath_container[a2].RC_value;  //������������У�����Ϊ>,���������� 
}
bool compare_back(int a1, int a2)
{
	return BackwardPath_container[a1].RC_value < BackwardPath_container[a2].RC_value;  //������������У�����Ϊ>,���������� 
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
	return true;//Ϊ��
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
		cout<< "ERROR:û����3-cycle��� " << endl;
		cin.get();
	}

#if Frameworks == 0
	if (true == elementaty)
	{
		//���жϲ����next_modifynode�Ƿ񹹳ɻ�
		int lineid, rowid;
		rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
		lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
		if (0 == (passnode[rowid] & passnodelist[lineid]))
		{
			//�����ɻ�
			passnode[rowid] = passnode[rowid] + passnodelist[lineid];
			elementaty = true;
		}
		else
		{
			//�γɻ�
			elementaty = false;
		}
	}
#else
	//���жϲ����next_modifynode�Ƿ񹹳ɻ�
	int lineid, rowid;
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
	if (0 == (passnode[rowid] & passnodelist[lineid]))
	{
		//�����ɻ�,���벻�ظ���
		passnode[rowid] = passnode[rowid] + passnodelist[lineid];
		elementaty = elementaty & true;
	}
	else
	{
		//�γɻ������ü����ظ������ڵ�
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
	if (true == fromlabel.elementaty)//ֻ�е�ǰlabel������ʱ���ż�����¼passnode
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
	//�����ǽ���
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
	//�������һ����
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
	//���жϲ����next_modifynode�Ƿ񹹳ɻ�
	int lineid, rowid;
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
	if (0 == (passnode[rowid] & passnodelist[lineid]))
	{
		//�����ɻ�
		passnode[rowid] = passnode[rowid] + passnodelist[lineid];
		return true;
	}
	else
	{
		//�γɻ�
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
	passnode_2cycle[rowid] = passnode_2cycle[rowid] | passnodelist[lineid];	//������ֻҪ��һ�����ھͿ�����
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
	//���жϲ����next_modifynode�Ƿ񹹳ɻ�
	int lineid, rowid;
	rowid = int((next_modifynode + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = next_modifynode - rowid*Conf::EACH_LONG_NUM;
	if (0 == (passnode[rowid] & passnodelist[lineid]))
	{
		//�����ɻ�,���벻�ظ���
		passnode[rowid] = passnode[rowid] + passnodelist[lineid];
		elementaty = elementaty & true;
	}
	else
	{
		//�γɻ������ü����ظ������ڵ�
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
	//����
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
	//ǰ������
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

	//��������
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

	//�������
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

	//DSSR���
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


	//��ʼ��bucket
	ForwardBucket = new NodeBucket[p.Customer_Num+1];
	BackwardBucket = new NodeBucket[p.Customer_Num + 1];
	for (int i = 0; i<p.Customer_Num + 1; i++)
	{
		ForwardBucket[i].Path_index = new int[Conf::MAX_PATH_NUM];
		BackwardBucket[i].Path_index = new int[Conf::MAX_PATH_NUM];
		//������Completion_bounds
		ForwardBucket[i].Completion_bounds = new float[int(p.Depot[current_depot].endTW)];
		BackwardBucket[i].Completion_bounds = new float[int(p.Depot[current_depot].endTW)];
		for(int j = 0; j < int(p.Depot[current_depot].endTW); j++)
		{
			ForwardBucket[i].Completion_bounds[j] = MINNUM;
			BackwardBucket[i].Completion_bounds[j] = MINNUM;
		}
	}
	Ini_compleBound_bucket(p);

	//��ʼ��ngset
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
	//��ʼ������
	passnodelist = new long[Conf::EACH_LONG_NUM];
	passnodelist[0] = 1;
	for (int i = 1; i < Conf::EACH_LONG_NUM; i++)
	{
		passnodelist[i] = passnodelist[i - 1] * 2;
	}
	temppass = new long[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
	temp_Index = new int[p.Customer_Num + 1];
	
	//��ʼ��vector
	elimi_nodes.resize(Conf::MAX_NODES);
	temp_dual = new float[p.Customer_Num];
}

void Subproblem::Get_dual_info(int obj_depot, float remain_arc, BranchABound & BB, RMP & lp, Problem & p, SDC &Cuts_sdc)
{
	current_depot = obj_depot;
	arc_remain_proportion = remain_arc;
	fix_cost = lp.Vehicle_dual[current_depot];

	//��ʼ��һ��Ѳ���
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
	//�ع�ӳ���ϵ��FeasiExten_Index
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
	//����FeasiExten_Index
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
	//����FeasiExten_Index
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
	//���´ӳ�վ�����ܹ������customer
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

	//����arc_remain_proportion����FeasiExten_Index
	if (arc_remain_proportion<(1 - MINDOUBLE))
	{
		int temp_posi, i, j, k;
		int ordered_num;	//�Ѿ�����õ�����
		int savenum;
		//���ճɱ�+RCֵ��������
		//ǰ��
		for (i = 0; i < p.Customer_Num; i++)
		{
			if (ForfeasiExten_Index_num[i] <= 25)continue; //PSP������һ���ڵ�ķ�����������25��
			savenum = max(25, int(arc_remain_proportion*ForfeasiExten_Index_num[i])); 
			for (j = 0; j <ForfeasiExten_Index_num[i]; j++)
			{
				temp_Index[j] = ForfeasiExten_Index[i][j];
			}
			//��ʼ����
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
					//ǰ����������
					for (k = ordered_num - 1; k > temp_posi; k--)
					{
						ForfeasiExten_Index[i][k] = ForfeasiExten_Index[i][k - 1];
					}
					//�ٸ�ֵ
					ForfeasiExten_Index[i][temp_posi] = temp_Index[j];
				}
			}
			ForfeasiExten_Index_num[i] = savenum;
		}
		//����
		for (i = 0; i < p.Customer_Num; i++)
		{
			if (BackfeasiExten_Index_num[i] <= 1)continue;
			savenum = int(arc_remain_proportion*BackfeasiExten_Index_num[i]);
			for (j = 0; j <BackfeasiExten_Index_num[i]; j++)
			{
				temp_Index[j] = BackfeasiExten_Index[i][j];
			}
			//��ʼ����
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
					//ǰ����������
					for (k = ordered_num - 1; k > temp_posi; k--)
					{
						BackfeasiExten_Index[i][k] = BackfeasiExten_Index[i][k - 1];
					}
					//�ٸ�ֵ
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

		temp_time = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot];		//ʱ�������Ҫת��
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
		//ǰ��
		ForwardBucket[i].Bucket_max = int(floor(p.Max_arrivaltime[current_depot] - p.Allnode[i].startTW - p.Allnode[i].servicetime));
		ForwardBucket[i].Bucket_min = int(floor(max(p.Time_network[i][p.Customer_Num+ current_depot], p.Max_arrivaltime[current_depot]- p.Allnode[i].endTW - p.Allnode[i].servicetime)));
		//����
		BackwardBucket[i].Bucket_max = int(floor(p.Allnode[i].endTW));
		BackwardBucket[i].Bucket_min = int(floor(p.Allnode[i].startTW));
	}

}

void Subproblem::Initial_Hpoints(Problem &p)
{
	float bound = 0.48;

	if (true == Conf::Not_DynamicBound)
	{
		//��lastest_departuretime������surplus_timeΪ�ؼ���Դ����ʼ��������ͬ
		Hpoint_Back = 0;
		Hpoint_For = p.Max_arrivaltime[current_depot];
	}
	else
	{
		//��lastest_departuretime������surplus_timeΪ�ؼ���Դ����ʼ��������ͬ
		Hpoint_Back = (1 - bound)*p.Max_arrivaltime[current_depot];
		Hpoint_For = bound*p.Max_arrivaltime[current_depot];
	}
}

void Subproblem::Ini_ngset(BranchABound & BB, RMP & lp, Problem & p)
{
	int temp_posi,i,j,k;
	int ordered_num;	//neighbourhood���Ѿ�����õ�����
	bool check;
	//���ݶ�ż�������ҵ�i�ڵ�����������NEIGHBOURHOOD_NUM���ڵ�,����i�ڵ�һ����neighbourhood��(����)
	ng_clear();

	//ȷ��ng�Ĵ�С
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
	//����ÿ��customer��ngset��pr06һ������0.001s
	//����RCֵ��������
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
				//ǰ����������
				for (k = ordered_num - 1; k > temp_posi; k--)
				{
					neighbourhood[i][k] = neighbourhood[i][k - 1];
				}
				//�ٸ�ֵ
				neighbourhood[i][temp_posi] = j;
			}
			}
		neighbourhood_num[i] = ordered_num;
		}
#endif

	//���i�ڵ��Ƿ���i�ڵ�������У�������������ĩβλ��
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
		if (true == check)//i�ڵ㲻��i�ڵ��������
		{
			neighbourhood[i][neighbourhood_num[i] - 1] = i;
		}
	}
	//תΪ�����ƣ�����neighbourhood_passnode��
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
	//ת��neighbourhood�õ�ng_memory_passnode
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

	//����չ�����У�����label����node_id�洢��������p.Customer_Num��Bucket
	//��ÿ��customer�д洢��label���Ѿ���չ��������
	//��ÿ��Mapping_Reduced�еĵ�
	/////////////////////////////////////�����ǩ////////////////////////////////////////////////////
	for (i=0;i<BackfeasiExten_Index_num[p.Customer_Num];i++)
	{
		//�ͻ�
		temp_customer = BackfeasiExten_Index[p.Customer_Num][i];
#if EXACTEXTEND==0
		if (true== restricted_extend && (BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] - lp.Customer_dual[temp_customer])>-MINDOUBLE)continue;
#endif
		if (BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num+current_depot]>MAXNUM - 1) continue;
		//��չ�����Ӧ��
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//ȷ������BackwardPath_container��λ��
		temp_back_posi = Seek_containerIndex(BACKWARD);
		//������BackwardPath_container��temp_back_posiλ�ù�������
		BackwardPath_container[temp_back_posi].exist_state = 0;
		BackwardPath_container[temp_back_posi].extend_state = 1;
		//��չ��������:�ӽ���㵽������
		BackwardPath_container[temp_back_posi].customeList[0] = temp_customer;
		BackwardPath_container[temp_back_posi].customeList_num = 1;
		//������Դ��Ϣ
		BackwardPath_container[temp_back_posi].availablecapacity = p.Veh.Veh_Capacity - p.Allnode[temp_customer].demand;
		BackwardPath_container[temp_back_posi].accu_duration =BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot]+p.Allnode[temp_customer].servicetime;//ʱ�������Ҫת��
		BackwardPath_container[temp_back_posi].consumed_time = max(BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot], p.Max_arrivaltime[current_depot] - p.Allnode[temp_customer].endTW - p.Allnode[temp_customer].servicetime);//ʱ�������Ҫת��
		BackwardPath_container[temp_back_posi].surplus_time = p.Max_arrivaltime[current_depot] - BackwardPath_container[temp_back_posi].consumed_time;
		if (BackwardPath_container[temp_back_posi].surplus_time<Hpoint_For)
		{
			BackwardPath_container[temp_back_posi].extend_state = 0;
		}
		//��չʱ��·����RCֱֵ�Ӱ���temp_customer���е�node
#if KPATHCUT +RCCUT==0
		BackwardPath_container[temp_back_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] - lp.Customer_dual[temp_customer] - fix_cost;
#else
		BackwardPath_container[temp_back_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] - lp.RobustCut_dual[temp_customer][p.Customer_Num + current_depot] - lp.Customer_dual[temp_customer] - fix_cost;
#endif
		
		BackwardPath_container[temp_back_posi].augment_ornot = false;

		//���þ����Ľڵ�
		BackwardPath_container[temp_back_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			BackwardPath_container[temp_back_posi].passnode[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_2cycle[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_ngpath[j] = 0;
			BackwardPath_container[temp_back_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary�йص�
		BackwardPath_container[temp_back_posi].elementaty = true;
		BackwardPath_container[temp_back_posi].prenode = -1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode);
		//ngpath�йص�
		BackwardPath_container[temp_back_posi].ngpath[BackwardPath_container[temp_back_posi].ngpath_num] = modify_Node;
		BackwardPath_container[temp_back_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode_ngpath);

		//SDCʹ�õ�label resource
#if Frameworks == 1
		for (j = 0; j < passnode_length; j++)
		{
			BackwardPath_container[temp_back_posi].passnode_SDCnode[j] = 0;
		}
		if (1==Cuts_sdc.SDC_indicator[temp_customer])
		{
			//������SDCnode�ĵ�һ��λ��
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
		//��ʼ��state
		//���Ȱ�state_src����Ϊ0
		for (j = 0; j < Cuts_src.processed_num; j++)
		{
			BackwardPath_container[temp_back_posi].state_src[j] = 0;
			//�жϽڵ�temp_customer�Ƿ��ڵ�j��Cuts_src��subset��
			if (1==Cuts_src.SRC_subset_indicator[j][temp_customer])
			{
				BackwardPath_container[temp_back_posi].state_src[j] = Cuts_src.add_state(Cuts_src.SRC_subset_len[j]);
			}
		}
#endif
		incre_deleted = Delelted_Backwardindex_num;	//��ʼ��
		//������ж��Ƿ�֧��
		if (false == domination_2cycle_ngpath_cuts(BACKWARD, temp_customer, lp,BackwardPath_container[temp_back_posi], Cuts_src))
		{
			incre_deleted = Delelted_Backwardindex_num - incre_deleted;
			//�������֧�䣬�����BackwardBucket[temp_customer]��
			//������BackwardPath_container��ȷ�����
			BackwardPath_container[temp_back_posi].exist_state = 1;
			//Ȼ�����BackwardPath_container��Delelted_Backwardindex�е����
			Add_container(BACKWARD, temp_back_posi, incre_deleted);
			//�����ӵ�BackwardBucket[temp_customer]��
			BackwardBucket[temp_customer].Path_index[BackwardBucket[temp_customer].Path_Num] = temp_back_posi;
			BackwardBucket[temp_customer].Path_Num = BackwardBucket[temp_customer].Path_Num + 1;
			//Bac_unprocessed_label_num = Bac_unprocessed_label_num + 1;
			//Bac_generated_label_num = Bac_generated_label_num + 1;
		}
	}
	/////////////////////////////////////ǰ���ǩ////////////////////////////////////////////////////
	for (i = 0; i < ForfeasiExten_Index_num[p.Customer_Num]; i++)
	{
		//�ͻ�
		temp_customer = ForfeasiExten_Index[p.Customer_Num][i];
#if EXACTEXTEND==0
		if (true == restricted_extend && (BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - lp.Customer_dual[temp_customer])>-MINDOUBLE)continue;
#endif
		if (BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer]>MAXNUM - 1) continue;
		//��չ�����Ӧ��
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//ȷ������ForwardPath_container��λ��
		temp_for_posi = Seek_containerIndex(FORWARD);
		//��ForwardPath_container��temp_for_posiλ�ù�������
		ForwardPath_container[temp_for_posi].exist_state = 0;
		ForwardPath_container[temp_for_posi].extend_state = 1;
		//��չ��������:�ӽ���㵽������
		ForwardPath_container[temp_for_posi].customeList[0] = temp_customer;
		ForwardPath_container[temp_for_posi].customeList_num = 1;
		//��Դ����
		ForwardPath_container[temp_for_posi].usedcapacity = p.Allnode[temp_customer].demand;
		ForwardPath_container[temp_for_posi].accu_duration =BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] + p.Allnode[temp_customer].servicetime;  //ʱ�������Ҫת��
		ForwardPath_container[temp_for_posi].arrivaltime = max(BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer], p.Allnode[temp_customer].startTW); //ʱ�������Ҫת��


		if (ForwardPath_container[temp_for_posi].arrivaltime > Hpoint_For)
		{
			ForwardPath_container[temp_for_posi].extend_state = 0;
		}
		//��չʱ��·����RCֱֵ�Ӱ���temp_customer���е�node
#if KPATHCUT +RCCUT==0
		ForwardPath_container[temp_for_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - lp.Customer_dual[temp_customer] - fix_cost;
#else
		ForwardPath_container[temp_for_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - lp.RobustCut_dual[p.Customer_Num + current_depot][temp_customer] - lp.Customer_dual[temp_customer] - fix_cost;
#endif
		ForwardPath_container[temp_for_posi].augment_ornot = false;

		//���þ����Ľڵ�
		ForwardPath_container[temp_for_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			ForwardPath_container[temp_for_posi].passnode[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_2cycle[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_ngpath[j] = 0;
			ForwardPath_container[temp_for_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary�йص�
		ForwardPath_container[temp_for_posi].elementaty = true;
		ForwardPath_container[temp_for_posi].prenode = -1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode);
		//ngpath�йص�
		ForwardPath_container[temp_for_posi].ngpath[ForwardPath_container[temp_for_posi].ngpath_num] = modify_Node;
		ForwardPath_container[temp_for_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode_ngpath);

		//SDCʹ�õ�label resource
#if Frameworks == 1
		for (j = 0; j < passnode_length; j++)
		{
			ForwardPath_container[temp_for_posi].passnode_SDCnode[j] = 0;
		}
		if (1 == Cuts_sdc.SDC_indicator[temp_customer])
		{
			//������SDCnode�ĵ�һ��λ��
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
		//��ʼ��state
		//���Ȱ�state_src����Ϊ0
		for (j = 0; j < Cuts_src.processed_num; j++)
		{
			ForwardPath_container[temp_for_posi].state_src[j] = 0;
			//�жϽڵ�temp_customer�Ƿ��ڵ�j��Cuts_src��subset��
			if (1 == Cuts_src.SRC_subset_indicator[j][temp_customer])
			{
				ForwardPath_container[temp_for_posi].state_src[j] = Cuts_src.add_state(Cuts_src.SRC_subset_len[j]);
			}
		}
#endif
		incre_deleted = Delelted_Forwardindex_num;	//��ʼ��
		//������ж��Ƿ�֧��
		if (false == domination_2cycle_ngpath_cuts(FORWARD, temp_customer, lp, ForwardPath_container[temp_for_posi], Cuts_src))
		{
			incre_deleted = Delelted_Forwardindex_num - incre_deleted;
			//�������֧�䣬�����ForwardBucket[temp_customer]��
			//������ForwardPath_container��ȷ�����
			ForwardPath_container[temp_for_posi].exist_state = 1;
			//Ȼ�����ForwardPath_container��Delelted_Forwardindex�е����
			Add_container(FORWARD, temp_for_posi, incre_deleted);
			//�����ӵ�ForwardBucket[temp_customer]��
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
			//Delelted_Backwardindex�������
			temp_posi = Delelted_Backwardindex[Delelted_Backwardindex_num - 1];
		}
		else
		{
			//BackwardPath_container˳�����
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
		//Delelted_Foundindex_num�������
		temp_posi = Delelted_Foundindex[Delelted_Foundindex_num - 1];
	}
	else
	{
		//FoundPath_container˳�����
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
		//���Ȱ�objbucket.Path_index���һ��Ԫ����objbucket.Path_index[pathindex]����
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
		//��ʼ��ģ��
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
	float RC_increment = 0;							//��L1֧��L2����L1��RCֵ����Ҫ��L2��RCֵ����RC_increment
													//EPO����£�RC_incrementΪ0
													//CPA����£�RC_increment����Ϊһ������
	int node_num=0,store_prenode = -1;
	int next_Labelindex;							//container�е���һ��label�����
	bool controled = false;							//����·�Ƿ�֧�䣬true��֧�䣬false����֧��
	bool ini_ornot = true;							//����Ҫ��ʼ��ģ��
	bool complete_donimated_byNgpath = false;

	if (direction == FORWARD)//����
	{
		///////////////////////////////////////////////////////////////////���ж�����·nextV�Ƿ�ForwardBucket[nextcustomer]��ĳ����·֧��////////////////////////////////////////////////////////////////////////////
		//�Ƚϵ����յ�Ϊnextcustomer������·��
		for(k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�ForwardBucket[nextcustomer]ÿ��label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			//·����֧���ϵΪ����P1·���ľ��������cycle_eli_num���ڵ㼯��ΪS1��P2·�����������cycle_eli_num���ڵ㼯��ΪS2
			//S1��Ӧ����Դ������ΪR1(�����ɱ�)��S2��Ӧ����Դ������ΪR2�������ɱ���
			//���1��S1��S2���Ӽ������R1<=R2��P1֧��P2
			//���2�������������һ��·��P3���յ���P1��P2��ͬ�������㣺1) S2�Ĳ����ǣ�S1�Ĳ�������S3�Ĳ��������Ӽ���2��R1<=R2��R3<=R2 ,��ôP2���Ա�֧��

			//���ǵ�·�������ڵ�һ��Ϊnextcustomer
			//���ȿ���ForwardPath_container[next_Labelindex]�Ƿ�ΪnextV���Ӽ�
			//if (ForwardPath_container[next_Labelindex].nodenum>nextV.nodenum)continue;		//���ForwardPath_container[next_Labelindex]�Ľڵ���������·�󣬼���ѭ��
			//�����ҵ�֧��nextV��ForwardPath_container[next_Labelindex],֧������ʹ�����ɵ�SPPRC׼��
			if (ForwardPath_container[next_Labelindex].usedcapacity>nextV.usedcapacity)continue;
			if (ForwardPath_container[next_Labelindex].arrivaltime>nextV.arrivaltime/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (ForwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif
			RC_increment = Calculate_RCincrement(ForwardPath_container[next_Labelindex], nextV, lp);	//����RC_increment
			if (ForwardPath_container[next_Labelindex].RC_value+ RC_increment>nextV.RC_value + MINDOUBLE)continue;
#if SRCUT == 1
			if (ForwardPath_container[next_Labelindex].RC_value- Calculate_SRCdual_Gap(ForwardPath_container[next_Labelindex], nextV, lp, Cuts_src)>nextV.RC_value + MINDOUBLE)continue;
#endif

			//��������������˵��ForwardPath_container[next_Labelindex]���ٿ��Բ���֧��nextV
#if NGDOI == 0		//ng��ȫ֧��
			//ForwardPath_container[next_Labelindex]��ng-path��nextV��ng-path���Ӽ�����ForwardPath_container[next_Labelindex]֧��nextV
			//ng-path
			//�ж��Ƿ���ȫ֧��
			if (true == check_subset(ForwardPath_container[next_Labelindex].passnode_ngpath, nextV.passnode_ngpath))
			{
				controled = true;				//�����������������������·��֧��
				return controled;
			}
			//���ȣ��ҵ�ForwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ���
			for (i = 0; i < ForwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//�ҵ�ForwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬�����ȫ֧��
							controled = true;				//�����������������������·��֧��
							return controled;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ForwardPath_container[next_Labelindex]��ng-path��nextV��ng-path���Ӽ�����ForwardPath_container[next_Labelindex]֧��nextV
			//ng-path
			//���ȣ��ҵ�ForwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ���
			for (i = 0; i < ForwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//�ҵ�ForwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬�����ȫ֧��
							controled = true;				//�����������������������·��֧��
							return controled;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//�����������������������·��֧��
				return controled;
			}
#endif
			if (false== exact_ornot)
			{
#if TCYCLE == 1		//��ȷ��2cycle֧�����
				if (true == check_subset(ForwardPath_container[next_Labelindex].passnode_2cycle, nextV.passnode_2cycle))
				{
					//���ж�ForwardPath_container[next_Labelindex].passnode_2cycle�Ƿ�Ϊ��nextV.passnode_2cycle���Ӽ�
					//�����ͬ����nextV��֧��
					controled = true;				//�����������������������·��֧��
					return controled;
				}
				else
				{
					//���ж��Ƿ�������������·���Ҷ�֧��nextV��·��
					//������ڣ���nextV��֧��
					if (false == if_union_domination(ini_ornot, ForwardPath_container[next_Labelindex].passnode_2cycle))
					{
						//�ȳ�ʼ��ģ��
						ini_ornot = false;
						continue;
					}
					else
					{
						controled = true;				//�����������������������·��֧��
						return controled;
					}
				}
#elif TCYCLE == 2	//����ʽ��2cycle֧�����
				if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					//���ж�ForwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ��Ƿ���ͬ
					//�����ͬ����nextV��֧��
					controled = true;
					return controled;
				}
				else
				{
					//ForwardBucket[nextcustomer]�����ٴ�������ForwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ㲻ͬ��
					//������ForwardPath_container[next_Labelindex]��prenodeҲ����ͬ
					if (ForwardPath_container[next_Labelindex].prenode != store_prenode)
					{
						if (1 == node_num)
						{
							//����Ѿ��ҵ���������������ForwardPath_container[next_Labelindex]����nextV��֧��
							controled = true;
							return controled;
						}
						else
						{
							//��¼�ҵ���ForwardPath_container[next_Labelindex]
							store_prenode = ForwardPath_container[next_Labelindex].prenode;
							node_num = node_num + 1;
						}
					}
				}
#endif
			}
		}
		////////////////////////////////////////////////////////���ж�ForwardBucket[nextcustomer]��ĳ����·�Ƿ�����·nextV֧��////////////////////////////////////////////////////////
		for (k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�ForwardBucket[nextcustomer]ÿ��label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			if (0 == ForwardPath_container[next_Labelindex].exist_state)
			{
				continue;//���ForwardPath_container[next_Labelindex]�Ѿ���֧����ˣ���û�б�Ҫȥ����
			}
			//�����ҵ���nextV֧���ForwardPath_container[next_Labelindex]��ʹ��SPPRC֧��׼��
			//���nextV��֧��ForwardPath_container[next_Labelindex]������
			if (nextV.usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
			if (nextV.arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
			RC_increment = Calculate_RCincrement(nextV, ForwardPath_container[next_Labelindex], lp);	//����RC_increment
			if (nextV.RC_value+ RC_increment>ForwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#if SRCUT == 1
			if (nextV.RC_value - Calculate_SRCdual_Gap(ForwardPath_container[next_Labelindex], nextV, lp, Cuts_src)>ForwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#endif

			//��������������˵��nextV���ٿ��Բ���֧��ForwardPath_container[next_Labelindex]
#if NGDOI == 0		//ng��ȫ֧��
			//ng-path
			//�ж��Ƿ���ȫ֧��
			if (true == check_subset(nextV.passnode_ngpath, ForwardPath_container[next_Labelindex].passnode_ngpath))
			{
				//ͬ�£��ȴ���container,�ٴ�������
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
				continue;
			}
			//��ΪForwardPath_container[next_Labelindex].feasible_extensions_byngpath�Ѿ������¹�������ֻ��Ҫ���nextV��ForwardPath_container[next_Labelindex]�ľֲ�֧���ϵ
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//�ҵ�nextV.passnode_ngpath��ForwardPath_container[next_Labelindex].passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬����ForwardPath_container[next_Labelindex]����ȫ֧��
							//ɾ��ForwardBucket[nextcustomer].Path_index[k]���;
							Delete_pathindex(direction, nextcustomer, k);
							//ɾ��ForwardPath_container��Ԫ�أ�����¼��Delelted_Forwardindex��
							Delete_container(direction, next_Labelindex);
							//�ȱ�עForwardPath_container��exist_state״̬
							ForwardPath_container[next_Labelindex].exist_state = 0;
							//ע�������
							k = k - 1;
							break;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ng-path
			//��ΪForwardPath_container[next_Labelindex].feasible_extensions_byngpath�Ѿ������¹�������ֻ��Ҫ���nextV��ForwardPath_container[next_Labelindex]�ľֲ�֧���ϵ
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//�ҵ�nextV.passnode_ngpath��ForwardPath_container[next_Labelindex].passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬����ForwardPath_container[next_Labelindex]����ȫ֧��
							//ɾ��ForwardBucket[nextcustomer].Path_index[k]���;
							Delete_pathindex(direction, nextcustomer, k);
							//ɾ��ForwardPath_container��Ԫ�أ�����¼��Delelted_Forwardindex��
							Delete_container(direction, next_Labelindex);
							//�ȱ�עForwardPath_container��exist_state״̬
							ForwardPath_container[next_Labelindex].exist_state = 0;
							//ע�������
							k = k - 1;
							break;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
			if (true == complete_donimated_byNgpath)
			{
				//ͬ�ϣ��ȴ���container,�ٴ�������
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}
#endif

			if (0 == ForwardPath_container[next_Labelindex].exist_state)continue;

			if (false==exact_ornot)
			{
#if TCYCLE == 1		//��ȷ��2cycle֧�����
				if (true == check_subset(nextV.passnode_2cycle, ForwardPath_container[next_Labelindex].passnode_2cycle))
				{
					//���ж�nextV.passnode_2cycle�Ƿ�ΪForwardPath_container[next_Labelindex].passnode_2cycle���Ӽ�
					//�����ͬ����ForwardPath_container[next_Labelindex]��֧��
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					ForwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					if_union_domination(true, nextV.passnode_2cycle);
					//����֪����nextV�Ѿ�SPPRC֧��ForwardPath_container[next_Labelindex]
					//���ж��Ƿ�������������·����nextV���֧��ForwardPath_container[next_Labelindex]
					//������ڣ���ForwardPath_container[next_Labelindex]��֧��
					for (i = 0; i < ForwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = ForwardBucket[nextcustomer].Path_index[i];
						if (0 == ForwardPath_container[temp_label].exist_state)
						{
							continue;//���ForwardPath_container[temp_label]�Ѿ���֧����ˣ���û�б�Ҫȥ����
						}
						//�ҵ�֧��ForwardPath_container[next_Labelindex]��ForwardPath_container[temp_label]
						if (ForwardPath_container[temp_label].usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
						if (ForwardPath_container[temp_label].arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
						if (ForwardPath_container[temp_label].accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp);	//����RC_increment
						if (ForwardPath_container[temp_label].RC_value+ RC_increment>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (ForwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp, Cuts_src)>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//ForwardPath_container[temp_label]ҲSPPRC֧��ForwardPath_container[next_Labelindex]
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
#elif TCYCLE == 2	//����ʽ��2cycle֧�����
				//�����֧�䣺
				//���ж�nextV�����ڶ��ڵ��Ƿ��ForwardPath_container[next_Labelindex]�ĵ����ڶ����ڵ���ͬ
				if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					ForwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					//����֪����nextV�Ѿ�SPPRC֧��ForwardPath_container[next_Labelindex]
					//���ж��Ƿ�������������·����nextV���֧��ForwardPath_container[next_Labelindex]
					//������ڣ���ForwardPath_container[next_Labelindex]��֧��
					for (i = 0; i < ForwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = ForwardBucket[nextcustomer].Path_index[i];
						if (0 == ForwardPath_container[temp_label].exist_state)
						{
							continue;//���ForwardPath_container[temp_label]�Ѿ���֧����ˣ���û�б�Ҫȥ����
						}
						//�ҵ�֧��ForwardPath_container[next_Labelindex]��ForwardPath_container[temp_label]
						if (ForwardPath_container[temp_label].usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
						if (ForwardPath_container[temp_label].arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
						if (ForwardPath_container[temp_label].accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp);	//����RC_increment
						if (ForwardPath_container[temp_label].RC_value+ RC_increment>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (ForwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(ForwardPath_container[temp_label], ForwardPath_container[next_Labelindex], lp, Cuts_src)>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//ForwardPath_container[temp_label]ҲSPPRC֧��ForwardPath_container[next_Labelindex]
						//ForwardPath_container[temp_label]��nextV�ĵ����ڶ����ڵ㲻ͬ
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
	else//����
	{
		///////////////////////////////////////////////////////////////////���ж�����·nextV�Ƿ�ĳ����·֧��////////////////////////////////////////////////////////////////////////////
		//�Ƚϵ����յ�Ϊnextcustomer������·��
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�BackwardBucket[nextcustomer]ÿ��label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			//ԭ��ͬ����
			//�����ҵ�֧��nextV��BackwardPath_container[next_Labelindex],֧������ʹ�����ɵ�SPPRC׼��
			if (BackwardPath_container[next_Labelindex].availablecapacity <nextV.availablecapacity)continue;	//���curS��ĵ�k����·��ʣ��ռ������·С������ѭ��
			if (BackwardPath_container[next_Labelindex].surplus_time<nextV.surplus_time/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (BackwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif	
			RC_increment = Calculate_RCincrement(BackwardPath_container[next_Labelindex], nextV, lp);	//����RC_increment
			if (BackwardPath_container[next_Labelindex].RC_value+ RC_increment>nextV.RC_value + MINDOUBLE)continue;//���curS��ĵ�k����·��Ŀ��ֵ������·���ã�����ѭ��
#if SRCUT == 1
			if (BackwardPath_container[next_Labelindex].RC_value - Calculate_SRCdual_Gap(BackwardPath_container[next_Labelindex], nextV, lp, Cuts_src)>nextV.RC_value + MINDOUBLE)continue;
#endif			

			//��������������˵��BackwardPath_container[next_Labelindex]���ٿ��Բ���֧��nextV

#if NGDOI == 0		//ng��ȫ֧��
			//ng-path
			//�ж��Ƿ���ȫ֧��
			if (true == check_subset(BackwardPath_container[next_Labelindex].passnode_ngpath, nextV.passnode_ngpath))
			{
				controled = true;				//�����������������������·��֧��
				return controled;
			}
			//���ȣ��ҵ�BackwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ���
			for (i = 0; i < BackwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//�ҵ�BackwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬�����ȫ֧��
							controled = true;				//�����������������������·��֧��
							return controled;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ng-path
			//���ȣ��ҵ�BackwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ���
			for (i = 0; i < BackwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//�ҵ�BackwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬�����ȫ֧��
							controled = true;				//�����������������������·��֧��
							return controled;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//�����������������������·��֧��
				return controled;
			}
#endif

			if (false==exact_ornot)
			{
#if TCYCLE == 1		//��ȷ��2cycle֧�����
				if (true == check_subset(BackwardPath_container[next_Labelindex].passnode_2cycle, nextV.passnode_2cycle))
				{
					//���ж�BackwardPath_container[next_Labelindex].passnode_2cycle�Ƿ�Ϊ��nextV.passnode_2cycle���Ӽ�
					//�����ͬ����nextV��֧��
					controled = true;				//�����������������������·��֧��
					return controled;
				}
				else
				{
					//���ж��Ƿ�������������·���Ҷ�֧��nextV��·��
					//������ڣ���nextV��֧��
					if (false == if_union_domination(ini_ornot, BackwardPath_container[next_Labelindex].passnode_2cycle))
					{
						//�ȳ�ʼ��ģ��
						ini_ornot = false;
						continue;
					}
					else
					{
						controled = true;				//�����������������������·��֧��
						return controled;
					}
				}

#elif TCYCLE == 2	//����ʽ��2cycle֧�����
				if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					//���ж�BackwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ��Ƿ���ͬ
					//�����ͬ����nextV��֧��
					controled = true;
					return controled;
				}
				else
				{
					//BackwardBucket[nextcustomer]�����ٴ�������BackwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ㲻ͬ��
					//������BackwardPath_container[next_Labelindex]��prenodeҲ����ͬ
					if (BackwardPath_container[next_Labelindex].prenode != store_prenode)
					{
						if (1 == node_num)
						{
							//����Ѿ��ҵ���������������BackwardPath_container[next_Labelindex]����nextV��֧��
							controled = true;
							return controled;
						}
						else
						{
							//��¼�ҵ���BackwardPath_container[next_Labelindex]
							store_prenode = BackwardPath_container[next_Labelindex].prenode;
							node_num = node_num + 1;
						}
					}
				}
#endif
			}
		}
		////////////////////////////////////////////////////////���ж�BackwardPath_container[temp_nextcus]��ĳ����·�Ƿ�����·nextV֧��////////////////////////////////////////////////////////
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�BackwardBucket[nextcustomer]ÿ��label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			if (0 == BackwardPath_container[next_Labelindex].exist_state)continue;//���BackwardPath_container[next_Labelindex]�Ѿ���֧����ˣ���û�б�Ҫȥ����
			//�����ҵ���nextV֧���BackwardPath_container[next_Labelindex]��ʹ��SPPRC֧��׼��
			//���nextV��֧��BackwardPath_container[next_Labelindex]������
			if (nextV.availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
			if (nextV.surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif	
			RC_increment = Calculate_RCincrement(nextV, BackwardPath_container[next_Labelindex], lp);	//����RC_increment
			if (nextV.RC_value+ RC_increment>BackwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#if SRCUT == 1
			if (nextV.RC_value - Calculate_SRCdual_Gap(nextV, BackwardPath_container[next_Labelindex], lp, Cuts_src)>BackwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;
#endif	

			//��������������˵��nextV���ٿ��Բ���֧��BackwardPath_container[next_Labelindex]
#if NGDOI == 0		//ng��ȫ֧��
			//�ж��Ƿ���ȫ֧��
			//ng-path
			if (true == check_subset(nextV.passnode_ngpath, BackwardPath_container[next_Labelindex].passnode_ngpath))
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				BackwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
				continue;
			}
			//��ΪBackwardPath_container[next_Labelindex].feasible_extensions_byngpath�Ѿ������¹�������ֻ��Ҫ���nextV��BackwardPath_container[next_Labelindex]�ľֲ�֧���ϵ
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//�ҵ�nextV.passnode_ngpath��BackwardPath_container[next_Labelindex].passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬����BackwardPath_container[next_Labelindex]����ȫ֧��
							//ɾ��BackwardBucket[nextcustomer].Path_index[k]���;
							Delete_pathindex(direction, nextcustomer, k);
							//ɾ��BackwardPath_container��Ԫ�أ�����¼��Delelted_Backwardindex��
							Delete_container(direction, next_Labelindex);
							//�ȱ�עBackwardPath_container��exist_state״̬
							BackwardPath_container[next_Labelindex].exist_state = 0;
							//��ɾ��������Ԫ��,ע��������뿼�ǵ�������������
							k = k - 1;
							break;
						}
					}
				}
			}
#elif NGDOI == 1	//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ng-path
			//if (false == nextV.check_subset(nextV.passnode_ngpath, BackwardPath_container[next_Labelindex].passnode_ngpath))continue;
			//��ΪBackwardPath_container[next_Labelindex].feasible_extensions_byngpath�Ѿ������¹�������ֻ��Ҫ���nextV��BackwardPath_container[next_Labelindex]�ľֲ�֧���ϵ
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//�ҵ�nextV.passnode_ngpath��BackwardPath_container[next_Labelindex].passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬����BackwardPath_container[next_Labelindex]����ȫ֧��
							//ɾ��BackwardBucket[nextcustomer].Path_index[k]���;
							Delete_pathindex(direction, nextcustomer, k);
							//ɾ��BackwardPath_container��Ԫ�أ�����¼��Delelted_Backwardindex��
							Delete_container(direction, next_Labelindex);
							//�ȱ�עBackwardPath_container��exist_state״̬
							BackwardPath_container[next_Labelindex].exist_state = 0;
							//��ɾ��������Ԫ��,ע��������뿼�ǵ�������������
							k = k - 1;
							break;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
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
#if TCYCLE == 1		//��ȷ��2cycle֧�����
				if (true == check_subset(nextV.passnode_2cycle, BackwardPath_container[next_Labelindex].passnode_2cycle))
				{
					//���ж�nextV.passnode_2cycle�Ƿ�ΪBackwardPath_container[next_Labelindex].passnode_2cycle���Ӽ�
					//�����ͬ����BackwardPath_container[next_Labelindex]��֧��
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					BackwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					if_union_domination(true, nextV.passnode_2cycle);
					//����֪����nextV�Ѿ�SPPRC֧��BackwardPath_container[next_Labelindex]
					//���ж��Ƿ�������������·����nextV���֧��BackwardPath_container[next_Labelindex]
					//������ڣ���BackwardPath_container[next_Labelindex]��֧��
					for (i = 0; i < BackwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = BackwardBucket[nextcustomer].Path_index[i];
						if (0 == BackwardPath_container[temp_label].exist_state)
						{
							continue;//���BackwardPath_container[temp_label]�Ѿ���֧����ˣ���û�б�Ҫȥ����
						}
						//�ҵ�֧��BackwardPath_container[next_Labelindex]��BackwardPath_container[temp_label]
						if (BackwardPath_container[temp_label].usedcapacity>BackwardPath_container[next_Labelindex].usedcapacity)continue;
						if (BackwardPath_container[temp_label].arrivaltime>BackwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
						if (BackwardPath_container[temp_label].accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp);	//����RC_increment
						if (BackwardPath_container[temp_label].RC_value+ RC_increment>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (BackwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp, Cuts_src)>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//BackwardPath_container[temp_label]ҲSPPRC֧��BackwardPath_container[next_Labelindex]
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
#elif TCYCLE == 2	//����ʽ��2cycle֧�����
				//�����֧�䣺
				//���ж�nextV�����ڶ��ڵ��Ƿ��BackwardPath_container[next_Labelindex]�ĵ����ڶ����ڵ���ͬ
				if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
				{
					Delete_pathindex(direction, nextcustomer, k);
					Delete_container(direction, next_Labelindex);
					BackwardPath_container[next_Labelindex].exist_state = 0;
					k = k - 1;
				}
				else
				{
					//����֪����nextV�Ѿ�SPPRC֧��BackwardPath_container[next_Labelindex]
					//���ж��Ƿ�������������·����nextV���֧��BackwardPath_container[next_Labelindex]
					//������ڣ���nextV��֧��
					for (i = 0; i < BackwardBucket[nextcustomer].Path_Num; i++)
					{
						int temp_label = BackwardBucket[nextcustomer].Path_index[i];
						if (0 == BackwardPath_container[temp_label].exist_state)continue;//���BackwardPath_container[temp_label]�Ѿ���֧����ˣ���û�б�Ҫȥ����
						 //�ҵ�֧��BackwardPath_container[next_Labelindex]��BackwardPath_container[temp_label]
						if (BackwardPath_container[temp_label].availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
						if (BackwardPath_container[temp_label].surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
						if (BackwardPath_container[temp_label].accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif
						RC_increment = Calculate_RCincrement(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp);	//����RC_increment
						if (BackwardPath_container[temp_label].RC_value+ RC_increment>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#if SRCUT == 1
						if (BackwardPath_container[temp_label].RC_value - Calculate_SRCdual_Gap(BackwardPath_container[temp_label], BackwardPath_container[next_Labelindex], lp, Cuts_src)>BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;
#endif

						//BackwardPath_container[temp_label]ҲSPPRC֧��BackwardPath_container[next_Labelindex]
						//BackwardPath_container[temp_label]��nextV�ĵ����ڶ����ڵ㲻ͬ
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
	//û���ҵ��ͷŵ�����listitor�ĺ�������Ϊ�ֲ�����û��newӦ��û�������
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
	
	//�����Ƕ�ÿ��customer�ϵ�label��������չ
	//��ֹ������û��customer�ϴ��ڿ�����չ��label��
	do
	{
		solved_customers = 0;
		for (temp_itor=0; temp_itor<p.Customer_Num; temp_itor++)
		{
			//ȷ��ѭ��˳��,������ȷ�����ĸ�label��չ
			//��������ֻȷ�����ĸ�customer��ʼ��չ
			Extend_Customer = Search_Nextcustomer(temp_itor, p);
			//��Extend_Customer�ϵ�����label������չ
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
	//����1������customer˳������
	temp_ExtenCustomer = itor;
	//
#elif SEARCH == 1	
	//����2�����տ�ʼ����ʱ��������ҵ�customer
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

	//��չ�����е�label��չ���������
	//����֧��ֻ��cutomer�Ϸ�����ͬһ��customer�µ�����label�����洢�ڳ�����
	//��չ����Ҳ�ǣ��ӳ�������չ��ֱ����һ��������ͣ�£�������dominance rule
	if (direction == BACKWARD)
	{
		//�Ƚϵ�cur_customer���յ㣩�ϵ�����·��
		for (int i = 0; i<BackwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = BackwardBucket[cur_customer].Path_index[i];		//BackwardPath_container�е����
			//��һ��BackwardPath_container[curLabel]һ���ǿ�����չ��
			if (0 == BackwardPath_container[curLabel].extend_state)continue;
			//����ÿ��������չ
			for (int k = 0; k<BackfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//��ȡ�µ�label��λ��
				next_customer = BackfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Backwardindex_num;	//��ʼ��
				if (true == Extend_Onestep(direction, cur_customer, BackwardPath_container[curLabel], next_customer, BackwardPath_container[nextLabel], BB,lp, p, Cuts_src, Cuts_sdc))
				{
					//����չ�ɹ�����Ҫ�ж�֧������
					if (false == domination_2cycle_ngpath_cuts(direction, next_customer, lp, BackwardPath_container[nextLabel], Cuts_src))
					{
						//Delelted_Backwardindex_num����
						incre_deleted = Delelted_Backwardindex_num - incre_deleted;
						//�������֧�䣬�����BackwardBucket[next_customer]��
						//������BackwardPath_container��ȷ�����
						BackwardPath_container[nextLabel].exist_state = 1;
						//Ȼ�����BackwardPath_container��Delelted_Backwardindex�е����
						Add_container(direction, nextLabel, incre_deleted);
						//�����ӵ�BackwardBucket[next_customer]��
						BackwardBucket[next_customer].Path_index[BackwardBucket[next_customer].Path_Num] = nextLabel;
						BackwardBucket[next_customer].Path_Num = BackwardBucket[next_customer].Path_Num + 1;
						//˵����������չ
						solved_ornot = false;
					}
				}
			}
			//BackwardPath_container[curLabel]�Ѿ���չ��ϣ�תΪ���ñ�ǩ
			//��������չ
			BackwardPath_container[curLabel].extend_state = 0;
		}
	}
	else
	{
		//�Ƚϵ�cur_customer���յ㣩�ϵ�����·��
		for (int i = 0; i<ForwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = ForwardBucket[cur_customer].Path_index[i];		//ForwardPath_container�е����
			//��һ��ForwardPath_container[curLabel]һ���ǿ�����չ��
			if (0 == ForwardPath_container[curLabel].extend_state)continue;
			//����ÿ��������չ
			for (int k = 0; k<ForfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//��ȡ�µ�label��λ��
				next_customer = ForfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Forwardindex_num;	//��ʼ��
				if (true == Extend_Onestep(direction, cur_customer, ForwardPath_container[curLabel], next_customer, ForwardPath_container[nextLabel], BB,lp, p, Cuts_src, Cuts_sdc))
				{
					//����չ�ɹ�����Ҫ�ж�֧������
					if (false == domination_2cycle_ngpath_cuts(direction, next_customer, lp, ForwardPath_container[nextLabel], Cuts_src))
					{
						//Delelted_Forwardindex_num����
						incre_deleted = Delelted_Forwardindex_num - incre_deleted;
						//�������֧�䣬�����ForwardBucket[next_customer]��
						//������ForwardPath_container��ȷ�����
						ForwardPath_container[nextLabel].exist_state = 1;
						//Ȼ�����ForwardPath_container��Delelted_Forwardindex�е����
						Add_container(direction, nextLabel, incre_deleted);
						//�����ӵ�ForwardBucket[next_customer]��
						ForwardBucket[next_customer].Path_index[ForwardBucket[next_customer].Path_Num] = nextLabel;
						ForwardBucket[next_customer].Path_Num = ForwardBucket[next_customer].Path_Num + 1;
						//˵����������չ
						solved_ornot = false;
					}
				}
			}
			//ForwardPath_container[curLabel]�Ѿ���չ��ϣ�תΪ���ñ�ǩ
			//��������չ
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

		//�����ж��Ƿ��γ�1-cycle��
		if (currentCus == nextCus)return false;

		//����ʽ��չ
#if EXACTEXTEND ==0
		if (true == restricted_extend && (/*curlabel.RC_value+*/BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus] - lp.Customer_dual[nextCus]) >-MINDOUBLE)return false;
#endif	

		if (false == exact_ornot)
		{
			//�����Ƿ���γ�k-cycle
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

		//Ȼ������ng֧��
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return false;
		}
		//����ؼ���Դ�Ƿ񳬹��ؼ���,�Լ���Դ����
		if (false== satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return false;
		//�����Ƿ���γ�ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return false;

		//����SDC��Դ
#if Frameworks == 1
		//������ЧSDCnode��nextlabel
		nextlabel.Copy_SDCnode(curlabel, Cuts_sdc);
#if SDCTYPE == 0
		//���ȿ�nextCus�Ƿ�����Cuts_sdc.SDC_nodes
		if (1 == Cuts_sdc.SDC_indicator[nextCus])
		{
			//�ٿ�nextCus�Ƿ��Ѿ���curlabel����
			if (false == belong_toset(next_modifyNode, curlabel.passnode))
			{
				//��nextCus����nextlabel��SDCnode��
				SDC_modifiedRC = Insert_toSDC(nextCus, nextlabel, lp, Cuts_sdc);
			}
		}
#elif SDCTYPE == 1
		//�������ж�nextCus�Ƿ���Ҫ����nextlabel.SDCnode��
		if (Cuts_sdc.processed_num>0)
		{
			if (false == Copy_Augngset_SDC(nextCus, nextlabel, curlabel))
			{
				//Augng-infeasible
				//������temp_modify������SDCnode����Ȼ����customer_no
				SDC_modifiedRC = 0;
			}
			else
			{
				//��nextCus������Cuts_sdc.SDC_nodes�У�����Ҫ���루��������
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
		//�Ե�ǰ������Ч��Cuts_src
		for (int i = 0; i < Cuts_src.processed_num; i++)
		{
			//���ȼ̳�curlabel�ϵ�state_src
			nextlabel.state_src[i] = curlabel.state_src[i];
			//Ȼ�����state_src
			//��һ���жϻ�[nextCus][currentCus]�Ƿ���Cuts_src.SRC_LimitArcSet_indicator[i]��
			if (0== Cuts_src.SRC_LimitArcSet_indicator[i][nextCus][currentCus])
			{
				nextlabel.state_src[i] = 0;
			}
			//�ڶ����ж�nextCus�Ƿ���Cuts_src.SRC_subset_indicator[i]��
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

		//·�����ٿ�����չ��nextlabel
		//�ȳ�ʼ��
		nextlabel.augment_ornot = false;
		
		//�й���Դ�ĸ��¶��ں���satisfying_resource_constrains��

		//����customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num]= nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//������ngpath�й�����
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//������kcycle�йص�
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];
		if (false==exact_ornot)
		{
#if TCYCLE == 1
			//�Ȱ�passnode_ngpath���Ƹ�passnode_2cycle
			nextlabel.copy_passnode_2cycle(nextlabel.passnode_ngpath);
			//Ȼ���next_modifyNode��passnode_2cycle��ȥ����������������һ���ڵ㣩
			remove_fromPassnode(next_modifyNode, nextlabel.passnode_2cycle);
			//����ϵ����ڶ����ڵ㣨prenode��
			nextlabel.Union_2cycle(nextlabel.prenode);
#endif
		}

		//������elementary�йص�
		nextlabel.copy_passnode(curlabel);
		nextlabel.update_passnode_kcycle(Conf::CYCLE_NUM, next_modifyNode);

		//����޸ĳɱ�(reduced cost)
		nextlabel.RC_value = curlabel.RC_value+ Add_RC(direction, currentCus, curlabel, nextCus,BB,lp,p)+ SDC_modifiedRC+ SRC_modifiedRC;

		return true;
	}
	else
	{
		if (BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus]>MAXNUM - 1)return false;
		next_modifyNode = Mapping_fromRN_toMN[nextCus];

		//�����ж��Ƿ��γ�1-cycle��
		if (currentCus == nextCus)return false;
		
		//����ʽ��չ
#if EXACTEXTEND ==0
		if (true == restricted_extend && (/*curlabel.RC_value +*/ BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus] - lp.Customer_dual[nextCus]) >-MINDOUBLE)return false;
#endif

		if (false == exact_ornot)
		{
			//�����Ƿ���γ�k-cycle
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

		//Ȼ������ng֧��
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return false;
		}
		//����ؼ���Դ�Ƿ񳬹��ؼ���,�Լ���Դ����
		if (false == satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return false;
		//�����Ƿ���γ�ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return false;
		
		//����SDC��Դ
#if Frameworks == 1
		//������ЧSDCnode��nextlabel
		nextlabel.Copy_SDCnode(curlabel, Cuts_sdc);
#if SDCTYPE == 0
		//���ȿ�nextCus�Ƿ�����Cuts_sdc.SDC_nodes
		if (1 == Cuts_sdc.SDC_indicator[nextCus])
		{
			//�ٿ�nextCus�Ƿ��Ѿ���curlabel����
			if (false == belong_toset(next_modifyNode, curlabel.passnode))
			{
				//��nextCus����nextlabel��SDCnode��
				SDC_modifiedRC = Insert_toSDC(nextCus, nextlabel, lp, Cuts_sdc);
			}
		}
#elif SDCTYPE == 1
		//�������ж�nextCus�Ƿ���Ҫ����nextlabel.SDCnode��
		if (Cuts_sdc.processed_num>0)
		{
			if (false == Copy_Augngset_SDC(nextCus, nextlabel, curlabel))
			{
				//Augng-infeasible
				//������temp_modify������SDCnode����Ȼ����customer_no
				SDC_modifiedRC = 0;
			}
			else
			{
				//��nextCus������Cuts_sdc.SDC_nodes�У�����Ҫ���루��������
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
		//�Ե�ǰ������Ч��Cuts_src
		for (int i = 0; i < Cuts_src.processed_num; i++)
		{
			//���ȼ̳�curlabel�ϵ�state_src
			nextlabel.state_src[i] = curlabel.state_src[i];
			//Ȼ�����state_src
			//��һ���жϻ�[currentCus][nextCus]�Ƿ���Cuts_src.SRC_LimitArcSet_indicator[i]��
			if (0== Cuts_src.SRC_LimitArcSet_indicator[i][currentCus][nextCus])
			{
				nextlabel.state_src[i] = 0;
			}
			//�ڶ����ж�nextCus�Ƿ���Cuts_src.SRC_subset_indicator[i]��
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
		//·�����ٿ�����չ��nextlabel
		//�ȳ�ʼ��
		nextlabel.augment_ornot = false;
	
		//�й���Դ�ĸ��¶��ں���satisfying_resource_constrains��

		//����customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num] = nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//������ngpath�й�����
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//������kcycle�йص�
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];
		if (false==exact_ornot)
		{
#if TCYCLE == 1
			//�Ȱ�passnode_ngpath���Ƹ�passnode_2cycle
			nextlabel.copy_passnode_2cycle(nextlabel.passnode_ngpath);
			//Ȼ���next_modifyNode��passnode_2cycle��ȥ����������������һ���ڵ㣩
			remove_fromPassnode(next_modifyNode, nextlabel.passnode_2cycle);
			//����ϵ����ڶ����ڵ㣨prenode��
			nextlabel.Union_2cycle(nextlabel.prenode);
#endif
		}

		//������elementary�йص�
		nextlabel.copy_passnode(curlabel);
		nextlabel.update_passnode_kcycle(Conf::CYCLE_NUM, next_modifyNode);

		//����޸ĳɱ�(reduced cost)
		nextlabel.RC_value = curlabel.RC_value + Add_RC(direction, currentCus, curlabel, nextCus, BB,lp,p) + SDC_modifiedRC+ SRC_modifiedRC;

		return true;
	}
}

bool Subproblem::satisfying_resource_constrains(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, Problem & p)
{
	if (direction == BACKWARD)
	{
		//������
		nextlabel.availablecapacity = curlabel.availablecapacity - p.Allnode[nextCus].demand;
		if (nextlabel.availablecapacity<0)return false;
		//duration
		nextlabel.accu_duration = curlabel.accu_duration + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus]+ p.Allnode[nextCus].servicetime;		//ʱ�������Ҫת��
#if DURATIONORNOT == 1
		if (nextlabel.accu_duration>p.Veh.Duration)return false;
#endif
		//ʱ�䴰
		nextlabel.consumed_time = max((curlabel.consumed_time + p.Allnode[currentCus].servicetime + BB.branch[BB.cur_node].CostNetwork_Branch[nextCus][currentCus]), //ʱ�������Ҫת��
			(p.Max_arrivaltime[current_depot] - p.Allnode[nextCus].endTW - p.Allnode[nextCus].servicetime));
		if (nextlabel.consumed_time>(p.Max_arrivaltime[current_depot] - p.Allnode[nextCus].startTW - p.Allnode[nextCus].servicetime))return false;
		nextlabel.surplus_time = p.Max_arrivaltime[current_depot] - nextlabel.consumed_time;
		//��Դ����
		if (nextlabel.surplus_time >= Hpoint_For)
		{
			//��������չ
			nextlabel.extend_state = 1;
			return true;
		}
		else
		{
			//�������һ����չ
			nextlabel.extend_state = 0;
			return true;
		}
	}
	else
	{
		//������
		nextlabel.usedcapacity = curlabel.usedcapacity + p.Allnode[nextCus].demand;
		if (nextlabel.usedcapacity>p.Veh.Veh_Capacity)return false;
		//duration
		nextlabel.accu_duration = curlabel.accu_duration + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus]+p.Allnode[nextCus].servicetime;  //ʱ�������Ҫת��
#if DURATIONORNOT == 1
		if (nextlabel.accu_duration>p.Veh.Duration)return false;
#endif
		//ʱ�䴰
		nextlabel.arrivaltime = max(curlabel.arrivaltime + p.Allnode[currentCus].servicetime + BB.branch[BB.cur_node].CostNetwork_Branch[currentCus][nextCus], p.Allnode[nextCus].startTW);  //ʱ�������Ҫת��
		if (nextlabel.arrivaltime <= p.Allnode[nextCus].endTW)
		{
			//��Դ����
			if (nextlabel.arrivaltime <= Hpoint_For)
			{
				//��������չ
				nextlabel.extend_state = 1;
				return true;
			}
			else
			{
				//�������һ����չ
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
		return true;//�γ�ng-path
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
		cout << "ERROR��δָ��cost����" << endl;
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

	//srand((unsigned)time(0));  //�������ʼ����Ϊ�˲����������//����ע��

	//����,���ʱ��̫����ע��
	//ʹForwardBucket����BackwardBucket�е�label��RCֵ����
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

	//ǰ�����ÿ��customer
	for (int for_customer=0; for_customer<p.Customer_Num; for_customer++)
	{
		//����for_customer�ϵ�����ǰ��label
		for (i=0;i<ForwardBucket[for_customer].Path_Num;i++)
		{
			for_Label = ForwardBucket[for_customer].Path_index[i]; //ǰ��label�����

			for (int back_customer = 0; back_customer < p.Customer_Num; back_customer++)
			{
				//ǰ��label�뷴��label�����һ������ͬʱû�б�Ҫ�ϲ�
				//��Ϊ�ܴ�����һ�Ժϲ���label
				if(for_customer== back_customer)continue;
				//����back_customer�ϵ�����ǰ��label
				for(j=0;j<BackwardBucket[back_customer].Path_Num;j++)
				{
					back_Label = BackwardBucket[back_customer].Path_index[j]; //����label�����

					//EPO�����ͬʱ��������ֵ��
					//��һ����RC��С��label�������Ƿ����
					//�ڶ���������RC<0�ĳ���label

					//CPA�����ֻ����һ��ֵ��
					//RC��С��label�������Ƿ����

					//����ForwardPath_container[for_Label]��BackwardPath_container[back_Label]�ϲ����RCֵ
					//EPO�����Ҫȥ��һ��fix_cost
					//CPA�����Ҫȥ��һ��fix_cost��ǰ��label��SDCnode���ظ����Ӧ��SDC_dual
					RC_modify= Calculate_Joinmodify(ForwardPath_container[for_Label], BackwardPath_container[back_Label],lp);
#if KPATHCUT +RCCUT==0
					temp_RC = ForwardPath_container[for_Label].RC_value + BackwardPath_container[back_Label].RC_value
						+ BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer] + RC_modify;
#else
					temp_RC = ForwardPath_container[for_Label].RC_value + BackwardPath_container[back_Label].RC_value
						+ BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer] - lp.RobustCut_dual[for_customer][back_customer] + RC_modify;
#endif


#if SRCUT == 1
					//����ÿһ��Cuts_src
					for (int k = 0; k < Cuts_src.processed_num; k++)
					{
						//���黡[for_customer][back_customer]�Ƿ���Cuts_src.SRC_LimitArcSet_indicator[k]��
						if (1== Cuts_src.SRC_LimitArcSet_indicator[k][for_customer][back_customer])
						{
							if (ForwardPath_container[for_Label].state_src[k]+ BackwardPath_container[back_Label].state_src[k]>=1)
							{
								temp_RC = temp_RC - lp.SRC_dual[k];
							}
						}
					}
#endif

					//���ж���Դ������
					if (temp_RC >= -Conf::Exact_threshold || temp_RC>reduce_cost)continue;
					if (ForwardPath_container[for_Label].usedcapacity >BackwardPath_container[back_Label].availablecapacity)continue;	 //����Ƿ���������Լ��
					if ((ForwardPath_container[for_Label].arrivaltime + p.Allnode[for_customer].servicetime +BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer]
						+ p.Allnode[back_customer].servicetime + BackwardPath_container[back_Label].consumed_time)>p.Max_arrivaltime[current_depot])continue;	//����Ƿ�����ʱ�䴰Լ��
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
						+ BackwardPath_container[back_Label].accu_duration - redundant_ST) >p.Veh.Duration)continue;	 //����Ƿ�����durationԼ��
#endif
					
					//����ϲ�label��������Լ��
#if Frameworks == 0
					//�ȴ���FoundPath_container
					if (true == ForwardPath_container[for_Label].elementaty &&		//����ǰ����label�Ƿ���ڻ�
						true == BackwardPath_container[back_Label].elementaty)
					{
						//�ϳɺ��label���ܴ��ڻ�
						if (false == check_route_samenode(ForwardPath_container[for_Label].passnode, BackwardPath_container[back_Label].passnode))
						{
							if (true == Update_FoundPath(temp_RC, for_Label, ForwardPath_container[for_Label], back_Label, BackwardPath_container[back_Label], p))
							{
								//�ҵ��㹻���elementary��·��
								return true;
							}
						}
					}
					else
					{
						//DSSR-NG�������չ����ǰ����ߺ����л���label
						//�����л���label�Բ��ܴ���2-cycle������ng-infeasible��
						if (ForwardPath_container[for_Label].prenode> 0 && Mapping_Reduced[ForwardPath_container[for_Label].prenode] == back_customer)continue;			//ǰ��·�����������·��ĩλ���γ�2-cycle
						if (BackwardPath_container[back_Label].prenode> 0 && Mapping_Reduced[BackwardPath_container[back_Label].prenode] == for_customer)continue;		//ǰ��·�����������·��ĩλ���γ�2-cycle
						if (true == check_route_samenode(ForwardPath_container[for_Label].passnode_ngpath, BackwardPath_container[back_Label].passnode_ngpath))continue;				//ǰ��ͺ���label��ngpath���ܾ�����ͬ�ĵ�
						//�ٴ���Shortest_path
						if (temp_RC < Shortest_path.RC_value)
						{
							Shortest_path.RC_value = temp_RC;
							Shortest_forlabel = for_Label;
							Shortest_backlabel = back_Label;
						}
					}
#else
					//CPA����ºϲ�·��ʱ��
					//���ȣ������γ�2-cycle
					if (ForwardPath_container[for_Label].prenode> 0 && Mapping_Reduced[ForwardPath_container[for_Label].prenode] == back_customer)continue;			//ǰ��·�����������·��ĩλ���γ�2-cycle
					if (BackwardPath_container[back_Label].prenode> 0 && Mapping_Reduced[BackwardPath_container[back_Label].prenode] == for_customer)continue;		//ǰ��·�����������·��ĩλ���γ�2-cycle
					//��Σ��ϲ��������ng-feasible��
					if (true == check_route_samenode(ForwardPath_container[for_Label].passnode_ngpath, BackwardPath_container[back_Label].passnode_ngpath))continue;				//ǰ��ͺ���label��ngpath���ܾ�����ͬ�ĵ�

					if (true == Update_FoundPath(temp_RC, for_Label, ForwardPath_container[for_Label], back_Label, BackwardPath_container[back_Label], p))
					{
						return true;
					}
#endif
				}
			}
		}
	}
	//����ж���FoundPath_container�Ƿ��ҵ�elementary��·��
#if Frameworks == 0
	if (reduce_cost<-Conf::Exact_threshold)
	{
		//����һ��RC<0�ĳ���·��
		return true;
	}
	else
	{
		//���ҷǳ���·���Ƿ�RC<0
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
	//��elementary��·����RCֵ�㹻��ʱ��Ӧ��Ѱ���������·��
	if (tempRC>-Conf::Exact_threshold)
	{
		return false;
	}

#if FOUND == 0
	//��ȡFoundPath_container���
	foundIndex = Get_FoundIndex();
	//�ϲ�·��
	FoundPath_container[foundIndex].nodesNum = forlabel.customeList_num + backlabel.customeList_num +2; //����û�������յ�
	FoundPath_container[foundIndex].nodes[0] = p.Customer_Num+current_depot;
	//ǰ��
	for (int i = 0; i < forlabel.customeList_num; i++)
		FoundPath_container[foundIndex].nodes[1+i] = forlabel.customeList[i];
	//����
	i = forlabel.customeList_num;
	for (j = backlabel.customeList_num - 1; j >= 0; j--)
	{
		FoundPath_container[foundIndex].nodes[1+i] = backlabel.customeList[j];
		i++;
	}
	FoundPath_container[foundIndex].nodes[FoundPath_container[foundIndex].nodesNum -1] = p.Customer_Num+ current_depot;
	//����RCֵ
	FoundPath_container[foundIndex].Reducedcost = tempRC;
	//��ӽ���FoundPath_container
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
	//��¼�ϲ������
	foundIndex = forlabel.customeList[forlabel.customeList_num-1];
	//��ÿ���ϲ�������¼
	if (tempRC<-Conf::Exact_threshold && tempRC<FoundPath_container[foundIndex].Reducedcost- MINDOUBLE)
	{
		//����RCֵ
		FoundPath_container[foundIndex].Reducedcost = tempRC;
		FoundPath_container[foundIndex].pre_label=forLabel_index;
		FoundPath_container[foundIndex].succ_label=backLabel_index;
		if (tempRC<reduce_cost)
		{
			reduce_cost = tempRC;
		}
		//��ӽ���FoundPath_container
		return false;
	}
	return false;
#elif FOUND == 2	

#elif FOUND == 3	
	if (tempRC<FoundPath_container[0].Reducedcost)
	{
		foundIndex = 0;
		//����RCֵ
		FoundPath_container[foundIndex].Reducedcost = tempRC;
		FoundPath_container[foundIndex].pre_label = forLabel_index;
		FoundPath_container[foundIndex].succ_label = backLabel_index;
		//��ӽ���FoundPath_container
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
	//ǰ��label�л�
	if (false== ForwardPath_container[Shortest_forlabel].elementaty)
	{
		Extend_ngset(FORWARD, p);
	}
	//����label�л�
	if (false == BackwardPath_container[Shortest_backlabel].elementaty)
	{
		Extend_ngset(BACKWARD, p);
	}
}

void Subproblem::Update_compleBound(Problem & p)
{
	int bucket_index,label_index;
	//���ȸ���ÿ��customer��ÿ��bucket�ϵ�bound
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//ǰ���������ҷ���label
		for (int j = 0; j < BackwardBucket[i].Path_Num; j++)
		{
			label_index = BackwardBucket[i].Path_index[j];
			bucket_index = int(ceil(BackwardPath_container[label_index].consumed_time));
			if (BackwardPath_container[label_index].RC_value<ForwardBucket[i].Completion_bounds[bucket_index] || ForwardBucket[i].Completion_bounds[bucket_index]<MINNUM+1)
			{
				ForwardBucket[i].Completion_bounds[bucket_index] = BackwardPath_container[label_index].RC_value;
			}
		}
		//������������ǰ��label
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
	//Ȼ������������ÿ��customer��bucket�ϵ�bound
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
		//�����ж�check_modifyNode��ngset���Ƿ��Ѿ�����add_modifyNode
		//���add_modifyNode�Ѿ���check_modifyNode��ngset��,�Ͳ��ö��������
		if (false==belong_toset(add_modifyNode, neighbourhood_passnode[check_modifyNode]))
		{
			//����Ͱ���check_modifyNode��ngset�����λ�ü���add_modifyNode
			//neighbourhood
			neighbourhood[check_modifyNode][neighbourhood_num[check_modifyNode]] = add_modifyNode;
			neighbourhood_num[check_modifyNode] = neighbourhood_num[check_modifyNode] +1;
			//neighbourhood_passnode
			Insert_toPassnode(add_modifyNode, neighbourhood_passnode[check_modifyNode]);
			//ng_memory_passnode
			Insert_toPassnode(check_modifyNode, ng_memory_passnode[add_modifyNode]);

#if DSSRNG==1	
			//�洢dssr-ngset�ĸ���
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
	//������Shortest_path.passnode
	for (int i = 0; i < passnode_length; i++)
	{
		Shortest_path.passnode[i] = 0;
	}

	if (BACKWARD == direction)
	{
		//��label�Ŀ�ʼλ�ã�˳���ж��Ƿ���ڻ�
		//Ѱ��BackwardPath_container[Shortest_backlabel]�е����л�
		for (int i = 0; i<BackwardPath_container[Shortest_backlabel].customeList_num; i++)
		{
			//Ѱ��һ��modifyNode(������BackwardPath_container[Shortest_backlabel]�����ɸ�node����)
			next_modifyNode = Mapping_fromRN_toMN[BackwardPath_container[Shortest_backlabel].customeList[i]];
			//����Shortest_path��passnode
			if (false == Shortest_path.update_elementary_passnode(next_modifyNode))
			{
				//�γɻ������cycle���ظ���Ϊnext_modifyNode
				//��λ��i����̽��ֱ���ҵ���һ����ͬnext_modifyNode��node���γ�cycle
				Generate_cycle(direction, i, p);
				//��ÿ��cycle�еĽڵ㶼����ngset
				Add_ngset_byCycle(p);
			}
		}
	}
	else
	{
		//��label�Ŀ�ʼλ�ã�˳���ж��Ƿ���ڻ�
		//Ѱ��ForwardPath_container[Shortest_forlabel]�е����л�
		for (int i = 0; i<ForwardPath_container[Shortest_forlabel].customeList_num; i++)
		{
			//Ѱ��һ��modifyNode(������ForwardPath_container[Shortest_forlabel]�����ɸ�node����)
			next_modifyNode = Mapping_fromRN_toMN[ForwardPath_container[Shortest_forlabel].customeList[i]];
			//����Shortest_path��passnode
			if (false == Shortest_path.update_elementary_passnode(next_modifyNode))
			{
				//�γɻ������cycle���ظ���Ϊnext_modifyNode
				//��λ��i����̽��ֱ���ҵ���һ����ͬnext_modifyNode��node���γ�cycle
				Generate_cycle(direction,i, p);
				//��ÿ��cycle�еĽڵ㶼����ngset
				Add_ngset_byCycle(p);
			}
		}
	}
}

void Subproblem::Label_update(Problem & p)
{
	int label_index;
	//DSSR-ng����£�label��Ҫ����2����Ϣ��
	//�Ƿ�ɾ����exist_state���Ƿ�����չ��extend_state

	//elementaryΪtrue��label����
	//�������Hpoints��extend_state=1������extend_state=0
	//����extend_stateΪ0����1��ֻҪ���µ�label��ngset�����䣬feasible_extensions_byngpath��augment_ornot����
	//ǰ��
	for (int k = 0; k < p.Customer_Num; k++)
	{
		for (int i = 0; i < ForwardBucket[k].Path_Num; i++)
		{
			label_index = ForwardBucket[k].Path_index[i];
			if (0 == ForwardPath_container[label_index].exist_state) continue;
			if (true == ForwardPath_container[label_index].elementaty)
			{
				//����
				if (ForwardPath_container[label_index].arrivaltime <= Hpoint_For)
				{
					ForwardPath_container[label_index].extend_state = 1;
					//����ngpath��ngpath_num��passnode_ngpath
					generate_ngpath_Byroute(ForwardPath_container[label_index]);
					//����feasible_extensions_byngpath��augment_ornot
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
				//���������Ҫɾ��
				Delete_pathindex(FORWARD, k, i);
				Delete_container(FORWARD, label_index);
				ForwardPath_container[label_index].exist_state = 0;
				i = i - 1;
#elif NEWNG == 1	
				//ʣ����Щ��elementary
				//����ngpath�Ƿ����㣬�����������£�
				if (true == Check_ngpath_Byroute(ForwardPath_container[label_index]))
				{
					//����Hpoints�ж�extend_state
					if (ForwardPath_container[label_index].arrivaltime <= Hpoint_For)
					{
						ForwardPath_container[label_index].extend_state = 1;
						//����feasible_extensions_byngpath��augment_ornot
						for (int j = 0; j < passnode_length; j++)
						{
							ForwardPath_container[label_index].feasible_extensions_byngpath[j] = 0;
						}
						ForwardPath_container[label_index].augment_ornot = false;
					}
				}
				else
				{
					//���������Ҫɾ��
					Delete_pathindex(FORWARD, k, i);
					Delete_container(FORWARD, label_index);
					ForwardPath_container[label_index].exist_state = 0;
					i = i - 1;
				}
#endif

			}
		}
	}
	//����
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
					//����ngpath��ngpath_num��passnode_ngpath
					generate_ngpath_Byroute(BackwardPath_container[label_index]);
					//����feasible_extensions_byngpath��augment_ornot
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
				//ʣ����Щ��elementary
				//����ngpath�Ƿ����㣬�����������£�
				if (true == Check_ngpath_Byroute(BackwardPath_container[label_index]))
				{
					//����Hpoints�ж�extend_state
					if (BackwardPath_container[label_index].surplus_time >= Hpoint_For)
					{
						BackwardPath_container[label_index].extend_state = 1;
						//����feasible_extensions_byngpath��augment_ornot
						for (int j = 0; j < passnode_length; j++)
						{
							BackwardPath_container[label_index].feasible_extensions_byngpath[j] = 0;
						}
						BackwardPath_container[label_index].augment_ornot = false;
					}
				}
				else
				{
					//���������Ҫɾ��
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
	//����passnode_ngpath
	for (int i = 0; i < passnode_length; i++)
	{
		objlabel.passnode_ngpath[i] = 0;
	}
	//��һ��customer
	Insert_toPassnode(Mapping_fromRN_toMN[objlabel.customeList[0]], objlabel.passnode_ngpath);
	//֮���customer
	for (int i = 1; i < objlabel.customeList_num; i++)
	{
		next_modifynode = Mapping_fromRN_toMN[objlabel.customeList[i]];
		for (int j = 0; j < passnode_length; j++)
		{
			objlabel.passnode_ngpath[j] = objlabel.passnode_ngpath[j] & neighbourhood_passnode[next_modifynode][j];
		}
		//�������һ����
		Insert_toPassnode(next_modifynode, objlabel.passnode_ngpath);
	}
	//ngpath
	//����ngpath
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
	//����passnode_ngpath
	for (int i = 0; i < passnode_length; i++)
	{
		objlabel.passnode_ngpath[i] = 0;
	}
	//��һ��customer
	Insert_toPassnode(Mapping_fromRN_toMN[objlabel.customeList[0]], objlabel.passnode_ngpath);
	//֮���customer
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
		//�������һ����
		Insert_toPassnode(next_modifynode, objlabel.passnode_ngpath);
	}
	//ngpath
	//����ngpath
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
		//�ܹ��γɳ���·���Ľڵ�
		if (FoundPath_container[i].Reducedcost+ MINDOUBLE<MAXNUM)
		{
			forLabel_index = FoundPath_container[i].pre_label;
			backLabel_index = FoundPath_container[i].succ_label;
			//�ϲ�·��
			FoundPath_container[i].nodesNum = ForwardPath_container[forLabel_index].customeList_num + BackwardPath_container[backLabel_index].customeList_num + 2; //����û�������յ�
			FoundPath_container[i].nodes[0] = p.Customer_Num+current_depot;
			//ǰ��
			for (int j = 0; j < ForwardPath_container[forLabel_index].customeList_num; j++)
				FoundPath_container[i].nodes[1 + j] = ForwardPath_container[forLabel_index].customeList[j];
			//����
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
	//�ϲ�·��
	FoundPath_container[0].nodesNum = ForwardPath_container[forLabel_index].customeList_num + BackwardPath_container[backLabel_index].customeList_num + 2; //����û�������յ�
	FoundPath_container[0].nodes[0] = p.Customer_Num + current_depot;;
	//ǰ��
	for (int i = 0; i < forlabel.customeList_num; i++)
		FoundPath_container[0].nodes[1 + i] = ForwardPath_container[forLabel_index].customeList[i];
	//����
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
	//����׼һ��RCֵ��ͬ
	for (int i=0;i<p.Customer_Num;i++)
	{
		if (FoundPath_container[i].Reducedcost>-Conf::Exact_threshold)continue;

		for (int j = i+1; j<p.Customer_Num; j++)
		{
			if (fabs(FoundPath_container[i].Reducedcost - FoundPath_container[j].Reducedcost)<-0.01*reduce_cost)
			{
				//�ظ���·��
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
	//���������������ڲ���neighbourhood
	ng_clear();
	int i,j,cur_customer;

	//�ҵ�dssr_ngSet�г�����Mapping_Reduced�е�ngset
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
	//תΪ�����ƣ�����neighbourhood_passnode��
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
	//ת��neighbourhood�õ�ng_memory_passnode
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
	//���մ�С�����ҵ�����λ��
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
		cout << "ERROR: SDCnode��������" << endl;
		cin.get();
	}
	//����ǰɾ��ĩβ�ڵ�
	objlabel.SDCnode.pop_back();
	//������SDCnode�ĵ�inposi��λ��
	objlabel.SDCnode.insert(objlabel.SDCnode.begin() + inposi, customer_no);
	objlabel.SDCnode_num = objlabel.SDCnode_num + 1;

	temp_modify = - lp.SDC_dual[customer_no];
	return  temp_modify;
}

void Subproblem::Insert_intoVector(int insert_posi, int customer_no, Path & objlabel)
{
	//����ֵ
	for (int i = objlabel.SDCnode_num; i>insert_posi; i--)
	{
		objlabel.SDCnode[i] = objlabel.SDCnode[i - 1];
	}
	//�����
	objlabel.SDCnode[insert_posi] = customer_no;
}

float Subproblem::Calculate_RCincrement(Path & better_label, Path & dominated_label, RMP & lp)
{
	float temp_RCincrement = 0;

#if Frameworks == 1
	//�ҵ�better_label.SDCnode��dominated_label.SDCnode�Ĳ
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
	//�ҵ�forlabel.SDCnode��backlabel.SDCnode���ظ���
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
		//�ڵ�i��Cuts_src��Ӧ��state�У�better_label��state_srcҪ�ϸ����dominated_label��state_src�����ۻ�һ�ζ�ż����
		if (better_label.state_src[i]>dominated_label.state_src[i]+ MINDOUBLE)
		{
			diff = diff + lp.SRC_dual[i];
		}
	}

	return diff;
}

bool Subproblem::DP_Ng_Cuts_Dssr_Stable(BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc)
{
	//////////////////////////////////////////////////////////////////��ʼ����ʼ////////////////////////////////////////////////////////////////////////////////////////////////////
	//cout << "��������";
	long start = clock();
	//Ԥ��������
	//����1������ڵ������reduced costΪ������ô�ýڵ�һ�����������reduced cost��С��·�������Ž⣩�ϡ�
	//���ǣ�������ǵ�RMP�ļ����ԣ�reduced costΪ���Ľڵ�Ӧ�ñ����������·�����Ϊ��������ԣ�Ҫ�󾭹�ָ���Ľڵ㣩�����£�reduced cost��С�������������ҪԤ������ռ䣩
	//����2���������һ���������㣺�û���Ȩ��+����β�ڵ��reduced costΪ��ʱ����ô�û����п��������·�ϡ�
	//Ȼ�����������п�����ĳ��state(һ��·��)�б�֧�䣻
	//���ߣ������û���β����ص���������ϵͳ�����㣺�����ýڵ��·���ᱻ֧�䣬��ô�����иû���β����Ա�������������������������ϸ񣬹���Ԥ������ռ䣩
	//�ٻ��ߣ�����û����ɵ���������ϵͳ�����㣺�߸û���·���ᱻ֧�䣬��ô�����иû����Ա��������������������������ǰһ�����ɣ�����Ԥ������ռ䣩

	//�Ե�ǰ������
	//Get_inverseBasis(temp_op, routeset);
	Initial_Hpoints(p);			//���ֽ�����½縳ֵ
	//Ԥ�������ɵ�ǰPSP�µ�ngset
#if DSSRNG+AUGMENT >0	//��EPO-dssr����CPA-ng����¶���Ҫ��ngset������չ
	Set_dssr_ngset(p);
#else
	Ini_ngset(BB, lp, p);
#endif
		
	//////////////////////////////////////////////////////////////////DSSR���////////////////////////////////////////////////////////////////////////////////////////////////////
	int iter_count = 0;
	do
	{
		//���ɳ�ʼ�У����ɹ����е���֧��
		Initial_lables(BB,lp, p, Cuts_src, Cuts_sdc);  //�������й���reduced cost
		//labeling���̣���Ҫ����ΪSearch strategy��Dominance rule
		DP_Labeling(FORWARD, BB,lp, p, Cuts_src, Cuts_sdc);	//������չ
		DP_Labeling(BACKWARD, BB,lp, p, Cuts_src, Cuts_sdc);	//������չ
		////////////////////////////////////////////////////////�ϲ��⣬�ϲ����������/////////////////////////////////////////////////////////////////////////////////////////////
		//��DSSR-NG����µĺϲ���ngset����չĿ��ֻ��Ϊ��ʹǰ����ߺ����label�����ֻ�(elementary)
		//DSSR-NG�����ngset��չ����Ϊ��ǰ����ߺ���label�л����Һϲ���RCֵС��0
		//DSSR-NG����ֹ����Ϊ��ǰ��ͺ���label�޻����Һϲ���Ϊelementary������С��RCֵ���ڵ���0
		if (true == combinebidirection_kcycle_ngpath_cuts_stable(BB, lp, p, Cuts_src))
		{
			//Ҫô�ҵ���RC<-Conf::Exact_threshold��elementary·��
			//Ҫôǰ�����޻���label�ܹ��γɵ�elementary·����RC��>-Conf::Exact_threshold
			break;
		}
		else
		{
			//�ȸ���Completion Bound
			//��ΪֻҪ���½磬��˲����ں�ngset���º�label�Ƿ����
			if (true==exact_ornot)
			{
				Update_compleBound(p);
			}
			//���ݷ�elementary��·������ngset������չ
			//���ҵ�Shortest_path�еĻ�������ngset
			Update_ngset(p);
			//��Σ�����ForwardPath_container��BackwardPath_container�з�elementary��label��
			//��һ��ɾ������ngset�󲻿��е�label
			//�ڶ������µ�label��������չ��������extend_state=1
			Label_update(p);
			//Label_clear(p);
		}
		iter_count++;
	} while (1);
	////////////////////////////////////////////////////////////////////����//////////////////////////////////////////////////////////////////////////////////////////////////////
	//���ȹ���FoundPath_container
	Construct_FoundPath(p);
#if FOUND==1
	//�ٶ�FoundPath_container���д���:ɾ���ظ���
	Delete_duplicate(p);
#endif

	long end = clock();
	//PSPָ��
	PSP_time = float(end - start) / 1000;
	all_label_num = ForwardPath_container_num + BackwardPath_container_num;
	cout << "�����⻨�ѣ�" << PSP_time << "��    ||     ";
	cout <<"RCֵ��" << reduce_cost << endl;
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
	//�ȹ�������
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
	//��ʼ��һ��Ѳ���
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
	//�ع�ӳ���ϵ��FeasiExten_Index
	Build_Mapping_simple(BB, lp, p);
	Feasible_Extension_simple(BB, lp, p);
	Reset_Maxtime(BB, lp, p);
	//��ʼlabeling
	Initial_Hpoints(p);			//���ֽ�����½縳ֵ
	//Ԥ�������ɵ�ǰPSP�µ�ngset
	Ini_ngset_simple(BB, lp, p);
	//���ɳ�ʼ�У����ɹ����е���֧��
	Initial_lables_simple(BB, lp, p);  //�������й���reduced cost
	//labeling���̣���Ҫ����ΪSearch strategy��Dominance rule
	if (false == DP_Labeling_simple(FORWARD, BB, lp, p))		//������չ
	{
		p.Max_arrivaltime[current_depot] = temp_Max_arrivaltime;
		return true;
	}
	if (false == DP_Labeling_simple(BACKWARD, BB, lp, p))		//������չ
	{
		p.Max_arrivaltime[current_depot] = temp_Max_arrivaltime;
		return true;
	}
	////////////////////////////////////////////////////////�ϲ��⣬�ϲ����������/////////////////////////////////////////////////////////////////////////////////////////////
	//��ֹ����Ϊ��ֻҪ����һ������subset���е��·�����ͷ���true
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

	//����FeasiExten_Index
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
	//���´ӳ�վ�����ܹ������customer
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
	int ordered_num;	//neighbourhood���Ѿ�����õ�����
	bool check;
	//���ݶ�ż�������ҵ�i�ڵ�����������NEIGHBOURHOOD_NUM���ڵ�,����i�ڵ�һ����neighbourhood��(����)
	ng_clear();

	//ȷ��ng�Ĵ�С
	int temp_ngnum=fmin(5, Modifiednetwork_Nodes_num);
	//����ÿ��customer��ngset��pr06һ������0.001s
	//����RCֵ��������
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
				//ǰ����������
				for (k = ordered_num - 1; k > temp_posi; k--)
				{
					neighbourhood[i][k] = neighbourhood[i][k - 1];
				}
				//�ٸ�ֵ
				neighbourhood[i][temp_posi] = j;
			}
		}
		neighbourhood_num[i] = ordered_num;
	}

	//���i�ڵ��Ƿ���i�ڵ�������У�������������ĩβλ��
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
		if (true == check)//i�ڵ㲻��i�ڵ��������
		{
			neighbourhood[i][neighbourhood_num[i] - 1] = i;
		}
	}
	//תΪ�����ƣ�����neighbourhood_passnode��
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
	//ת��neighbourhood�õ�ng_memory_passnode
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

	//����չ�����У�����label����node_id�洢��������p.Customer_Num��Bucket
	//��ÿ��customer�д洢��label���Ѿ���չ��������
	//��ÿ��Mapping_Reduced�еĵ�
	/////////////////////////////////////�����ǩ////////////////////////////////////////////////////
	for (i = 0; i<BackfeasiExten_Index_num[p.Customer_Num]; i++)
	{
		//�ͻ�
		temp_customer = BackfeasiExten_Index[p.Customer_Num][i];
		if (BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot]>MAXNUM - 1) continue;
		//��չ�����Ӧ��
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//ȷ������BackwardPath_container��λ��
		temp_back_posi = Seek_containerIndex(BACKWARD);
		//������BackwardPath_container��temp_back_posiλ�ù�������
		BackwardPath_container[temp_back_posi].exist_state = 0;
		BackwardPath_container[temp_back_posi].extend_state = 1;
		//��չ��������:�ӽ���㵽������
		BackwardPath_container[temp_back_posi].customeList[0] = temp_customer;
		BackwardPath_container[temp_back_posi].customeList_num = 1;
		//������Դ��Ϣ
		BackwardPath_container[temp_back_posi].availablecapacity = p.Veh.Veh_Capacity - p.Allnode[temp_customer].demand;
		BackwardPath_container[temp_back_posi].accu_duration = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot] + p.Allnode[temp_customer].servicetime;//ʱ�������Ҫת��
		BackwardPath_container[temp_back_posi].consumed_time = max(BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot], p.Max_arrivaltime[current_depot] - p.Allnode[temp_customer].endTW - p.Allnode[temp_customer].servicetime);//ʱ�������Ҫת��
		BackwardPath_container[temp_back_posi].surplus_time = p.Max_arrivaltime[current_depot] - BackwardPath_container[temp_back_posi].consumed_time;
		if (BackwardPath_container[temp_back_posi].surplus_time<Hpoint_For)
		{
			BackwardPath_container[temp_back_posi].extend_state = 0;
		}
		//��չʱ��·����RCֱֵ�Ӱ���temp_customer���е�node
		BackwardPath_container[temp_back_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_customer][p.Customer_Num + current_depot]  - temp_dual[temp_customer] - fix_cost;
		BackwardPath_container[temp_back_posi].augment_ornot = false;

		//���þ����Ľڵ�
		BackwardPath_container[temp_back_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			BackwardPath_container[temp_back_posi].passnode[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_2cycle[j] = 0;
			BackwardPath_container[temp_back_posi].passnode_ngpath[j] = 0;
			BackwardPath_container[temp_back_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary�йص�
		BackwardPath_container[temp_back_posi].elementaty = true;
		BackwardPath_container[temp_back_posi].prenode = -1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode);
		//ngpath�йص�
		BackwardPath_container[temp_back_posi].ngpath[BackwardPath_container[temp_back_posi].ngpath_num] = modify_Node;
		BackwardPath_container[temp_back_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, BackwardPath_container[temp_back_posi].passnode_ngpath);

		incre_deleted = Delelted_Backwardindex_num;	//��ʼ��
		//������ж��Ƿ�֧��
		if (false == domination_2cycle_ngpath_cuts_simple(BACKWARD, temp_customer, lp, BackwardPath_container[temp_back_posi]))
		{
			incre_deleted = Delelted_Backwardindex_num - incre_deleted;
			//�������֧�䣬�����BackwardBucket[temp_customer]��
			//������BackwardPath_container��ȷ�����
			BackwardPath_container[temp_back_posi].exist_state = 1;
			//Ȼ�����BackwardPath_container��Delelted_Backwardindex�е����
			Add_container(BACKWARD, temp_back_posi, incre_deleted);
			//�����ӵ�BackwardBucket[temp_customer]��
			BackwardBucket[temp_customer].Path_index[BackwardBucket[temp_customer].Path_Num] = temp_back_posi;
			BackwardBucket[temp_customer].Path_Num = BackwardBucket[temp_customer].Path_Num + 1;
		}
	}
	/////////////////////////////////////ǰ���ǩ////////////////////////////////////////////////////
	for (i = 0; i < ForfeasiExten_Index_num[p.Customer_Num]; i++)
	{
		//�ͻ�
		temp_customer = ForfeasiExten_Index[p.Customer_Num][i];
		if (BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer]>MAXNUM - 1) continue;
		//��չ�����Ӧ��
		modify_Node = Mapping_fromRN_toMN[temp_customer];
		//ȷ������ForwardPath_container��λ��
		temp_for_posi = Seek_containerIndex(FORWARD);
		//��ForwardPath_container��temp_for_posiλ�ù�������
		ForwardPath_container[temp_for_posi].exist_state = 0;
		ForwardPath_container[temp_for_posi].extend_state = 1;
		//��չ��������:�ӽ���㵽������
		ForwardPath_container[temp_for_posi].customeList[0] = temp_customer;
		ForwardPath_container[temp_for_posi].customeList_num = 1;
		//��Դ����
		ForwardPath_container[temp_for_posi].usedcapacity = p.Allnode[temp_customer].demand;
		ForwardPath_container[temp_for_posi].accu_duration = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] + p.Allnode[temp_customer].servicetime;  //ʱ�������Ҫת��
		ForwardPath_container[temp_for_posi].arrivaltime = max(BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer], p.Allnode[temp_customer].startTW); //ʱ�������Ҫת��

		if (ForwardPath_container[temp_for_posi].arrivaltime > Hpoint_For)
		{
			ForwardPath_container[temp_for_posi].extend_state = 0;
		}
		//��չʱ��·����RCֱֵ�Ӱ���temp_customer���е�node
		ForwardPath_container[temp_for_posi].RC_value = BB.branch[BB.cur_node].CostNetwork_Branch[p.Customer_Num + current_depot][temp_customer] - temp_dual[temp_customer] - fix_cost;
		ForwardPath_container[temp_for_posi].augment_ornot = false;

		//���þ����Ľڵ�
		ForwardPath_container[temp_for_posi].ngpath_num = 0;
		for (j = 0; j < passnode_length; j++)
		{
			ForwardPath_container[temp_for_posi].passnode[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_2cycle[j] = 0;
			ForwardPath_container[temp_for_posi].passnode_ngpath[j] = 0;
			ForwardPath_container[temp_for_posi].feasible_extensions_byngpath[j] = 0;
		}
		//elementary�йص�
		ForwardPath_container[temp_for_posi].elementaty = true;
		ForwardPath_container[temp_for_posi].prenode = -1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode);
		//ngpath�йص�
		ForwardPath_container[temp_for_posi].ngpath[ForwardPath_container[temp_for_posi].ngpath_num] = modify_Node;
		ForwardPath_container[temp_for_posi].ngpath_num = 1;
		Insert_toPassnode(modify_Node, ForwardPath_container[temp_for_posi].passnode_ngpath);

		incre_deleted = Delelted_Forwardindex_num;	//��ʼ��
		//������ж��Ƿ�֧��
		if (false == domination_2cycle_ngpath_cuts_simple(FORWARD, temp_customer, lp, ForwardPath_container[temp_for_posi]))
		{
			incre_deleted = Delelted_Forwardindex_num - incre_deleted;
			//�������֧�䣬�����ForwardBucket[temp_customer]��
			//������ForwardPath_container��ȷ�����
			ForwardPath_container[temp_for_posi].exist_state = 1;
			//Ȼ�����ForwardPath_container��Delelted_Forwardindex�е����
			Add_container(FORWARD, temp_for_posi, incre_deleted);
			//�����ӵ�ForwardBucket[temp_customer]��
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

	//�����Ƕ�ÿ��customer�ϵ�label��������չ
	//��ֹ������û��customer�ϴ��ڿ�����չ��label��
	do
	{
		solved_customers = 0;
		for (temp_itor = 0; temp_itor<p.Customer_Num; temp_itor++)
		{
			//ȷ��ѭ��˳��,������ȷ�����ĸ�label��չ
			//��������ֻȷ�����ĸ�customer��ʼ��չ
			Extend_Customer = Search_Nextcustomer(temp_itor, p);
			//��Extend_Customer�ϵ�����label������չ
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

	//��չ�����е�label��չ���������
	//����֧��ֻ��cutomer�Ϸ�����ͬһ��customer�µ�����label�����洢�ڳ�����
	//��չ����Ҳ�ǣ��ӳ�������չ��ֱ����һ��������ͣ�£�������dominance rule
	if (direction == BACKWARD)
	{
		//�Ƚϵ�cur_customer���յ㣩�ϵ�����·��
		for (int i = 0; i<BackwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = BackwardBucket[cur_customer].Path_index[i];		//BackwardPath_container�е����
																		//��һ��BackwardPath_container[curLabel]һ���ǿ�����չ��
			if (0 == BackwardPath_container[curLabel].extend_state)continue;
			//����ÿ��������չ
			for (int k = 0; k<BackfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//��ȡ�µ�label��λ��
				next_customer = BackfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Backwardindex_num;	//��ʼ��
				temp_check = Extend_Onestep_simple(direction, cur_customer, BackwardPath_container[curLabel], next_customer, BackwardPath_container[nextLabel], BB, lp, p);
				if (1 == temp_check)
				{
					//����չ�ɹ�����Ҫ�ж�֧������
					if (false == domination_2cycle_ngpath_cuts_simple(direction, next_customer, lp, BackwardPath_container[nextLabel]))
					{
						//Delelted_Backwardindex_num����
						incre_deleted = Delelted_Backwardindex_num - incre_deleted;
						//�������֧�䣬�����BackwardBucket[next_customer]��
						//������BackwardPath_container��ȷ�����
						BackwardPath_container[nextLabel].exist_state = 1;
						//Ȼ�����BackwardPath_container��Delelted_Backwardindex�е����
						Add_container(direction, nextLabel, incre_deleted);
						//�����ӵ�BackwardBucket[next_customer]��
						BackwardBucket[next_customer].Path_index[BackwardBucket[next_customer].Path_Num] = nextLabel;
						BackwardBucket[next_customer].Path_Num = BackwardBucket[next_customer].Path_Num + 1;
						//˵����������չ
						solved_ornot = 0;
					}
				}
				else if (2 == temp_check)
				{
					return 2;
				}
			}
			//BackwardPath_container[curLabel]�Ѿ���չ��ϣ�תΪ���ñ�ǩ
			//��������չ
			BackwardPath_container[curLabel].extend_state = 0;
		}
	}
	else
	{
		//�Ƚϵ�cur_customer���յ㣩�ϵ�����·��
		for (int i = 0; i<ForwardBucket[cur_customer].Path_Num; i++)
		{
			curLabel = ForwardBucket[cur_customer].Path_index[i];		//ForwardPath_container�е����
																		//��һ��ForwardPath_container[curLabel]һ���ǿ�����չ��
			if (0 == ForwardPath_container[curLabel].extend_state)continue;
			//����ÿ��������չ
			for (int k = 0; k<ForfeasiExten_Index_num[cur_customer]; k++)
			{
				nextLabel = Seek_containerIndex(direction);//��ȡ�µ�label��λ��
				next_customer = ForfeasiExten_Index[cur_customer][k];
				incre_deleted = Delelted_Forwardindex_num;	//��ʼ��
				temp_check = Extend_Onestep_simple(direction, cur_customer, ForwardPath_container[curLabel], next_customer, ForwardPath_container[nextLabel], BB, lp, p);
				if (1 == temp_check)
				{
					//����չ�ɹ�����Ҫ�ж�֧������
					if (false == domination_2cycle_ngpath_cuts_simple(direction, next_customer, lp, ForwardPath_container[nextLabel]))
					{
						//Delelted_Forwardindex_num����
						incre_deleted = Delelted_Forwardindex_num - incre_deleted;
						//�������֧�䣬�����ForwardBucket[next_customer]��
						//������ForwardPath_container��ȷ�����
						ForwardPath_container[nextLabel].exist_state = 1;
						//Ȼ�����ForwardPath_container��Delelted_Forwardindex�е����
						Add_container(direction, nextLabel, incre_deleted);
						//�����ӵ�ForwardBucket[next_customer]��
						ForwardBucket[next_customer].Path_index[ForwardBucket[next_customer].Path_Num] = nextLabel;
						ForwardBucket[next_customer].Path_Num = ForwardBucket[next_customer].Path_Num + 1;
						//˵����������չ
						solved_ornot = 0;
					}
				}
				else if(2== temp_check)
				{
					return 2;
				}
			}
			//ForwardPath_container[curLabel]�Ѿ���չ��ϣ�תΪ���ñ�ǩ
			//��������չ
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

		//�����ж��Ƿ��γ�1-cycle��
		if (currentCus == nextCus)return 0;

		//�����Ƿ���γ�k-cycle
		if (true == if_2cycle(curlabel, next_modifyNode))return 0;

		//Ȼ������ng֧��
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return 0;
		}
		//����ؼ���Դ�Ƿ񳬹��ؼ���,�Լ���Դ����
		if (false == satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return 0;
		//�����Ƿ���γ�ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return 0;

		//·�����ٿ�����չ��nextlabel
		//�ȳ�ʼ��
		nextlabel.augment_ornot = false;

		//�й���Դ�ĸ��¶��ں���satisfying_resource_constrains��

		//����customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num] = nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//������ngpath�й�����
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//������kcycle�йص�
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];

		//������elementary�йص�
		nextlabel.copy_passnode_simple(curlabel);
		nextlabel.update_passnode_kcycle_simple(Conf::CYCLE_NUM, next_modifyNode);

		//����޸ĳɱ�(reduced cost)
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
		//�����ж��Ƿ��γ�1-cycle��
		if (currentCus == nextCus)return 0;

		//�����Ƿ���γ�k-cycle
		if (true == if_2cycle(curlabel, next_modifyNode))return 0;

		//Ȼ������ng֧��
		if (true == curlabel.augment_ornot)
		{
			if (false == belong_toset(next_modifyNode, curlabel.feasible_extensions_byngpath))return 0;
		}
		//����ؼ���Դ�Ƿ񳬹��ؼ���,�Լ���Դ����
		if (false == satisfying_resource_constrains(direction, currentCus, curlabel, nextCus, nextlabel, BB, p))return 0;
		//�����Ƿ���γ�ng-path
		if (true == if_ng_path(next_modifyNode, curlabel))return 0;

		//·�����ٿ�����չ��nextlabel
		//�ȳ�ʼ��
		nextlabel.augment_ornot = false;

		//�й���Դ�ĸ��¶��ں���satisfying_resource_constrains��

		//����customeList
		nextlabel.Copy_Customer(curlabel);
		nextlabel.customeList[nextlabel.customeList_num] = nextCus;
		nextlabel.customeList_num = nextlabel.customeList_num + 1;

		//������ngpath�й�����
		nextlabel.copy_passnode_ngpath(curlabel);
		nextlabel.update_passnode_ngpath(neighbourhood_passnode[next_modifyNode], next_modifyNode, curlabel);

		//������kcycle�йص�
		nextlabel.prenode = Mapping_fromRN_toMN[currentCus];

		//������elementary�йص�
		nextlabel.copy_passnode_simple(curlabel);
		nextlabel.update_passnode_kcycle_simple(Conf::CYCLE_NUM, next_modifyNode);

		//����޸ĳɱ�(reduced cost)
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
	int next_Labelindex;							//container�е���һ��label�����
	bool controled = false;							//����·�Ƿ�֧�䣬true��֧�䣬false����֧��
	bool ini_ornot = true;							//����Ҫ��ʼ��ģ��
	bool complete_donimated_byNgpath = false;

	if (direction == FORWARD)//����
	{
		///////////////////////////////////////////////////////////////////���ж�����·nextV�Ƿ�ForwardBucket[nextcustomer]��ĳ����·֧��////////////////////////////////////////////////////////////////////////////
		//�Ƚϵ����յ�Ϊnextcustomer������·��
		for (k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�ForwardBucket[nextcustomer]ÿ��label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			//·����֧���ϵΪ����P1·���ľ��������cycle_eli_num���ڵ㼯��ΪS1��P2·�����������cycle_eli_num���ڵ㼯��ΪS2
			//S1��Ӧ����Դ������ΪR1(�����ɱ�)��S2��Ӧ����Դ������ΪR2�������ɱ���
			//���1��S1��S2���Ӽ������R1<=R2��P1֧��P2
			//���2�������������һ��·��P3���յ���P1��P2��ͬ�������㣺1) S2�Ĳ����ǣ�S1�Ĳ�������S3�Ĳ��������Ӽ���2��R1<=R2��R3<=R2 ,��ôP2���Ա�֧��

			//���ǵ�·�������ڵ�һ��Ϊnextcustomer
			//���ȿ���ForwardPath_container[next_Labelindex]�Ƿ�ΪnextV���Ӽ�
			//if (ForwardPath_container[next_Labelindex].nodenum>nextV.nodenum)continue;		//���ForwardPath_container[next_Labelindex]�Ľڵ���������·�󣬼���ѭ��
			//�����ҵ�֧��nextV��ForwardPath_container[next_Labelindex],֧������ʹ�����ɵ�SPPRC׼��
			if (ForwardPath_container[next_Labelindex].usedcapacity>nextV.usedcapacity)continue;
			if (ForwardPath_container[next_Labelindex].arrivaltime>nextV.arrivaltime/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (ForwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif
			if (ForwardPath_container[next_Labelindex].RC_value >nextV.RC_value + MINDOUBLE)continue;

			//��������������˵��ForwardPath_container[next_Labelindex]���ٿ��Բ���֧��nextV
			//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ForwardPath_container[next_Labelindex]��ng-path��nextV��ng-path���Ӽ�����ForwardPath_container[next_Labelindex]֧��nextV
			//ng-path
			//���ȣ��ҵ�ForwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ���
			for (i = 0; i < ForwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//�ҵ�ForwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬�����ȫ֧��
							controled = true;				//�����������������������·��֧��
							return controled;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//�����������������������·��֧��
				return controled;
			}
			//����ʽ��2cycle֧�����
			if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				//���ж�ForwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ��Ƿ���ͬ
				//�����ͬ����nextV��֧��
				controled = true;
				return controled;
			}
			else
			{
				//ForwardBucket[nextcustomer]�����ٴ�������ForwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ㲻ͬ��
				//������ForwardPath_container[next_Labelindex]��prenodeҲ����ͬ
				if (ForwardPath_container[next_Labelindex].prenode != store_prenode)
				{
					if (1 == node_num)
					{
						//����Ѿ��ҵ���������������ForwardPath_container[next_Labelindex]����nextV��֧��
						controled = true;
						return controled;
					}
					else
					{
						//��¼�ҵ���ForwardPath_container[next_Labelindex]
						store_prenode = ForwardPath_container[next_Labelindex].prenode;
						node_num = node_num + 1;
					}
				}
			}
		}
		////////////////////////////////////////////////////////���ж�ForwardBucket[nextcustomer]��ĳ����·�Ƿ�����·nextV֧��////////////////////////////////////////////////////////
		for (k = 0; k < ForwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�ForwardBucket[nextcustomer]ÿ��label
			next_Labelindex = ForwardBucket[nextcustomer].Path_index[k];
			if (0 == ForwardPath_container[next_Labelindex].exist_state)
			{
				continue;//���ForwardPath_container[next_Labelindex]�Ѿ���֧����ˣ���û�б�Ҫȥ����
			}
			//�����ҵ���nextV֧���ForwardPath_container[next_Labelindex]��ʹ��SPPRC֧��׼��
			//���nextV��֧��ForwardPath_container[next_Labelindex]������
			if (nextV.usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
			if (nextV.arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
			if (nextV.RC_value>ForwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;

			//��������������˵��nextV���ٿ��Բ���֧��ForwardPath_container[next_Labelindex]
			//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ng-path
			//��ΪForwardPath_container[next_Labelindex].feasible_extensions_byngpath�Ѿ������¹�������ֻ��Ҫ���nextV��ForwardPath_container[next_Labelindex]�ľֲ�֧���ϵ
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//�ҵ�nextV.passnode_ngpath��ForwardPath_container[next_Labelindex].passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬����ForwardPath_container[next_Labelindex]����ȫ֧��
							//ɾ��ForwardBucket[nextcustomer].Path_index[k]���;
							Delete_pathindex(direction, nextcustomer, k);
							//ɾ��ForwardPath_container��Ԫ�أ�����¼��Delelted_Forwardindex��
							Delete_container(direction, next_Labelindex);
							//�ȱ�עForwardPath_container��exist_state״̬
							ForwardPath_container[next_Labelindex].exist_state = 0;
							//ע�������
							k = k - 1;
							break;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
			if (true == complete_donimated_byNgpath)
			{
				//ͬ�ϣ��ȴ���container,�ٴ�������
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}

			if (0 == ForwardPath_container[next_Labelindex].exist_state)continue;

			//����ʽ��2cycle֧�����
			//�����֧�䣺
			//���ж�nextV�����ڶ��ڵ��Ƿ��ForwardPath_container[next_Labelindex]�ĵ����ڶ����ڵ���ͬ
			if (ForwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				ForwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}
			else
			{
				//����֪����nextV�Ѿ�SPPRC֧��ForwardPath_container[next_Labelindex]
				//���ж��Ƿ�������������·����nextV���֧��ForwardPath_container[next_Labelindex]
				//������ڣ���ForwardPath_container[next_Labelindex]��֧��
				for (i = 0; i < ForwardBucket[nextcustomer].Path_Num; i++)
				{
					int temp_label = ForwardBucket[nextcustomer].Path_index[i];
					if (0 == ForwardPath_container[temp_label].exist_state)
					{
						continue;//���ForwardPath_container[temp_label]�Ѿ���֧����ˣ���û�б�Ҫȥ����
					}
					//�ҵ�֧��ForwardPath_container[next_Labelindex]��ForwardPath_container[temp_label]
					if (ForwardPath_container[temp_label].usedcapacity>ForwardPath_container[next_Labelindex].usedcapacity)continue;
					if (ForwardPath_container[temp_label].arrivaltime>ForwardPath_container[next_Labelindex].arrivaltime)continue;
#if DURATIONORNOT == 1
					if (ForwardPath_container[temp_label].accu_duration>ForwardPath_container[next_Labelindex].accu_duration)continue;
#endif
					if (ForwardPath_container[temp_label].RC_value>ForwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;

					//ForwardPath_container[temp_label]ҲSPPRC֧��ForwardPath_container[next_Labelindex]
					//ForwardPath_container[temp_label]��nextV�ĵ����ڶ����ڵ㲻ͬ
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
	else//����
	{
		///////////////////////////////////////////////////////////////////���ж�����·nextV�Ƿ�ĳ����·֧��////////////////////////////////////////////////////////////////////////////
		//�Ƚϵ����յ�Ϊnextcustomer������·��
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�BackwardBucket[nextcustomer]ÿ��label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			//ԭ��ͬ����
			//�����ҵ�֧��nextV��BackwardPath_container[next_Labelindex],֧������ʹ�����ɵ�SPPRC׼��
			if (BackwardPath_container[next_Labelindex].availablecapacity <nextV.availablecapacity)continue;	//���curS��ĵ�k����·��ʣ��ռ������·С������ѭ��
			if (BackwardPath_container[next_Labelindex].surplus_time<nextV.surplus_time/*-0.0001*/)continue;
#if DURATIONORNOT == 1
			if (BackwardPath_container[next_Labelindex].accu_duration>nextV.accu_duration/*-0.0001*/)continue;
#endif	
			if (BackwardPath_container[next_Labelindex].RC_value >nextV.RC_value + MINDOUBLE)continue;//���curS��ĵ�k����·��Ŀ��ֵ������·���ã�����ѭ��
			

			//��������������˵��BackwardPath_container[next_Labelindex]���ٿ��Բ���֧��nextV
			//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ng-path
			//���ȣ��ҵ�BackwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ���
			for (i = 0; i < BackwardPath_container[next_Labelindex].ngpath_num; i++)
			{
				//�ҵ�BackwardPath_container[next_Labelindex].passnode_ngpath��nextV.passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬�����ȫ֧��
							controled = true;				//�����������������������·��֧��
							return controled;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
			if (true == complete_donimated_byNgpath)
			{
				controled = true;				//�����������������������·��֧��
				return controled;
			}

			//����ʽ��2cycle֧�����
			if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				//���ж�BackwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ��Ƿ���ͬ
				//�����ͬ����nextV��֧��
				controled = true;
				return controled;
			}
			else
			{
				//BackwardBucket[nextcustomer]�����ٴ�������BackwardPath_container[next_Labelindex]��nextV�����ڶ����ڵ㲻ͬ��
				//������BackwardPath_container[next_Labelindex]��prenodeҲ����ͬ
				if (BackwardPath_container[next_Labelindex].prenode != store_prenode)
				{
					if (1 == node_num)
					{
						//����Ѿ��ҵ���������������BackwardPath_container[next_Labelindex]����nextV��֧��
						controled = true;
						return controled;
					}
					else
					{
						//��¼�ҵ���BackwardPath_container[next_Labelindex]
						store_prenode = BackwardPath_container[next_Labelindex].prenode;
						node_num = node_num + 1;
					}
				}
			}
		}
		////////////////////////////////////////////////////////���ж�BackwardPath_container[temp_nextcus]��ĳ����·�Ƿ�����·nextV֧��////////////////////////////////////////////////////////
		for (k = 0; k < BackwardBucket[nextcustomer].Path_Num; k++)
		{
			//�ҵ�BackwardBucket[nextcustomer]ÿ��label
			next_Labelindex = BackwardBucket[nextcustomer].Path_index[k];
			if (0 == BackwardPath_container[next_Labelindex].exist_state)continue;//���BackwardPath_container[next_Labelindex]�Ѿ���֧����ˣ���û�б�Ҫȥ����
																				  //�����ҵ���nextV֧���BackwardPath_container[next_Labelindex]��ʹ��SPPRC֧��׼��
																				  //���nextV��֧��BackwardPath_container[next_Labelindex]������
			if (nextV.availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
			if (nextV.surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
			if (nextV.accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif	
			if (nextV.RC_value >BackwardPath_container[next_Labelindex].RC_value + MINDOUBLE)continue;


			//��������������˵��nextV���ٿ��Բ���֧��BackwardPath_container[next_Labelindex]
			//multi-ng֧��
			complete_donimated_byNgpath = true;
			//ng-path
			//if (false == nextV.check_subset(nextV.passnode_ngpath, BackwardPath_container[next_Labelindex].passnode_ngpath))continue;
			//��ΪBackwardPath_container[next_Labelindex].feasible_extensions_byngpath�Ѿ������¹�������ֻ��Ҫ���nextV��BackwardPath_container[next_Labelindex]�ľֲ�֧���ϵ
			for (i = 0; i < nextV.ngpath_num; i++)
			{
				//�ҵ�nextV.passnode_ngpath��BackwardPath_container[next_Labelindex].passnode_ngpath�Ĳ�����ÿ��Ԫ��
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
							//�������֧�䣬����BackwardPath_container[next_Labelindex]����ȫ֧��
							//ɾ��BackwardBucket[nextcustomer].Path_index[k]���;
							Delete_pathindex(direction, nextcustomer, k);
							//ɾ��BackwardPath_container��Ԫ�أ�����¼��Delelted_Backwardindex��
							Delete_container(direction, next_Labelindex);
							//�ȱ�עBackwardPath_container��exist_state״̬
							BackwardPath_container[next_Labelindex].exist_state = 0;
							//��ɾ��������Ԫ��,ע��������뿼�ǵ�������������
							k = k - 1;
							break;
						}
					}
				}
			}
			//�ж��Ƿ���ȫ֧��
			if (true == complete_donimated_byNgpath)
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				BackwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}

			if (0 == BackwardPath_container[next_Labelindex].exist_state)continue;

			//����ʽ��2cycle֧�����
			//�����֧�䣺
			//���ж�nextV�����ڶ��ڵ��Ƿ��BackwardPath_container[next_Labelindex]�ĵ����ڶ����ڵ���ͬ
			if (BackwardPath_container[next_Labelindex].prenode == nextV.prenode)
			{
				Delete_pathindex(direction, nextcustomer, k);
				Delete_container(direction, next_Labelindex);
				BackwardPath_container[next_Labelindex].exist_state = 0;
				k = k - 1;
			}
			else
			{
				//����֪����nextV�Ѿ�SPPRC֧��BackwardPath_container[next_Labelindex]
				//���ж��Ƿ�������������·����nextV���֧��BackwardPath_container[next_Labelindex]
				//������ڣ���nextV��֧��
				for (i = 0; i < BackwardBucket[nextcustomer].Path_Num; i++)
				{
					int temp_label = BackwardBucket[nextcustomer].Path_index[i];
					if (0 == BackwardPath_container[temp_label].exist_state)continue;//���BackwardPath_container[temp_label]�Ѿ���֧����ˣ���û�б�Ҫȥ����
																						//�ҵ�֧��BackwardPath_container[next_Labelindex]��BackwardPath_container[temp_label]
					if (BackwardPath_container[temp_label].availablecapacity<BackwardPath_container[next_Labelindex].availablecapacity)continue;
					if (BackwardPath_container[temp_label].surplus_time<BackwardPath_container[next_Labelindex].surplus_time)continue;
#if DURATIONORNOT == 1
					if (BackwardPath_container[temp_label].accu_duration>BackwardPath_container[next_Labelindex].accu_duration)continue;
#endif
					if (BackwardPath_container[temp_label].RC_value >BackwardPath_container[next_Labelindex].RC_value - MINDOUBLE)continue;

					//BackwardPath_container[temp_label]ҲSPPRC֧��BackwardPath_container[next_Labelindex]
					//BackwardPath_container[temp_label]��nextV�ĵ����ڶ����ڵ㲻ͬ
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
	//û���ҵ��ͷŵ�����listitor�ĺ�������Ϊ�ֲ�����û��newӦ��û�������
	return controled;
}

bool Subproblem::combinebidirection_kcycle_ngpath_cuts_stable_simple(BranchABound & BB, RMP & lp, Problem & p)
{
	int i, j;
	int for_Label, back_Label;
	float redundant_ST;
	bool check = false;

	//ǰ�����ÿ��customer
	for (int for_customer = 0; for_customer<p.Customer_Num; for_customer++)
	{
		//����for_customer�ϵ�����ǰ��label
		for (i = 0; i<ForwardBucket[for_customer].Path_Num; i++)
		{
			for_Label = ForwardBucket[for_customer].Path_index[i]; //ǰ��label�����

			for (int back_customer = 0; back_customer < p.Customer_Num; back_customer++)
			{
				//ǰ��label�뷴��label�����һ������ͬʱû�б�Ҫ�ϲ�
				//��Ϊ�ܴ�����һ�Ժϲ���label
				if (for_customer == back_customer)continue;
				//����back_customer�ϵ�����ǰ��label
				for (j = 0; j<BackwardBucket[back_customer].Path_Num; j++)
				{
					back_Label = BackwardBucket[back_customer].Path_index[j]; //����label�����

					//���ж���Դ������
					if (ForwardPath_container[for_Label].usedcapacity >BackwardPath_container[back_Label].availablecapacity)continue;	 //����Ƿ���������Լ��
					if ((ForwardPath_container[for_Label].arrivaltime + p.Allnode[for_customer].servicetime + BB.branch[BB.cur_node].CostNetwork_Branch[for_customer][back_customer]
						+ p.Allnode[back_customer].servicetime + BackwardPath_container[back_Label].consumed_time)>p.Max_arrivaltime[current_depot])continue;	//����Ƿ�����ʱ�䴰Լ��
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
						+ BackwardPath_container[back_Label].accu_duration - redundant_ST) >p.Veh.Duration)continue;	 //����Ƿ�����durationԼ��
#endif
					//����ϲ�label��������Լ��
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

