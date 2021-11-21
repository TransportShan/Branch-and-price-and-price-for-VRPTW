#include "stdafx.h"
#include "BranchABound.h"


NewConstraint::NewConstraint()
{
}


NewConstraint::~NewConstraint()
{
}

BranchNode::BranchNode()
{
}


BranchNode::~BranchNode()
{
}

void BranchNode::Copy_Constraints(BranchNode preNode)
{
	addCons_Total_num = preNode.addCons_Total_num;
	for (int i = 0; i<addCons_Total_num; i++)
	{
		if (1 == preNode.addCons_Total[i].type)
		{
			//弧相关约束
			addCons_Total[i].type = preNode.addCons_Total[i].type;
			addCons_Total[i].startnode = preNode.addCons_Total[i].startnode;
			addCons_Total[i].endnode = preNode.addCons_Total[i].endnode;
			addCons_Total[i].arc_state = preNode.addCons_Total[i].arc_state;
		}
	}
}

BranchABound::BranchABound()
{
}


BranchABound::~BranchABound()
{
}

void BranchABound::Claim_Branch(Problem & p)
{
	//辅助量
	vehicle_num = new float[p.Depot_Num];
	flow_onarc = new float*[p.Customer_Num];
	customer_branch = new int[p.Customer_Num];
	for (int i = 0; i < p.Customer_Num; i++)
	{
		flow_onarc[i] = new float[p.Customer_Num];
	}

	//分支点
	branch = new BranchNode[Conf::MAX_BRANCH_NODES];
	for (int i = 0; i < Conf::MAX_BRANCH_NODES; i++)
	{
		branch[i].depth = -1;
		branch[i].branch_state = -1;
		branch[i].pre_nodeid = -1;
		branch[i].branch_ornot = false;
		branch[i].branched_num = 0;
		branch[i].state = -1;
		branch[i].LB_value = MAXNUM;

		branch[i].branch_strategy = -1;
		branch[i].addCons_New_num = 0;
		branch[i].branchcons_num = 0;
		branch[i].branch_Startnode = new int[Conf::MAX_ADD];
		branch[i].branch_Endnode = new int[Conf::MAX_ADD];
	}
	best_upper = MAXNUM;
	best_lower = MAXNUM;


	//对根节点0分枝
	exist_nodeNum = 1;
	cur_node = 0;
	Toslove_num = 1;
	//给根节点开辟空间并赋值上下界
	branch[cur_node].LBsol_index = new int[2 * p.Customer_Num];
	branch[cur_node].LBsol_value = new float[2 * p.Customer_Num];
	branch[cur_node].LBsol_num = 0;
	branch[cur_node].used_column = new int[Conf::MAX_COLUMN_IN_RMP];
	branch[cur_node].used_column_num = 0;
	branch[cur_node].total_column_num = 0;
	branch[cur_node].UB_value = MAXNUM;
	branch[cur_node].LB_value = MAXNUM;
	branch[cur_node].vehicleNum_lower = new int[p.Depot_Num];
	branch[cur_node].vehicleNum_upper = new int[p.Depot_Num];

	for (int i = 0; i < p.Depot_Num; i++)
	{
		branch[cur_node].vehicleNum_lower[i] = 0;
		branch[cur_node].vehicleNum_upper[i] = p.Vehicle_Num;
	}

	branch[cur_node].addCons_Total_num = 0;
	branch[cur_node].node_id = 0;
	branch[cur_node].depth = 0;
	branch[cur_node].branch_state = 0;
	branch[cur_node].pre_nodeid = -1;

	//分支树上的网络
	branch[cur_node].CostNetwork_Branch = new float*[p.Customer_Num + p.Depot_Num];
	for (int i = 0; i < p.Customer_Num + p.Depot_Num; i++)
	{
		branch[cur_node].CostNetwork_Branch[i] = new float[p.Customer_Num + p.Depot_Num];
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			branch[cur_node].CostNetwork_Branch[i][j] = 0;
		}
	}
	branch[cur_node].valid_Cus = new int[p.Customer_Num];
	//初始化节点的网络
	for (int i = 0; i < p.Customer_Num + p.Depot_Num; i++)
	{
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			branch[cur_node].CostNetwork_Branch[i][j] = p.Cost_network[i][j];
		}
	}
	for (int i = 0; i < p.Customer_Num; i++)
	{
		branch[cur_node].valid_Cus[i] = 1;
	}
}

void BranchABound::Choose_solveNode(void)
{
	cur_node = Toslove_index[2 - Toslove_num];
}

void BranchABound::releaseNode(Problem & p)
{
	//更新母节点的branched_num
	int temp_mother = branch[cur_node].pre_nodeid;

	branch[temp_mother].branched_num = branch[temp_mother].branched_num + 1;
	if (2 == branch[temp_mother].branched_num)
	{
		//释放母节点的列池
		delete[] branch[temp_mother].used_column;
		delete[] branch[temp_mother].LBsol_index;
		delete[] branch[temp_mother].LBsol_value;

		//释放母节点的网络
		for (int j = 0; j<p.Customer_Num + p.Depot_Num; j++)
		{
			delete[] branch[temp_mother].CostNetwork_Branch[j];
		}
		delete[] branch[temp_mother].CostNetwork_Branch;
		delete[] branch[temp_mother].valid_Cus;
	}
}

void BranchABound::Update_bestLB(SolINFOR & SolInfo)
{
	SolInfo.Best_Solution.OBJ_lower = branch[best_node_lower].LB_value;

	SolInfo.Best_Solution.best_LBsol_num = branch[best_node_lower].LBsol_num;
	for (int i = 0; i<branch[best_node_lower].LBsol_num; i++)
	{
		SolInfo.Best_Solution.best_LBsol_index[i] = branch[best_node_lower].LBsol_index[i];
		SolInfo.Best_Solution.best_LBsol_value[i] = branch[best_node_lower].LBsol_value[i];
	}
}
