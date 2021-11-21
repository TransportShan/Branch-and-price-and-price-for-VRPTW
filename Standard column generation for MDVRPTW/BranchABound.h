#ifndef BRANCHABOUND_H_
#define BRANCHABOUND_H_

#include "Problem.h"
#include "Solution.h"

using namespace std;


//我们这里采用最优搜索策略
//采用动态内存管理策略
//每次一个BranchNode前，先开辟动态数组；每次对BranchNode的左右两个子节点分支后，释放动态数组
class NewConstraint
{
public:
	NewConstraint();
	virtual ~NewConstraint();

public:
	int type;									//初始化为-1，=1表示与弧相关
	//第一种分支策略
	int startnode;								//分支弧的起点
	int endnode;								//分支弧的起点
	bool arc_state;								//表示对在某天上某个客户的分支状态：
												//=true表示弧(startnode,endnode)在branch_day上一定被经过；=false表示弧(startnode,endnode)在branch_day上一定被不被经过
};


class BranchNode
{
public:
	BranchNode();
	virtual ~BranchNode();

	void Copy_Constraints(BranchNode preNode);

public:
	//节点上CG结束后的状态
	int node_id;								//节点的编号，0对应根节点

	int depth;									//该分支节点上的分支深度，从0开始
	int branch_state;							//该分支节点上的状态：
												//初始化为-1；=0时表示未求解LP，=1时表示已求解未分支，=2时表示已求解且分支
	int pre_nodeid;								//母节点的node_id
	int Son_node[2];

	bool branch_ornot;							//=true表示已经分支结束，=false表示还未分支
	int branched_num;							//已经处理分支的数量，初始化为0，一旦达到2，则释放该点上的动态数组
	int state;									//该node调用CG结束后的求解结果，初始化为-1表示还没有求解,0表示无解(剪枝)，1表示整数解(定界)，2表示实数解(分支)
	float LB_value;								//表示该节点上CG结束后的目标函数值
	float UB_value;								//表示CG结束后的目标函数值

	int *used_column;							//该node下，CG结束时，符合该node约束的列在列池中的序号索引，大小为[MAX_COLUMN_IN_RMP]
	int used_column_num;						//该node下，CG结束时，used_column的总数
	int total_column_num;						//该node下，CG结束时列池总数

	int *LBsol_index;							//该node下，CG结束时RMP最优解中>0的变量在列池中的位置，大小为[2*Customer_Num]
	float *LBsol_value;							//该node下，CG结束时RMP最优解中>0的变量的值，大小为[2*Customer_Num]
	int LBsol_num;								//该node下，CG结束时RMP最优解中>0的变量的个数

	//该分支上的要求
	//与车辆数相关的
	int *vehicleNum_upper;						//表示每个场站出发的车辆数上限,大小为[p.Depot_Num]
	int *vehicleNum_lower;						//表示每个场站出发的车辆数下限,大小为[p.Depot_Num]
	//与客户或者弧相关的
	NewConstraint *addCons_New;					//在该node上新加入的约束，大小为[MAX_ADD]
	int addCons_New_num;						//在该node上新加入的约束的数量
	NewConstraint *addCons_Total;				//包括新加入的约束在内，在该node上所有考虑的约束，大小为[int(MAX_BRANCH_NODES/2)]
	int addCons_Total_num;						//在该node上所有考虑的约束的数量

	//在每个BranchNode的网络，在每个分支节点上需要更新
	float **CostNetwork_Branch;					//每个分枝节点上的费用网络，若两点之间没有路径相连，那么距离为MAXNUM，[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num]
	int *valid_Cus;								//有效的customer集合，表示分支过程中，哪些点还可以根据对偶变量值不在网络中被考虑，大小为[Customer_Num]
												//1表示该customer在该天有效，否则为0

	//CG求解后：
	//对子节点的要求，用于构建addCons_New
	int branch_strategy;						//分支策略，初始化为-1，：
												//0：表示对车辆数分支；1：表示对弧分支
	int branch_depot;							//指示对从哪个场站出发的车辆数分支
	int branch_vehicleNum;						//指示车辆数的分界点（左枝上界，右枝下界）
												//左枝<= floor(BB.vehicle_num[i]),右枝>=floor(BB.vehicle_num[i])+1
	int branchcons_num;							//branch_Startnode或者branch_Endnode的大小
	int *branch_Startnode;						//指示分支弧的起点，大小为[Conf::MAX_ADD]
	int *branch_Endnode;						//指示分支弧的终点，大小为[Conf::MAX_ADD]
};


class BranchABound
{
public:
	BranchABound();
	virtual ~BranchABound();

	void Claim_Branch(Problem &p);
	void Choose_solveNode(void);					//选择下一次求解的节点
	void releaseNode(Problem &p);

	void Update_bestLB(SolINFOR &SolInfo);
public:
	BranchNode *branch;								//寻找BranchNode的最多数量，大小为[Conf::MAX_BRANCH_NODES]
	int exist_nodeNum;								//branch中已经存在的node数量
	int best_node_upper;							//上界值最好的分支点的序号
	int best_node_lower;							//下界值最好的分支点的序号
	int cur_node;									//当前分枝的节点的序号
	float best_upper;								//分支树上已经找到的最好上界
	float best_lower;								//分支树上已经找到的最好下界

	//下一次从那个母节点开始分支
	int mother_node;

	//求解序列
	int Toslove_index[2];							//大小为[2]，分表
	int Toslove_num;								//Toslove_index的内容

	//辅助量
	float *vehicle_num;								//从每个场站出发的车辆数，大小为[p.Depot_Num]
	float **flow_onarc;								//每条弧上的累积流量，大小为[p.Customer_Num]*[p.Customer_Num]，我们这里只对客户点之间的弧进行分支（不考虑从每个场站出发的流）
	int *customer_branch;							//如果客户可以分支为1，否则为0，大小为[p.Customer_Num]
};

#endif