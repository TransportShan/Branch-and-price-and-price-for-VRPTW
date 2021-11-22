#ifndef CUT_H_
#define CUT_H_

#include <algorithm>

#include "Problem.h"
#include "Utility.h"

class Repetition	//中间参数，当前RMP解在每个点的经过次数，辅助在RMP求解后找到新的SDC
{
public:
	Repetition();
	virtual ~Repetition();

public:
	int Customer_id;		//客户的索引
	float Indicator;		//每个客户被经过的次数
	int Prohibit_ornot;		//=1表示不能在该点上加入SDC，=0表示
};

class SDC
{
public:
	SDC();
	virtual ~SDC();

	void Ini_SDC(Problem &p);									//初始化SDC
	int Find_SDC(Utility & Results, Problem &p);				//对Rep_customers按照Indicator降序排序，以找到被重复经过次数最多的customer
	void Reset_RepCus(Problem &p);								//重置Rep_customers
	void Cpoy_Rep_customers(Problem &p);						//给Rep_customers_copy赋值
public:
	//记录已经生成的SDC
	int *SDC_nodes;							//记录应用SDC的node的索引集合，大小为[p.Customer_Num]
	int *SDC_indicator;						//如果该客户上应用SDC，则为1，否则为0，大小为[p.Customer_Num]
	int processed_num;						//SDC_nodes中已经在主问题写成约束的数量，也可以看做SDC_nodes的有效大小
	int added_num;							//新加入，待处理成约束的数量
											//记录每个SDC对应CPLEX环境中Rmp_ctall的序号
	int *SDC_cons_index;					//每个SDC_nodes对应的SDCcuts在Rmp_ctall中的序号，大小为[p.Customer_Num]

	Repetition *Rep_customers;				//用来存储RMP当前解每个客户被经过的次数，大小为[p.Customer_Num]
	Repetition *Rep_customers_copy;			//Rep_customers的副本，当CustomerToSDC==1时有用

											//ng-cycle-SDC相关
	bool ngset_change;						//根据RMP最优LP解，是否有节点的ngset被拓展，被拓展为true，否则为false，初始化为false
	int *ngset_addNum;						//CPA框架下，一次重优化，每个节点的ngset是否被拓展，被拓展为1，否则为0，初始化为0，大小为[p.Customer_Num]
};


class KPATH
{
public:
	KPATH();
	virtual ~KPATH();

	void Ini_KPATH(Problem &p);										//初始化KPATH
	bool Already_inKpath(int *obj_subset, int obj_subset_num);		//判断obj_subset是否与当前新加入（最新的added_num个）的Kpath重合
	void Add_Kpath(int *obj_subset, int obj_subset_num);			//把obj_subset加入当前KPATH中
public:
	//记录已经生成的Kpath
	int **Kpath_subset;						//记录每个Kpath有效不等式对应的节点集合，大小为[Conf::MAX_KPATH_NUM]*[Conf::MAX_SUBSET_KPATH]
	int *Kpath_subset_len;					//记录每个Kpath有效不等式对应的节点集合的大小，大小为[Conf::MAX_KPATH_NUM]
	int **Kpath_indicator;					//如果一个Kpath_subset包含该客户上则为1，否则为0，大小为[Conf::MAX_KPATH_NUM]*[p.Depot_num+p.Customer_Num]
	int processed_num;						//Kpath_subset中已经在主问题写成约束的数量
	int added_num;							//新加入，待处理成约束的数量
	//记录每个Kpath对应CPLEX环境中Rmp_ctall的序号
	int *Kpath_cons_index;					//每个Kpath_subset对应的Kpath在Rmp_ctall中的序号，大小为[Conf::MAX_KPATH_NUM]
};

class RCC
{
public:
	RCC();
	virtual ~RCC();

	void Ini_RCC(Problem &p);																							//初始化RCC
	bool Already_inRCC(long* obj_subset_passnode, Problem &p);															//判断obj_subset是否与当前新加入（最新的added_num个）的RCC重合
	void Add_RCC(int obj_RHS, int *obj_subset, int obj_subset_num, long* obj_subset_passnode, Problem &p);				//把obj_subset加入当前RCC中
public:
	//记录已经生成的Kpath
	int **RCC_subset;						//记录每个RCC有效不等式对应的节点集合，大小为[Conf::MAX_RCC_NUM]*[Conf::MAX_SUBSET_RCC]
	int *RCC_subset_len;					//记录每个RCC有效不等式对应的节点集合的大小，大小为[Conf::MAX_RCC_NUM]
	int *RCC_RHS;							//记录每个RCC有效不等式的右侧常数，大小为[Conf::MAX_RCC_NUM]
	long **RCC_subset_passnode;				//记录RCC_subset的二进制，大小为[Conf::MAX_RCC_NUM]*[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1]
	int **RCC_indicator;					//如果一个RCC_subset包含该客户上则为1，否则为0，大小为[Conf::MAX_RCC_NUM]*[p.Depot_num+p.Customer_Num]
	int processed_num;						//RCC_subset中已经在主问题写成约束的数量
	int added_num;							//新加入，待处理成约束的数量
											//记录每个RCC对应CPLEX环境中Rmp_ctall的序号
	int *RCC_cons_index;					//每个RCC_subset对应的RCC_subset在Rmp_ctall中的序号，大小为[Conf::MAX_RCC_NUM]
};

//subset-row cuts
class SRC
{
public:
	SRC();
	virtual ~SRC();

	void Ini_SRC(Problem &p);																							//初始化SRC
	float add_state(int SRC_length);																					//根据第SRC的集合C的大小，确定每次state递增的数量
	int Get_RHS(int SRC_length);																						//根据第SRC的集合C的大小，返回SRC右侧系数
	int Get_insert_no(float cur_violation);																				//获取violation中的插入位置
	void Insert_VioandRoute(int posi,float cur_violation);																//将cur_violation和temp_2D分别插入violation和subset_route的第posi个位置
	bool Check_SameSubset(int S1, int S2);																				//如果SRC_subset[S1]与SRC_subset[S1]相同则返回true，否则返回false
public:
	//记录已经生成的SRC
	int **SRC_subset;						//记录每个SRC有效不等式中节点集合C，大小为[Conf::MAX_SR_NUM]*[Conf::MAX_SR_SET]
	int *SRC_subset_len;					//记录每个SRC有效不等式中集合C的大小，大小为[Conf::MAX_SR_NUM]
	int *SRC_RHS;							//记录每个SRC有效不等式的右侧常数，大小为[Conf::MAX_SR_NUM]
	int **SRC_subset_indicator;				//如果一个SRC_subset包含该客户上则为1，否则为0，大小为[Conf::MAX_SR_NUM]*[p.Depot_num+p.Customer_Num]

	int **SRC_LimitVertexSet;				//记录每个SRC有效不等式中limit_memory中节点的集合，大小为[Conf::MAX_SR_NUM]*[p.Customer_Num]
	int *SRC_LimitVertexSet_num;			//记录每个SRC有效不等式中limit_memory中节点的数量，大小为[Conf::MAX_SR_NUM]
	int **SRC_LimitVertexSet_indicator;		//记录每个SRC有效不等式中limit_memory中节点的集合，如果包含该点则为1，否则为0，大小为[Conf::MAX_SR_NUM]*[p.Depot_num+p.Customer_Num]

	int ***SRC_LimitArcSet_indicator;		//记录每个SRC有效不等式中limit_memory中弧的集合，如果经过该弧则为1，否则为0，大小为[Conf::MAX_SR_NUM]*[p.Depot_num+p.Customer_Num]*[p.Depot_num+p.Customer_Num]

	int processed_num;						//SRC_subset中已经在主问题写成约束的数量
	int added_num;							//新加入，待处理成约束的数量
	//记录每个SRC对应CPLEX环境中Rmp_ctall的序号
	int *SRC_cons_index;					//每个SRC对应的SRC_subset在Rmp_ctall中的序号，大小为[Conf::MAX_SR_NUM]
	//辅助separation algorithm的变量
	vector<float> violation;				//分离过程中记录前Conf::MAX_ADD_SR个违反约束程度最大的SRC的惩罚值，大小为[Conf::MAX_ADD_SR+1]
	vector<vector<int>> subset_route;		//分离过程中记录前Conf::MAX_ADD_SR个违反约束程度最大的SRC的subset和哪些路径，大小为[Conf::MAX_ADD_SR+1]*[Conf::MAX_SR_SET+100]
											//第0列为subset个数;第1-Conf::MAX_SR_SET列为subset的节点序号;第Conf::MAX_SR_SET+1列为该集合C所在路径的个数;第Conf::MAX_SR_SET+2-Conf::MAX_SR_SET+99为基解中路径的序号
	vector<int> temp_2D;					//向subset_route插入一行的过程中，临时存储，这里开辟是为了减少函数中反复动态开辟，大小为[Conf::MAX_SR_SET+100]
	int violation_num;						//分离过程中，找到的SRC数量
};


#endif