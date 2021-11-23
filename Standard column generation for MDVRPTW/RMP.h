#ifndef RMP_H_
#define RMP_H_

#include "Problem.h"
#include "Cpelx_object.h"
#include "ColumnPool.h"
#include "Cut.h"
#include "BranchABound.h"
#include "Solution.h"
#include "Utility.h"

class RMP
{
public:
	RMP();
	virtual ~RMP();

	void Ini_RMP(Problem &p);																																				//在程序开始初始化
	bool Initial_Rmpnode(SolINFOR &SolInfo, BranchABound & BB,ColumnPool &colp,Problem &p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src);								//在每个分枝数节点上更新RMP
	bool Bulid_RmpMatrix(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem &p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src);								//构建求解RMP的系数矩阵
	bool StandardRMP_SolvebyCPLEX_Real(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem &p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src);					//求解RMP的实数解，获取对偶值
	bool StandardRMP_SolvebyCPLEX_Int(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp);																				//求解RMP的整数解，获取对偶值
	
	void Update_RMP(BranchABound &BB, ColumnPool &colp);																													//根据ColumnPool和Subproblem更新RMP中的信息
	bool Strengthen_elementary(Utility & Results, SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem &p, SDC &Cuts_sdc);										//根据RMP的求解信息SolInfo，更新elementary constraints
																																											//并判断CPA框架的终止条件
																																											//包括SRC和dssr_ngSet-
	bool Calculate_SDCindicator(SolINFOR &SolInfo, ColumnPool &colp, Problem &p, SDC &Cuts_sdc);
	void DeleteColumn_byCycle(BranchABound &BB, ColumnPool &colp, Problem &p);																								//根据RMP当前解的cycle信息，删除所有包含该cycle的列
	void Update_flow(SolINFOR &SolInfo, ColumnPool &colp, Problem &p);																										//根据RMP当前解，更新每条弧上的流量
	void Update_AugNgset(SolINFOR &SolInfo, ColumnPool &colp, Problem &p, SDC &Cuts_sdc);
	int Get_SRC_Cof(ColumnPool &colp,int ColIndex, SRC &Cuts_src, int SrcIndex);
	//每个node上结束时调用
	void reset_RmpMatrix_byclass(void);																																		//对Cpelx_object中参数使用end进行释放
public:
	//分支树上每个节点上必须初始化的参数
	int Added_columnNum_once;					//一次迭代中，从PSP中传进来的列的数量
	int Rmp_CandidateColumn_Num;				//表示当前RMP使用的列的数量
	Cpelx_object Rmp_matrix;					//存储求解RMP的cplex参数

	//指示哪些列可以被删除（目标系数为MAXNUM）
	int *Todelete_col;							//存储可以下次被删除的变量（列）的序号，这个序号不是整个列池的序号，而是分支节点上Rmp_CandidateColumn_Num的序号，大小为[int(Conf::MAX_COLUMN_IN_RMP/10)]
	int Todelete_col_num;						//Todelete_col的大小

	//求解RMP返回的变量
	float OBJ_value;							//RMP当前的最优目标函数值
	float *Customer_dual;						//与客户对应的对偶变量，大小为[p.Customer_Num]
	float *Vehicle_dual;						//与车辆数对应的对偶变量，大小为[p.Depot_Num]
	//Strong degree constraints
	float *SDC_dual;							//Strong Degree Constraints对应的对偶变量，大小为[p.Customer_Num]
	//robust valid cuts对应的对偶变量被集成到弧上
	float **RobustCut_dual;						//每条弧上集成的obust valid cuts 对应的对偶变量的总和，大小为[p.Customer_Num+p.Depot_Num]*[p.Customer_Num+p.Depot_Num]，初始化为0
	float **Arc_flow;							//RMP当前解下的每条弧上的流量总和，大小为[p.Customer_Num+p.Depot_Num]*[p.Customer_Num+p.Depot_Num]
	//nonrobust valid cuts对应的对偶变量
	float *SRC_dual;							//subset row cuts对应的对偶变量，大小为[Conf::MAX_SR_NUM]
};

#endif

//IloCplex::clearModel;	//Deletes CPLEX problem and frees associated memory
//IloCplex::setVectors;	//Copies a starting basis and/or starting solution into a CPLEX problem
//IloCplex::extract;	//Copies LP data, including variable names and constraint names, into a CPLEX problem