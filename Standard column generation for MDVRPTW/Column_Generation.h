#ifndef COLUMN_GENERATION_H_
#define COLUMN_GENERATION_H_

#include "Problem.h"
#include "Initialization.h"
#include "RMP.h"
#include "Subproblem.h"
#include "OUTPUT.h"
#include "Solution.h"
#include "ColumnPool.h"
#include "Subproblem.h"
#include "BranchABound.h"
#include "Utility.h"

class Column_Generation
{
public:
	Column_Generation();
	virtual ~Column_Generation();

	void Claim_CG(void);

	void SolveNode_ByCG(Utility & Results, SolINFOR &SolInfo,BranchABound & BB, Initialization & IniC, ColumnPool &pool, Problem &p, RMP &LP, Subproblem & subp, SRC &Cuts_src, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc);	//ʹ��column generation���node�µ�vrp����

	bool Solve_PricingSubproble(bool exact, Utility & Results, BranchABound & BB, Subproblem &subp, RMP &lp, ColumnPool &colp, Problem &p, SRC &temp_src, SDC &Cuts_sdc);

	bool Detect_branch(SolINFOR &SolInfo, BranchABound & BB, ColumnPool &colp, Problem &p);			//��CG��������̽���Ƿ�õ�һ�������⣬������������ҳ����Ǹ��������з�֧
	bool Branching(BranchABound & BB, Problem &p);
	void Update_used_column(BranchABound & BB, ColumnPool &colp, Problem &p);
	void Update_network(BranchABound & BB, Problem &p);
	bool Check_state(SolINFOR &SolInfo, BranchABound & BB, ColumnPool &colp, Problem &p);			//����branch[cur_node]�ķ�֧״̬
	int Show_UBsolutions_mdvrptw(const string & file_name, SolINFOR &SolInfo, ColumnPool &pool, Problem p);
	int Show_LBsolutions_mdvrptw(const string & file_name, SolINFOR &SolInfo, ColumnPool &pool, Problem p);
	bool Choose_branchNode(SolINFOR &SolInfo, BranchABound & BB, ColumnPool &pool, Problem p);
	void Update_Solvednode(SolINFOR &SolInfo, BranchABound & BB, ColumnPool &pool);

	bool Generate_Kpath(SolINFOR &SolInfo, BranchABound &BB, Subproblem &subp, ColumnPool &colp, Problem &p, RMP &LP, KPATH &Cuts_kpath);
	void Extend_Kpath_cut(float cur_RHS, int *cur_Kpath, int cur_Kpath_num, long *cur_Kpath_passnode, BranchABound & BB, Subproblem &subp, Problem &p, RMP &LP, KPATH &Cuts_kpath);
	bool Generate_RCC(SolINFOR &SolInfo, BranchABound &BB, Subproblem &subp, ColumnPool &colp, Problem &p, RMP &LP, RCC &Cuts_rcc);
	void Extend_RCC_cut(float cur_load,float cur_RHS, int *cur_RCC, int cur_RCC_num, long *cur_RCC_passnode, BranchABound & BB, Subproblem &subp, Problem &p, RMP &LP, RCC &Cuts_rcc);
	bool Generate_SRC(SolINFOR &SolInfo, BranchABound &BB, Subproblem &subp, ColumnPool &colp, Problem &p, RMP &LP, SRC &Cuts_src);
public:
	int solved_depotNum;			//PSP=2ʱʹ�ñ��������ܷ����е�������ĸ���
	int Tosolve_depotno;			//PSP=2ʱʹ�ñ�������ǰ����PSP����������
	float Remain_pro;

	//�������ʱ���ļ�������
	string UBresult_file_name;
	string LBresult_file_name;
};

#endif