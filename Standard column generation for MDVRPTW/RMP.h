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

	void Ini_RMP(Problem &p);																																				//�ڳ���ʼ��ʼ��
	bool Initial_Rmpnode(SolINFOR &SolInfo, BranchABound & BB,ColumnPool &colp,Problem &p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src);								//��ÿ����֦���ڵ��ϸ���RMP
	bool Bulid_RmpMatrix(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem &p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src);								//�������RMP��ϵ������
	bool StandardRMP_SolvebyCPLEX_Real(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem &p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src);					//���RMP��ʵ���⣬��ȡ��żֵ
	bool StandardRMP_SolvebyCPLEX_Int(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp);																				//���RMP�������⣬��ȡ��żֵ
	
	void Update_RMP(BranchABound &BB, ColumnPool &colp);																													//����ColumnPool��Subproblem����RMP�е���Ϣ
	bool Strengthen_elementary(Utility & Results, SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem &p, SDC &Cuts_sdc);										//����RMP�������ϢSolInfo������elementary constraints
																																											//���ж�CPA��ܵ���ֹ����
																																											//����SRC��dssr_ngSet-
	bool Calculate_SDCindicator(SolINFOR &SolInfo, ColumnPool &colp, Problem &p, SDC &Cuts_sdc);
	void DeleteColumn_byCycle(BranchABound &BB, ColumnPool &colp, Problem &p);																								//����RMP��ǰ���cycle��Ϣ��ɾ�����а�����cycle����
	void Update_flow(SolINFOR &SolInfo, ColumnPool &colp, Problem &p);																										//����RMP��ǰ�⣬����ÿ�����ϵ�����
	void Update_AugNgset(SolINFOR &SolInfo, ColumnPool &colp, Problem &p, SDC &Cuts_sdc);
	int Get_SRC_Cof(ColumnPool &colp,int ColIndex, SRC &Cuts_src, int SrcIndex);
	//ÿ��node�Ͻ���ʱ����
	void reset_RmpMatrix_byclass(void);																																		//��Cpelx_object�в���ʹ��end�����ͷ�
public:
	//��֧����ÿ���ڵ��ϱ����ʼ���Ĳ���
	int Added_columnNum_once;					//һ�ε����У���PSP�д��������е�����
	int Rmp_CandidateColumn_Num;				//��ʾ��ǰRMPʹ�õ��е�����
	Cpelx_object Rmp_matrix;					//�洢���RMP��cplex����

	//ָʾ��Щ�п��Ա�ɾ����Ŀ��ϵ��ΪMAXNUM��
	int *Todelete_col;							//�洢�����´α�ɾ���ı������У�����ţ������Ų��������гص���ţ����Ƿ�֧�ڵ���Rmp_CandidateColumn_Num����ţ���СΪ[int(Conf::MAX_COLUMN_IN_RMP/10)]
	int Todelete_col_num;						//Todelete_col�Ĵ�С

	//���RMP���صı���
	float OBJ_value;							//RMP��ǰ������Ŀ�꺯��ֵ
	float *Customer_dual;						//��ͻ���Ӧ�Ķ�ż��������СΪ[p.Customer_Num]
	float *Vehicle_dual;						//�복������Ӧ�Ķ�ż��������СΪ[p.Depot_Num]
	//Strong degree constraints
	float *SDC_dual;							//Strong Degree Constraints��Ӧ�Ķ�ż��������СΪ[p.Customer_Num]
	//robust valid cuts��Ӧ�Ķ�ż���������ɵ�����
	float **RobustCut_dual;						//ÿ�����ϼ��ɵ�obust valid cuts ��Ӧ�Ķ�ż�������ܺͣ���СΪ[p.Customer_Num+p.Depot_Num]*[p.Customer_Num+p.Depot_Num]����ʼ��Ϊ0
	float **Arc_flow;							//RMP��ǰ���µ�ÿ�����ϵ������ܺͣ���СΪ[p.Customer_Num+p.Depot_Num]*[p.Customer_Num+p.Depot_Num]
	//nonrobust valid cuts��Ӧ�Ķ�ż����
	float *SRC_dual;							//subset row cuts��Ӧ�Ķ�ż��������СΪ[Conf::MAX_SR_NUM]
};

#endif

//IloCplex::clearModel;	//Deletes CPLEX problem and frees associated memory
//IloCplex::setVectors;	//Copies a starting basis and/or starting solution into a CPLEX problem
//IloCplex::extract;	//Copies LP data, including variable names and constraint names, into a CPLEX problem