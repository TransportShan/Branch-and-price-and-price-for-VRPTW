#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "Problem.h"

class Solution
{
public:
	Solution();
	virtual ~Solution();

public:
	//�Ͻ��
	float OBJ_upper;

	int *best_UPsol_order;						//�洢�����Ͻ�ֵ�д���0�ı�������ţ���СΪ[2*Customer_Num]
	int *best_UPsol_index;						//�洢�ܹ��ҵ��������Ͻ�����г��е�λ�ã���СΪ[2*Customer_Num]
	float *best_UPsol_value;					//�洢�ܹ��ҵ��������Ͻ���ֵ����СΪ[2*Customer_Num]
	int best_UPsol_num;							//���ٸ���0��
	//�½��
	float OBJ_lower;

	int *best_LBsol_order;						//�洢�����½�ֵ�д���0�ı�������ţ���СΪ[2*Customer_Num]
	int *best_LBsol_index;						//�洢�ܹ��ҵ��������½�����г��е�λ�ã���СΪ[2*Customer_Num]
	float *best_LBsol_value;					//�洢�ܹ��ҵ��������½���ֵ����СΪ[2*Customer_Num]
	int best_LBsol_num;							//���ٸ���0��
};


class SolINFOR
{
public:
	SolINFOR();
	virtual ~SolINFOR();

	void Claim_Solution(Problem &p);
	void CopyToBest_Upper(void);
	void CopyToBest_Lower(void);
	bool check_IntSol(void);

public:
	Solution Best_Solution;						//�Ѿ��ҵ�����ý�
	Solution Cur_solution;						//��ǰ�ҵ�����ý�
};

#endif