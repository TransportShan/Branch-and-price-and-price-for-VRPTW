#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "Problem.h"

class Solution
{
public:
	Solution();
	virtual ~Solution();

public:
	//上界解
	float OBJ_upper;

	int *best_UPsol_order;						//存储最优上界值中大于0的变量的序号，大小为[2*Customer_Num]
	int *best_UPsol_index;						//存储能够找到的最优上界解在列池中的位置，大小为[2*Customer_Num]
	float *best_UPsol_value;					//存储能够找到的最优上界解的值，大小为[2*Customer_Num]
	int best_UPsol_num;							//多少个非0解
	//下界解
	float OBJ_lower;

	int *best_LBsol_order;						//存储最优下界值中大于0的变量的序号，大小为[2*Customer_Num]
	int *best_LBsol_index;						//存储能够找到的最优下界解在列池中的位置，大小为[2*Customer_Num]
	float *best_LBsol_value;					//存储能够找到的最优下界解的值，大小为[2*Customer_Num]
	int best_LBsol_num;							//多少个非0解
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
	Solution Best_Solution;						//已经找到的最好解
	Solution Cur_solution;						//当前找到的最好解
};

#endif