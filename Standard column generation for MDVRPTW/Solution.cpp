#include "stdafx.h"
#include "Solution.h"


Solution::Solution()
{
}


Solution::~Solution()
{
}

SolINFOR::SolINFOR()
{
}

SolINFOR::~SolINFOR()
{
}

void SolINFOR::Claim_Solution(Problem & p)
{
	Best_Solution.best_UPsol_index = new int[2 *p.Customer_Num];
	Best_Solution.best_UPsol_value = new float[2 *p.Customer_Num];
	Best_Solution.best_UPsol_order = new int[2 * p.Customer_Num];

	Best_Solution.best_LBsol_index = new int[2 *p.Customer_Num];
	Best_Solution.best_LBsol_value = new float[2 *p.Customer_Num];
	Best_Solution.best_LBsol_order = new int[2 * p.Customer_Num];

	Cur_solution.best_UPsol_index = new int[2 *p.Customer_Num];
	Cur_solution.best_UPsol_value = new float[2 *p.Customer_Num];
	Cur_solution.best_UPsol_order = new int[2 * p.Customer_Num];

	Cur_solution.best_LBsol_index = new int[2 *p.Customer_Num];
	Cur_solution.best_LBsol_value = new float[2 *p.Customer_Num];
	Cur_solution.best_LBsol_order = new int[2 * p.Customer_Num];
}

void SolINFOR::CopyToBest_Upper(void)
{
	Best_Solution.OBJ_upper = Cur_solution.OBJ_lower;

	Best_Solution.best_UPsol_num = Cur_solution.best_LBsol_num;
	for (int i = 0; i<Cur_solution.best_LBsol_num; i++)
	{
		Best_Solution.best_UPsol_index[i] = Cur_solution.best_LBsol_index[i];
		Best_Solution.best_UPsol_value[i] = Cur_solution.best_LBsol_value[i];
	}
}

void SolINFOR::CopyToBest_Lower(void)
{
	Best_Solution.OBJ_lower = Cur_solution.OBJ_lower;

	Best_Solution.best_LBsol_num = Cur_solution.best_LBsol_num;
	for (int i = 0; i<Cur_solution.best_LBsol_num; i++)
	{
		Best_Solution.best_LBsol_index[i] = Cur_solution.best_LBsol_index[i];
		Best_Solution.best_LBsol_value[i] = Cur_solution.best_LBsol_value[i];
	}
}

bool SolINFOR::check_IntSol(void)
{
	for (int i = 0; i<Cur_solution.best_LBsol_num; i++)
	{
		if (Cur_solution.best_LBsol_value[i]-floor(Cur_solution.best_LBsol_value[i])>0.001)
		{
			return false;
		}
	}

	return true;
}
