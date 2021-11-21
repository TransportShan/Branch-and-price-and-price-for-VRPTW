#include "stdafx.h"
#include "RMP.h"
#include "Conf.h"



RMP::RMP()
{
}

RMP::~RMP()
{

}

//检验包含关系
bool Contained_Byset(int check_node, int* obj_set, int obj_set_length)
{
	bool contained_ornot = false;

	for (int i = 0; i < obj_set_length; i++)
	{
		if (check_node== obj_set[i])
		{
			contained_ornot = true;
			break;
		}
	}

	return contained_ornot;
}



//检验RMP当前解是否满足SDC约束
void Verify_SDCcons(SolINFOR &SolInfo, ColumnPool &colp, Problem & p,SDC &Cuts_sdc)
{
	int temp_index, temp_cus;
	//检验是否存在不能约束的node
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 1; j++)
		{
			temp_cus = colp.Col[temp_index].nodes[j];
			if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 2 && 1==Cuts_sdc.SDC_indicator[temp_cus])
			{
				cout << "ERROR：SDC约束无效" << endl;
				cin.get();
			}
		}
	}
	//检验所有约束
	float *temp_RHS;
	temp_RHS = new float[p.Customer_Num+ p.Customer_Num];
	for (int i = 0; i < p.Customer_Num + p.Customer_Num; i++)
	{
		temp_RHS[i] = 0;
	}
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 1; j++)
		{
			temp_cus = colp.Col[temp_index].nodes[j];
			if (colp.Col[temp_index].Customer_indicator[temp_cus]>1)
			{
				temp_RHS[temp_cus] = temp_RHS[temp_cus] + SolInfo.Cur_solution.best_LBsol_value[i] * 1;
			}
			else
			{
				temp_RHS[temp_cus] = temp_RHS[temp_cus] + SolInfo.Cur_solution.best_LBsol_value[i] * colp.Col[temp_index].Customer_indicator[temp_cus];
			}
			if (1 == Cuts_sdc.SDC_indicator[temp_cus])
			{
				if (colp.Col[temp_index].Customer_indicator[temp_cus] > 1)
				{
					temp_RHS[p.Customer_Num + temp_cus] = temp_RHS[p.Customer_Num + temp_cus] + SolInfo.Cur_solution.best_LBsol_value[i] * fmin(1, colp.Col[temp_index].Customer_indicator[temp_cus])/ colp.Col[temp_index].Customer_indicator[temp_cus];
				}
				else
				{
					temp_RHS[p.Customer_Num + temp_cus] = temp_RHS[p.Customer_Num + temp_cus] + SolInfo.Cur_solution.best_LBsol_value[i] * fmin(1, colp.Col[temp_index].Customer_indicator[temp_cus]);
				}
				
			}
		}
	}
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (temp_RHS[i]<1- MINDOUBLE)
		{
			cout << "ERROR：node约束无效" << endl;
			cin.get();
		}
	}
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (1 == Cuts_sdc.SDC_indicator[i] && temp_RHS[p.Customer_Num+i]<1- MINDOUBLE)
		{
			cout << "ERROR：SDC约束无效" << endl;
			cin.get();
		}
	}
	delete temp_RHS;
}

void RMP::Ini_RMP(Problem & p)
{
	Customer_dual = new float[p.Customer_Num];
	Vehicle_dual = new float[p.Depot_Num];
	SDC_dual = new float[p.Customer_Num];
	SRC_dual = new float[Conf::MAX_SR_NUM];

	Todelete_col = new int[int(Conf::MAX_COLUMN_IN_RMP / 10)];
	Todelete_col_num = 0;

#if KPATHCUT +RCCUT>0
	RobustCut_dual = new float*[p.Customer_Num+ p.Depot_Num];
	Arc_flow = new float*[p.Customer_Num + p.Depot_Num];
	for (int i = 0; i < p.Customer_Num + p.Depot_Num; i++)
	{
		RobustCut_dual[i]= new float[p.Customer_Num + p.Depot_Num];
		Arc_flow[i] = new float[p.Customer_Num + p.Depot_Num];
	}
#endif
}

bool RMP::Initial_Rmpnode(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem &p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src)
{
	bool feasiblesolution = true;
	Added_columnNum_once = 0;
	Rmp_CandidateColumn_Num = BB.branch[BB.cur_node].used_column_num;

	//初始化SDC
#if Frameworks == 1
	Cuts_sdc.added_num = Cuts_sdc.processed_num;
	Cuts_sdc.processed_num = 0;
	//对偶值初始化为负值
	for (int i = 0; i < p.Customer_Num; i++)
	{
		SDC_dual[i] = -1;
	}
#endif


#if KPATHCUT +RCCUT>0
	for (int i = 0; i < p.Customer_Num + p.Depot_Num; i++)
	{
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			RobustCut_dual[i][j] = 0;
		}
	}
#endif
	//初始化kpath
#if KPATHCUT == 1
	if (BB.branch[BB.cur_node].depth <= Conf::MAX_ROBUST_DEPTH)
	{
		Cuts_kpath.added_num = Cuts_kpath.processed_num;
		Cuts_kpath.processed_num = 0;
	}
	else
	{
		Cuts_kpath.added_num = 0;
		Cuts_kpath.processed_num = 0;
	}
#endif
	//初始化RCC
#if RCCUT == 1
	if (BB.branch[BB.cur_node].depth <= Conf::MAX_ROBUST_DEPTH)
	{
		Cuts_rcc.added_num = Cuts_rcc.processed_num;
		Cuts_rcc.processed_num = 0;
	}
	else
	{
		Cuts_rcc.added_num = 0;
		Cuts_rcc.processed_num = 0;
	}
#endif
	//初始化src
#if SRCUT == 1
	if (BB.branch[BB.cur_node].depth <= Conf::MAX_ROBUST_DEPTH)
	{
		Cuts_src.added_num = Cuts_src.processed_num;
		Cuts_src.processed_num = 0;
	}
	else
	{
		Cuts_src.added_num = 0;
		Cuts_src.processed_num = 0;
	}
#endif

	//再初始化Rmp_Matrix
	Rmp_matrix.ini_parameter();
	feasiblesolution= Bulid_RmpMatrix(SolInfo, BB,colp,p, Cuts_kpath, Cuts_rcc,Cuts_sdc, Cuts_src);

	return feasiblesolution;
}

bool RMP::Bulid_RmpMatrix(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem & p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src)
{
	int temp_factor = 0;
	float temp_value;
	bool feasible_ornot = true;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//构建CPLEX模型有两种模式：1,以行添加；2,以列添加
	//列生成中采用以行添加的模式构建初始模型；
	//然后在列生成过程中采用添加列的模式；
	//对于SDCcuts（只增不减），我们采取添加不断添加行的模式
	//对于SRcuts（可能会减少），我们可以提前生成空白行（系数为0，右侧常量为0，即恒成立），后修改行的系数的形式，使用到IloRange::setLinearCoef (for matrix)和IloRange::setBounds (for rhs)函数
	//经过预实验，预留空白行的方式的CPLEX运算时间增加的比较快
	//后修改：不再预留空白行，而是在删除SR-cuts时，直接替换系数或者修改系数为空白行
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//目标函数
	Rmp_obj = IloAdd(Rmp_model, IloMinimize(Rmp_matrix.env));
	////以下通过方式添加行的方式先构建初始模型RMP_model
	IloInt i,j,k,r;
	//定义每类约束的范围
	//set-covering约束
#if Frameworks == 0
	for (i = 0; i < p.Customer_Num; i++)
	{
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 1, IloInfinity));
	}
#else
	for (i = 0; i < p.Customer_Num; i++)
	{
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 1, 1));
	}
#endif
	//vehicle number约束
	for (i = 0; i < p.Depot_Num; i++)
	{
		Rmp_ctall.add(IloRange(Rmp_matrix.env, BB.branch[BB.cur_node].vehicleNum_lower[i], BB.branch[BB.cur_node].vehicleNum_upper[i]));
	}
	//SDCcuts
#if Frameworks == 1
	for (i = 0; i < Cuts_sdc.added_num; i++)
	{
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 1, IloInfinity));
		Cuts_sdc.SDC_cons_index[i] = p.Customer_Num+ p.Depot_Num+i;
	}
#endif

	//如果加入cut
	//那么先向Rmp_model中添加行，再添加列
	//robust cuts
#if KPATHCUT == 1
	for (i = 0; i < Cuts_kpath.added_num; i++)
	{
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 2, IloInfinity));
		Cuts_kpath.Kpath_cons_index[i] = p.Customer_Num + p.Depot_Num + Cuts_sdc.added_num + i;
	}
#endif
#if RCCUT == 1
	for (i = 0; i < Cuts_rcc.added_num; i++)
	{
		Rmp_ctall.add(IloRange(Rmp_matrix.env, Cuts_rcc.RCC_RHS[i], IloInfinity));
		Cuts_rcc.RCC_cons_index[i] = p.Customer_Num + p.Depot_Num + Cuts_sdc.added_num+ Cuts_kpath.added_num + i;
	}
#endif
	//nonrobust cuts
#if SRCUT == 1
	for (i = 0; i < Cuts_src.added_num; i++)
	{
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 0, Cuts_src.SRC_RHS[i]));
		Cuts_src.SRC_cons_index[i] = p.Customer_Num + p.Depot_Num + Cuts_sdc.added_num + Cuts_kpath.added_num+ Cuts_rcc.added_num + i;
	}
#endif

	//加入到模型中
	Rmp_model.add(Rmp_ctall);
	//对一列来说，一行一行输入系数
	int column_index;
	for (i = 0; i < Rmp_CandidateColumn_Num; i++)
	{
		column_index = BB.branch[BB.cur_node].used_column[i];
		//先赋值给目标函数的系数
		IloNumColumn col = Rmp_obj(colp.Col[column_index].Totalcost);
		//然后赋值给约束的系数
		for (j = 0; j < p.Customer_Num; j++)
		{
			col += Rmp_ctall[j](colp.Col[column_index].Customer_indicator[j]);
		}
		for (j = 0; j < p.Depot_Num; j++)
		{
			col += Rmp_ctall[p.Customer_Num+j](colp.Col[column_index].depot_indicator[j]);
		}

		//SDC 
#if Frameworks == 1
		//系数=1+ Augng-feasible-cycle的数量
		int temp_factor;
		for (j = 0; j < Cuts_sdc.added_num; j++)
		{
#if SDCTYPE == 0
			temp_factor = fmin(1, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]);
#elif SDCTYPE == 1
			temp_factor = 1;
			for (k = 0; k < colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]; k++)
			{
				//判断路径colp.Col[column_index]上以colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]][k]为起点的弧是否Augng-feasible
				if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]][k], colp.Col[column_index]))
				{
					colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]][k] = -1;
				}
				else
				{
					temp_factor = temp_factor + 1;
				}
			}
			//Augng_infeasible的cycle从colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]中剔除
			colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]);
			temp_factor = fmin(temp_factor, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]);
#endif
			col += Rmp_ctall[p.Customer_Num + p.Depot_Num + j](temp_factor);	//不确定向CPLEX约束中直接传入min/max表达式是否会影响运行效率
																				//记录SDC_cof
			colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]] = temp_factor;
		}
#endif

		//注意Cut对应的行也要添加列
#if KPATHCUT == 1
		//每个kpath
		for (j = 0; j < Cuts_kpath.added_num; j++)
		{
			temp_factor = 0;
			//一个kpath内包含的每个点
			for (k = 0; k < Cuts_kpath.Kpath_subset_len[j]; k++)
			{
				//列colp.Col[column_index]到达kpath包含的点cuts_kpath.Kpath_subset[j][k]之前经过的点
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_kpath.Kpath_subset[j][k]]; r++)
				{
					//点colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[j][k]][r]必须不被kpath包含
					if (0 == Cuts_kpath.Kpath_indicator[j][colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[j][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			col += Rmp_ctall[Cuts_kpath.Kpath_cons_index[j]](temp_factor);
		}
#endif
#if RCCUT == 1
		//每个rcc
		for (j = 0; j < Cuts_rcc.added_num; j++)
		{
			temp_factor = 0;
			//一个rcc内包含的每个点
			for (k = 0; k < Cuts_rcc.RCC_subset_len[j]; k++)
			{
				//列colp.Col[column_index]到达rcc包含的点Cuts_rcc.RCC_subset[j][k]之前经过的点
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_rcc.RCC_subset[j][k]]; r++)
				{
					//点colp.Col[column_index].pre_arcs[Cuts_kpath.RCC_subset[j][k]][r]必须不被rcc包含
					if (0 == Cuts_rcc.RCC_indicator[j][colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[j][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			col += Rmp_ctall[Cuts_rcc.RCC_cons_index[j]](temp_factor);
		}
#endif
#if SRCUT == 1
		//每个SR-cuts
		for (j = 0; j < Cuts_src.added_num; j++)
		{
			temp_factor = 0;
			//一个SR-cuts内包含的每个点
			for (k = 0; k < Cuts_src.SRC_subset_len[j]; k++)
			{
				//列colp.Col[column_index]到达SR-cuts包含的点Cuts_src.SRC_subset[j][k]的次数
				temp_factor = temp_factor + colp.Col[column_index].Customer_indicator[Cuts_src.SRC_subset[j][k]];
			}
			temp_factor = Get_SRC_Cof(temp_factor,j, Cuts_src);
			col += Rmp_ctall[Cuts_src.SRC_cons_index[j]](temp_factor);
		}
#endif

		//加入到该列对应的变量中
		Rmp_routevar_x.add(IloNumVar(col, 0, IloInfinity, ILOFLOAT));
		col.end();
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Rmp_cplex.setOut(Rmp_matrix.env.getNullStream());
	Rmp_cplex.setWarning(Rmp_matrix.env.getNullStream());
	Rmp_cplex.solve();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//判断模型求解情况
	if (Rmp_cplex.getStatus() == IloAlgorithm::Infeasible || Rmp_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unknown || Rmp_cplex.getStatus() == IloAlgorithm::Error)
	{
		Rmp_matrix.env.out() << "No Solution" << endl;
		feasible_ornot = false;
	}
	else
	{
		//获取求解基解信息，为下次计算提供初始解
		Rmp_cplex.getBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);
		//获取目标函数值
		OBJ_value = Rmp_cplex.getObjValue();
		SolInfo.Cur_solution.OBJ_lower = OBJ_value;
		//获取决策变量的值
		SolInfo.Cur_solution.best_LBsol_num = 0;
		for (i = 0; i < Rmp_CandidateColumn_Num; i++)
		{
			temp_value = Rmp_cplex.getValue(Rmp_routevar_x[i]);
			if (temp_value>MINDOUBLE)
			{
				SolInfo.Cur_solution.best_LBsol_value[SolInfo.Cur_solution.best_LBsol_num] = temp_value;
				SolInfo.Cur_solution.best_LBsol_index[SolInfo.Cur_solution.best_LBsol_num] = BB.branch[BB.cur_node].used_column[i];
				SolInfo.Cur_solution.best_LBsol_order[SolInfo.Cur_solution.best_LBsol_num] = i;
				SolInfo.Cur_solution.best_LBsol_num = SolInfo.Cur_solution.best_LBsol_num + 1;
			}
		}
		//获取对偶最优解,也就是约束对应的影子价格
		for (i = 0; i<p.Customer_Num; i++)
		{
			Customer_dual[i] = Rmp_cplex.getDual(Rmp_ctall[i]);//getDual方法的参数必须是IloRangeArray类，所以这也是取约束别名的原因
		}
		for (i = 0; i < p.Depot_Num; i++)
		{
			Vehicle_dual[i] = Rmp_cplex.getDual(Rmp_ctall[p.Customer_Num+i]);
		}
		//SDC的对偶变量
#if Frameworks == 1
		for (i = 0; i < Cuts_sdc.added_num; i++)
		{
			SDC_dual[Cuts_sdc.SDC_nodes[i]] = Rmp_cplex.getDual(Rmp_ctall[Cuts_sdc.SDC_cons_index[i]]);
		}
		Cuts_sdc.processed_num = Cuts_sdc.processed_num + Cuts_sdc.added_num;
		Cuts_sdc.added_num = 0;
		Cuts_sdc.ngset_change = false;
#endif

		//Robust-cut的对偶变量都被整合到弧RobustCut_dual上
#if KPATHCUT == 1
		//每个kpath
		for (i = 0; i < Cuts_kpath.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_kpath.Kpath_cons_index[i]]);
			//所有从kpath子集外入射弧
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0==Cuts_kpath.Kpath_indicator[i][j])
				{
					//一个kpath内包含所有节点
					for (k = 0; k < Cuts_kpath.Kpath_subset_len[i]; k++)
					{
						RobustCut_dual[j][Cuts_kpath.Kpath_subset[i][k]]+= temp_value;
					}
				}
			}
		}
		Cuts_kpath.processed_num = Cuts_kpath.processed_num + Cuts_kpath.added_num;
		Cuts_kpath.added_num = 0;
#endif
#if RCCUT == 1
		//每个kpath
		for (i = 0; i < Cuts_rcc.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_rcc.RCC_cons_index[i]]);
			//所有从kpath子集外入射弧
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0 == Cuts_rcc.RCC_indicator[i][j])
				{
					//一个kpath内包含所有节点
					for (k = 0; k < Cuts_rcc.RCC_subset_len[i]; k++)
					{
						RobustCut_dual[j][Cuts_rcc.RCC_subset[i][k]] += temp_value;
					}
				}
			}
		}
		Cuts_rcc.processed_num = Cuts_rcc.processed_num + Cuts_rcc.added_num;
		Cuts_rcc.added_num = 0;
#endif
		//nonrobust-cut对应的对偶变量不需要整合到弧上
#if SRCUT == 1
		//每个SRC
		for (i = 0; i < Cuts_src.added_num; i++)
		{
			SRC_dual[i] = Rmp_cplex.getDual(Rmp_ctall[Cuts_src.SRC_cons_index[i]]);
		}
		Cuts_src.processed_num = Cuts_src.processed_num + Cuts_src.added_num;
		Cuts_src.added_num = 0;
#endif
	}

	return feasible_ornot;
}


bool RMP::StandardRMP_SolvebyCPLEX_Real(SolINFOR &SolInfo, BranchABound &BB, ColumnPool &colp, Problem & p, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc, SRC &Cuts_src)
{
	bool feasible_ornot =true;
	int column_index = 0;
	float temp_value;
	int temp_factor = 0;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////构建cplex环境下的模型
	IloInt i,j,k,r;
	//先warm start
	Rmp_cplex.setBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);

	//根据Todelete_col，修改目标函数系数
	for (i = 0; i < Todelete_col_num; i++)
	{
		Rmp_obj.setLinearCoef(Rmp_routevar_x[Todelete_col[i]], MAXNUM);
	}
	Todelete_col_num = 0;
	
	//如果加入cut
	//那么先向Rmp_model中添加行，再添加列
#if Frameworks == 1

#if SDCTYPE == 1
	//先修改SDC中的系数
	if (true == Cuts_sdc.ngset_change)
	{
		for (i = 0; i < Rmp_CandidateColumn_Num - Added_columnNum_once; i++)
		{
			column_index = BB.branch[BB.cur_node].used_column[i];
			//至少有环才有可能系数改变
			if (false == colp.Col[column_index].elementary_ornot)
			{
				for (j = 0; j < Cuts_sdc.processed_num; j++)
				{
					//节点Cuts_sdc.SDC_nodes[j]的Aug_ngset发生变化了，才有可能系数改变
					if (1 == Cuts_sdc.ngset_addNum[Cuts_sdc.SDC_nodes[j]])
					{
						//在Cuts_sdc.SDC_nodes[j]客户点有cycle才有可能系数改变
						if (colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[j]]>0)
						{
							temp_factor = 1;
							for (k = 0; k < colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[j]]; k++)
							{
								//判断路径colp.Col[column_index]上以colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k]为起点的弧是否Augng-feasible
								if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k], colp.Col[column_index]))
								{
									colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k] = -1;
								}
								else
								{
									temp_factor = temp_factor + 1;
								}
							}
							//只有系数确实改变了，才调用setLinearCoef
							if (temp_factor != colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[j]])
							{
								Rmp_ctall[Cuts_sdc.SDC_cons_index[j]].setLinearCoef(Rmp_routevar_x[i], temp_factor);
								//Augng_infeasible的cycle从colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]]中剔除
								colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[j]);
								//记录SDC_cof
								colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[j]] = temp_factor;
							}
						}
					}
				}
			}
		}
	}
#endif

	//再添加行
	for (i = 0; i < Cuts_sdc.added_num; i++)
	{
		IloExpr c_sdc(Rmp_matrix.env);
		for (j = 0; j < Rmp_CandidateColumn_Num - Added_columnNum_once; j++)
		{
			column_index = BB.branch[BB.cur_node].used_column[j];
#if SDCTYPE == 0
			temp_factor = fmin(1, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]]);
#elif SDCTYPE == 1
			temp_factor = 1;
			for (k = 0; k < colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]]; k++)
			{
				//判断路径colp.Col[column_index]上以colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]][k]为起点的弧是否Augng-feasible
				if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]][k], colp.Col[column_index]))
				{
					colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]][k] = -1;
				}
				else
				{
					temp_factor = temp_factor + 1;
				}
			}
			//Augng_infeasible的cycle从colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]]中剔除
			colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]);
			temp_factor = fmin(temp_factor, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]]);
#endif
			c_sdc += Rmp_routevar_x[j] * temp_factor;
			//记录SDC_cof
			colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]] = temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 1, c_sdc, IloInfinity));
		//记录SDC序号
		Cuts_sdc.SDC_cons_index[Cuts_sdc.processed_num + i] = Rmp_ctall.getSize() - 1;
		//新SDC添加到Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_sdc.SDC_cons_index[Cuts_sdc.processed_num + i]]);
		c_sdc.end();
		//每添加一行（约束）就添加对应的一个初始状态
		allcon_stat.add(IloCplex::AtLower);
						}
#endif
	//添加kpath-cut
#if KPATHCUT == 1
	for (i = 0; i < Cuts_kpath.added_num; i++)
	{
		IloExpr c_Kpath(Rmp_matrix.env);
		//构建左侧系数
		for (j = 0; j < Rmp_CandidateColumn_Num - Added_columnNum_once; j++)
		{
			column_index = BB.branch[BB.cur_node].used_column[j];
			temp_factor = 0;
			//Kpath中包含的每个点
			for (k = 0; k < Cuts_kpath.Kpath_subset_len[Cuts_kpath.processed_num+i]; k++)
			{
				//列colp.Col[column_index]到达kpath中点cuts_kpath.Kpath_subset[Cuts_kpath.processed_num+i][k]之前经过的点
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_kpath.Kpath_subset[Cuts_kpath.processed_num + i][k]]; r++)
				{
					//点colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[Cuts_kpath.processed_num+i][k]][r]必须不被kpath包含
					if (0 == Cuts_kpath.Kpath_indicator[Cuts_kpath.processed_num + i][colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[Cuts_kpath.processed_num + i][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			c_Kpath+= Rmp_routevar_x[j] * temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env,2, c_Kpath, IloInfinity));
		//记录SDC序号
		Cuts_kpath.Kpath_cons_index[Cuts_kpath.processed_num + i] = Rmp_ctall.getSize() - 1;
		//新SDC添加到Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_kpath.Kpath_cons_index[Cuts_kpath.processed_num + i]]);
		c_Kpath.end();
		//每添加一行（约束）就添加对应的一个初始状态
		allcon_stat.add(IloCplex::AtLower);
	}
#endif
#if RCCUT == 1
	for (i = 0; i < Cuts_rcc.added_num; i++)
	{
		IloExpr c_RCC(Rmp_matrix.env);
		//构建左侧系数
		for (j = 0; j < Rmp_CandidateColumn_Num - Added_columnNum_once; j++)
		{
			column_index = BB.branch[BB.cur_node].used_column[j];
			temp_factor = 0;
			//rcc中包含的每个点
			for (k = 0; k < Cuts_rcc.RCC_subset_len[Cuts_rcc.processed_num + i]; k++)
			{
				//列colp.Col[column_index]到达rcc中点Cuts_rcc.RCC_subset[Cuts_rcc.processed_num+i][k]之前经过的点
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_rcc.RCC_subset[Cuts_rcc.processed_num + i][k]]; r++)
				{
					//点colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[Cuts_rcc.processed_num+i][k]][r]必须不被kpath包含
					if (0 == Cuts_rcc.RCC_indicator[Cuts_rcc.processed_num + i][colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[Cuts_rcc.processed_num + i][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			c_RCC += Rmp_routevar_x[j] * temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env, Cuts_rcc.RCC_RHS[Cuts_rcc.processed_num + i], c_RCC, IloInfinity));
		//记录SDC序号
		Cuts_rcc.RCC_cons_index[Cuts_rcc.processed_num + i] = Rmp_ctall.getSize() - 1;
		//新SDC添加到Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_rcc.RCC_cons_index[Cuts_rcc.processed_num + i]]);
		c_RCC.end();
		//每添加一行（约束）就添加对应的一个初始状态
		allcon_stat.add(IloCplex::AtLower);
	}
#endif
	//添加一个新的subset-row cuts
#if SRCUT == 1
	for (i = 0; i < Cuts_src.added_num; i++)
	{
		IloExpr c_SRC(Rmp_matrix.env);
		//构建左侧系数
		for (j = 0; j < Rmp_CandidateColumn_Num - Added_columnNum_once; j++)
		{
			column_index = BB.branch[BB.cur_node].used_column[j];
			temp_factor = 0;
			//一个SR-cuts内包含的每个点
			for (k = 0; k < Cuts_src.SRC_subset_len[Cuts_src.processed_num+i]; k++)
			{
				//列colp.Col[column_index]到达SR-cuts包含的点Cuts_src.SRC_subset[Cuts_src.processed_num+i][k]的次数
				temp_factor = temp_factor + colp.Col[column_index].Customer_indicator[Cuts_src.SRC_subset[Cuts_src.processed_num + i][k]];
			}
			temp_factor = Get_SRC_Cof(temp_factor, Cuts_src.processed_num + i, Cuts_src);
			c_SRC += Rmp_routevar_x[j] * temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 0, c_SRC, Cuts_src.SRC_RHS[Cuts_src.processed_num + i]));
		//记录SRC序号
		Cuts_src.SRC_cons_index[Cuts_src.processed_num + i] = Rmp_ctall.getSize() - 1;
		//新SRC添加到Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_src.SRC_cons_index[Cuts_src.processed_num + i]]);
		c_SRC.end();
		//每添加一行（约束）就添加对应的一个初始状态
		allcon_stat.add(IloCplex::AtLower);
	}
#endif

	//再添加列
	for (i = Added_columnNum_once; i > 0; i--)
	{
		column_index = BB.branch[BB.cur_node].used_column[Rmp_CandidateColumn_Num - i];
		IloNumColumn col = Rmp_obj(colp.Col[column_index].Totalcost);
		for (j = 0; j < p.Customer_Num; j++)
		{
			col += Rmp_ctall[j](colp.Col[column_index].Customer_indicator[j]);
		}
		for (j = 0; j < p.Depot_Num; j++)
		{
			col += Rmp_ctall[p.Customer_Num + j](colp.Col[column_index].depot_indicator[j]);
		}

		//SDC 
#if Frameworks == 1
		for (j = 0; j < Cuts_sdc.processed_num + Cuts_sdc.added_num; j++)
		{
#if SDCTYPE == 0
			temp_factor = fmin(1, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[j]]);
#elif SDCTYPE == 1
			temp_factor = 1;
			for (k = 0; k < colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[j]]; k++)
			{
				//判断路径colp.Col[column_index]上以colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k]为起点的弧是否Augng-feasible
				if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k], colp.Col[column_index]))
				{
					colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k] = -1;
				}
				else
				{
					temp_factor = temp_factor + 1;
				}
			}
			//Augng_infeasible的cycle从colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]]中剔除
			colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[j]);
			temp_factor = fmin(temp_factor, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[j]]);
#endif
			col += Rmp_ctall[Cuts_sdc.SDC_cons_index[j]](temp_factor);	//不确定向CPLEX约束中直接传入min/max表达式是否会影响运行效率
																		//记录SDC_cof
			colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[j]] = temp_factor;
		}
#endif
		//注意Cut对应的行也要添加列
#if KPATHCUT == 1
		//每个kpath
		for (j = 0; j < Cuts_kpath.processed_num+Cuts_kpath.added_num; j++)
		{
			temp_factor = 0;
			//一个kpath内包含的每个点
			for (k = 0; k < Cuts_kpath.Kpath_subset_len[j]; k++)
			{
				//列colp.Col[column_index]到达kpath包含的点cuts_kpath.Kpath_subset[j][k]之前经过的点
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_kpath.Kpath_subset[j][k]]; r++)
				{
					//点colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[j][k]][r]必须不被kpath包含
					if (0 == Cuts_kpath.Kpath_indicator[j][colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[j][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			col += Rmp_ctall[Cuts_kpath.Kpath_cons_index[j]](temp_factor);
		}
#endif
#if RCCUT == 1
		//每个rcc
		for (j = 0; j < Cuts_rcc.processed_num + Cuts_rcc.added_num; j++)
		{
			temp_factor = 0;
			//一个rcc内包含的每个点
			for (k = 0; k < Cuts_rcc.RCC_subset_len[j]; k++)
			{
				//列colp.Col[column_index]到达kpath包含的点Cuts_rcc.RCC_subset[j][k]之前经过的点
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_rcc.RCC_subset[j][k]]; r++)
				{
					//点colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[j][k]][r]必须不被rcc包含
					if (0 == Cuts_rcc.RCC_indicator[j][colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[j][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			col += Rmp_ctall[Cuts_rcc.RCC_cons_index[j]](temp_factor);
		}
#endif
#if SRCUT == 1
		//每个subset-row cuts
		for (j = 0; j < Cuts_src.processed_num + Cuts_src.added_num; j++)
		{
			temp_factor = 0;
			//一个SR-cuts内包含的每个点
			for (k = 0; k < Cuts_src.SRC_subset_len[j]; k++)
			{
				//列colp.Col[column_index]到达SR-cuts包含的点Cuts_src.SRC_subset[j][k]的次数
				temp_factor = temp_factor + colp.Col[column_index].Customer_indicator[Cuts_src.SRC_subset[j][k]];
			}
			temp_factor = Get_SRC_Cof(temp_factor, j, Cuts_src);
			col += Rmp_ctall[Cuts_src.SRC_cons_index[j]](temp_factor);
		}
#endif

		Rmp_routevar_x.add(IloNumVar(col, 0, IloInfinity, ILOFLOAT));
		col.end();
		//每添加一列就添加对应的一个初始解
		var_stat.add(IloCplex::AtLower);
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//先warm start
	//Rmp_cplex.setBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);
	Rmp_cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal);
	Rmp_cplex.setOut(Rmp_matrix.env.getNullStream());
	Rmp_cplex.setWarning(Rmp_matrix.env.getNullStream());
	Rmp_cplex.solve();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//判断模型求解情况
	if (Rmp_cplex.getStatus() == IloAlgorithm::Infeasible || Rmp_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unknown || Rmp_cplex.getStatus() == IloAlgorithm::Error)
	{
		Rmp_matrix.env.out() << "ERROR:No Solution" << endl;
		cin.get();
	}
	else
	{
		//获取求解基解信息，为下次计算提供初始解
		Rmp_cplex.getBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);
		//获取目标函数值
		OBJ_value = Rmp_cplex.getObjValue();
		SolInfo.Cur_solution.OBJ_lower= OBJ_value;
		//获取决策变量的值
		SolInfo.Cur_solution.best_LBsol_num = 0;
		for (i = 0; i < Rmp_CandidateColumn_Num; i++)
		{
			temp_value= Rmp_cplex.getValue(Rmp_routevar_x[i]);
			if (temp_value>MINDOUBLE)
			{
				SolInfo.Cur_solution.best_LBsol_value[SolInfo.Cur_solution.best_LBsol_num] = temp_value;
				SolInfo.Cur_solution.best_LBsol_index[SolInfo.Cur_solution.best_LBsol_num] = BB.branch[BB.cur_node].used_column[i];
				SolInfo.Cur_solution.best_LBsol_order[SolInfo.Cur_solution.best_LBsol_num] = i;
				SolInfo.Cur_solution.best_LBsol_num = SolInfo.Cur_solution.best_LBsol_num + 1;
			}
		}
		//获取对偶最优解,也就是约束对应的影子价格
		for (i = 0; i<p.Customer_Num; i++)
		{
			Customer_dual[i] = Rmp_cplex.getDual(Rmp_ctall[i]);//getDual方法的参数必须是IloRangeArray类，所以这也是取约束别名的原因
		}
		for (i = 0; i < p.Depot_Num; i++)
		{
			Vehicle_dual[i] = Rmp_cplex.getDual(Rmp_ctall[p.Customer_Num + i]);
		}
		//SDC对应的对偶变量
#if Frameworks == 1
		for (i = 0; i < Cuts_sdc.processed_num + Cuts_sdc.added_num; i++)
		{
			SDC_dual[Cuts_sdc.SDC_nodes[i]] = Rmp_cplex.getDual(Rmp_ctall[Cuts_sdc.SDC_cons_index[i]]);
		}
		Cuts_sdc.processed_num = Cuts_sdc.processed_num + Cuts_sdc.added_num;
		Cuts_sdc.added_num = 0;
		if (true == Cuts_sdc.ngset_change)
		{
			for (i = 0; i < p.Customer_Num; i++)
			{
				Cuts_sdc.ngset_addNum[i] = 0;
			}
			Cuts_sdc.ngset_change = false;
		}
		//Verify_SDCcons(SolInfo, colp, p,Cuts_sdc);
#endif

		//Robust-cut的对偶变量都被整合到弧RobustCut_dual上
#if KPATHCUT+RCCUT >0
		if (Cuts_kpath.processed_num + Cuts_kpath.added_num+ Cuts_rcc.processed_num + Cuts_rcc.added_num > 0)
		{
			//重置
			for (i = 0; i < p.Customer_Num + p.Depot_Num; i++)
			{
				for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
				{
					RobustCut_dual[i][j] = 0;
				}
			}
		}
#endif
#if KPATHCUT == 1
		//每个kpath
		for (i = 0; i < Cuts_kpath.processed_num + Cuts_kpath.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_kpath.Kpath_cons_index[i]]);
			//所有从kpath子集外入射弧
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0 == Cuts_kpath.Kpath_indicator[i][j])
				{
					//一个kpath内包含所有节点
					for (k = 0; k < Cuts_kpath.Kpath_subset_len[i]; k++)
					{
						RobustCut_dual[j][Cuts_kpath.Kpath_subset[i][k]] += temp_value;
					}
				}
			}
		}
		Cuts_kpath.processed_num = Cuts_kpath.processed_num + Cuts_kpath.added_num;
		Cuts_kpath.added_num = 0;
#endif
#if RCCUT == 1
		//每个rcc
		for (i = 0; i < Cuts_rcc.processed_num + Cuts_rcc.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_rcc.RCC_cons_index[i]]);
			//所有从rcc子集外入射弧
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0 == Cuts_rcc.RCC_indicator[i][j])
				{
					//一个rcc内包含所有节点
					for (k = 0; k < Cuts_rcc.RCC_subset_len[i]; k++)
					{
						RobustCut_dual[j][Cuts_rcc.RCC_subset[i][k]] += temp_value;
					}
				}
			}
		}
		Cuts_rcc.processed_num = Cuts_rcc.processed_num + Cuts_rcc.added_num;
		Cuts_rcc.added_num = 0;
#endif
#if SRCUT == 1
		for (i = 0; i < Cuts_src.processed_num + Cuts_src.added_num; i++)
		{
			SDC_dual[i] = Rmp_cplex.getDual(Rmp_ctall[Cuts_src.SRC_cons_index[i]]);
		}
		Cuts_src.processed_num = Cuts_src.processed_num + Cuts_src.added_num;
		Cuts_src.added_num = 0;
#endif

		//Positive_variable_num = 0;
		////获取决策变量的值
		//for (i = 0; i<temp_rmp.Rmp_candidate_route_num; i++)
		//{
		//	routeset[branchnodes[node].used_column[i]].solution = temp_rmp.RMP_result[i];
		//	if (temp_rmp.RMP_result[i]>0.0001)
		//	{
		//		S[Positive_variable_num] = branchnodes[node].used_column[i];
		//		Positive_variable_num = Positive_variable_num + 1;
		//	}
		//}
	}

	return feasible_ornot;
}

bool RMP::StandardRMP_SolvebyCPLEX_Int(SolINFOR &SolInfo, BranchABound &BB, ColumnPool & colp)
{
	bool feasible_ornot = true;
	int column_index = 0;
	float temp_value;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////构建cplex环境下的模型
	IloInt i;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//转化Rmp_routevar_x的类型从float到int
	Rmp_model.add(IloConversion(Rmp_matrix.env, Rmp_routevar_x, ILOINT));
	Rmp_cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal);
	Rmp_cplex.setParam(Rmp_cplex.EpGap, 0.05);
	//Rmp_cplex.setParam(Rmp_cplex.TiLim, 30); //设置求解时间
	Rmp_cplex.setOut(Rmp_matrix.env.getNullStream());
	Rmp_cplex.setWarning(Rmp_matrix.env.getNullStream());
	Rmp_cplex.solve();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//判断模型求解情况
	if (Rmp_cplex.getStatus() == IloAlgorithm::Infeasible || Rmp_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unknown || Rmp_cplex.getStatus() == IloAlgorithm::Error)
	{
		Rmp_matrix.env.out() << "No Solution" << endl;
		feasible_ornot = false;
		OBJ_value = MAXNUM;
		//cin.get();
	}
	else
	{
		//获取目标函数值
		OBJ_value = Rmp_cplex.getObjValue();
		SolInfo.Cur_solution.OBJ_upper = OBJ_value;
		//获取决策变量的值
		SolInfo.Cur_solution.best_UPsol_num = 0;
		for (i = 0; i < Rmp_CandidateColumn_Num; i++)
		{
			temp_value = Rmp_cplex.getValue(Rmp_routevar_x[i]);
			if (temp_value>MINDOUBLE)
			{
				SolInfo.Cur_solution.best_UPsol_value[SolInfo.Cur_solution.best_UPsol_num] = temp_value;
				SolInfo.Cur_solution.best_UPsol_index[SolInfo.Cur_solution.best_UPsol_num] = BB.branch[BB.cur_node].used_column[i];
				SolInfo.Cur_solution.best_UPsol_num = SolInfo.Cur_solution.best_UPsol_num + 1;
			}
		}
	}

	return feasible_ornot;
}

void RMP::Update_RMP(BranchABound &BB, ColumnPool & colp)
{
	//Added_columnNum_once = colp.CandidateColumn_Num - Rmp_CandidateColumn_Num;
	Added_columnNum_once = BB.branch[BB.cur_node].used_column_num- Rmp_CandidateColumn_Num;
	Rmp_CandidateColumn_Num = Rmp_CandidateColumn_Num + Added_columnNum_once;
}

bool RMP::Strengthen_elementary(Utility & Results, SolINFOR & SolInfo, BranchABound &BB, ColumnPool & colp, Problem &p, SDC & Cuts_sdc)
{
	bool ele_ornot = true;
	int temp_index, temp_cus;
	float temp_indicator;

#if AUGMENT == 1
	do
	{
		if (true == Calculate_SDCindicator(SolInfo, colp, p, Cuts_sdc))
		{
			break;
		}
		else
		{
			//对Cuts_sdc.Rep_customers按照Indicator降序排序，找出最多前Conf::MAX_ADD_SDC个重复经过的客户
			Cuts_sdc.Find_SDC(Results, p);
#if SDCTYPE == 1
			//更新Aug_ngset
			Update_AugNgset(SolInfo, colp, p, Cuts_sdc);
#endif
			ele_ornot = false;
			break;
		}
	} while (1);
#else
	Todelete_col_num = 0;
	//遍历当前解SolInfo.Cur_solution中的所有路径
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		//遍历当前路径经过的每个客户
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 1; j++)
		{
			temp_cus = colp.Col[temp_index].nodes[j];
			//寻找环
			if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 2)
			{
				//形成环，则该cycle的重复点为temp_cus
				//从位置j向后探索直到找到第一个相同的temp_cus，形成cycle
				p.Generate_cycle(colp.Col[temp_index].nodesNum, colp.Col[temp_index].nodes, j);
				//对每个cycle中的节点都更新ngset
#if ClearColumn == 0
				p.Add_ngset_byCycle();
				//只要成环，就不允许再出现在RMP中
				colp.Col[temp_index].Totalcost = MAXNUM;
				//更新Todelete_col
				Todelete_col[Todelete_col_num] = SolInfo.Cur_solution.best_LBsol_order[i];
				Todelete_col_num = Todelete_col_num + 1;
#else
				if (false == p.Add_ngset_byCycle())
				{
					//当列池中一个cycle重复出现时，就需要检查该分支点上的整个列池
					//并删除所有包含该cycle的column（把Totalcost置为MAXNUM）
					DeleteColumn_byCycle(BB, colp, p);
				}
				else
				{
					//只要成环，就不允许再出现在RMP中
					colp.Col[temp_index].Totalcost = MAXNUM;
					//更新Todelete_col
					Todelete_col[Todelete_col_num] = SolInfo.Cur_solution.best_LBsol_order[i];
					Todelete_col_num = Todelete_col_num + 1;
				}
#endif
				ele_ornot = false;
				//这里只是找到第一个环
				break;
			}
		}
	}
#endif

	return ele_ornot;
}

bool RMP::Calculate_SDCindicator(SolINFOR & SolInfo, ColumnPool & colp, Problem & p, SDC & Cuts_sdc)
{
	bool ele_ornot = true;
	int temp_index, temp_cus;
	float temp_indicator;

	//重置Rep_customers
	Cuts_sdc.Reset_RepCus(p);
	//遍历当前解SolInfo.Cur_solution中的所有路径
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		//遍历当前路径经过的每个客户
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 1; j++)
		{
			temp_cus = colp.Col[temp_index].nodes[j];
#if SDCindicator == 0
			if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 2)
			{
				//更新重复经过次数
#if SDCTYPE == 0
				temp_indicator = 1;
#elif SDCTYPE == 1
				temp_indicator = 1;
#endif
				Cuts_sdc.Rep_customers[temp_cus].Indicator = Cuts_sdc.Rep_customers[temp_cus].Indicator + temp_indicator;
				ele_ornot = false;
			}
#else
			if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 1)
			{
#if SDCTYPE == 0
				temp_indicator = -SolInfo.Cur_solution.best_LBsol_value[i] / colp.Col[temp_index].Customer_indicator[temp_cus];
#elif SDCTYPE == 1
				temp_indicator = -SolInfo.Cur_solution.best_LBsol_value[i] / colp.Col[temp_index].Customer_indicator[temp_cus];
#endif
				Cuts_sdc.Rep_customers[temp_cus].Indicator = Cuts_sdc.Rep_customers[temp_cus].Indicator + temp_indicator;
				if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 2)
				{
					ele_ornot = false;
				}
			}
#endif
		}
	}

	return ele_ornot;
}

void RMP::DeleteColumn_byCycle(BranchABound &BB, ColumnPool & colp, Problem & p)
{
	//注意：以下方法不保证一定准确删除包含p.cycle_node的列
	//只是很大概率上保证能够删除

	int temp_index;
	bool temp_check;
	//遍历列池
	for (int i = 0; i < BB.branch[BB.cur_node].used_column_num; i++)
	{
		temp_index= BB.branch[BB.cur_node].used_column[i];
		//首先p.cycle_node的端点必须被经过两次
		if (colp.Col[temp_index].Customer_indicator[p.cycle_node[0]] >= 2)
		{
			temp_check = true;
			//路径colp.Col[temp_index]上p.cycle_node[j]的succ_arcs包含p.cycle_node[j+1]
			for (int j = 0; j < p.cycle_node_num-1; j++)
			{
				if (false==Contained_Byset(p.cycle_node[j+1], colp.Col[temp_index].succ_arcs[p.cycle_node[j]], colp.Col[temp_index].succ_arcs_Num[p.cycle_node[j]]))
				{
					temp_check = false;
					break;
				}
			}
			if (true== temp_check)
			{
				//重置Totalcost为MAXNUM
				colp.Col[temp_index].Totalcost = MAXNUM;
				//加入Todelete_col
				Todelete_col[Todelete_col_num] = i;
				Todelete_col_num = Todelete_col_num + 1;
			}
		}
	}
}

void RMP::Update_flow(SolINFOR & SolInfo, ColumnPool & colp, Problem & p)
{
	int temp_start, temp_end;
	//重置Arc_flow
	for (int i = 0; i < p.Customer_Num+p.Depot_Num; i++)
	{
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			Arc_flow[i][j] = 0;
		}
	}
	//赋值Arc_flow
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		for (int j = 0; j < colp.Col[SolInfo.Cur_solution.best_LBsol_index[i]].nodesNum - 1; j++)
		{
			temp_start = colp.Col[SolInfo.Cur_solution.best_LBsol_index[i]].nodes[j];
			temp_end = colp.Col[SolInfo.Cur_solution.best_LBsol_index[i]].nodes[j + 1];
			Arc_flow[temp_start][temp_end] += SolInfo.Cur_solution.best_LBsol_value[i];
		}
	}
}

void RMP::Update_AugNgset(SolINFOR & SolInfo, ColumnPool & colp, Problem & p, SDC & Cuts_sdc)
{
#if Frameworks == 1
	int temp_index;
	//遍历每个需要拓展ngset的点
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (Cuts_sdc.ngset_addNum[i]>0)
		{
			//遍历当前解SolInfo.Cur_solution中的所有路径
			for (int j = 0; j < SolInfo.Cur_solution.best_LBsol_num; j++)
			{
				temp_index = SolInfo.Cur_solution.best_LBsol_index[j];
				//该路径上是否存在以i为起点的cycle
				for (int k = 0; k < colp.Col[temp_index].cus_cyclePosi_num[i]; k++)
				{
					//对该路径上每个以i为起点的cycle都拓展ngset
					//从位置colp.Col[temp_index].cus_cyclePosi[i][k]向前探索直到找到第一个相同的i，形成cycle
					p.Generate_cycle_back(colp.Col[temp_index].nodesNum, colp.Col[temp_index].nodes, colp.Col[temp_index].cus_cyclePosi[i][k]);
					//对每个cycle中的节点都更新ngset
					p.Add_Angngset_byCycle();
				}
			}
		}
	}
#endif
}

int RMP::Get_SRC_Cof(int visitnum, int SRC_no, SRC & Cuts_src)
{
	float multiplier = 0;

	if (3==Cuts_src.SRC_subset_len[SRC_no])
	{
		multiplier = Conf::SR_MULTIPY_3;
	}
	else if(4 == Cuts_src.SRC_subset_len[SRC_no])
	{
		multiplier = Conf::SR_MULTIPY_4;
	}
	else if (5 == Cuts_src.SRC_subset_len[SRC_no])
	{
		multiplier = Conf::SR_MULTIPY_5;
	}
	else
	{
		cout << "ERROR：SRC不等式错误" << endl;
		cin.get();
	}

	int cof = floor(visitnum*multiplier);
	return cof;
}

void RMP::reset_RmpMatrix_byclass(void)
{
	Rmp_routevar_x.end();
	Rmp_ctall.end();
	Rmp_obj.end();
	Rmp_cplex.clearModel();		//这个可能没有用
	Rmp_cplex.end();
	var_stat.end();
	allcon_stat.end();

	Rmp_matrix.env.end();
}
