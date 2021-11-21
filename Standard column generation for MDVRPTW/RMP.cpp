#include "stdafx.h"
#include "RMP.h"
#include "Conf.h"



RMP::RMP()
{
}

RMP::~RMP()
{

}

//���������ϵ
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



//����RMP��ǰ���Ƿ�����SDCԼ��
void Verify_SDCcons(SolINFOR &SolInfo, ColumnPool &colp, Problem & p,SDC &Cuts_sdc)
{
	int temp_index, temp_cus;
	//�����Ƿ���ڲ���Լ����node
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 1; j++)
		{
			temp_cus = colp.Col[temp_index].nodes[j];
			if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 2 && 1==Cuts_sdc.SDC_indicator[temp_cus])
			{
				cout << "ERROR��SDCԼ����Ч" << endl;
				cin.get();
			}
		}
	}
	//��������Լ��
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
			cout << "ERROR��nodeԼ����Ч" << endl;
			cin.get();
		}
	}
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (1 == Cuts_sdc.SDC_indicator[i] && temp_RHS[p.Customer_Num+i]<1- MINDOUBLE)
		{
			cout << "ERROR��SDCԼ����Ч" << endl;
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

	//��ʼ��SDC
#if Frameworks == 1
	Cuts_sdc.added_num = Cuts_sdc.processed_num;
	Cuts_sdc.processed_num = 0;
	//��żֵ��ʼ��Ϊ��ֵ
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
	//��ʼ��kpath
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
	//��ʼ��RCC
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
	//��ʼ��src
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

	//�ٳ�ʼ��Rmp_Matrix
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
	//����CPLEXģ��������ģʽ��1,������ӣ�2,�������
	//�������в���������ӵ�ģʽ������ʼģ�ͣ�
	//Ȼ���������ɹ����в�������е�ģʽ��
	//����SDCcuts��ֻ�������������ǲ�ȡ��Ӳ�������е�ģʽ
	//����SRcuts�����ܻ���٣������ǿ�����ǰ���ɿհ��У�ϵ��Ϊ0���Ҳೣ��Ϊ0����������������޸��е�ϵ������ʽ��ʹ�õ�IloRange::setLinearCoef (for matrix)��IloRange::setBounds (for rhs)����
	//����Ԥʵ�飬Ԥ���հ��еķ�ʽ��CPLEX����ʱ�����ӵıȽϿ�
	//���޸ģ�����Ԥ���հ��У�������ɾ��SR-cutsʱ��ֱ���滻ϵ�������޸�ϵ��Ϊ�հ���
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Ŀ�꺯��
	Rmp_obj = IloAdd(Rmp_model, IloMinimize(Rmp_matrix.env));
	////����ͨ����ʽ����еķ�ʽ�ȹ�����ʼģ��RMP_model
	IloInt i,j,k,r;
	//����ÿ��Լ���ķ�Χ
	//set-coveringԼ��
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
	//vehicle numberԼ��
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

	//�������cut
	//��ô����Rmp_model������У��������
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

	//���뵽ģ����
	Rmp_model.add(Rmp_ctall);
	//��һ����˵��һ��һ������ϵ��
	int column_index;
	for (i = 0; i < Rmp_CandidateColumn_Num; i++)
	{
		column_index = BB.branch[BB.cur_node].used_column[i];
		//�ȸ�ֵ��Ŀ�꺯����ϵ��
		IloNumColumn col = Rmp_obj(colp.Col[column_index].Totalcost);
		//Ȼ��ֵ��Լ����ϵ��
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
		//ϵ��=1+ Augng-feasible-cycle������
		int temp_factor;
		for (j = 0; j < Cuts_sdc.added_num; j++)
		{
#if SDCTYPE == 0
			temp_factor = fmin(1, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]);
#elif SDCTYPE == 1
			temp_factor = 1;
			for (k = 0; k < colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]; k++)
			{
				//�ж�·��colp.Col[column_index]����colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]][k]Ϊ���Ļ��Ƿ�Augng-feasible
				if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]][k], colp.Col[column_index]))
				{
					colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]][k] = -1;
				}
				else
				{
					temp_factor = temp_factor + 1;
				}
			}
			//Augng_infeasible��cycle��colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]���޳�
			colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]);
			temp_factor = fmin(temp_factor, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]]);
#endif
			col += Rmp_ctall[p.Customer_Num + p.Depot_Num + j](temp_factor);	//��ȷ����CPLEXԼ����ֱ�Ӵ���min/max���ʽ�Ƿ��Ӱ������Ч��
																				//��¼SDC_cof
			colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + j]] = temp_factor;
		}
#endif

		//ע��Cut��Ӧ����ҲҪ�����
#if KPATHCUT == 1
		//ÿ��kpath
		for (j = 0; j < Cuts_kpath.added_num; j++)
		{
			temp_factor = 0;
			//һ��kpath�ڰ�����ÿ����
			for (k = 0; k < Cuts_kpath.Kpath_subset_len[j]; k++)
			{
				//��colp.Col[column_index]����kpath�����ĵ�cuts_kpath.Kpath_subset[j][k]֮ǰ�����ĵ�
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_kpath.Kpath_subset[j][k]]; r++)
				{
					//��colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[j][k]][r]���벻��kpath����
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
		//ÿ��rcc
		for (j = 0; j < Cuts_rcc.added_num; j++)
		{
			temp_factor = 0;
			//һ��rcc�ڰ�����ÿ����
			for (k = 0; k < Cuts_rcc.RCC_subset_len[j]; k++)
			{
				//��colp.Col[column_index]����rcc�����ĵ�Cuts_rcc.RCC_subset[j][k]֮ǰ�����ĵ�
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_rcc.RCC_subset[j][k]]; r++)
				{
					//��colp.Col[column_index].pre_arcs[Cuts_kpath.RCC_subset[j][k]][r]���벻��rcc����
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
		//ÿ��SR-cuts
		for (j = 0; j < Cuts_src.added_num; j++)
		{
			temp_factor = 0;
			//һ��SR-cuts�ڰ�����ÿ����
			for (k = 0; k < Cuts_src.SRC_subset_len[j]; k++)
			{
				//��colp.Col[column_index]����SR-cuts�����ĵ�Cuts_src.SRC_subset[j][k]�Ĵ���
				temp_factor = temp_factor + colp.Col[column_index].Customer_indicator[Cuts_src.SRC_subset[j][k]];
			}
			temp_factor = Get_SRC_Cof(temp_factor,j, Cuts_src);
			col += Rmp_ctall[Cuts_src.SRC_cons_index[j]](temp_factor);
		}
#endif

		//���뵽���ж�Ӧ�ı�����
		Rmp_routevar_x.add(IloNumVar(col, 0, IloInfinity, ILOFLOAT));
		col.end();
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Rmp_cplex.setOut(Rmp_matrix.env.getNullStream());
	Rmp_cplex.setWarning(Rmp_matrix.env.getNullStream());
	Rmp_cplex.solve();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//�ж�ģ��������
	if (Rmp_cplex.getStatus() == IloAlgorithm::Infeasible || Rmp_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unknown || Rmp_cplex.getStatus() == IloAlgorithm::Error)
	{
		Rmp_matrix.env.out() << "No Solution" << endl;
		feasible_ornot = false;
	}
	else
	{
		//��ȡ��������Ϣ��Ϊ�´μ����ṩ��ʼ��
		Rmp_cplex.getBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);
		//��ȡĿ�꺯��ֵ
		OBJ_value = Rmp_cplex.getObjValue();
		SolInfo.Cur_solution.OBJ_lower = OBJ_value;
		//��ȡ���߱�����ֵ
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
		//��ȡ��ż���Ž�,Ҳ����Լ����Ӧ��Ӱ�Ӽ۸�
		for (i = 0; i<p.Customer_Num; i++)
		{
			Customer_dual[i] = Rmp_cplex.getDual(Rmp_ctall[i]);//getDual�����Ĳ���������IloRangeArray�࣬������Ҳ��ȡԼ��������ԭ��
		}
		for (i = 0; i < p.Depot_Num; i++)
		{
			Vehicle_dual[i] = Rmp_cplex.getDual(Rmp_ctall[p.Customer_Num+i]);
		}
		//SDC�Ķ�ż����
#if Frameworks == 1
		for (i = 0; i < Cuts_sdc.added_num; i++)
		{
			SDC_dual[Cuts_sdc.SDC_nodes[i]] = Rmp_cplex.getDual(Rmp_ctall[Cuts_sdc.SDC_cons_index[i]]);
		}
		Cuts_sdc.processed_num = Cuts_sdc.processed_num + Cuts_sdc.added_num;
		Cuts_sdc.added_num = 0;
		Cuts_sdc.ngset_change = false;
#endif

		//Robust-cut�Ķ�ż�����������ϵ���RobustCut_dual��
#if KPATHCUT == 1
		//ÿ��kpath
		for (i = 0; i < Cuts_kpath.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_kpath.Kpath_cons_index[i]]);
			//���д�kpath�Ӽ������仡
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0==Cuts_kpath.Kpath_indicator[i][j])
				{
					//һ��kpath�ڰ������нڵ�
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
		//ÿ��kpath
		for (i = 0; i < Cuts_rcc.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_rcc.RCC_cons_index[i]]);
			//���д�kpath�Ӽ������仡
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0 == Cuts_rcc.RCC_indicator[i][j])
				{
					//һ��kpath�ڰ������нڵ�
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
		//nonrobust-cut��Ӧ�Ķ�ż��������Ҫ���ϵ�����
#if SRCUT == 1
		//ÿ��SRC
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
	////����cplex�����µ�ģ��
	IloInt i,j,k,r;
	//��warm start
	Rmp_cplex.setBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);

	//����Todelete_col���޸�Ŀ�꺯��ϵ��
	for (i = 0; i < Todelete_col_num; i++)
	{
		Rmp_obj.setLinearCoef(Rmp_routevar_x[Todelete_col[i]], MAXNUM);
	}
	Todelete_col_num = 0;
	
	//�������cut
	//��ô����Rmp_model������У��������
#if Frameworks == 1

#if SDCTYPE == 1
	//���޸�SDC�е�ϵ��
	if (true == Cuts_sdc.ngset_change)
	{
		for (i = 0; i < Rmp_CandidateColumn_Num - Added_columnNum_once; i++)
		{
			column_index = BB.branch[BB.cur_node].used_column[i];
			//�����л����п���ϵ���ı�
			if (false == colp.Col[column_index].elementary_ornot)
			{
				for (j = 0; j < Cuts_sdc.processed_num; j++)
				{
					//�ڵ�Cuts_sdc.SDC_nodes[j]��Aug_ngset�����仯�ˣ����п���ϵ���ı�
					if (1 == Cuts_sdc.ngset_addNum[Cuts_sdc.SDC_nodes[j]])
					{
						//��Cuts_sdc.SDC_nodes[j]�ͻ�����cycle���п���ϵ���ı�
						if (colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[j]]>0)
						{
							temp_factor = 1;
							for (k = 0; k < colp.Col[column_index].cus_cyclePosi_num[Cuts_sdc.SDC_nodes[j]]; k++)
							{
								//�ж�·��colp.Col[column_index]����colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k]Ϊ���Ļ��Ƿ�Augng-feasible
								if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k], colp.Col[column_index]))
								{
									colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k] = -1;
								}
								else
								{
									temp_factor = temp_factor + 1;
								}
							}
							//ֻ��ϵ��ȷʵ�ı��ˣ��ŵ���setLinearCoef
							if (temp_factor != colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[j]])
							{
								Rmp_ctall[Cuts_sdc.SDC_cons_index[j]].setLinearCoef(Rmp_routevar_x[i], temp_factor);
								//Augng_infeasible��cycle��colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]]���޳�
								colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[j]);
								//��¼SDC_cof
								colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[j]] = temp_factor;
							}
						}
					}
				}
			}
		}
	}
#endif

	//�������
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
				//�ж�·��colp.Col[column_index]����colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]][k]Ϊ���Ļ��Ƿ�Augng-feasible
				if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]][k], colp.Col[column_index]))
				{
					colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]][k] = -1;
				}
				else
				{
					temp_factor = temp_factor + 1;
				}
			}
			//Augng_infeasible��cycle��colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]]���޳�
			colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]);
			temp_factor = fmin(temp_factor, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]]);
#endif
			c_sdc += Rmp_routevar_x[j] * temp_factor;
			//��¼SDC_cof
			colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[Cuts_sdc.processed_num + i]] = temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 1, c_sdc, IloInfinity));
		//��¼SDC���
		Cuts_sdc.SDC_cons_index[Cuts_sdc.processed_num + i] = Rmp_ctall.getSize() - 1;
		//��SDC��ӵ�Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_sdc.SDC_cons_index[Cuts_sdc.processed_num + i]]);
		c_sdc.end();
		//ÿ���һ�У�Լ��������Ӷ�Ӧ��һ����ʼ״̬
		allcon_stat.add(IloCplex::AtLower);
						}
#endif
	//���kpath-cut
#if KPATHCUT == 1
	for (i = 0; i < Cuts_kpath.added_num; i++)
	{
		IloExpr c_Kpath(Rmp_matrix.env);
		//�������ϵ��
		for (j = 0; j < Rmp_CandidateColumn_Num - Added_columnNum_once; j++)
		{
			column_index = BB.branch[BB.cur_node].used_column[j];
			temp_factor = 0;
			//Kpath�а�����ÿ����
			for (k = 0; k < Cuts_kpath.Kpath_subset_len[Cuts_kpath.processed_num+i]; k++)
			{
				//��colp.Col[column_index]����kpath�е�cuts_kpath.Kpath_subset[Cuts_kpath.processed_num+i][k]֮ǰ�����ĵ�
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_kpath.Kpath_subset[Cuts_kpath.processed_num + i][k]]; r++)
				{
					//��colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[Cuts_kpath.processed_num+i][k]][r]���벻��kpath����
					if (0 == Cuts_kpath.Kpath_indicator[Cuts_kpath.processed_num + i][colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[Cuts_kpath.processed_num + i][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			c_Kpath+= Rmp_routevar_x[j] * temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env,2, c_Kpath, IloInfinity));
		//��¼SDC���
		Cuts_kpath.Kpath_cons_index[Cuts_kpath.processed_num + i] = Rmp_ctall.getSize() - 1;
		//��SDC��ӵ�Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_kpath.Kpath_cons_index[Cuts_kpath.processed_num + i]]);
		c_Kpath.end();
		//ÿ���һ�У�Լ��������Ӷ�Ӧ��һ����ʼ״̬
		allcon_stat.add(IloCplex::AtLower);
	}
#endif
#if RCCUT == 1
	for (i = 0; i < Cuts_rcc.added_num; i++)
	{
		IloExpr c_RCC(Rmp_matrix.env);
		//�������ϵ��
		for (j = 0; j < Rmp_CandidateColumn_Num - Added_columnNum_once; j++)
		{
			column_index = BB.branch[BB.cur_node].used_column[j];
			temp_factor = 0;
			//rcc�а�����ÿ����
			for (k = 0; k < Cuts_rcc.RCC_subset_len[Cuts_rcc.processed_num + i]; k++)
			{
				//��colp.Col[column_index]����rcc�е�Cuts_rcc.RCC_subset[Cuts_rcc.processed_num+i][k]֮ǰ�����ĵ�
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_rcc.RCC_subset[Cuts_rcc.processed_num + i][k]]; r++)
				{
					//��colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[Cuts_rcc.processed_num+i][k]][r]���벻��kpath����
					if (0 == Cuts_rcc.RCC_indicator[Cuts_rcc.processed_num + i][colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[Cuts_rcc.processed_num + i][k]][r]])
					{
						temp_factor = temp_factor + 1;
					}
				}
			}
			c_RCC += Rmp_routevar_x[j] * temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env, Cuts_rcc.RCC_RHS[Cuts_rcc.processed_num + i], c_RCC, IloInfinity));
		//��¼SDC���
		Cuts_rcc.RCC_cons_index[Cuts_rcc.processed_num + i] = Rmp_ctall.getSize() - 1;
		//��SDC��ӵ�Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_rcc.RCC_cons_index[Cuts_rcc.processed_num + i]]);
		c_RCC.end();
		//ÿ���һ�У�Լ��������Ӷ�Ӧ��һ����ʼ״̬
		allcon_stat.add(IloCplex::AtLower);
	}
#endif
	//���һ���µ�subset-row cuts
#if SRCUT == 1
	for (i = 0; i < Cuts_src.added_num; i++)
	{
		IloExpr c_SRC(Rmp_matrix.env);
		//�������ϵ��
		for (j = 0; j < Rmp_CandidateColumn_Num - Added_columnNum_once; j++)
		{
			column_index = BB.branch[BB.cur_node].used_column[j];
			temp_factor = 0;
			//һ��SR-cuts�ڰ�����ÿ����
			for (k = 0; k < Cuts_src.SRC_subset_len[Cuts_src.processed_num+i]; k++)
			{
				//��colp.Col[column_index]����SR-cuts�����ĵ�Cuts_src.SRC_subset[Cuts_src.processed_num+i][k]�Ĵ���
				temp_factor = temp_factor + colp.Col[column_index].Customer_indicator[Cuts_src.SRC_subset[Cuts_src.processed_num + i][k]];
			}
			temp_factor = Get_SRC_Cof(temp_factor, Cuts_src.processed_num + i, Cuts_src);
			c_SRC += Rmp_routevar_x[j] * temp_factor;
		}
		Rmp_ctall.add(IloRange(Rmp_matrix.env, 0, c_SRC, Cuts_src.SRC_RHS[Cuts_src.processed_num + i]));
		//��¼SRC���
		Cuts_src.SRC_cons_index[Cuts_src.processed_num + i] = Rmp_ctall.getSize() - 1;
		//��SRC��ӵ�Rmp_model
		Rmp_model.add(Rmp_ctall[Cuts_src.SRC_cons_index[Cuts_src.processed_num + i]]);
		c_SRC.end();
		//ÿ���һ�У�Լ��������Ӷ�Ӧ��һ����ʼ״̬
		allcon_stat.add(IloCplex::AtLower);
	}
#endif

	//�������
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
				//�ж�·��colp.Col[column_index]����colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k]Ϊ���Ļ��Ƿ�Augng-feasible
				if (false == p.Check_Augng_Feasible(colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k], colp.Col[column_index]))
				{
					colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]][k] = -1;
				}
				else
				{
					temp_factor = temp_factor + 1;
				}
			}
			//Augng_infeasible��cycle��colp.Col[column_index].cus_cyclePosi[Cuts_sdc.SDC_nodes[j]]���޳�
			colp.Col[column_index].Update_cus_cyclePosi(Cuts_sdc.SDC_nodes[j]);
			temp_factor = fmin(temp_factor, colp.Col[column_index].Customer_indicator[Cuts_sdc.SDC_nodes[j]]);
#endif
			col += Rmp_ctall[Cuts_sdc.SDC_cons_index[j]](temp_factor);	//��ȷ����CPLEXԼ����ֱ�Ӵ���min/max���ʽ�Ƿ��Ӱ������Ч��
																		//��¼SDC_cof
			colp.Col[column_index].SDC_cof[Cuts_sdc.SDC_nodes[j]] = temp_factor;
		}
#endif
		//ע��Cut��Ӧ����ҲҪ�����
#if KPATHCUT == 1
		//ÿ��kpath
		for (j = 0; j < Cuts_kpath.processed_num+Cuts_kpath.added_num; j++)
		{
			temp_factor = 0;
			//һ��kpath�ڰ�����ÿ����
			for (k = 0; k < Cuts_kpath.Kpath_subset_len[j]; k++)
			{
				//��colp.Col[column_index]����kpath�����ĵ�cuts_kpath.Kpath_subset[j][k]֮ǰ�����ĵ�
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_kpath.Kpath_subset[j][k]]; r++)
				{
					//��colp.Col[column_index].pre_arcs[Cuts_kpath.Kpath_subset[j][k]][r]���벻��kpath����
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
		//ÿ��rcc
		for (j = 0; j < Cuts_rcc.processed_num + Cuts_rcc.added_num; j++)
		{
			temp_factor = 0;
			//һ��rcc�ڰ�����ÿ����
			for (k = 0; k < Cuts_rcc.RCC_subset_len[j]; k++)
			{
				//��colp.Col[column_index]����kpath�����ĵ�Cuts_rcc.RCC_subset[j][k]֮ǰ�����ĵ�
				for (r = 0; r < colp.Col[column_index].pre_arcs_Num[Cuts_rcc.RCC_subset[j][k]]; r++)
				{
					//��colp.Col[column_index].pre_arcs[Cuts_rcc.RCC_subset[j][k]][r]���벻��rcc����
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
		//ÿ��subset-row cuts
		for (j = 0; j < Cuts_src.processed_num + Cuts_src.added_num; j++)
		{
			temp_factor = 0;
			//һ��SR-cuts�ڰ�����ÿ����
			for (k = 0; k < Cuts_src.SRC_subset_len[j]; k++)
			{
				//��colp.Col[column_index]����SR-cuts�����ĵ�Cuts_src.SRC_subset[j][k]�Ĵ���
				temp_factor = temp_factor + colp.Col[column_index].Customer_indicator[Cuts_src.SRC_subset[j][k]];
			}
			temp_factor = Get_SRC_Cof(temp_factor, j, Cuts_src);
			col += Rmp_ctall[Cuts_src.SRC_cons_index[j]](temp_factor);
		}
#endif

		Rmp_routevar_x.add(IloNumVar(col, 0, IloInfinity, ILOFLOAT));
		col.end();
		//ÿ���һ�о���Ӷ�Ӧ��һ����ʼ��
		var_stat.add(IloCplex::AtLower);
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//��warm start
	//Rmp_cplex.setBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);
	Rmp_cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal);
	Rmp_cplex.setOut(Rmp_matrix.env.getNullStream());
	Rmp_cplex.setWarning(Rmp_matrix.env.getNullStream());
	Rmp_cplex.solve();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//�ж�ģ��������
	if (Rmp_cplex.getStatus() == IloAlgorithm::Infeasible || Rmp_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unknown || Rmp_cplex.getStatus() == IloAlgorithm::Error)
	{
		Rmp_matrix.env.out() << "ERROR:No Solution" << endl;
		cin.get();
	}
	else
	{
		//��ȡ��������Ϣ��Ϊ�´μ����ṩ��ʼ��
		Rmp_cplex.getBasisStatuses(var_stat, Rmp_routevar_x, allcon_stat, Rmp_ctall);
		//��ȡĿ�꺯��ֵ
		OBJ_value = Rmp_cplex.getObjValue();
		SolInfo.Cur_solution.OBJ_lower= OBJ_value;
		//��ȡ���߱�����ֵ
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
		//��ȡ��ż���Ž�,Ҳ����Լ����Ӧ��Ӱ�Ӽ۸�
		for (i = 0; i<p.Customer_Num; i++)
		{
			Customer_dual[i] = Rmp_cplex.getDual(Rmp_ctall[i]);//getDual�����Ĳ���������IloRangeArray�࣬������Ҳ��ȡԼ��������ԭ��
		}
		for (i = 0; i < p.Depot_Num; i++)
		{
			Vehicle_dual[i] = Rmp_cplex.getDual(Rmp_ctall[p.Customer_Num + i]);
		}
		//SDC��Ӧ�Ķ�ż����
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

		//Robust-cut�Ķ�ż�����������ϵ���RobustCut_dual��
#if KPATHCUT+RCCUT >0
		if (Cuts_kpath.processed_num + Cuts_kpath.added_num+ Cuts_rcc.processed_num + Cuts_rcc.added_num > 0)
		{
			//����
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
		//ÿ��kpath
		for (i = 0; i < Cuts_kpath.processed_num + Cuts_kpath.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_kpath.Kpath_cons_index[i]]);
			//���д�kpath�Ӽ������仡
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0 == Cuts_kpath.Kpath_indicator[i][j])
				{
					//һ��kpath�ڰ������нڵ�
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
		//ÿ��rcc
		for (i = 0; i < Cuts_rcc.processed_num + Cuts_rcc.added_num; i++)
		{
			temp_value = Rmp_cplex.getDual(Rmp_ctall[Cuts_rcc.RCC_cons_index[i]]);
			//���д�rcc�Ӽ������仡
			for (j = 0; j < p.Customer_Num + p.Depot_Num; j++)
			{
				if (0 == Cuts_rcc.RCC_indicator[i][j])
				{
					//һ��rcc�ڰ������нڵ�
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
		////��ȡ���߱�����ֵ
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
	////����cplex�����µ�ģ��
	IloInt i;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//ת��Rmp_routevar_x�����ʹ�float��int
	Rmp_model.add(IloConversion(Rmp_matrix.env, Rmp_routevar_x, ILOINT));
	Rmp_cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal);
	Rmp_cplex.setParam(Rmp_cplex.EpGap, 0.05);
	//Rmp_cplex.setParam(Rmp_cplex.TiLim, 30); //�������ʱ��
	Rmp_cplex.setOut(Rmp_matrix.env.getNullStream());
	Rmp_cplex.setWarning(Rmp_matrix.env.getNullStream());
	Rmp_cplex.solve();
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//�ж�ģ��������
	if (Rmp_cplex.getStatus() == IloAlgorithm::Infeasible || Rmp_cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unbounded || Rmp_cplex.getStatus() == IloAlgorithm::Unknown || Rmp_cplex.getStatus() == IloAlgorithm::Error)
	{
		Rmp_matrix.env.out() << "No Solution" << endl;
		feasible_ornot = false;
		OBJ_value = MAXNUM;
		//cin.get();
	}
	else
	{
		//��ȡĿ�꺯��ֵ
		OBJ_value = Rmp_cplex.getObjValue();
		SolInfo.Cur_solution.OBJ_upper = OBJ_value;
		//��ȡ���߱�����ֵ
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
			//��Cuts_sdc.Rep_customers����Indicator���������ҳ����ǰConf::MAX_ADD_SDC���ظ������Ŀͻ�
			Cuts_sdc.Find_SDC(Results, p);
#if SDCTYPE == 1
			//����Aug_ngset
			Update_AugNgset(SolInfo, colp, p, Cuts_sdc);
#endif
			ele_ornot = false;
			break;
		}
	} while (1);
#else
	Todelete_col_num = 0;
	//������ǰ��SolInfo.Cur_solution�е�����·��
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		//������ǰ·��������ÿ���ͻ�
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 1; j++)
		{
			temp_cus = colp.Col[temp_index].nodes[j];
			//Ѱ�һ�
			if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 2)
			{
				//�γɻ������cycle���ظ���Ϊtemp_cus
				//��λ��j���̽��ֱ���ҵ���һ����ͬ��temp_cus���γ�cycle
				p.Generate_cycle(colp.Col[temp_index].nodesNum, colp.Col[temp_index].nodes, j);
				//��ÿ��cycle�еĽڵ㶼����ngset
#if ClearColumn == 0
				p.Add_ngset_byCycle();
				//ֻҪ�ɻ����Ͳ������ٳ�����RMP��
				colp.Col[temp_index].Totalcost = MAXNUM;
				//����Todelete_col
				Todelete_col[Todelete_col_num] = SolInfo.Cur_solution.best_LBsol_order[i];
				Todelete_col_num = Todelete_col_num + 1;
#else
				if (false == p.Add_ngset_byCycle())
				{
					//���г���һ��cycle�ظ�����ʱ������Ҫ���÷�֧���ϵ������г�
					//��ɾ�����а�����cycle��column����Totalcost��ΪMAXNUM��
					DeleteColumn_byCycle(BB, colp, p);
				}
				else
				{
					//ֻҪ�ɻ����Ͳ������ٳ�����RMP��
					colp.Col[temp_index].Totalcost = MAXNUM;
					//����Todelete_col
					Todelete_col[Todelete_col_num] = SolInfo.Cur_solution.best_LBsol_order[i];
					Todelete_col_num = Todelete_col_num + 1;
				}
#endif
				ele_ornot = false;
				//����ֻ���ҵ���һ����
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

	//����Rep_customers
	Cuts_sdc.Reset_RepCus(p);
	//������ǰ��SolInfo.Cur_solution�е�����·��
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		//������ǰ·��������ÿ���ͻ�
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 1; j++)
		{
			temp_cus = colp.Col[temp_index].nodes[j];
#if SDCindicator == 0
			if (colp.Col[temp_index].Customer_indicator[temp_cus] >= 2)
			{
				//�����ظ���������
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
	//ע�⣺���·�������֤һ��׼ȷɾ������p.cycle_node����
	//ֻ�Ǻܴ�����ϱ�֤�ܹ�ɾ��

	int temp_index;
	bool temp_check;
	//�����г�
	for (int i = 0; i < BB.branch[BB.cur_node].used_column_num; i++)
	{
		temp_index= BB.branch[BB.cur_node].used_column[i];
		//����p.cycle_node�Ķ˵���뱻��������
		if (colp.Col[temp_index].Customer_indicator[p.cycle_node[0]] >= 2)
		{
			temp_check = true;
			//·��colp.Col[temp_index]��p.cycle_node[j]��succ_arcs����p.cycle_node[j+1]
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
				//����TotalcostΪMAXNUM
				colp.Col[temp_index].Totalcost = MAXNUM;
				//����Todelete_col
				Todelete_col[Todelete_col_num] = i;
				Todelete_col_num = Todelete_col_num + 1;
			}
		}
	}
}

void RMP::Update_flow(SolINFOR & SolInfo, ColumnPool & colp, Problem & p)
{
	int temp_start, temp_end;
	//����Arc_flow
	for (int i = 0; i < p.Customer_Num+p.Depot_Num; i++)
	{
		for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
		{
			Arc_flow[i][j] = 0;
		}
	}
	//��ֵArc_flow
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
	//����ÿ����Ҫ��չngset�ĵ�
	for (int i = 0; i < p.Customer_Num; i++)
	{
		if (Cuts_sdc.ngset_addNum[i]>0)
		{
			//������ǰ��SolInfo.Cur_solution�е�����·��
			for (int j = 0; j < SolInfo.Cur_solution.best_LBsol_num; j++)
			{
				temp_index = SolInfo.Cur_solution.best_LBsol_index[j];
				//��·�����Ƿ������iΪ����cycle
				for (int k = 0; k < colp.Col[temp_index].cus_cyclePosi_num[i]; k++)
				{
					//�Ը�·����ÿ����iΪ����cycle����չngset
					//��λ��colp.Col[temp_index].cus_cyclePosi[i][k]��ǰ̽��ֱ���ҵ���һ����ͬ��i���γ�cycle
					p.Generate_cycle_back(colp.Col[temp_index].nodesNum, colp.Col[temp_index].nodes, colp.Col[temp_index].cus_cyclePosi[i][k]);
					//��ÿ��cycle�еĽڵ㶼����ngset
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
		cout << "ERROR��SRC����ʽ����" << endl;
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
	Rmp_cplex.clearModel();		//�������û����
	Rmp_cplex.end();
	var_stat.end();
	allcon_stat.end();

	Rmp_matrix.env.end();
}
