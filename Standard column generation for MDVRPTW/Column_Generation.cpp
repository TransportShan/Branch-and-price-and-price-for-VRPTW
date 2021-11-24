#include "stdafx.h"
#include "Column_Generation.h"
#include "Conf.h"

Column_Generation::Column_Generation()
{	
}


Column_Generation::~Column_Generation()
{
}

void Column_Generation::Claim_CG(void)
{
	Remain_pro = Conf::Remain_proIni;
	UBresult_file_name = "./result./UBsolution.csv";
	LBresult_file_name = "./result./LBsolution.csv";
}

void Verify_Routes(float objRC, int startnode, int routeLen, int* route, Subproblem & subp, RMP lp, Problem p, SDC &Cuts_sdc)
{
	float *temp_Arrivaltime;				//����·����ǰ�㣨���㼯���һ���㣩�ĵ���ʱ��,��СΪ[Conf::MAX_NODES]

	int temp_RouteLength;					//����·�������ڵ����
	float temp_RCvalue;						//����·����RCֵ
	float temp_Load;						//����·����װ����
	float temp_Travelcost;					//����·����ʵ�ʳɱ������ȣ�
	float temp_Duration;					//����·���ĵ�ǰ�Ѿ��ۻ�������ʱ��
	
	temp_Arrivaltime = new float[Conf::MAX_NODES];

	//��������
	if (route[0] != startnode)
	{
		//������
		cout <<"ERROR: ·�������㲻��" << endl;
		cin.get();
	}
	else
	{
		temp_RouteLength = routeLen;
	}

	//����ʱ��-ʱ�䴰
	temp_Arrivaltime[0] = 0;
	float serviceT = 0;	//����ʱ��
	float travelT = 0;	//����ʱ��
	for (int i = 1; i<temp_RouteLength; i++)
	{
		serviceT = p.Allnode[route[i - 1]].servicetime;
		travelT = p.Calculate_traveltime(route[i - 1], route[i]);
		temp_Arrivaltime[i] = max(temp_Arrivaltime[i - 1] + serviceT + travelT, p.Allnode[route[i]].startTW);
		if (temp_Arrivaltime[i]>p.Allnode[route[i]].endTW)
		{
			cout << "ERROR��Arrivaltime" << endl;
			cin.get();
		}
	}

	//capacity �� duration
	temp_Load = 0;
	temp_Travelcost = 0;
	temp_Duration = 0;
	for (int i = 0; i<temp_RouteLength - 1; i++)
	{
		temp_Travelcost = temp_Travelcost + p.Calculate_travelcost(route[i], route[i+1]);
		temp_Duration = temp_Duration + p.Calculate_traveltime(route[i], route[i+1])+ p.Allnode[route[i+1]].servicetime;
		temp_Load = temp_Load +p.Allnode[route[i + 1]].demand;
	}
#if DURATIONORNOT == 1
	if (temp_Duration>p.Veh.Duration || temp_Load>p.Veh.Veh_Capacity)
	{
		cout << "ERROR��Duration or Capacity" << endl;
		cin.get();
	}
#else
	if (temp_Load>p.Veh.Veh_Capacity)
	{
		cout << "ERROR��Capacity" << endl;
		cin.get();
	}
#endif

	//RC
	temp_RCvalue = -lp.Vehicle_dual[route[0]-p.Customer_Num] + temp_Travelcost;
	for (int i = 1; i<temp_RouteLength - 1; i++)
	{
		temp_RCvalue = temp_RCvalue - lp.Customer_dual[route[i]];
	}
#if Frameworks == 1
	//����SDC��Ӧ��RCֵ
	int *check_SDCnodes;
	check_SDCnodes = new int[p.Customer_Num];
	for (int i = 0; i < Cuts_sdc.processed_num; i++)
	{
		check_SDCnodes[Cuts_sdc.SDC_nodes[i]] = Cuts_sdc.SDC_indicator[Cuts_sdc.SDC_nodes[i]];
	}
	
	for (int i = 1; i<temp_RouteLength - 1; i++)
	{
		//������Cuts_sdc.SDC_nodes��
		if (1== Cuts_sdc.SDC_indicator[route[i]])
		{
			//Ȼ����check_SDCnodes��һ�α�����
			if (1==check_SDCnodes[route[i]])
			{
				temp_RCvalue = temp_RCvalue - lp.SDC_dual[route[i]];
				check_SDCnodes[route[i]] = 0;
			}
		}
	}
	delete check_SDCnodes;
#endif
	if (fabs(temp_RCvalue-objRC)>100*MINDOUBLE)
	{
		cout << "ERROR��RC" << endl;
		cin.get();
	}

	delete temp_Arrivaltime;
}

void Column_Generation::SolveNode_ByCG(Utility & Results, SolINFOR &SolInfo, BranchABound & BB, Initialization & IniC, ColumnPool & pool, Problem & p, RMP & lp_node, Subproblem & subp, SRC &Cuts_src, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc)
{
	//ÿ��CG���ò���
	solved_depotNum = 0;
	Tosolve_depotno = 0;
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//�����㷨��������
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��¼CGʱ��
	clock_t CG_start = clock();				//CG��ʼʱ��
	clock_t	CG_end;							//CG����ʱ��
	//�㷨��ֹ��������
	bool Termination = false;				//��TerminationΪtrueʱ����node��CG�㷨����
	bool inter_Terminate;					//�ڲ�ѭ������ֹ��������inter_TerminateΪtrueʱ��������ѭ��
	int iter_num_CG = 1;					//��¼��������,���һ��RMP�����Ϊһ�ε���
	int iter_num_Reopt = 0;					//CPA����������Ż������һ��SDC���Ĵ���
	//����DSSR����µ�ngset
#if INHERIT == 0
	p.Set_dssr_ngset();
#endif
	//�����㷨��ʼ�׶�ʹ������ʽ���
	bool algorithm_state = false;
	//��һ�����RMP���⣬����CPLEX����
	//��ʼ���ýڵ���RMP,�ټ��鵱ǰ�г��ܷ����
	if (false == lp_node.Initial_Rmpnode(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc,Cuts_sdc, Cuts_src))
	{
		//Ӧ�ø���һ����ʼ��
		//����ͼ򵥵ذ��½�ֵ��ֵΪMAXNUM
		BB.branch[BB.cur_node].LB_value = MAXNUM;
		BB.branch[BB.cur_node].UB_value = MAXNUM;
		return;
	}
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��ѭ����ʼ
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	do//��ѭ��
	{
		inter_Terminate = false;
#if Frameworks == 1
		Remain_pro = Conf::Remain_proIni;
#endif	
		do//��ѭ��
		{
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//���PSP����
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			if (true == Solve_PricingSubproble(algorithm_state, Results, BB, subp, lp_node, pool, p, Cuts_src, Cuts_sdc))
			{
				if (Remain_pro<1 - MINDOUBLE)
				{
					Remain_pro = fmin(1.0, 2 * Remain_pro);
				}
				else if (true==subp.restricted_extend)
				{
					Remain_pro = fmin(1.0, 2 * Conf::Remain_proIni);
					subp.restricted_extend = false;
				}
				else
				{
#if EXACTPSP == 0	
#if Frameworks == 0
					Termination = true;
#else
					//CPA����ж�PSP���������RMP����ֻ�г���·����������ѭ��
					inter_Terminate= true;
#endif				
#elif EXACTPSP == 1	
#if Frameworks == 1
					cout<< "ERROR:CPA����²���ʹ��DSSR���" << endl;
					cin.get();
#endif	
					//һ������ʽDP�޷��ҵ��⣬�͵��þ�ȷDP
					if (true == algorithm_state)
					{
						Termination = true;
					}
					else
					{
						algorithm_state = true;
					}
#endif
				}
			}
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//SRC_cuts�ع�
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
#if SRCUT == 1
			//�ж��Ƿ���Ҫ���ûع���������label�����������ʱ����Ҫ���ûع�
			//if (subp.all_label_num / subp.init_label_num >= 50)
			//{
			//	//��node��ɾȥ�����ӵ�src
			//	update_src_separation(true, node, temp_src, branchnodes);
			//	//���ûع�
			//	roll_back(temp_src);
			//}
#endif
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//���RMP����
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//�ȸ���RMP�����еľ�����Ϣ
			lp_node.Update_RMP(BB, pool);
			//����CPLEX���RMP�����Լ����Ӧ��Ӱ�Ӽ۸�
			lp_node.StandardRMP_SolvebyCPLEX_Real(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc, Cuts_sdc, Cuts_src);
			//��¼��������
			iter_num_CG++;
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//�����гأ�S��Is��
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//����RMP�����������S
			//Update_posi_baseornot(S, Positive_variable_num, routeset); //�ù��ܲ���Ҫ��Ϊ��ʡ����ʱ�����ע�͵�//pvrptwδ�޸ģ�ע����mdvrptwʱҪȡ��ע��
			//����RMP����������ǳ�վԼ����������Is����ѡ��
			//Compatible_byM3_depot_real_eletransformation(S, Positive_variable_num, routeset, temp_op);//pvrptwδ�޸ģ�ע����mdvrptwʱҪȡ��ע��
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//��������ż��
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//��¼��������ż�Ŀ�ʼʱ��
			//AugmentDUAL_start = clock();
			//���ݻ��ֽ��Is����CPLEX��������ż����(���ǳ�վԼ��)����ѡ��
			//AugmentDUAL_solve_byCPLEX_depot_Real_getbasis_disturb_dual(temp_op, temp_rmp, routeset, S, Positive_variable_num);
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//������ѭ��
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
#if Frameworks == 0
			//EPO���ֱ������ѭ�� 
			inter_Terminate = true;
#endif
		} while (false== inter_Terminate);

#if Frameworks == 1
		//CPA������ȸ����ϴ�RMP����ж���ֹ��������RMP�Ľⶼ�ǳ���·�����㷨����
		//������Ҫ���SDC-cuts
		if (true== lp_node.Strengthen_elementary(Results, SolInfo, BB, pool, p, Cuts_sdc))
		{
			Termination = true;
		}
		else
		{
			//��������SDC_cuts���RMP���õ�SDC_dual
			lp_node.Added_columnNum_once = 0;
			lp_node.StandardRMP_SolvebyCPLEX_Real(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc, Cuts_sdc, Cuts_src);
			//��¼��������
			iter_num_CG++;
			iter_num_Reopt++;
		}
#endif
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
		//������ѭ��
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
		//����cut
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
#if KPATHCUT+RCCUT+SRCUT >0
		if (true == Termination && BB.branch[BB.cur_node].depth <=Conf::MAX_ROBUST_DEPTH)
		{
			//���Ȳ�����������
			if (false== SolInfo.check_IntSol())
			{
#if KPATHCUT+RCCUT >0
				//����RMP��ǰ����Ϣ���õ����⻡�ϵ�����
				lp_node.Update_flow(SolInfo, pool, p);
#endif
				//����Kpath-cuts
#if KPATHCUT ==1
				Termination = Termination && Generate_Kpath(SolInfo, BB, subp, pool, p, lp_node, Cuts_kpath);
#endif
				//����RCC-cuts
#if RCCUT ==1
				Termination = Termination && Generate_RCC(SolInfo, BB, subp, pool, p, lp_node, Cuts_rcc);
#endif
				//����subset-row cuts
#if SRCUT == 1
				//|C|=3ʱ������������Ԫ��
				//|C|>3ʱ��ֻ�鿴�ڲ�������ΪConf::MAX_SR_DISTANCE����
				Termination = Termination && Generate_SRC(SolInfo, BB, subp, pool, p, lp_node, Cuts_src);
#endif
				//����������cuts��Ķ�ż����
				if (false == Termination)
				{
					//��������cuts���RMP
					lp_node.Added_columnNum_once = 0;
					lp_node.StandardRMP_SolvebyCPLEX_Real(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc, Cuts_sdc, Cuts_src);
					//��¼��������
					iter_num_CG++;
					iter_num_Reopt++;
				}
			}
		}
#endif

		if (true== Termination)
		{
			BB.branch[BB.cur_node].LB_value = lp_node.OBJ_value;
			break;
		}
	} while (pool.CandidateColumn_Num<Conf::MAX_COLUMN_IN_RMP-1000);	//��ֹ������ע��𳬽�
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��ѭ������
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	
#if UPBOUND == 1
	if (false== lp_node.StandardRMP_SolvebyCPLEX_Int(SolInfo, BB, pool))
	{
		BB.branch[BB.cur_node].UB_value = MAXNUM;
	}
	BB.branch[BB.cur_node].UB_value = lp_node.OBJ_value;
#endif
	//��CPLEX�������õ����ͷ�,ͬʱ�����µ�CPLEX����
	lp_node.reset_RmpMatrix_byclass();
	lp_node.Rmp_matrix.ini_cplex();
	//��¼rootnode�ϵ�ָ��
	CG_end = clock();
	Results.Reopt_num = iter_num_Reopt;
	Results.CGitr_num = iter_num_CG;
	Results.Total_time = (float)(CG_end - CG_start) / CLOCKS_PER_SEC;
	Results.SDC_num = Cuts_sdc.processed_num;
	//���rootnode�ϵ�ָ��
	if (0 == BB.cur_node)
	{
		Results.Show_RootNode_results();
	}
	Results.Reset_UTILITY();
	//���¸÷�֧�ڵ��ϵ�״̬
	Update_Solvednode(SolInfo, BB, pool);
}

bool Column_Generation::Solve_PricingSubproble(bool exact, Utility & Results, BranchABound & BB, Subproblem & subp, RMP &lp, ColumnPool & colp, Problem & p, SRC & temp_src, SDC &Cuts_sdc)
{
	//��ȷ��⣬������2cycle
	subp.exact_ornot = exact;
	bool solution_state = true;	//=trueʱ��ʾ������ʹ������ʽ�����Ѿ������ҵ�RC<0,
								//=falseʱ��ʾ������ʹ������ʽ�����ҵ�RC<0

#if PRICING == 0
	//���ȸ���RMP��ż��Ϣ��������
	//�ж�RCֵ�������еĹ���
	for (int i = 0; i < p.Depot_Num; i++)
	{
		//��һ��:��RMP��ȡ��ż��Ϣ��ָ������
		subp.Get_dual_info(i, Remain_pro, BB, lp, p, Cuts_sdc);
		//�ڶ���:��̬�滮��ESPPRC
		if (true == subp.DP_Ng_Cuts_Dssr_Stable(BB, lp, p, temp_src, Cuts_sdc))
		{
			solution_state = false;
		}
		//������:��PSP���ҵ���·�����ݸ�colp
		for (int j = 0; j<subp.FoundPath_container_num; j++)
		{
			if (subp.FoundPath_container[j].Reducedcost>-Conf::Exact_threshold)continue;
			//Verify_Routes(subp.FoundPath_container[j].Reducedcost, i+p.Customer_Num, subp.FoundPath_container[j].nodesNum, subp.FoundPath_container[j].nodes, subp, lp, p, Cuts_sdc);
			colp.Add_columns(true, BB, subp.FoundPath_container[j], p);
#if ADDCOL == 1
			colp.Add_auxi_columns(BB, p);
#endif
		}
		//���Ĳ�:��¼PSPָ��
		Results.PSP_label_num.push_back(subp.all_label_num);
		Results.PSP_time.push_back(subp.PSP_time);
	}
	return solution_state;
#elif PRICING == 1
	subp.Get_dual_info(Tosolve_depotno, Remain_pro, BB, lp, p);
	if (true == subp.DP_Ng_Cuts_Dssr_Stable(BB, lp, p, temp_src, Cuts_sdc))
	{
		for (int j = 0; j<subp.FoundPath_container_num; j++)
		{
			if (subp.FoundPath_container[j].Reducedcost>-Conf::Exact_threshold)continue;
			//Verify_Routes(subp.FoundPath_container[j].Reducedcost, Tosolve_depotno + p.Customer_Num, subp.FoundPath_container[j].nodesNum, subp.FoundPath_container[j].nodes,subp, lp, p, Cuts_sdc);
			colp.Add_columns(true, BB, subp.FoundPath_container[j], p);
#if ADDCOL == 1
			colp.Add_auxi_columns(BB, p);
#endif
		}
		solved_depotNum = 0;
		return false;
	}
	else
	{
		solved_depotNum = solved_depotNum + 1;
		if (p.Depot_Num == solved_depotNum)
		{
			Tosolve_depotno = 0;
			solved_depotNum = 0;
			return true;
		}

		if (Tosolve_depotno + 1 >= p.Depot_Num)
		{
			Tosolve_depotno = 0;
		}
		else
		{
			Tosolve_depotno = Tosolve_depotno + 1;
		}
		return false;
	}
	//��¼PSPָ��
	Results.PSP_label_num.push_back(subp.all_label_num);
	Results.PSP_time.push_back(subp.PSP_time);
#endif
}


bool Column_Generation::Detect_branch(SolINFOR &SolInfo, BranchABound & BB, ColumnPool & colp, Problem & p)
{
	//���ж����CG�����Ƿ��ҵ�������
	if (BB.branch[BB.cur_node].LB_value>MAXNUM - 1 || BB.branch[BB.cur_node].LB_value>Conf::PRE_BOUND)
	{
		//��֧�޽�
		//��֦������Ҫ�����·�֧
		return false;
	}
	//�п����޷�ֱ�ӵ���CPLEX���IP�������Ҫ�������ж�BB.branch[BB.cur_node].LB_value�Ƿ��BB.branch[BB.cur_node].UB_value���
	int temp_index,temp_start,temp_end;
	float temp_value, min_value;

#if BRANCHVEH == 1
	//��һ�׶�
	//����vehicle_num
	BB.vehicle_num = 0;
	//��ֵvehicle_num
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		BB.vehicle_num[colp.Col[SolInfo.Cur_solution.best_LBsol_index[i]].Depot-p.Customer_Num] = BB.vehicle_num[colp.Col[SolInfo.Cur_solution.best_LBsol_index[i]].Depot - p.Customer_Num] + SolInfo.Cur_solution.best_LBsol_value[i];
	}
	//�ҵ���ӽ�0.5��vehicle_num
	min_value = MAXNUM;
	for (int i = 0; i < p.Depot_Num; i++)
	{
		temp_value = BB.vehicle_num[i] - floor(BB.vehicle_num[i]);
		if (fabs(0.5 - temp_value)<min_value)
		{
			min_value = fabs(0.5 - temp_value);
			BB.branch[BB.cur_node].branch_depot = i;
			//��֦<= floor(BB.vehicle_num[i]),��֦>=floor(BB.vehicle_num[i])+1
			BB.branch[BB.cur_node].branch_vehicleNum = int(floor(BB.vehicle_num[i]));
		}
	}
	if (fabs(min_value - 0.5)>10 * MINDOUBLE)
	{
		//��ǰ��һ������������
		//ֻ��ÿ��ʹ�õĳ�������֧�Ϳ���
		BB.branch[BB.cur_node].branch_strategy = 0;
		return true;
	}
#endif

	//��ڶ��׶�����
	//����flow_onday
	for (int j = 0; j < p.Customer_Num; j++)
	{
		for (int k = 0; k < p.Customer_Num; k++)
		{
			BB.flow_onarc[j][k] = 0;
		}
	}
	//��ֵflow_onday
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		temp_index = SolInfo.Cur_solution.best_LBsol_index[i];
		for (int j = 1; j < colp.Col[temp_index].nodesNum - 2; j++)
		{
			temp_start = colp.Col[temp_index].nodes[j];
			temp_end = colp.Col[temp_index].nodes[j+1];
			BB.flow_onarc[temp_start][temp_end] = BB.flow_onarc[temp_start][temp_end] + SolInfo.Cur_solution.best_LBsol_value[i];
		}
	}
	//��BB.flow_onarcѰ�����BB.branch[BB.cur_node].branchcons_num�������з�֧
	//������customer_branch
	for (int i = 0; i < p.Customer_Num; i++)
	{
		BB.customer_branch[i] = 1;
	}
	do
	{
		//�ҵ���ӽ�0.5��flow_onday
		min_value = MAXNUM;
		for (int j = 0; j < p.Customer_Num; j++)
		{
			if (0 == BB.customer_branch[j])continue;
			for (int k = 0; k < p.Customer_Num; k++)
			{
				if (0 == BB.customer_branch[k])continue;
				if (j == k)continue;
				if (fabs(0.5 - BB.flow_onarc[j][k])<min_value)
				{
					min_value = fabs(0.5 - BB.flow_onarc[j][k]);
					temp_start = j;
					temp_end = k;
					if (min_value<MINDOUBLE)
					{
						break;
					}
				}
			}
			if (min_value<MINDOUBLE)
			{
				break;
			}
		}
		if (fabs(min_value - 0.5)>10 * MINDOUBLE)
		{
			BB.branch[BB.cur_node].branch_Startnode[BB.branch[BB.cur_node].branchcons_num] = temp_start;
			BB.branch[BB.cur_node].branch_Endnode[BB.branch[BB.cur_node].branchcons_num] = temp_end;
			BB.branch[BB.cur_node].branchcons_num = BB.branch[BB.cur_node].branchcons_num + 1;
			BB.customer_branch[temp_start] = 0;
			BB.customer_branch[temp_end] = 0;
		}
		if (Conf::MAX_ADD == BB.branch[BB.cur_node].branchcons_num || fabs(min_value - 0.5)<10 * MINDOUBLE)
		{
			break;
		}
	} while (1);

	if (BB.branch[BB.cur_node].branchcons_num>0)
	{
		//��ǰ��һ������������
		//ֻ��ÿ��ʹ�õĳ�������֧�Ϳ���
		BB.branch[BB.cur_node].branch_strategy = 1;
		return true;
	}
	else
	{
		//��ǰ��Ϊ������
		//��֧���磬����Ҫ�����·�֧
		BB.branch[BB.cur_node].UB_value = BB.branch[BB.cur_node].LB_value;
		return false;
	}
}

bool Column_Generation::Branching(BranchABound & BB, Problem &p)
{
	int left_index,right_index;
	int temp_index;
	left_index = BB.exist_nodeNum;
	right_index = BB.exist_nodeNum+1;
	if (left_index>=Conf::MAX_BRANCH_NODES || right_index >= Conf::MAX_BRANCH_NODES)
	{
		return true;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//��֦
	BB.branch[left_index].node_id = left_index;
	BB.branch[left_index].depth = BB.branch[BB.mother_node].depth+1;
	BB.branch[left_index].branch_state = 0;
	BB.branch[left_index].pre_nodeid = BB.mother_node;
	BB.branch[left_index].LB_value = MAXNUM;
	BB.branch[left_index].UB_value = MAXNUM;
	BB.branch[left_index].used_column_num = 0;
	BB.branch[left_index].total_column_num = 0;
	//���ٿռ�
	BB.branch[left_index].addCons_New = new NewConstraint[Conf::MAX_ADD];
	for (int i = 0; i<Conf::MAX_ADD; i++)
	{
		BB.branch[left_index].addCons_New[i].type = -1;
	}
	BB.branch[left_index].addCons_Total = new NewConstraint[BB.branch[BB.mother_node].addCons_Total_num + Conf::MAX_ADD];
	for (int i = 0; i<BB.branch[BB.mother_node].addCons_Total_num + Conf::MAX_ADD; i++)
	{
		BB.branch[left_index].addCons_Total[i].type = -1;
	}
	//����ĸ�ڵ��Լ��
	BB.branch[left_index].Copy_Constraints(BB.branch[BB.mother_node]);
	BB.branch[left_index].addCons_Total_num = BB.branch[BB.mother_node].addCons_Total_num;
	//����̳�ĸ�ڵ���ڳ�������Լ��
	//�ȿ��ٿռ䣬�����ͷ�
	BB.branch[left_index].vehicleNum_lower = new int[p.Depot_Num];
	BB.branch[left_index].vehicleNum_upper = new int[p.Depot_Num];
	//�ٸ�ֵ
	for (int i = 0; i < p.Depot_Num; i++)
	{
		BB.branch[left_index].vehicleNum_lower[i] = BB.branch[BB.mother_node].vehicleNum_lower[i];
		BB.branch[left_index].vehicleNum_upper[i] = BB.branch[BB.mother_node].vehicleNum_upper[i];
	}
	//�����Լ��
	if (0== BB.branch[BB.mother_node].branch_strategy)
	{
		//�Գ�վ�����ĳ������ķ�֧
		BB.branch[left_index].addCons_New_num = 0;
		BB.branch[left_index].vehicleNum_upper[BB.branch[BB.mother_node].branch_depot] = BB.branch[BB.mother_node].branch_vehicleNum;
	}
	else if(1 == BB.branch[BB.mother_node].branch_strategy)
	{
		//�Ի��ķ�֧
		BB.branch[left_index].addCons_New_num = BB.branch[BB.mother_node].branchcons_num;
		for (int i = 0; i < BB.branch[left_index].addCons_New_num; i++)
		{
			temp_index = i;
			BB.branch[left_index].addCons_New[temp_index].type = 1;
			BB.branch[left_index].addCons_New[temp_index].startnode = BB.branch[BB.mother_node].branch_Startnode[i];
			BB.branch[left_index].addCons_New[temp_index].endnode = BB.branch[BB.mother_node].branch_Endnode[i];
			BB.branch[left_index].addCons_New[temp_index].arc_state = true;
		}
	}
	else
	{
		cout <<"ERROR: ��֧����"<< endl;
		cin.get();
	}
	//����addCons_Total
	for (int i = 0; i<BB.branch[left_index].addCons_New_num; i++)
	{
		temp_index = BB.branch[left_index].addCons_Total_num;
		if(1 == BB.branch[left_index].addCons_New[i].type)
		{
			//�����Լ��
			BB.branch[left_index].addCons_Total[temp_index].type = BB.branch[left_index].addCons_New[i].type;
			BB.branch[left_index].addCons_Total[temp_index].startnode = BB.branch[left_index].addCons_New[i].startnode;
			BB.branch[left_index].addCons_Total[temp_index].endnode = BB.branch[left_index].addCons_New[i].endnode;
			BB.branch[left_index].addCons_Total[temp_index].arc_state = BB.branch[left_index].addCons_New[i].arc_state;
		}
		else
		{
			cout << "ERROR: ��֧����" << endl;
			cin.get();
		}
		BB.branch[left_index].addCons_Total_num = BB.branch[left_index].addCons_Total_num + 1;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//��֦
	BB.branch[right_index].node_id = right_index;
	BB.branch[right_index].depth = BB.branch[BB.mother_node].depth + 1;
	BB.branch[right_index].branch_state = 0;
	BB.branch[right_index].pre_nodeid = BB.mother_node;
	BB.branch[right_index].LB_value = MAXNUM;
	BB.branch[right_index].UB_value = MAXNUM;
	BB.branch[right_index].used_column_num = 0;
	BB.branch[right_index].total_column_num = 0;
	//���ٿռ�
	BB.branch[right_index].addCons_New = new NewConstraint[Conf::MAX_ADD];
	for (int i = 0; i<Conf::MAX_ADD; i++)
	{
		BB.branch[right_index].addCons_New[i].type = -1;
	}
	BB.branch[right_index].addCons_Total = new NewConstraint[BB.branch[BB.mother_node].addCons_Total_num + Conf::MAX_ADD];
	for (int i = 0; i<BB.branch[BB.mother_node].addCons_Total_num + Conf::MAX_ADD; i++)
	{
		BB.branch[right_index].addCons_Total[i].type = -1;
	}
	//����ĸ�ڵ��Լ��
	BB.branch[right_index].Copy_Constraints(BB.branch[BB.mother_node]);
	BB.branch[right_index].addCons_Total_num = BB.branch[BB.mother_node].addCons_Total_num;
	//����̳�ĸ�ڵ���ڳ�������Լ��
	//�ȿ��ٿռ䣬�����ͷ�
	BB.branch[right_index].vehicleNum_lower = new int[p.Depot_Num];
	BB.branch[right_index].vehicleNum_upper = new int[p.Depot_Num];
	//�ٸ�ֵ
	for (int i = 0; i < p.Depot_Num; i++)
	{
		BB.branch[right_index].vehicleNum_lower[i] = BB.branch[BB.mother_node].vehicleNum_lower[i];
		BB.branch[right_index].vehicleNum_upper[i] = BB.branch[BB.mother_node].vehicleNum_upper[i];
	}
	//�����Լ��
	if (0 == BB.branch[BB.mother_node].branch_strategy)
	{
		//�Գ������ķ�֧
		BB.branch[right_index].addCons_New_num = 0;
		BB.branch[right_index].vehicleNum_lower[BB.branch[BB.mother_node].branch_depot] = BB.branch[BB.mother_node].branch_vehicleNum;
	}
	else if (1 == BB.branch[BB.mother_node].branch_strategy)
	{
		//�Ի��ķ�֧
		BB.branch[right_index].addCons_New_num = BB.branch[BB.mother_node].branchcons_num;
		for (int i = 0; i < BB.branch[right_index].addCons_New_num; i++)
		{
			temp_index = i;
			BB.branch[right_index].addCons_New[temp_index].type = 1;
			BB.branch[right_index].addCons_New[temp_index].startnode = BB.branch[BB.mother_node].branch_Startnode[i];
			BB.branch[right_index].addCons_New[temp_index].endnode = BB.branch[BB.mother_node].branch_Endnode[i];
			BB.branch[right_index].addCons_New[temp_index].arc_state = false;
		}
	}
	else
	{
		cout << "ERROR: ��֧����" << endl;
		cin.get();
	}
	//����addCons_Total
	for (int i = 0; i<BB.branch[right_index].addCons_New_num; i++)
	{
		temp_index = BB.branch[right_index].addCons_Total_num;
		if (1 == BB.branch[right_index].addCons_New[i].type)
		{
			//�����Լ��
			BB.branch[right_index].addCons_Total[temp_index].type = BB.branch[right_index].addCons_New[i].type;
			BB.branch[right_index].addCons_Total[temp_index].startnode = BB.branch[right_index].addCons_New[i].startnode;
			BB.branch[right_index].addCons_Total[temp_index].endnode = BB.branch[right_index].addCons_New[i].endnode;
			BB.branch[right_index].addCons_Total[temp_index].arc_state = BB.branch[right_index].addCons_New[i].arc_state;
		}
		else
		{
			cout << "ERROR: ��֧����" << endl;
			cin.get();
		}
		BB.branch[right_index].addCons_Total_num = BB.branch[right_index].addCons_Total_num + 1;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//���·�֧��
	BB.exist_nodeNum = BB.exist_nodeNum + 2;
	//��֧������ĸ�ڵ���Ϊ��֧����״̬
	BB.branch[BB.mother_node].branch_ornot = true;
	BB.branch[BB.mother_node].branched_num = 0;
	BB.branch[BB.mother_node].branch_state = 2;
	//�ͷ�ĸ�ڵ��ϵĶ�̬����
	if (BB.branch[BB.mother_node].node_id>0)
	{
		delete[] BB.branch[BB.mother_node].vehicleNum_upper;
		delete[] BB.branch[BB.mother_node].vehicleNum_lower;
		delete[] BB.branch[BB.mother_node].addCons_New;
		delete[] BB.branch[BB.mother_node].addCons_Total;
		delete[] BB.branch[BB.mother_node].branch_Startnode;
		delete[] BB.branch[BB.mother_node].branch_Endnode;
	}
	//������������
	BB.Toslove_index[0] = left_index;
	BB.Toslove_index[1] = right_index;
	BB.branch[BB.mother_node].Son_node[0] = left_index;
	BB.branch[BB.mother_node].Son_node[1] = right_index;
	BB.Toslove_num = 2;
	return false;
}

void Column_Generation::Update_used_column(BranchABound & BB, ColumnPool & colp, Problem &p)
{
	//������������
	//��һ���ж���Щ���ܴ�ĸ�ڵ�̳�
	//�ڶ����ж�ĸ�ڵ���г��Ƿ����ͷ� 

	//���ȿ��ٿռ�
	BB.branch[BB.cur_node].used_column = new int[Conf::MAX_COLUMN_IN_RMP];
	BB.branch[BB.cur_node].LBsol_index = new int[2 * p.Customer_Num];
	BB.branch[BB.cur_node].LBsol_value = new float[2 * p.Customer_Num];
	BB.branch[BB.cur_node].LBsol_num = 0;

	int temp_mother, temp_index, temp_start, temp_end;
	bool check;
	temp_mother = BB.branch[BB.cur_node].pre_nodeid;
	BB.branch[BB.cur_node].used_column_num = 0;

	//���жϴ�0��used_column_num�����Ƿ��ܻ�����Լ��
	for (int i = 0; i < BB.branch[temp_mother].used_column_num; i++)
	{
		check = true;
		temp_index = BB.branch[temp_mother].used_column[i];
		//�Խڵ�BB.branch[temp_mother].used_columnΪģ�壬�޸�branchnodes[node].used_column
		//���ÿ��Լ���Ƿ�����,�йس�������Լ����ÿ�еĿ�����û��Ӱ��
		for (int j = 0; j < BB.branch[BB.cur_node].addCons_New_num; j++)
		{
			if (1 == BB.branch[BB.cur_node].addCons_New[j].type)
			{
				//�����Լ��
				temp_start = BB.branch[BB.cur_node].addCons_New[j].startnode;
				temp_end = BB.branch[BB.cur_node].addCons_New[j].endnode;
				if (true == BB.branch[BB.cur_node].addCons_New[j].arc_state)			//Ҫ��û�һ��������
				{
					//��һ�������������BB.branch[BB.cur_node].addCons_New[j].startnode�������ﲻ��BB.branch[BB.cur_node].addCons_New[j].endnode���յ�,��ô��Լ��[j]������
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//ֻҪ��һ���������Ͳ�����
						for (int k = 0; k <colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] != temp_end)
							{
								//��Լ��[j]������
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
					//�ڶ�����������һ���������BB.branch[BB.cur_node].addCons_New[j].startnode��������BB.branch[BB.cur_node].addCons_New[j].endnode���յ�,��ô��Լ��[j]������
					if (colp.Col[temp_index].Customer_indicator[temp_end] >= 1)
					{
						//ֻҪ��һ���������Ͳ�����
						for (int k = 0; k < colp.Col[temp_index].pre_arcs_Num[temp_end]; k++)
						{
							if (colp.Col[temp_index].pre_arcs[temp_end][k] != temp_start)
							{
								//��Լ��[j]������
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
				}
				else         //Ҫ��û�һ����������
				{
					//ֻ��Ҫ�������BB.branch[BB.cur_node].addCons_New[j].startnode���յ�BB.branch[BB.cur_node].addCons_New[j].endnode�Ļ����ڼ�������Լ��
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//ֻҪ��һ���������Ͳ�����
						for (int k = 0; k < colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] == temp_end)
							{
								//��Լ��[j]������
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
				}
			}
			else
			{
				cout << "ERROR: ��֧����" << endl;
				cin.get();
			}
		}
		if (true == check)
		{
			BB.branch[BB.cur_node].used_column[BB.branch[BB.cur_node].used_column_num] = temp_index;
			BB.branch[BB.cur_node].used_column_num = BB.branch[BB.cur_node].used_column_num + 1;
		}
	}

	//���жϴ�BB.branch[BB.cur_node].total_column_num��colp.CandidateColumn_Num�����Ƿ��ܻ�����Լ��
	for (int i = BB.branch[temp_mother].total_column_num; i<colp.CandidateColumn_Num; i++)
	{
		temp_index = i;
		check = true;
		for (int j = 0; j < BB.branch[BB.cur_node].addCons_Total_num; j++)	//�ж�ÿһ��Լ��
		{
			if (1 == BB.branch[BB.cur_node].addCons_Total[j].type)
			{
				//�����Լ��
				temp_start = BB.branch[BB.cur_node].addCons_Total[j].startnode;
				temp_end = BB.branch[BB.cur_node].addCons_Total[j].endnode;
				if (true == BB.branch[BB.cur_node].addCons_Total[j].arc_state)			//Ҫ��û�һ��������
				{
					//��һ�������������BB.branch[BB.cur_node].addCons_Total[j].startnode�������ﲻ��BB.branch[BB.cur_node].addCons_Total[j].endnode���յ�,��ô��Լ��[j]������
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//ֻҪ��һ���������Ͳ�����
						for (int k = 0; k < colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] != temp_end)
							{
								//��Լ��[j]������
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
					//�ڶ�����������һ���������BB.branch[BB.cur_node].addCons_Total[j].startnode��������BB.branch[BB.cur_node].addCons_Total[j].endnode���յ�,��ô��Լ��[j]������
					if (colp.Col[temp_index].Customer_indicator[temp_end] >= 1)
					{
						//ֻҪ��һ���������Ͳ�����
						for (int k = 0; k < colp.Col[temp_index].pre_arcs_Num[temp_end]; k++)
						{
							if (colp.Col[temp_index].pre_arcs[temp_end][k] != temp_start)
							{
								//��Լ��[j]������
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
				}
				else         //Ҫ��û�һ����������
				{
					//ֻ��Ҫ�������B.branch[BB.cur_node].addCons_Total[j].startnode���յ�BB.branch[BB.cur_node].addCons_Total[j].endnode�Ļ����ڼ�������Լ��
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//ֻҪ��һ���������Ͳ�����
						for (int k = 0; k < colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] == temp_end)
							{
								//��Լ��[j]������
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
				}
			}
			else
			{
				cout << "ERROR: ��֧����" << endl;
				cin.get();
			}
		}
		if (true == check)
		{
			BB.branch[BB.cur_node].used_column[BB.branch[BB.cur_node].used_column_num] = temp_index;
			BB.branch[BB.cur_node].used_column_num = BB.branch[BB.cur_node].used_column_num + 1;
		}
	}
}

void Column_Generation::Update_network(BranchABound & BB, Problem & p)
{
	int temp_mother = BB.branch[BB.cur_node].pre_nodeid;
	//�ȿ��ٿռ�
	BB.branch[BB.cur_node].CostNetwork_Branch = new float*[p.Customer_Num + p.Depot_Num];
	for (int i = 0; i < p.Customer_Num + p.Depot_Num; i++)
	{
		BB.branch[BB.cur_node].CostNetwork_Branch[i] = new float[p.Customer_Num + p.Depot_Num];
	}
	BB.branch[BB.cur_node].valid_Cus = new int[p.Customer_Num];

	//��ĸ�ڵ�̳�
	for (int j = 0; j < p.Customer_Num + p.Depot_Num; j++)
	{
		for (int k = 0; k < p.Customer_Num + p.Depot_Num; k++)
		{
			BB.branch[BB.cur_node].CostNetwork_Branch[j][k] = BB.branch[temp_mother].CostNetwork_Branch[j][k];
		}
	}
	for (int j = 0; j < p.Customer_Num; j++)
	{
		BB.branch[BB.cur_node].valid_Cus[j] = BB.branch[temp_mother].valid_Cus[j];
	}
	//������Լ������
	int temp_start, temp_end;
	float temp_value;
	for (int i = 0; i<BB.branch[BB.cur_node].addCons_New_num; i++)
	{
		if (1 == BB.branch[BB.cur_node].addCons_New[i].type)
		{
			//�����Լ��
			temp_start = BB.branch[BB.cur_node].addCons_New[i].startnode;
			temp_end = BB.branch[BB.cur_node].addCons_New[i].endnode;
			if (true == BB.branch[BB.cur_node].addCons_New[i].arc_state)			//Ҫ��û�һ��������
			{
				//CostNetwork_Branch
				temp_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][temp_end];
				//��һ�������BB.branch[BB.cur_node].addCons_New[i].startnode����������BB.branch[BB.cur_node].addCons_New[i].endnode�Ļ��ĳɱ���Ϊ������
				for (int k = 0; k < p.Customer_Num + p.Depot_Num; k++)
				{
					BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][k] = MAXNUM;
				}
				//�ڶ�����һ���������BB.branch[BB.cur_node].addCons_New[i].startnode��������BB.branch[BB.cur_node].addCons_New[i].endnode�Ļ��ĳɱ���Ϊ������
				for (int k = 0; k < p.Customer_Num + p.Depot_Num; k++)
				{
					BB.branch[BB.cur_node].CostNetwork_Branch[k][temp_end] = MAXNUM;
				}
				BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][temp_end] = temp_value;
			}
			else         //Ҫ��û�һ����������
			{
				//�����B.branch[BB.cur_node].addCons_New[i].startnode���յ�BB.branch[BB.cur_node].addCons_New[i].endnode�Ļ��ĳɱ���Ϊ������
				BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][temp_end] = MAXNUM;
			}
			//exempt_Cus
			BB.branch[BB.cur_node].valid_Cus[temp_start] = 0;
			BB.branch[BB.cur_node].valid_Cus[temp_end] = 0;
		}
		else
		{
			cout << "ERROR: ��֧����" << endl;
			cin.get();
		}
	}
}


bool Column_Generation::Check_state(SolINFOR & SolInfo, BranchABound & BB, ColumnPool & colp, Problem & p)
{
	float temp_LB;
	int nownode = BB.cur_node;
	//�ж���node������״̬state
	if (BB.branch[nownode].LB_value>Conf::PRE_BOUND || BB.branch[nownode].LB_value >= BB.best_upper || BB.branch[nownode].LB_value>MAXNUM - 1)
	{
		//��֦
		BB.branch[nownode].state = 0;
		cout << "��֦" << endl;
	}
	else if (BB.branch[nownode].LB_value == BB.branch[nownode].UB_value)	//�ҵ�������
	{
		//����
		BB.branch[nownode].state = 1;
		if (BB.branch[nownode].LB_value<BB.best_lower)
		{
			BB.best_lower = BB.branch[nownode].LB_value;
		}
		if (BB.branch[nownode].UB_value < BB.best_upper)
		{
			//�����Ͻ�
			BB.best_upper = BB.branch[nownode].UB_value;
			//��������������
			BB.best_node_upper = nownode;
			//���best_node_upper�ķ����������
			SolInfo.CopyToBest_Upper();
			Show_UBsolutions_mdvrptw(UBresult_file_name, SolInfo, colp, p);
		}
		cout << "����" << endl;
	}
	else
	{
		//��֧,������branch_state=1���Ҵ���best_lower�Ÿ���best_lower
		BB.branch[nownode].state = 2;
		//���ҵ���ǰLB_value��С��
		temp_LB = MAXNUM;
		for (int j = 0; j < BB.exist_nodeNum; j++)
		{
			if (1 == BB.branch[j].branch_state)
			{
				if (BB.branch[j].LB_value<temp_LB)
				{
					temp_LB = BB.branch[j].LB_value;
					BB.best_node_lower = j;
				}
			}
		}
		//Ȼ����LB_value��С�ı�BB.best_lower�������BB.best_lower
		if (0 == nownode || temp_LB>BB.best_lower + MINDOUBLE)
		{
			BB.best_lower = temp_LB;
			BB.Update_bestLB(SolInfo);
			Show_LBsolutions_mdvrptw(LBresult_file_name, SolInfo, colp, p);
		}
		cout << "��֧" << endl;
	}

	//�����������Toslove_index��ȥ��������һ����
	BB.Toslove_num = BB.Toslove_num - 1;

	if (((BB.best_upper - BB.best_lower) / BB.best_upper)<Conf::MIN_GAP)
	{
		//������ֹ����
		return true;
	}
	else
	{
		return false;
	}
}

int Column_Generation::Show_UBsolutions_mdvrptw(const string & file_name, SolINFOR & SolInfo, ColumnPool & pool, Problem p)
{
	ofstream oFile;
	int i, j,k;

	oFile.open(file_name.c_str(), ios::out | ios::trunc);

	if (!oFile.is_open())
	{
		cerr << "Can't open" << file_name << "for write.\n";
		return 0;
	}
	else
	{
		oFile << "Ŀ�꺯��Ϊ��" << ',' << SolInfo.Best_Solution.OBJ_upper << endl;

		for (k = 0; k < p.Depot_Num; k++)
		{
			for (i = 0; i<SolInfo.Best_Solution.best_UPsol_num; i++)
			{
				if ((k+p.Customer_Num) == pool.Col[SolInfo.Best_Solution.best_UPsol_index[i]].Depot)
				{
					oFile << "��" << ',' << k + 1 << ',' << "����վ:" << ',';
					oFile << "�ܷ���" << ',' << pool.Col[SolInfo.Best_Solution.best_UPsol_index[i]].Totalcost << ',';
					oFile << "����ֵ" << ',' << SolInfo.Best_Solution.best_UPsol_value[i] << ',';

					oFile << k + p.Customer_Num+1 << ',';
					for (j = 1; j < pool.Col[SolInfo.Best_Solution.best_UPsol_index[i]].nodesNum - 1; j++)
					{
						oFile << pool.Col[SolInfo.Best_Solution.best_UPsol_index[i]].nodes[j] + 1 << ',';
					}
					oFile << k + p.Customer_Num +1<< ',';
					oFile << endl;
				}
			}
		}
		oFile.close();
		return 1;
	}
}

int Column_Generation::Show_LBsolutions_mdvrptw(const string & file_name, SolINFOR & SolInfo, ColumnPool & pool, Problem p)
{
	ofstream oFile;
	int i, j,k;

	oFile.open(file_name.c_str(), ios::out | ios::trunc);

	if (!oFile.is_open())
	{
		cerr << "Can't open" << file_name << "for write.\n";
		return 0;
	}
	else
	{
		oFile << "Ŀ�꺯��Ϊ��" << ',' << SolInfo.Best_Solution.OBJ_lower << endl;
		for (k = 0; k < p.Depot_Num; k++)
		{
			for (i = 0; i<SolInfo.Best_Solution.best_LBsol_num; i++)
			{
				if ((k + p.Customer_Num) == pool.Col[SolInfo.Best_Solution.best_LBsol_index[i]].Depot)
				{
					oFile << "��" << ',' << k + 1 << ',' << "����վ:" << ',';
					oFile << "����" << ',' << pool.Col[SolInfo.Best_Solution.best_LBsol_index[i]].Totalcost << ',';
					oFile << "����ֵ" << ',' << SolInfo.Best_Solution.best_LBsol_value[i] << ',';
					oFile << "�г����" << ',' << SolInfo.Best_Solution.best_LBsol_index[i] << ',';

					oFile << k + p.Customer_Num << ',';
					for (j = 1; j < pool.Col[SolInfo.Best_Solution.best_LBsol_index[i]].nodesNum - 1; j++)
					{
						oFile << pool.Col[SolInfo.Best_Solution.best_LBsol_index[i]].nodes[j] + 1 << ',';
					}
					oFile << k + p.Customer_Num << ',';
					oFile << endl;
				}
			}
		}
		oFile.close();
		return 1;
	}
}

bool Column_Generation::Choose_branchNode(SolINFOR & SolInfo, BranchABound & BB, ColumnPool & pool, Problem p)
{
	//����ԭ���ǣ�����ȷ�֧����������Ͻ磬�����ŷ�֧
	int temp_pre = -1;
	//���ȶ�cur_node������ȷ�֧
	if (0 == BB.cur_node)
	{
		BB.mother_node = BB.cur_node;
		return false;
	}
	else if (2 == BB.branch[BB.branch[BB.cur_node].pre_nodeid].Son_node[0] || 2 == BB.branch[BB.branch[BB.cur_node].pre_nodeid].Son_node[1])
	{
		//���BB.cur_node���Է�֧����������������֧��ѡ��LB_value��С��Ϊ��һ�εķ�֧�ڵ�
		temp_pre = BB.branch[BB.cur_node].pre_nodeid;
#if QUICKBranch == 0
		if (BB.branch[BB.branch[temp_pre].Son_node[0]].LB_value<BB.branch[BB.branch[temp_pre].Son_node[1]].LB_value)
		{
			BB.mother_node = BB.branch[temp_pre].Son_node[0];
		}
		else
		{
			BB.mother_node = BB.branch[temp_pre].Son_node[1];
		}
#else
		if (2 == BB.branch[BB.branch[BB.cur_node].pre_nodeid].Son_node[0])
		{
			BB.mother_node = BB.branch[temp_pre].Son_node[0];
		}
		else
		{
			BB.mother_node = BB.branch[temp_pre].Son_node[1];
		}

#endif

		return false;
	}
	else
	{
		//cur_node�޽����������ʱ���޷���ȷ�֧����ʱ�������ŷ�֧
		float temp_min = MAXNUM;
		for (int j = 0; j < BB.exist_nodeNum; j++)
		{
			if (BB.branch[j].branch_state<2 && BB.branch[j].LB_value<BB.best_upper)
			{
				if (2 == BB.branch[j].state && BB.branch[j].LB_value<temp_min)
				{
					temp_min = BB.branch[j].LB_value;
					BB.mother_node = j;
				}
			}
		}
		if (temp_min <= Conf::heuristic_BOUND && temp_min<BB.best_upper)
		{
			return false;
		}
	}
	return true;
}

void Column_Generation::Update_Solvednode(SolINFOR & SolInfo, BranchABound & BB, ColumnPool & pool)
{
	BB.branch[BB.cur_node].branch_state = 1;
	BB.branch[BB.cur_node].total_column_num = pool.CandidateColumn_Num;
	//RMP�����Ϣ
	BB.branch[BB.cur_node].LB_value = SolInfo.Cur_solution.OBJ_lower;

	BB.branch[BB.cur_node].LBsol_num = SolInfo.Cur_solution.best_LBsol_num;
	for (int i = 0; i<SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		BB.branch[BB.cur_node].LBsol_index[i] = SolInfo.Cur_solution.best_LBsol_index[i];
		BB.branch[BB.cur_node].LBsol_value[i] = SolInfo.Cur_solution.best_LBsol_value[i];
	}
}

bool Column_Generation::Generate_Kpath(SolINFOR &SolInfo, BranchABound &BB, Subproblem &subp, ColumnPool &colp, Problem &p, RMP &LP, KPATH &Cuts_kpath)
{
	if (Cuts_kpath.processed_num>=Conf::MAX_KPATH_NUM)
	{
		return true;
	}
	//����һ����ʼKpath-cut
	int *temp_Kpath;
	temp_Kpath = new int[Conf::MAX_SUBSET_KPATH];
	int temp_Kpath_num = 0;
	//�����ó�ʼKpath-cut�İ����ڵ�Ķ�����
	long *temp_Kpath_passnode;
	temp_Kpath_passnode = new long[p.passnode_length];
	for (int i = 0; i <p.passnode_length; i++)
	{
		temp_Kpath_passnode[i] = 0;
	}
	Extend_Kpath_cut(0, temp_Kpath, temp_Kpath_num, temp_Kpath_passnode, BB, subp, p, LP, Cuts_kpath);
	delete temp_Kpath;
	delete temp_Kpath_passnode;

	if (Cuts_kpath.added_num >0)
	{
		Cuts_kpath.added_num = int(fmin(Cuts_kpath.added_num, Conf::MAX_KPATH_NUM - Cuts_kpath.processed_num));
		return false;
	}
	else
	{
		return true;
	}
}

void Column_Generation::Extend_Kpath_cut(float cur_RHS, int * cur_Kpath, int cur_Kpath_num, long * cur_Kpath_passnode, BranchABound & BB, Subproblem & subp, Problem & p, RMP & LP, KPATH & Cuts_kpath)
{
	//�ݹ���ֹ����
	//Kpath�ﵽConf::MAX_SUBSET_KPATH�������ֹ
	//�����Ѿ��ҵ�MAX_ADD_KPATH��Kpath
	if (cur_Kpath_num >= Conf::MAX_SUBSET_KPATH || Cuts_kpath.added_num>=Conf::MAX_ADD_KPATH)
	{
		return;
	}
	int lineid, rowid;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//������һ��Kpath-cut
	int *next_Kpath;
	next_Kpath = new int[Conf::MAX_SUBSET_KPATH];
	for (int i = 0; i < cur_Kpath_num; i++)
	{
		next_Kpath[i] = cur_Kpath[i];
	}
	int next_Kpath_num = 0;
	//�����ó�ʼKpath-cut�İ����ڵ�Ķ�����
	long *next_Kpath_passnode;
	next_Kpath_passnode = new long[p.passnode_length];
	//
	float next_RHS = 0;
	int count = 0;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//i������cur_Kpath
		if (false == p.belong_toset(i, cur_Kpath_passnode))
		{
			//��cur_Kpath���i���Ƿ���ڻ�
			next_RHS = 0;
			for (int j = 0; j < cur_Kpath_num; j++)
			{
				next_RHS -= (LP.Arc_flow[cur_Kpath[j]][i] + LP.Arc_flow[i][cur_Kpath[j]]);
			}
			//ֻѰ��֧��ͼ�ϵĻ�
			if (next_RHS<-MINDOUBLE || 0 == cur_Kpath_num)
			{
				//�ȼ���i���Inflow_node
				next_RHS += (cur_RHS + 1.0);
				//ϵ�����ܳ���2-Conf::MIN_KPATH_THRESHOLD
				if (next_RHS<2 - Conf::MIN_KPATH_THRESHOLD)
				{
					count++;
					//�����µ�Kpath
					next_Kpath[cur_Kpath_num] = i;
					next_Kpath_num = cur_Kpath_num + 1;
					for (int k = 0; k < p.passnode_length; k++)
					{
						next_Kpath_passnode[k] = cur_Kpath_passnode[k];
					}
					rowid = int((i + MINDOUBLE) / Conf::EACH_LONG_NUM);
					lineid = i - rowid*Conf::EACH_LONG_NUM;
					next_Kpath_passnode[rowid] += p.powerlist[lineid];
					Extend_Kpath_cut(next_RHS, next_Kpath, next_Kpath_num, next_Kpath_passnode, BB, subp, p, LP, Cuts_kpath);
				}
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (0== count && cur_RHS>1)
	{
		//�жϵ�ǰcur_Kpath�Ƿ��Ѿ���Kpath_subset��
		if (false == Cuts_kpath.Already_inKpath(cur_Kpath, cur_Kpath_num))
		{
			bool feasible = true;
			//�����µ�Kpath�����ж��Ҳ�ϵ���Ƿ�Ϊ2��TSPTW�޽⣩
			for (int i = 0; i < p.Depot_Num; i++)
			{
				if (false == subp.TSPTW_exist(i, cur_Kpath, cur_Kpath_num, BB, LP, p))
				{
					feasible = false;
					break;
				}
			}
			if (false== feasible)
			{
				//���cur_Kpath��Cuts_kpath��
				Cuts_kpath.Add_Kpath(cur_Kpath, cur_Kpath_num);
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	delete next_Kpath;
	delete next_Kpath_passnode;
}

bool Column_Generation::Generate_RCC(SolINFOR & SolInfo, BranchABound & BB, Subproblem & subp, ColumnPool & colp, Problem & p, RMP & LP, RCC & Cuts_rcc)
{
	if (Cuts_rcc.processed_num >= Conf::MAX_RCC_NUM)
	{
		return true;
	}
	//����һ����ʼRCC-cut
	int *temp_RCC;
	temp_RCC = new int[Conf::MAX_SUBSET_RCC];
	int temp_RCC_num = 0;
	//�����ó�ʼRCC-cut�İ����ڵ�Ķ�����
	long *temp_RCC_passnode;
	temp_RCC_passnode = new long[p.passnode_length];
	for (int i = 0; i <p.passnode_length; i++)
	{
		temp_RCC_passnode[i] = 0;
	}
	Extend_RCC_cut(0,0, temp_RCC, temp_RCC_num, temp_RCC_passnode, BB, subp, p, LP, Cuts_rcc);
	delete temp_RCC;
	delete temp_RCC_passnode;

	if (Cuts_rcc.added_num >0)
	{
		Cuts_rcc.added_num = int(fmin(Cuts_rcc.added_num, Conf::MAX_RCC_NUM - Cuts_rcc.processed_num));
		return false;
	}
	else
	{
		return true;
	}
}

void Column_Generation::Extend_RCC_cut(float cur_load, float cur_RHS, int * cur_RCC, int cur_RCC_num, long * cur_RCC_passnode, BranchABound & BB, Subproblem & subp, Problem & p, RMP & LP, RCC & Cuts_rcc)
{
	//�ݹ���ֹ����
	//RCC�ﵽConf::MAX_SUBSET_RCC�������ֹ
	//�����Ѿ��ҵ�MAX_ADD_RCC��RCC
	if (cur_RCC_num >= Conf::MAX_SUBSET_RCC || Cuts_rcc.added_num >= Conf::MAX_ADD_RCC || cur_RHS>=Conf::MAX_RCC_RHS)
	{
		return;
	}
	int lineid, rowid;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//������һ��RCC-cut
	int *next_RCC;
	next_RCC = new int[Conf::MAX_SUBSET_RCC];
	for (int i = 0; i < cur_RCC_num; i++)
	{
		next_RCC[i] = cur_RCC[i];
	}
	int next_RCC_num = 0;
	//�����ó�ʼRCC-cut�İ����ڵ�Ķ�����
	long *next_RCC_passnode;
	next_RCC_passnode = new long[p.passnode_length];
	//
	float next_load = 0;
	float next_RHS = 0;
	int count = 0;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//i������cur_RCC
		if (false == p.belong_toset(i, cur_RCC_passnode))
		{
			//��cur_RCC���i���Ƿ���ڻ�
			next_RHS = 0;
			for (int j = 0; j < cur_RCC_num; j++)
			{
				next_RHS -= (LP.Arc_flow[cur_RCC[j]][i] + LP.Arc_flow[i][cur_RCC[j]]);
			}
			//ֻѰ��֧��ͼ�ϵĻ�
			if (next_RHS<-MINDOUBLE || 0 == cur_RCC_num)
			{
				//�ȼ���i���Inflow_node
				next_RHS += (cur_RHS + 1.0);
				//ϵ�����ܳ���Conf::MAX_RCC_RHS-Conf::MIN_RCC_THRESHOLD
				if (next_RHS<Conf::MAX_RCC_RHS - Conf::MIN_RCC_THRESHOLD)
				{
					count++;
					//�����µ�RCC
					next_RCC[cur_RCC_num] = i;
					next_RCC_num = cur_RCC_num + 1;
					for (int k = 0; k < p.passnode_length; k++)
					{
						next_RCC_passnode[k] = cur_RCC_passnode[k];
					}
					rowid = int((i + MINDOUBLE) / Conf::EACH_LONG_NUM);
					lineid = i - rowid*Conf::EACH_LONG_NUM;
					next_RCC_passnode[rowid] += p.powerlist[lineid];
					next_load = cur_load + p.Allnode[i].demand;
					Extend_RCC_cut(next_load,next_RHS, next_RCC, next_RCC_num, next_RCC_passnode, BB, subp, p, LP, Cuts_rcc);
				}
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(cur_RHS<ceil(cur_load / p.Veh.Veh_Capacity))
	//if (0 == count && cur_RHS>Conf::MAX_RCC_RHS-1 && cur_load> p.Veh.Veh_Capacity*(Conf::MAX_RCC_RHS-1))
	{
		//�жϵ�ǰcur_RCC�Ƿ��Ѿ���RCC_subset��
		if (false == Cuts_rcc.Already_inRCC(cur_RCC_passnode, p))
		{
			//���cur_RCC��Cuts_rcc��
			Cuts_rcc.Add_RCC(ceil(cur_load / p.Veh.Veh_Capacity), cur_RCC, cur_RCC_num, cur_RCC_passnode, p);
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	delete next_RCC;
	delete next_RCC_passnode;
}

bool Column_Generation::Generate_SRC(SolINFOR & SolInfo, BranchABound & BB, Subproblem & subp, ColumnPool & colp, Problem & p, RMP & LP, SRC & Cuts_src)
{
	if (Cuts_src.processed_num >= Conf::MAX_SR_NUM)
	{
		return true;
	}

	float temp_vio = 0;
	int temp_visit_num = 0;
	int temp_RHS = 0;
	int insert_posi = 0;
	Cuts_src.violation_num = 0;
	//|C|=3ʱ
	if (Conf::MAX_SR_SET>=3)
	{
		temp_RHS = floor(Conf::SR_MULTIPY_3 * 3);
		//����������Ԫ��,ע������ϣ�����������
		for (int i = 0; i < p.Customer_Num; i++)
		{
			for (int j = i + 1; j < p.Customer_Num; j++)
			{
				for (int k = j + 1; k < p.Customer_Num; k++)
				{
					//���鵱ǰ����
					//�õ�ϵ��
					temp_vio = 0;
					Cuts_src.temp_2D[Conf::MAX_SR_SET + 1] = 0;
					for (int s = 0; s < SolInfo.Cur_solution.best_LBsol_num; s++)
					{
						temp_visit_num = colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[i] + colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[j] + colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[k];
						if (Conf::SR_MULTIPY_3*temp_visit_num >= 1)
						{
							Cuts_src.temp_2D[Conf::MAX_SR_SET + 1]++;
							Cuts_src.temp_2D[Conf::MAX_SR_SET + 1 + Cuts_src.temp_2D[Conf::MAX_SR_SET + 1]] = SolInfo.Cur_solution.best_LBsol_index[s];
							temp_vio = temp_vio + SolInfo.Cur_solution.best_LBsol_value[s] * floor(Conf::SR_MULTIPY_3*temp_visit_num);
						}
					}
					temp_vio = temp_vio - temp_RHS;
					//Υ���̶���Ҫ>=Conf::MIN_SR_THRESHOLD
					if (temp_vio >= Conf::MIN_SR_THRESHOLD)
					{
						//��violation��subset_route������
						//���ҵ�violation����λ��
						insert_posi = Cuts_src.Get_insert_no(temp_vio);
						//���insert_posiΪ-1���Ų��Ϻ�
						if (insert_posi >= 0)
						{
							//������temp_2D
							Cuts_src.temp_2D[0] = 3;

							Cuts_src.temp_2D[1] = i;
							Cuts_src.temp_2D[2] = j;
							Cuts_src.temp_2D[3] = k;
							//�ٲ���
							Cuts_src.Insert_VioandRoute(insert_posi, temp_vio);
						}
					}
				}
			}
		}
	}

	//|C|=4ʱ
	if (Conf::MAX_SR_SET >= 4)
	{
		temp_RHS = floor(Conf::SR_MULTIPY_4 * 4);
		//����������Ԫ��,��������C������������벻����MAX_SR_DISTANCE
		for (int n_1 = 0; n_1 < p.Customer_Num; n_1++)
		{
			for (int n_2 = n_1 + 1; n_2 < p.Customer_Num; n_2++)
			{
				if (p.Cost_network[n_1][n_2]>Conf::MAX_SR_DISTANCE)continue;
				for (int n_3 = n_2 + 1; n_3 < p.Customer_Num; n_3++)
				{
					if (p.Cost_network[n_1][n_3]>Conf::MAX_SR_DISTANCE)continue;
					if (p.Cost_network[n_2][n_3]>Conf::MAX_SR_DISTANCE)continue;
					for (int n_4 = n_3 + 1; n_4 < p.Customer_Num; n_4++)
					{
						if (p.Cost_network[n_1][n_4]>Conf::MAX_SR_DISTANCE)continue;
						if (p.Cost_network[n_2][n_4]>Conf::MAX_SR_DISTANCE)continue;
						if (p.Cost_network[n_3][n_4]>Conf::MAX_SR_DISTANCE)continue;
						//���϶�������|C|=4ʱ��C������������벻����MAX_SR_DISTANCE
						//���鵱ǰ����
						//�õ�ϵ��
						temp_vio = 0;
						Cuts_src.temp_2D[Conf::MAX_SR_SET + 1] = 0;
						for (int s = 0; s < SolInfo.Cur_solution.best_LBsol_num; s++)
						{
							temp_visit_num = colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_1] + colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_2]
								+ colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_3] + colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_4];
							if (Conf::SR_MULTIPY_4*temp_visit_num >= 1)
							{
								Cuts_src.temp_2D[Conf::MAX_SR_SET + 1]++;
								Cuts_src.temp_2D[Conf::MAX_SR_SET + 1 + Cuts_src.temp_2D[Conf::MAX_SR_SET + 1]] = SolInfo.Cur_solution.best_LBsol_index[s];
								temp_vio = temp_vio + SolInfo.Cur_solution.best_LBsol_value[s] * floor(Conf::SR_MULTIPY_4*temp_visit_num);
							}
						}
						temp_vio = temp_vio - temp_RHS;
						//Υ���̶���Ҫ>=Conf::MIN_SR_THRESHOLD
						if (temp_vio >= Conf::MIN_SR_THRESHOLD)
						{
							//��violation��subset_route������
							//���ҵ�violation����λ��
							insert_posi = Cuts_src.Get_insert_no(temp_vio);
							//���insert_posiΪ-1���Ų��Ϻ�
							if (insert_posi >= 0)
							{
								//������temp_2D
								Cuts_src.temp_2D[0] = 4;

								Cuts_src.temp_2D[1] = n_1;
								Cuts_src.temp_2D[2] = n_2;
								Cuts_src.temp_2D[3] = n_3;
								Cuts_src.temp_2D[4] = n_4;
								//�ٲ���
								Cuts_src.Insert_VioandRoute(insert_posi, temp_vio);
							}
						}
					}
				}
			}
		}
	}

	//|C|=5ʱ
	if (Conf::MAX_SR_SET >= 5)
	{
		temp_RHS = floor(Conf::SR_MULTIPY_5 * 5);
		//����������Ԫ��,��������C������������벻����MAX_SR_DISTANCE
		for (int n_1 = 0; n_1 < p.Customer_Num; n_1++)
		{
			for (int n_2 = n_1 + 1; n_2 < p.Customer_Num; n_2++)
			{
				if (p.Cost_network[n_1][n_2]>Conf::MAX_SR_DISTANCE)continue;
				for (int n_3 = n_2 + 1; n_3 < p.Customer_Num; n_3++)
				{
					if (p.Cost_network[n_1][n_3]>Conf::MAX_SR_DISTANCE)continue;
					if (p.Cost_network[n_2][n_3]>Conf::MAX_SR_DISTANCE)continue;
					for (int n_4 = n_3 + 1; n_4 < p.Customer_Num; n_4++)
					{
						if (p.Cost_network[n_1][n_4]>Conf::MAX_SR_DISTANCE)continue;
						if (p.Cost_network[n_2][n_4]>Conf::MAX_SR_DISTANCE)continue;
						if (p.Cost_network[n_3][n_4]>Conf::MAX_SR_DISTANCE)continue;
						for (int n_5 = n_4 + 1; n_5 < p.Customer_Num; n_5++)
						{
							if (p.Cost_network[n_1][n_5]>Conf::MAX_SR_DISTANCE)continue;
							if (p.Cost_network[n_2][n_5]>Conf::MAX_SR_DISTANCE)continue;
							if (p.Cost_network[n_3][n_5]>Conf::MAX_SR_DISTANCE)continue;
							if (p.Cost_network[n_4][n_5]>Conf::MAX_SR_DISTANCE)continue;
							//���϶�������|C|=5ʱ��C������������벻����MAX_SR_DISTANCE
							//���鵱ǰ����
							//�õ�ϵ��
							temp_vio = 0;
							Cuts_src.temp_2D[Conf::MAX_SR_SET + 1] = 0;
							for (int s = 0; s < SolInfo.Cur_solution.best_LBsol_num; s++)
							{
								temp_visit_num = colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_1] + colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_2]
									+ colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_3] + colp.Col[SolInfo.Cur_solution.best_LBsol_index[s]].Customer_indicator[n_4];
								if (Conf::SR_MULTIPY_5*temp_visit_num >= 1)
								{
									Cuts_src.temp_2D[Conf::MAX_SR_SET + 1]++;
									Cuts_src.temp_2D[Conf::MAX_SR_SET + 1 + Cuts_src.temp_2D[Conf::MAX_SR_SET + 1]] = SolInfo.Cur_solution.best_LBsol_index[s];
									temp_vio = temp_vio + SolInfo.Cur_solution.best_LBsol_value[s] * floor(Conf::SR_MULTIPY_5*temp_visit_num);
								}
							}
							temp_vio = temp_vio - temp_RHS;
							//Υ���̶���Ҫ>=Conf::MIN_SR_THRESHOLD
							if (temp_vio >= Conf::MIN_SR_THRESHOLD)
							{
								//��violation��subset_route������
								//���ҵ�violation����λ��
								insert_posi = Cuts_src.Get_insert_no(temp_vio);
								//���insert_posiΪ-1���Ų��Ϻ�
								if (insert_posi >= 0)
								{
									//������temp_2D
									Cuts_src.temp_2D[0] = 5;

									Cuts_src.temp_2D[1] = n_1;
									Cuts_src.temp_2D[2] = n_2;
									Cuts_src.temp_2D[3] = n_3;
									Cuts_src.temp_2D[4] = n_4;
									Cuts_src.temp_2D[5] = n_5;
									//�ٲ���
									Cuts_src.Insert_VioandRoute(insert_posi, temp_vio);
								}
							}
						}
					}
				}
			}
		}
	}
	if (Conf::MAX_SR_SET >= 6)
	{
		//������
		cout << "ERROR: ��û�б���" << endl;
		cin.get();
	}

	//����������ѡ��ǰCuts_src.violation_num��subset-row����RMP
	//���ȣ�����ǰCuts_src.violation_num��SRC��SRC_LimitVertexSet_indicator��SRC_LimitArcSet_indicator
	//Ȼ�󣬼����Ƿ������Ҫ�ϲ���SRC_LimitVertexSet_indicator
	int *augment_Vertex;
	int augment_Vertex_num = 0;
	augment_Vertex = new int[p.Customer_Num];
	float temp_state = 0;

	for (int i = 0; i < Cuts_src.violation_num; i++)
	{
		//������SRC_LimitVertexSet_indicator
		//SRC_subset_indicator
		Cuts_src.SRC_RHS[Cuts_src.processed_num + Cuts_src.added_num] = Cuts_src.Get_RHS(Cuts_src.subset_route[i][0]);
		Cuts_src.SRC_subset_len[Cuts_src.processed_num + Cuts_src.added_num] = Cuts_src.subset_route[i][0];
		Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num] = Cuts_src.subset_route[i][0];
		for (int s = 0; s < p.Depot_Num+p.Customer_Num; s++)
		{
			Cuts_src.SRC_subset_indicator[Cuts_src.processed_num+ Cuts_src.added_num][s] = 0;
			Cuts_src.SRC_LimitVertexSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][s] = 0;
		}
		for (int s = 0; s < Cuts_src.subset_route[i][0]; s++)
		{
			Cuts_src.SRC_subset[Cuts_src.processed_num + Cuts_src.added_num][s] = Cuts_src.subset_route[i][1 + s];
			Cuts_src.SRC_subset_indicator[Cuts_src.processed_num + Cuts_src.added_num][Cuts_src.subset_route[i][1+s]] = 1;
			//SRC_LimitVertexSet_indicator
			Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][s] = Cuts_src.subset_route[i][1 + s];
			Cuts_src.SRC_LimitVertexSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][Cuts_src.subset_route[i][1 + s]] = 1;
		}
		for (int j = 0; j < Cuts_src.subset_route[i][Conf::MAX_SR_SET + 1]; j++)
		{
			temp_state = 0;
			augment_Vertex_num = 0;
			for (int k = 0; k <colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodesNum ; k++)
			{
				if (1==Cuts_src.SRC_subset_indicator[Cuts_src.processed_num + Cuts_src.added_num][colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodes[k]])
				{
					temp_state = temp_state + Cuts_src.add_state(Cuts_src.subset_route[i][0]);
					if (temp_state >= 1)
					{
						for (int m = 0; m < augment_Vertex_num; m++)
						{
							if (1== Cuts_src.SRC_LimitVertexSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][augment_Vertex[m]]) continue;
							Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num]] = augment_Vertex[m];
							Cuts_src.SRC_LimitVertexSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][augment_Vertex[m]] = 1;
							Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num]++;
						}
						augment_Vertex_num = 0;
						temp_state = temp_state - 1;
					}
				}
				else if (temp_state>MINDOUBLE)
				{
					augment_Vertex[augment_Vertex_num] = colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodes[k];
					augment_Vertex_num++;
				}
			}
		}
		//������SRC_LimitArcSet_indicator
		for (int j = 0; j < p.Depot_Num + p.Customer_Num; j++)
		{	
			for (int k = 0; k < p.Depot_Num + p.Customer_Num; k++)
			{
				Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][j][k] = 0;
			}
		}
#if LIMITTPYE == 0
		//vertex-memory
		//����M��M
		for(int j = 0; j < Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num]; j++)
		{
			for (int k = j+1; k < Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num]; k++)
			{
				Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][j]][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k]] = 1;
				Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k]][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][j]] = 1;
			}
		}
#else
		//arc-memory
		//����·���ϵĻ������������M��C�ʹ�C��M
		//ÿ��·���ϵĻ�
		for (int j = 0; j < Cuts_src.subset_route[i][Conf::MAX_SR_SET + 1]; j++)
		{
			for (int k = 0; k < colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodesNum-1; k++)
			{
				if (1== Cuts_src.SRC_LimitVertexSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodes[k]]
					&& 1 == Cuts_src.SRC_LimitVertexSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodes[k+1]])
				{
					Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodes[k]][colp.Col[Cuts_src.subset_route[i][Conf::MAX_SR_SET + 2 + j]].nodes[k+1]] = 1;
				}
			}
		}
		//��M��C�ʹ�C��M
		for (int j = 0; j < Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num]; j++)
		{
			for (int k = 0; k < Cuts_src.SRC_subset_len[Cuts_src.processed_num + Cuts_src.added_num]; k++)
			{
				if (Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][j]== Cuts_src.SRC_subset[Cuts_src.processed_num + Cuts_src.added_num][k])continue;
				Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][j]][Cuts_src.SRC_subset[Cuts_src.processed_num + Cuts_src.added_num][k]] = 1;
				Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][Cuts_src.SRC_subset[Cuts_src.processed_num + Cuts_src.added_num][k]][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][j]] = 1;
			}
		}
#endif
		//��������Ƿ������Ҫ�ϲ���SRC_LimitVertexSet_indicator��SRC_LimitVertexSet_indicator
		for (int j = 0; j < Cuts_src.processed_num; j++)
		{
			//����Cuts_src.SRC_subset[j]�Ƿ���Cuts_src.SRC_subset[Cuts_src.processed_num + Cuts_src.added_num]��ȫ��ͬ
			if (true== Cuts_src.Check_SameSubset(j, Cuts_src.processed_num + Cuts_src.added_num))
			{
				//�ϲ�Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num]��Cuts_src.SRC_LimitVertexSet[j]
				for (int k = 0; k < Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num]; k++)
				{
					if (0==Cuts_src.SRC_LimitVertexSet_indicator[j][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k]])
					{
						Cuts_src.SRC_LimitVertexSet[j][Cuts_src.SRC_LimitVertexSet_num[j]] = Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k];
						Cuts_src.SRC_LimitVertexSet_num[j]++;
						Cuts_src.SRC_LimitVertexSet_indicator[j][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k]] = 1;
					}
				}
				//�ϲ�Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num]��Cuts_src.SRC_LimitArcSet_indicator[j]
				for (int k = 0; k < p.Depot_Num+p.Customer_Num; k++)
				{
					for (int s = 0; s < p.Depot_Num + p.Customer_Num; s++)
					{
						Cuts_src.SRC_LimitArcSet_indicator[j][k][s] = Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][k][s];
					}
				}
				Cuts_src.added_num = Cuts_src.added_num - 1;
				break;
			}
		}
		Cuts_src.added_num++;
	}

	if (Cuts_src.added_num >0)
	{
		Cuts_src.added_num = int(fmin(Cuts_src.added_num, Conf::MAX_SR_NUM - Cuts_src.processed_num));
		return false;
	}
	else
	{
		return true;
	}
}

