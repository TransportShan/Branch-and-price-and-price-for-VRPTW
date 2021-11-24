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
	float *temp_Arrivaltime;				//检验路径当前点（即点集最后一个点）的到达时间,大小为[Conf::MAX_NODES]

	int temp_RouteLength;					//检验路径经过节点个数
	float temp_RCvalue;						//检验路径的RC值
	float temp_Load;						//检验路径的装载量
	float temp_Travelcost;					//检验路径的实际成本（长度）
	float temp_Duration;					//检验路径的当前已经累积的运输时间
	
	temp_Arrivaltime = new float[Conf::MAX_NODES];

	//检查出发点
	if (route[0] != startnode)
	{
		//有问题
		cout <<"ERROR: 路径出发点不对" << endl;
		cin.get();
	}
	else
	{
		temp_RouteLength = routeLen;
	}

	//到达时间-时间窗
	temp_Arrivaltime[0] = 0;
	float serviceT = 0;	//服务时间
	float travelT = 0;	//运输时间
	for (int i = 1; i<temp_RouteLength; i++)
	{
		serviceT = p.Allnode[route[i - 1]].servicetime;
		travelT = p.Calculate_traveltime(route[i - 1], route[i]);
		temp_Arrivaltime[i] = max(temp_Arrivaltime[i - 1] + serviceT + travelT, p.Allnode[route[i]].startTW);
		if (temp_Arrivaltime[i]>p.Allnode[route[i]].endTW)
		{
			cout << "ERROR：Arrivaltime" << endl;
			cin.get();
		}
	}

	//capacity 和 duration
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
		cout << "ERROR：Duration or Capacity" << endl;
		cin.get();
	}
#else
	if (temp_Load>p.Veh.Veh_Capacity)
	{
		cout << "ERROR：Capacity" << endl;
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
	//构建SDC对应的RC值
	int *check_SDCnodes;
	check_SDCnodes = new int[p.Customer_Num];
	for (int i = 0; i < Cuts_sdc.processed_num; i++)
	{
		check_SDCnodes[Cuts_sdc.SDC_nodes[i]] = Cuts_sdc.SDC_indicator[Cuts_sdc.SDC_nodes[i]];
	}
	
	for (int i = 1; i<temp_RouteLength - 1; i++)
	{
		//首先在Cuts_sdc.SDC_nodes中
		if (1== Cuts_sdc.SDC_indicator[route[i]])
		{
			//然后在check_SDCnodes第一次被检验
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
		cout << "ERROR：RC" << endl;
		cin.get();
	}

	delete temp_Arrivaltime;
}

void Column_Generation::SolveNode_ByCG(Utility & Results, SolINFOR &SolInfo, BranchABound & BB, Initialization & IniC, ColumnPool & pool, Problem & p, RMP & lp_node, Subproblem & subp, SRC &Cuts_src, KPATH &Cuts_kpath, RCC &Cuts_rcc, SDC &Cuts_sdc)
{
	//每次CG重置参数
	solved_depotNum = 0;
	Tosolve_depotno = 0;
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//主体算法参数声明
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//记录CG时间
	clock_t CG_start = clock();				//CG开始时间
	clock_t	CG_end;							//CG结束时间
	//算法终止条件参数
	bool Termination = false;				//当Termination为true时，该node上CG算法结束
	bool inter_Terminate;					//内部循环的终止条件，当inter_Terminate为true时，进入主循环
	int iter_num_CG = 1;					//记录迭代次数,求解一次RMP问题记为一次迭代
	int iter_num_Reopt = 0;					//CPA框架下重新优化（添加一组SDC）的次数
	//重置DSSR框架下的ngset
#if INHERIT == 0
	p.Set_dssr_ngset();
#endif
	//设置算法开始阶段使用启发式求解
	bool algorithm_state = false;
	//第一次求解RMP问题，构建CPLEX环境
	//初始化该节点上RMP,再检验当前列池能否求解
	if (false == lp_node.Initial_Rmpnode(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc,Cuts_sdc, Cuts_src))
	{
		//应该给出一个初始解
		//这里就简单地把下界值赋值为MAXNUM
		BB.branch[BB.cur_node].LB_value = MAXNUM;
		BB.branch[BB.cur_node].UB_value = MAXNUM;
		return;
	}
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//主循环开始
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	do//外循环
	{
		inter_Terminate = false;
#if Frameworks == 1
		Remain_pro = Conf::Remain_proIni;
#endif	
		do//内循环
		{
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//求解PSP问题
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
					//CPA框架判断PSP的求解结果，RMP中若只有初等路径则，跳出内循环
					inter_Terminate= true;
#endif				
#elif EXACTPSP == 1	
#if Frameworks == 1
					cout<< "ERROR:CPA框架下不能使用DSSR框架" << endl;
					cin.get();
#endif	
					//一旦启发式DP无法找到解，就调用精确DP
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
			//SRC_cuts回滚
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
#if SRCUT == 1
			//判断是否需要调用回滚操作，当label的数量过多的时候需要调用回滚
			//if (subp.all_label_num / subp.init_label_num >= 50)
			//{
			//	//从node上删去新增加的src
			//	update_src_separation(true, node, temp_src, branchnodes);
			//	//调用回滚
			//	roll_back(temp_src);
			//}
#endif
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//求解RMP问题
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//先更新RMP问题中的矩阵信息
			lp_node.Update_RMP(BB, pool);
			//调用CPLEX求解RMP，求得约束对应的影子价格
			lp_node.StandardRMP_SolvebyCPLEX_Real(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc, Cuts_sdc, Cuts_src);
			//记录迭代次数
			iter_num_CG++;
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//划分列池（S，Is）
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//根据RMP求解结果，更新S
			//Update_posi_baseornot(S, Positive_variable_num, routeset); //该功能不必要，为节省运算时间可以注释掉//pvrptw未修改，注意在mdvrptw时要取消注释
			//根据RMP求解结果（考虑场站约束），更新Is（可选择）
			//Compatible_byM3_depot_real_eletransformation(S, Positive_variable_num, routeset, temp_op);//pvrptw未修改，注意在mdvrptw时要取消注释
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//求解增广对偶解
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//记录求解增广对偶的开始时间
			//AugmentDUAL_start = clock();
			//根据划分结果Is，用CPLEX求解增广对偶问题(考虑场站约束)（可选择）
			//AugmentDUAL_solve_byCPLEX_depot_Real_getbasis_disturb_dual(temp_op, temp_rmp, routeset, S, Positive_variable_num);
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//跳出内循环
			//----------------------------------------------------------------------------------------------------------------------------------------------------------
#if Frameworks == 0
			//EPO框架直接跳出循环 
			inter_Terminate = true;
#endif
		} while (false== inter_Terminate);

#if Frameworks == 1
		//CPA框架首先根据上次RMP求解判断终止条件：若RMP的解都是初等路径则算法结束
		//否则，需要添加SDC-cuts
		if (true== lp_node.Strengthen_elementary(Results, SolInfo, BB, pool, p, Cuts_sdc))
		{
			Termination = true;
		}
		else
		{
			//并求解添加SDC_cuts后的RMP，得到SDC_dual
			lp_node.Added_columnNum_once = 0;
			lp_node.StandardRMP_SolvebyCPLEX_Real(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc, Cuts_sdc, Cuts_src);
			//记录迭代次数
			iter_num_CG++;
			iter_num_Reopt++;
		}
#endif
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
		//跳出外循环
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
		//增加cut
		//----------------------------------------------------------------------------------------------------------------------------------------------------------
#if KPATHCUT+RCCUT+SRCUT >0
		if (true == Termination && BB.branch[BB.cur_node].depth <=Conf::MAX_ROBUST_DEPTH)
		{
			//首先不能是整数解
			if (false== SolInfo.check_IntSol())
			{
#if KPATHCUT+RCCUT >0
				//根据RMP当前解信息，得到任意弧上的流量
				lp_node.Update_flow(SolInfo, pool, p);
#endif
				//增加Kpath-cuts
#if KPATHCUT ==1
				Termination = Termination && Generate_Kpath(SolInfo, BB, subp, pool, p, lp_node, Cuts_kpath);
#endif
				//增加RCC-cuts
#if RCCUT ==1
				Termination = Termination && Generate_RCC(SolInfo, BB, subp, pool, p, lp_node, Cuts_rcc);
#endif
				//增加subset-row cuts
#if SRCUT == 1
				//|C|=3时，遍历所有三元组
				//|C|>3时，只查看内部间距最大为Conf::MAX_SR_DISTANCE的组
				Termination = Termination && Generate_SRC(SolInfo, BB, subp, pool, p, lp_node, Cuts_src);
#endif
				//求解满足添加cuts后的对偶变量
				if (false == Termination)
				{
					//并求解添加cuts后的RMP
					lp_node.Added_columnNum_once = 0;
					lp_node.StandardRMP_SolvebyCPLEX_Real(SolInfo, BB, pool, p, Cuts_kpath, Cuts_rcc, Cuts_sdc, Cuts_src);
					//记录迭代次数
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
	} while (pool.CandidateColumn_Num<Conf::MAX_COLUMN_IN_RMP-1000);	//终止条件，注意别超界
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//主循环结束
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	
#if UPBOUND == 1
	if (false== lp_node.StandardRMP_SolvebyCPLEX_Int(SolInfo, BB, pool))
	{
		BB.branch[BB.cur_node].UB_value = MAXNUM;
	}
	BB.branch[BB.cur_node].UB_value = lp_node.OBJ_value;
#endif
	//对CPLEX环境重置但不释放,同时开辟新的CPLEX环境
	lp_node.reset_RmpMatrix_byclass();
	lp_node.Rmp_matrix.ini_cplex();
	//记录rootnode上的指标
	CG_end = clock();
	Results.Reopt_num = iter_num_Reopt;
	Results.CGitr_num = iter_num_CG;
	Results.Total_time = (float)(CG_end - CG_start) / CLOCKS_PER_SEC;
	Results.SDC_num = Cuts_sdc.processed_num;
	//输出rootnode上的指标
	if (0 == BB.cur_node)
	{
		Results.Show_RootNode_results();
	}
	Results.Reset_UTILITY();
	//更新该分支节点上的状态
	Update_Solvednode(SolInfo, BB, pool);
}

bool Column_Generation::Solve_PricingSubproble(bool exact, Utility & Results, BranchABound & BB, Subproblem & subp, RMP &lp, ColumnPool & colp, Problem & p, SRC & temp_src, SDC &Cuts_sdc)
{
	//精确求解，不调用2cycle
	subp.exact_ornot = exact;
	bool solution_state = true;	//=true时表示子问题使用启发式规则已经不能找到RC<0,
								//=false时表示子问题使用启发式还能找到RC<0

#if PRICING == 0
	//首先根据RMP对偶信息更新网络
	//判断RC值并生成列的过程
	for (int i = 0; i < p.Depot_Num; i++)
	{
		//第一步:从RMP获取对偶信息，指定哪天
		subp.Get_dual_info(i, Remain_pro, BB, lp, p, Cuts_sdc);
		//第二步:动态规划解ESPPRC
		if (true == subp.DP_Ng_Cuts_Dssr_Stable(BB, lp, p, temp_src, Cuts_sdc))
		{
			solution_state = false;
		}
		//第三步:将PSP中找到的路径传递给colp
		for (int j = 0; j<subp.FoundPath_container_num; j++)
		{
			if (subp.FoundPath_container[j].Reducedcost>-Conf::Exact_threshold)continue;
			//Verify_Routes(subp.FoundPath_container[j].Reducedcost, i+p.Customer_Num, subp.FoundPath_container[j].nodesNum, subp.FoundPath_container[j].nodes, subp, lp, p, Cuts_sdc);
			colp.Add_columns(true, BB, subp.FoundPath_container[j], p);
#if ADDCOL == 1
			colp.Add_auxi_columns(BB, p);
#endif
		}
		//第四步:记录PSP指标
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
	//记录PSP指标
	Results.PSP_label_num.push_back(subp.all_label_num);
	Results.PSP_time.push_back(subp.PSP_time);
#endif
}


bool Column_Generation::Detect_branch(SolINFOR &SolInfo, BranchABound & BB, ColumnPool & colp, Problem & p)
{
	//先判断这次CG迭代是否找到整数解
	if (BB.branch[BB.cur_node].LB_value>MAXNUM - 1 || BB.branch[BB.cur_node].LB_value>Conf::PRE_BOUND)
	{
		//该支无解
		//剪枝，不需要再向下分支
		return false;
	}
	//有可能无法直接调用CPLEX求解IP，因此需要在这里判断BB.branch[BB.cur_node].LB_value是否和BB.branch[BB.cur_node].UB_value相等
	int temp_index,temp_start,temp_end;
	float temp_value, min_value;

#if BRANCHVEH == 1
	//第一阶段
	//重置vehicle_num
	BB.vehicle_num = 0;
	//赋值vehicle_num
	for (int i = 0; i < SolInfo.Cur_solution.best_LBsol_num; i++)
	{
		BB.vehicle_num[colp.Col[SolInfo.Cur_solution.best_LBsol_index[i]].Depot-p.Customer_Num] = BB.vehicle_num[colp.Col[SolInfo.Cur_solution.best_LBsol_index[i]].Depot - p.Customer_Num] + SolInfo.Cur_solution.best_LBsol_value[i];
	}
	//找到最接近0.5的vehicle_num
	min_value = MAXNUM;
	for (int i = 0; i < p.Depot_Num; i++)
	{
		temp_value = BB.vehicle_num[i] - floor(BB.vehicle_num[i]);
		if (fabs(0.5 - temp_value)<min_value)
		{
			min_value = fabs(0.5 - temp_value);
			BB.branch[BB.cur_node].branch_depot = i;
			//左枝<= floor(BB.vehicle_num[i]),右枝>=floor(BB.vehicle_num[i])+1
			BB.branch[BB.cur_node].branch_vehicleNum = int(floor(BB.vehicle_num[i]));
		}
	}
	if (fabs(min_value - 0.5)>10 * MINDOUBLE)
	{
		//当前解一定不是整数解
		//只对每天使用的车辆数分支就可以
		BB.branch[BB.cur_node].branch_strategy = 0;
		return true;
	}
#endif

	//向第二阶段搜索
	//重置flow_onday
	for (int j = 0; j < p.Customer_Num; j++)
	{
		for (int k = 0; k < p.Customer_Num; k++)
		{
			BB.flow_onarc[j][k] = 0;
		}
	}
	//赋值flow_onday
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
	//在BB.flow_onarc寻找最多BB.branch[BB.cur_node].branchcons_num个弧进行分支
	//先重置customer_branch
	for (int i = 0; i < p.Customer_Num; i++)
	{
		BB.customer_branch[i] = 1;
	}
	do
	{
		//找到最接近0.5的flow_onday
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
		//当前解一定不是整数解
		//只对每天使用的车辆数分支就可以
		BB.branch[BB.cur_node].branch_strategy = 1;
		return true;
	}
	else
	{
		//当前解为整数解
		//该支定界，不需要再向下分支
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
	//左枝
	BB.branch[left_index].node_id = left_index;
	BB.branch[left_index].depth = BB.branch[BB.mother_node].depth+1;
	BB.branch[left_index].branch_state = 0;
	BB.branch[left_index].pre_nodeid = BB.mother_node;
	BB.branch[left_index].LB_value = MAXNUM;
	BB.branch[left_index].UB_value = MAXNUM;
	BB.branch[left_index].used_column_num = 0;
	BB.branch[left_index].total_column_num = 0;
	//开辟空间
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
	//复制母节点的约束
	BB.branch[left_index].Copy_Constraints(BB.branch[BB.mother_node]);
	BB.branch[left_index].addCons_Total_num = BB.branch[BB.mother_node].addCons_Total_num;
	//必须继承母节点关于车辆数的约束
	//先开辟空间，别忘释放
	BB.branch[left_index].vehicleNum_lower = new int[p.Depot_Num];
	BB.branch[left_index].vehicleNum_upper = new int[p.Depot_Num];
	//再赋值
	for (int i = 0; i < p.Depot_Num; i++)
	{
		BB.branch[left_index].vehicleNum_lower[i] = BB.branch[BB.mother_node].vehicleNum_lower[i];
		BB.branch[left_index].vehicleNum_upper[i] = BB.branch[BB.mother_node].vehicleNum_upper[i];
	}
	//添加新约束
	if (0== BB.branch[BB.mother_node].branch_strategy)
	{
		//对场站发出的车辆数的分支
		BB.branch[left_index].addCons_New_num = 0;
		BB.branch[left_index].vehicleNum_upper[BB.branch[BB.mother_node].branch_depot] = BB.branch[BB.mother_node].branch_vehicleNum;
	}
	else if(1 == BB.branch[BB.mother_node].branch_strategy)
	{
		//对弧的分支
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
		cout <<"ERROR: 分支错误"<< endl;
		cin.get();
	}
	//补齐addCons_Total
	for (int i = 0; i<BB.branch[left_index].addCons_New_num; i++)
	{
		temp_index = BB.branch[left_index].addCons_Total_num;
		if(1 == BB.branch[left_index].addCons_New[i].type)
		{
			//弧相关约束
			BB.branch[left_index].addCons_Total[temp_index].type = BB.branch[left_index].addCons_New[i].type;
			BB.branch[left_index].addCons_Total[temp_index].startnode = BB.branch[left_index].addCons_New[i].startnode;
			BB.branch[left_index].addCons_Total[temp_index].endnode = BB.branch[left_index].addCons_New[i].endnode;
			BB.branch[left_index].addCons_Total[temp_index].arc_state = BB.branch[left_index].addCons_New[i].arc_state;
		}
		else
		{
			cout << "ERROR: 分支错误" << endl;
			cin.get();
		}
		BB.branch[left_index].addCons_Total_num = BB.branch[left_index].addCons_Total_num + 1;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//右枝
	BB.branch[right_index].node_id = right_index;
	BB.branch[right_index].depth = BB.branch[BB.mother_node].depth + 1;
	BB.branch[right_index].branch_state = 0;
	BB.branch[right_index].pre_nodeid = BB.mother_node;
	BB.branch[right_index].LB_value = MAXNUM;
	BB.branch[right_index].UB_value = MAXNUM;
	BB.branch[right_index].used_column_num = 0;
	BB.branch[right_index].total_column_num = 0;
	//开辟空间
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
	//复制母节点的约束
	BB.branch[right_index].Copy_Constraints(BB.branch[BB.mother_node]);
	BB.branch[right_index].addCons_Total_num = BB.branch[BB.mother_node].addCons_Total_num;
	//必须继承母节点关于车辆数的约束
	//先开辟空间，别忘释放
	BB.branch[right_index].vehicleNum_lower = new int[p.Depot_Num];
	BB.branch[right_index].vehicleNum_upper = new int[p.Depot_Num];
	//再赋值
	for (int i = 0; i < p.Depot_Num; i++)
	{
		BB.branch[right_index].vehicleNum_lower[i] = BB.branch[BB.mother_node].vehicleNum_lower[i];
		BB.branch[right_index].vehicleNum_upper[i] = BB.branch[BB.mother_node].vehicleNum_upper[i];
	}
	//添加新约束
	if (0 == BB.branch[BB.mother_node].branch_strategy)
	{
		//对车辆数的分支
		BB.branch[right_index].addCons_New_num = 0;
		BB.branch[right_index].vehicleNum_lower[BB.branch[BB.mother_node].branch_depot] = BB.branch[BB.mother_node].branch_vehicleNum;
	}
	else if (1 == BB.branch[BB.mother_node].branch_strategy)
	{
		//对弧的分支
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
		cout << "ERROR: 分支错误" << endl;
		cin.get();
	}
	//补齐addCons_Total
	for (int i = 0; i<BB.branch[right_index].addCons_New_num; i++)
	{
		temp_index = BB.branch[right_index].addCons_Total_num;
		if (1 == BB.branch[right_index].addCons_New[i].type)
		{
			//弧相关约束
			BB.branch[right_index].addCons_Total[temp_index].type = BB.branch[right_index].addCons_New[i].type;
			BB.branch[right_index].addCons_Total[temp_index].startnode = BB.branch[right_index].addCons_New[i].startnode;
			BB.branch[right_index].addCons_Total[temp_index].endnode = BB.branch[right_index].addCons_New[i].endnode;
			BB.branch[right_index].addCons_Total[temp_index].arc_state = BB.branch[right_index].addCons_New[i].arc_state;
		}
		else
		{
			cout << "ERROR: 分支错误" << endl;
			cin.get();
		}
		BB.branch[right_index].addCons_Total_num = BB.branch[right_index].addCons_Total_num + 1;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//更新分支树
	BB.exist_nodeNum = BB.exist_nodeNum + 2;
	//分支结束将母节点置为分支结束状态
	BB.branch[BB.mother_node].branch_ornot = true;
	BB.branch[BB.mother_node].branched_num = 0;
	BB.branch[BB.mother_node].branch_state = 2;
	//释放母节点上的动态数组
	if (BB.branch[BB.mother_node].node_id>0)
	{
		delete[] BB.branch[BB.mother_node].vehicleNum_upper;
		delete[] BB.branch[BB.mother_node].vehicleNum_lower;
		delete[] BB.branch[BB.mother_node].addCons_New;
		delete[] BB.branch[BB.mother_node].addCons_Total;
		delete[] BB.branch[BB.mother_node].branch_Startnode;
		delete[] BB.branch[BB.mother_node].branch_Endnode;
	}
	//加入带求解序列
	BB.Toslove_index[0] = left_index;
	BB.Toslove_index[1] = right_index;
	BB.branch[BB.mother_node].Son_node[0] = left_index;
	BB.branch[BB.mother_node].Son_node[1] = right_index;
	BB.Toslove_num = 2;
	return false;
}

void Column_Generation::Update_used_column(BranchABound & BB, ColumnPool & colp, Problem &p)
{
	//这里做两件事
	//第一，判断哪些列能从母节点继承
	//第二，判断母节点的列池是否能释放 

	//首先开辟空间
	BB.branch[BB.cur_node].used_column = new int[Conf::MAX_COLUMN_IN_RMP];
	BB.branch[BB.cur_node].LBsol_index = new int[2 * p.Customer_Num];
	BB.branch[BB.cur_node].LBsol_value = new float[2 * p.Customer_Num];
	BB.branch[BB.cur_node].LBsol_num = 0;

	int temp_mother, temp_index, temp_start, temp_end;
	bool check;
	temp_mother = BB.branch[BB.cur_node].pre_nodeid;
	BB.branch[BB.cur_node].used_column_num = 0;

	//先判断从0到used_column_num的列是否能还满足约束
	for (int i = 0; i < BB.branch[temp_mother].used_column_num; i++)
	{
		check = true;
		temp_index = BB.branch[temp_mother].used_column[i];
		//以节点BB.branch[temp_mother].used_column为模板，修改branchnodes[node].used_column
		//检查每条约束是否满足,有关车辆数的约束对每列的可行性没有影响
		for (int j = 0; j < BB.branch[BB.cur_node].addCons_New_num; j++)
		{
			if (1 == BB.branch[BB.cur_node].addCons_New[j].type)
			{
				//弧相关约束
				temp_start = BB.branch[BB.cur_node].addCons_New[j].startnode;
				temp_end = BB.branch[BB.cur_node].addCons_New[j].endnode;
				if (true == BB.branch[BB.cur_node].addCons_New[j].arc_state)			//要求该弧一定被经过
				{
					//第一，检查如果从起点BB.branch[BB.cur_node].addCons_New[j].startnode出发到达不是BB.branch[BB.cur_node].addCons_New[j].endnode的终点,那么则约束[j]不满足
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//只要有一条弧经过就不可以
						for (int k = 0; k <colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] != temp_end)
							{
								//则约束[j]不满足
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
					//第二，检查如果从一个不是起点BB.branch[BB.cur_node].addCons_New[j].startnode出发到达BB.branch[BB.cur_node].addCons_New[j].endnode的终点,那么则约束[j]不满足
					if (colp.Col[temp_index].Customer_indicator[temp_end] >= 1)
					{
						//只要有一条弧经过就不可以
						for (int k = 0; k < colp.Col[temp_index].pre_arcs_Num[temp_end]; k++)
						{
							if (colp.Col[temp_index].pre_arcs[temp_end][k] != temp_start)
							{
								//则约束[j]不满足
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
				else         //要求该弧一定不被经过
				{
					//只需要检查从起点BB.branch[BB.cur_node].addCons_New[j].startnode到终点BB.branch[BB.cur_node].addCons_New[j].endnode的弧存在即不满足约束
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//只要有一条弧经过就不可以
						for (int k = 0; k < colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] == temp_end)
							{
								//则约束[j]不满足
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
				cout << "ERROR: 分支错误" << endl;
				cin.get();
			}
		}
		if (true == check)
		{
			BB.branch[BB.cur_node].used_column[BB.branch[BB.cur_node].used_column_num] = temp_index;
			BB.branch[BB.cur_node].used_column_num = BB.branch[BB.cur_node].used_column_num + 1;
		}
	}

	//再判断从BB.branch[BB.cur_node].total_column_num到colp.CandidateColumn_Num的列是否能还满足约束
	for (int i = BB.branch[temp_mother].total_column_num; i<colp.CandidateColumn_Num; i++)
	{
		temp_index = i;
		check = true;
		for (int j = 0; j < BB.branch[BB.cur_node].addCons_Total_num; j++)	//判断每一条约束
		{
			if (1 == BB.branch[BB.cur_node].addCons_Total[j].type)
			{
				//弧相关约束
				temp_start = BB.branch[BB.cur_node].addCons_Total[j].startnode;
				temp_end = BB.branch[BB.cur_node].addCons_Total[j].endnode;
				if (true == BB.branch[BB.cur_node].addCons_Total[j].arc_state)			//要求该弧一定被经过
				{
					//第一，检查如果从起点BB.branch[BB.cur_node].addCons_Total[j].startnode出发到达不是BB.branch[BB.cur_node].addCons_Total[j].endnode的终点,那么则约束[j]不满足
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//只要有一条弧经过就不可以
						for (int k = 0; k < colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] != temp_end)
							{
								//则约束[j]不满足
								check = false;
								break;
							}
						}
						if (false == check)
						{
							break;
						}
					}
					//第二，检查如果从一个不是起点BB.branch[BB.cur_node].addCons_Total[j].startnode出发到达BB.branch[BB.cur_node].addCons_Total[j].endnode的终点,那么则约束[j]不满足
					if (colp.Col[temp_index].Customer_indicator[temp_end] >= 1)
					{
						//只要有一条弧经过就不可以
						for (int k = 0; k < colp.Col[temp_index].pre_arcs_Num[temp_end]; k++)
						{
							if (colp.Col[temp_index].pre_arcs[temp_end][k] != temp_start)
							{
								//则约束[j]不满足
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
				else         //要求该弧一定不被经过
				{
					//只需要检查从起点B.branch[BB.cur_node].addCons_Total[j].startnode到终点BB.branch[BB.cur_node].addCons_Total[j].endnode的弧存在即不满足约束
					if (colp.Col[temp_index].Customer_indicator[temp_start] >= 1)
					{
						//只要有一条弧经过就不可以
						for (int k = 0; k < colp.Col[temp_index].succ_arcs_Num[temp_start]; k++)
						{
							if (colp.Col[temp_index].succ_arcs[temp_start][k] == temp_end)
							{
								//则约束[j]不满足
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
				cout << "ERROR: 分支错误" << endl;
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
	//先开辟空间
	BB.branch[BB.cur_node].CostNetwork_Branch = new float*[p.Customer_Num + p.Depot_Num];
	for (int i = 0; i < p.Customer_Num + p.Depot_Num; i++)
	{
		BB.branch[BB.cur_node].CostNetwork_Branch[i] = new float[p.Customer_Num + p.Depot_Num];
	}
	BB.branch[BB.cur_node].valid_Cus = new int[p.Customer_Num];

	//从母节点继承
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
	//根据新约束更新
	int temp_start, temp_end;
	float temp_value;
	for (int i = 0; i<BB.branch[BB.cur_node].addCons_New_num; i++)
	{
		if (1 == BB.branch[BB.cur_node].addCons_New[i].type)
		{
			//弧相关约束
			temp_start = BB.branch[BB.cur_node].addCons_New[i].startnode;
			temp_end = BB.branch[BB.cur_node].addCons_New[i].endnode;
			if (true == BB.branch[BB.cur_node].addCons_New[i].arc_state)			//要求该弧一定被经过
			{
				//CostNetwork_Branch
				temp_value = BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][temp_end];
				//第一，从起点BB.branch[BB.cur_node].addCons_New[i].startnode出发到不是BB.branch[BB.cur_node].addCons_New[i].endnode的弧的成本都为正无穷
				for (int k = 0; k < p.Customer_Num + p.Depot_Num; k++)
				{
					BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][k] = MAXNUM;
				}
				//第二，从一个不是起点BB.branch[BB.cur_node].addCons_New[i].startnode出发到达BB.branch[BB.cur_node].addCons_New[i].endnode的弧的成本都为正无穷
				for (int k = 0; k < p.Customer_Num + p.Depot_Num; k++)
				{
					BB.branch[BB.cur_node].CostNetwork_Branch[k][temp_end] = MAXNUM;
				}
				BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][temp_end] = temp_value;
			}
			else         //要求该弧一定不被经过
			{
				//从起点B.branch[BB.cur_node].addCons_New[i].startnode到终点BB.branch[BB.cur_node].addCons_New[i].endnode的弧的成本都为正无穷
				BB.branch[BB.cur_node].CostNetwork_Branch[temp_start][temp_end] = MAXNUM;
			}
			//exempt_Cus
			BB.branch[BB.cur_node].valid_Cus[temp_start] = 0;
			BB.branch[BB.cur_node].valid_Cus[temp_end] = 0;
		}
		else
		{
			cout << "ERROR: 分支错误" << endl;
			cin.get();
		}
	}
}


bool Column_Generation::Check_state(SolINFOR & SolInfo, BranchABound & BB, ColumnPool & colp, Problem & p)
{
	float temp_LB;
	int nownode = BB.cur_node;
	//判断在node点的求解状态state
	if (BB.branch[nownode].LB_value>Conf::PRE_BOUND || BB.branch[nownode].LB_value >= BB.best_upper || BB.branch[nownode].LB_value>MAXNUM - 1)
	{
		//剪枝
		BB.branch[nownode].state = 0;
		cout << "剪枝" << endl;
	}
	else if (BB.branch[nownode].LB_value == BB.branch[nownode].UB_value)	//找到整数解
	{
		//定界
		BB.branch[nownode].state = 1;
		if (BB.branch[nownode].LB_value<BB.best_lower)
		{
			BB.best_lower = BB.branch[nownode].LB_value;
		}
		if (BB.branch[nownode].UB_value < BB.best_upper)
		{
			//更新上界
			BB.best_upper = BB.branch[nownode].UB_value;
			//更新最优整数解
			BB.best_node_upper = nownode;
			//输出best_node_upper的服务天分配结果
			SolInfo.CopyToBest_Upper();
			Show_UBsolutions_mdvrptw(UBresult_file_name, SolInfo, colp, p);
		}
		cout << "定界" << endl;
	}
	else
	{
		//分支,必须是branch_state=1，且大于best_lower才更新best_lower
		BB.branch[nownode].state = 2;
		//先找到当前LB_value最小的
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
		//然后，若LB_value最小的比BB.best_lower大，则更新BB.best_lower
		if (0 == nownode || temp_LB>BB.best_lower + MINDOUBLE)
		{
			BB.best_lower = temp_LB;
			BB.Update_bestLB(SolInfo);
			Show_LBsolutions_mdvrptw(LBresult_file_name, SolInfo, colp, p);
		}
		cout << "分支" << endl;
	}

	//最后从求解序列Toslove_index中去除倒数第一个点
	BB.Toslove_num = BB.Toslove_num - 1;

	if (((BB.best_upper - BB.best_lower) / BB.best_upper)<Conf::MIN_GAP)
	{
		//满足终止条件
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
		oFile << "目标函数为：" << ',' << SolInfo.Best_Solution.OBJ_upper << endl;

		for (k = 0; k < p.Depot_Num; k++)
		{
			for (i = 0; i<SolInfo.Best_Solution.best_UPsol_num; i++)
			{
				if ((k+p.Customer_Num) == pool.Col[SolInfo.Best_Solution.best_UPsol_index[i]].Depot)
				{
					oFile << "第" << ',' << k + 1 << ',' << "个场站:" << ',';
					oFile << "总费用" << ',' << pool.Col[SolInfo.Best_Solution.best_UPsol_index[i]].Totalcost << ',';
					oFile << "变量值" << ',' << SolInfo.Best_Solution.best_UPsol_value[i] << ',';

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
		oFile << "目标函数为：" << ',' << SolInfo.Best_Solution.OBJ_lower << endl;
		for (k = 0; k < p.Depot_Num; k++)
		{
			for (i = 0; i<SolInfo.Best_Solution.best_LBsol_num; i++)
			{
				if ((k + p.Customer_Num) == pool.Col[SolInfo.Best_Solution.best_LBsol_index[i]].Depot)
				{
					oFile << "第" << ',' << k + 1 << ',' << "个场站:" << ',';
					oFile << "费用" << ',' << pool.Col[SolInfo.Best_Solution.best_LBsol_index[i]].Totalcost << ',';
					oFile << "变量值" << ',' << SolInfo.Best_Solution.best_LBsol_value[i] << ',';
					oFile << "列池序号" << ',' << SolInfo.Best_Solution.best_LBsol_index[i] << ',';

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
	//总体原则是，先深度分支，如果超过上界，则最优分支
	int temp_pre = -1;
	//优先对cur_node进行深度分支
	if (0 == BB.cur_node)
	{
		BB.mother_node = BB.cur_node;
		return false;
	}
	else if (2 == BB.branch[BB.branch[BB.cur_node].pre_nodeid].Son_node[0] || 2 == BB.branch[BB.branch[BB.cur_node].pre_nodeid].Son_node[1])
	{
		//如果BB.cur_node可以分支，则在左右两个分支中选择LB_value最小作为下一次的分支节点
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
		//cur_node无解或者整数解时，无法深度分支；这时进行最优分支
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
	//RMP求解信息
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
	//声明一个初始Kpath-cut
	int *temp_Kpath;
	temp_Kpath = new int[Conf::MAX_SUBSET_KPATH];
	int temp_Kpath_num = 0;
	//声明该初始Kpath-cut的包含节点的二进制
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
	//递归终止条件
	//Kpath达到Conf::MAX_SUBSET_KPATH个点就终止
	//或者已经找到MAX_ADD_KPATH个Kpath
	if (cur_Kpath_num >= Conf::MAX_SUBSET_KPATH || Cuts_kpath.added_num>=Conf::MAX_ADD_KPATH)
	{
		return;
	}
	int lineid, rowid;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//声明下一个Kpath-cut
	int *next_Kpath;
	next_Kpath = new int[Conf::MAX_SUBSET_KPATH];
	for (int i = 0; i < cur_Kpath_num; i++)
	{
		next_Kpath[i] = cur_Kpath[i];
	}
	int next_Kpath_num = 0;
	//声明该初始Kpath-cut的包含节点的二进制
	long *next_Kpath_passnode;
	next_Kpath_passnode = new long[p.passnode_length];
	//
	float next_RHS = 0;
	int count = 0;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//i不属于cur_Kpath
		if (false == p.belong_toset(i, cur_Kpath_passnode))
		{
			//在cur_Kpath与点i间是否存在弧
			next_RHS = 0;
			for (int j = 0; j < cur_Kpath_num; j++)
			{
				next_RHS -= (LP.Arc_flow[cur_Kpath[j]][i] + LP.Arc_flow[i][cur_Kpath[j]]);
			}
			//只寻找支撑图上的弧
			if (next_RHS<-MINDOUBLE || 0 == cur_Kpath_num)
			{
				//先加上i点的Inflow_node
				next_RHS += (cur_RHS + 1.0);
				//系数不能超过2-Conf::MIN_KPATH_THRESHOLD
				if (next_RHS<2 - Conf::MIN_KPATH_THRESHOLD)
				{
					count++;
					//创建新的Kpath
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
		//判断当前cur_Kpath是否已经在Kpath_subset中
		if (false == Cuts_kpath.Already_inKpath(cur_Kpath, cur_Kpath_num))
		{
			bool feasible = true;
			//若是新的Kpath，则判断右侧系数是否为2（TSPTW无解）
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
				//添加cur_Kpath到Cuts_kpath中
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
	//声明一个初始RCC-cut
	int *temp_RCC;
	temp_RCC = new int[Conf::MAX_SUBSET_RCC];
	int temp_RCC_num = 0;
	//声明该初始RCC-cut的包含节点的二进制
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
	//递归终止条件
	//RCC达到Conf::MAX_SUBSET_RCC个点就终止
	//或者已经找到MAX_ADD_RCC个RCC
	if (cur_RCC_num >= Conf::MAX_SUBSET_RCC || Cuts_rcc.added_num >= Conf::MAX_ADD_RCC || cur_RHS>=Conf::MAX_RCC_RHS)
	{
		return;
	}
	int lineid, rowid;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//声明下一个RCC-cut
	int *next_RCC;
	next_RCC = new int[Conf::MAX_SUBSET_RCC];
	for (int i = 0; i < cur_RCC_num; i++)
	{
		next_RCC[i] = cur_RCC[i];
	}
	int next_RCC_num = 0;
	//声明该初始RCC-cut的包含节点的二进制
	long *next_RCC_passnode;
	next_RCC_passnode = new long[p.passnode_length];
	//
	float next_load = 0;
	float next_RHS = 0;
	int count = 0;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < p.Customer_Num; i++)
	{
		//i不属于cur_RCC
		if (false == p.belong_toset(i, cur_RCC_passnode))
		{
			//在cur_RCC与点i间是否存在弧
			next_RHS = 0;
			for (int j = 0; j < cur_RCC_num; j++)
			{
				next_RHS -= (LP.Arc_flow[cur_RCC[j]][i] + LP.Arc_flow[i][cur_RCC[j]]);
			}
			//只寻找支撑图上的弧
			if (next_RHS<-MINDOUBLE || 0 == cur_RCC_num)
			{
				//先加上i点的Inflow_node
				next_RHS += (cur_RHS + 1.0);
				//系数不能超过Conf::MAX_RCC_RHS-Conf::MIN_RCC_THRESHOLD
				if (next_RHS<Conf::MAX_RCC_RHS - Conf::MIN_RCC_THRESHOLD)
				{
					count++;
					//创建新的RCC
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
		//判断当前cur_RCC是否已经在RCC_subset中
		if (false == Cuts_rcc.Already_inRCC(cur_RCC_passnode, p))
		{
			//添加cur_RCC到Cuts_rcc中
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
	//|C|=3时
	if (Conf::MAX_SR_SET>=3)
	{
		temp_RHS = floor(Conf::SR_MULTIPY_3 * 3);
		//遍历所有三元组,注意是组合，而不是排列
		for (int i = 0; i < p.Customer_Num; i++)
		{
			for (int j = i + 1; j < p.Customer_Num; j++)
			{
				for (int k = j + 1; k < p.Customer_Num; k++)
				{
					//检验当前基解
					//得到系数
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
					//违反程度需要>=Conf::MIN_SR_THRESHOLD
					if (temp_vio >= Conf::MIN_SR_THRESHOLD)
					{
						//在violation和subset_route中排序
						//先找到violation插入位置
						insert_posi = Cuts_src.Get_insert_no(temp_vio);
						//如果insert_posi为-1则排不上号
						if (insert_posi >= 0)
						{
							//先生成temp_2D
							Cuts_src.temp_2D[0] = 3;

							Cuts_src.temp_2D[1] = i;
							Cuts_src.temp_2D[2] = j;
							Cuts_src.temp_2D[3] = k;
							//再插入
							Cuts_src.Insert_VioandRoute(insert_posi, temp_vio);
						}
					}
				}
			}
		}
	}

	//|C|=4时
	if (Conf::MAX_SR_SET >= 4)
	{
		temp_RHS = floor(Conf::SR_MULTIPY_4 * 4);
		//遍历所有四元组,但是限制C中任意两点距离不大于MAX_SR_DISTANCE
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
						//以上都是限制|C|=4时，C中任意两点距离不大于MAX_SR_DISTANCE
						//检验当前基解
						//得到系数
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
						//违反程度需要>=Conf::MIN_SR_THRESHOLD
						if (temp_vio >= Conf::MIN_SR_THRESHOLD)
						{
							//在violation和subset_route中排序
							//先找到violation插入位置
							insert_posi = Cuts_src.Get_insert_no(temp_vio);
							//如果insert_posi为-1则排不上号
							if (insert_posi >= 0)
							{
								//先生成temp_2D
								Cuts_src.temp_2D[0] = 4;

								Cuts_src.temp_2D[1] = n_1;
								Cuts_src.temp_2D[2] = n_2;
								Cuts_src.temp_2D[3] = n_3;
								Cuts_src.temp_2D[4] = n_4;
								//再插入
								Cuts_src.Insert_VioandRoute(insert_posi, temp_vio);
							}
						}
					}
				}
			}
		}
	}

	//|C|=5时
	if (Conf::MAX_SR_SET >= 5)
	{
		temp_RHS = floor(Conf::SR_MULTIPY_5 * 5);
		//遍历所有五元组,但是限制C中任意两点距离不大于MAX_SR_DISTANCE
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
							//以上都是限制|C|=5时，C中任意两点距离不大于MAX_SR_DISTANCE
							//检验当前基解
							//得到系数
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
							//违反程度需要>=Conf::MIN_SR_THRESHOLD
							if (temp_vio >= Conf::MIN_SR_THRESHOLD)
							{
								//在violation和subset_route中排序
								//先找到violation插入位置
								insert_posi = Cuts_src.Get_insert_no(temp_vio);
								//如果insert_posi为-1则排不上号
								if (insert_posi >= 0)
								{
									//先生成temp_2D
									Cuts_src.temp_2D[0] = 5;

									Cuts_src.temp_2D[1] = n_1;
									Cuts_src.temp_2D[2] = n_2;
									Cuts_src.temp_2D[3] = n_3;
									Cuts_src.temp_2D[4] = n_4;
									Cuts_src.temp_2D[5] = n_5;
									//再插入
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
		//有问题
		cout << "ERROR: 还没有编呢" << endl;
		cin.get();
	}

	//根据排序结果选择前Cuts_src.violation_num个subset-row进入RMP
	//首先，生成前Cuts_src.violation_num个SRC的SRC_LimitVertexSet_indicator和SRC_LimitArcSet_indicator
	//然后，检验是否存在需要合并的SRC_LimitVertexSet_indicator
	int *augment_Vertex;
	int augment_Vertex_num = 0;
	augment_Vertex = new int[p.Customer_Num];
	float temp_state = 0;

	for (int i = 0; i < Cuts_src.violation_num; i++)
	{
		//先生成SRC_LimitVertexSet_indicator
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
		//再生成SRC_LimitArcSet_indicator
		for (int j = 0; j < p.Depot_Num + p.Customer_Num; j++)
		{	
			for (int k = 0; k < p.Depot_Num + p.Customer_Num; k++)
			{
				Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num][j][k] = 0;
			}
		}
#if LIMITTPYE == 0
		//vertex-memory
		//即从M到M
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
		//除了路径上的弧，额外包括从M到C和从C到M
		//每条路径上的弧
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
		//从M到C和从C到M
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
		//检验检验是否存在需要合并的SRC_LimitVertexSet_indicator和SRC_LimitVertexSet_indicator
		for (int j = 0; j < Cuts_src.processed_num; j++)
		{
			//检验Cuts_src.SRC_subset[j]是否与Cuts_src.SRC_subset[Cuts_src.processed_num + Cuts_src.added_num]完全相同
			if (true== Cuts_src.Check_SameSubset(j, Cuts_src.processed_num + Cuts_src.added_num))
			{
				//合并Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num]到Cuts_src.SRC_LimitVertexSet[j]
				for (int k = 0; k < Cuts_src.SRC_LimitVertexSet_num[Cuts_src.processed_num + Cuts_src.added_num]; k++)
				{
					if (0==Cuts_src.SRC_LimitVertexSet_indicator[j][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k]])
					{
						Cuts_src.SRC_LimitVertexSet[j][Cuts_src.SRC_LimitVertexSet_num[j]] = Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k];
						Cuts_src.SRC_LimitVertexSet_num[j]++;
						Cuts_src.SRC_LimitVertexSet_indicator[j][Cuts_src.SRC_LimitVertexSet[Cuts_src.processed_num + Cuts_src.added_num][k]] = 1;
					}
				}
				//合并Cuts_src.SRC_LimitArcSet_indicator[Cuts_src.processed_num + Cuts_src.added_num]到Cuts_src.SRC_LimitArcSet_indicator[j]
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

