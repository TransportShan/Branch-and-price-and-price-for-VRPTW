#include "stdafx.h"
#include "ColumnPool.h"

ColumnPool::ColumnPool()
{
}

ColumnPool::~ColumnPool()
{
}

void ColumnPool::Ini(Problem & p)
{
	Col = new Columns[Conf::MAX_COLUMN_IN_RMP];
	CandidateColumn_Num = 0;

	for (int i = 0; i<Conf::MAX_COLUMN_IN_RMP; i++)
	{
		//��ʵ�ͻ�
		Col[i].nodes = new int[Conf::MAX_NODES];
		for (int j = 0; j < Conf::MAX_NODES; j++)
		{
			Col[i].nodes[j] = -1;
		}
		Col[i].nodesNum = 0;
		//
		Col[i].Customer_indicator = new int[p.Customer_Num + p.Depot_Num];
		Col[i].depot_indicator = new int[p.Depot_Num];
		for (int j = 0; j<(p.Customer_Num + p.Depot_Num); j++)
		{
			Col[i].Customer_indicator[j] = 0;
		}
		for (int j = 0; j<p.Depot_Num; j++)
		{
			Col[i].depot_indicator[j] = 0;
		}
#if Frameworks == 1
		//��ng-cycle-SDC��ص�
		Col[i].cus_cyclePosi = new int*[p.Customer_Num];
		Col[i].cus_cyclePosi_num = new int[p.Customer_Num];
		Col[i].SDC_cof = new int[p.Customer_Num];
		for (int j = 0; j < p.Customer_Num; j++)
		{
			Col[i].cus_cyclePosi[j] = new int[int(Conf::MAX_NODES / 3)];
			for (int k = 0; k < int(Conf::MAX_NODES / 3); k++)
			{
				Col[i].cus_cyclePosi[j][k] = 0;
			}
			Col[i].cus_cyclePosi_num[j] = 0;
			Col[i].SDC_cof[j] = 0;
		}
		Col[i].elementary_ornot = true;
#endif
		//branch�йص�
		Col[i].pre_arcs = new int*[p.Customer_Num + p.Depot_Num];
		Col[i].succ_arcs = new int*[p.Customer_Num + p.Depot_Num];
		Col[i].pre_arcs_Num = new int[p.Customer_Num + p.Depot_Num];
		Col[i].succ_arcs_Num = new int[p.Customer_Num + p.Depot_Num];
		for (int j = 0; j<(p.Customer_Num + p.Depot_Num); j++)
		{
			Col[i].pre_arcs[j] = new int[Conf::MAX_PASS_NUM];
			Col[i].succ_arcs[j] = new int[Conf::MAX_PASS_NUM];
			Col[i].pre_arcs_Num[j] = 0;
			Col[i].succ_arcs_Num[j] = 0;
		}
		for (int j = 0; j<(p.Customer_Num + p.Depot_Num); j++)
		{
			for (int k = 0; k < Conf::MAX_PASS_NUM; k++)
			{
				Col[i].pre_arcs[j][k] = -1;
				Col[i].succ_arcs[j][k] = -1;
			}
		}
	}
	//�м���
	Auxi_column.nodes = new int[Conf::MAX_NODES];
	for (int j = 0; j < Conf::MAX_NODES; j++)
	{
		Auxi_column.nodes[j] = -1;
	}
	Auxi_column.nodesNum = 0;
}



void ColumnPool::Add_columns(bool calculatecost_or_not, BranchABound & BB, Columns & add_route, Problem & p)
{
	if (0 == add_route.nodesNum)
	{
		cout << "ERROR: ��ӵ���û�о����κνڵ�" << endl;
		return;
	}

	if (CandidateColumn_Num<Conf::MAX_COLUMN_IN_RMP) //�����·�����Բ�����ֱ�Ӽ���
	{
		if (CandidateColumn_Num + 1<Conf::MAX_COLUMN_IN_RMP)
		{
			if (true == calculatecost_or_not)//�����Ҫ���¼����·���ĳɱ�
			{
				//�ȼ����·����Ӧ�ĳɱ�
				if (false==Objective_function(BB, add_route,p))
				{
					cout <<"ERRPR: �ɱ�����" << endl;
					cin.get();
				}
			}
			//�޸��г�routeset
			if (add_route.Depot<0)	//������Խڵ��ɳڱ���
			{
				Col[CandidateColumn_Num].Depot = -1;
				Col[CandidateColumn_Num].nodesNum = 3;

				Col[CandidateColumn_Num].nodes[0] = -1;
				Col[CandidateColumn_Num].nodes[1] = add_route.nodes[1];
				Col[CandidateColumn_Num].nodes[2] = -1;
				Col[CandidateColumn_Num].Customer_indicator[add_route.nodes[1]] = 1;

				for (int i = 0; i < p.Depot_Num; i++)
				{
					Col[CandidateColumn_Num].depot_indicator[i] = 0;
				}
			}
			else
			{
				//�޸����ڵĳ�վ
				Col[CandidateColumn_Num].Depot = add_route.nodes[0];
				//�޸�·������
				Col[CandidateColumn_Num].nodesNum = add_route.nodesNum;
				//�޸�·��
				for (int j = 0; j<add_route.nodesNum; j++)
				{
					Col[CandidateColumn_Num].nodes[j] = add_route.nodes[j];
				}
				//���λ��ÿ�����Ƿ񱻾�������ǰ��ڵ��Ƕ��٣�����ڵ��Ƕ���
				Col[CandidateColumn_Num].Customer_indicator[add_route.nodes[0]] = 1;
				Col[CandidateColumn_Num].succ_arcs[add_route.nodes[0]][Col[CandidateColumn_Num].succ_arcs_Num[add_route.nodes[0]]]= add_route.nodes[1];
				Col[CandidateColumn_Num].succ_arcs_Num[add_route.nodes[0]] = Col[CandidateColumn_Num].succ_arcs_Num[add_route.nodes[0]] + 1;
				for (int j = 1; j<add_route.nodesNum - 1; j++)
				{
					Col[CandidateColumn_Num].Customer_indicator[add_route.nodes[j]] = Col[CandidateColumn_Num].Customer_indicator[add_route.nodes[j]]+1;
					Col[CandidateColumn_Num].succ_arcs[add_route.nodes[j]][Col[CandidateColumn_Num].succ_arcs_Num[add_route.nodes[j]]] = add_route.nodes[j + 1];
					Col[CandidateColumn_Num].succ_arcs_Num[add_route.nodes[j]] = Col[CandidateColumn_Num].succ_arcs_Num[add_route.nodes[j]] + 1;
					Col[CandidateColumn_Num].pre_arcs[add_route.nodes[j]][Col[CandidateColumn_Num].pre_arcs_Num[add_route.nodes[j]]] = add_route.nodes[j - 1];
					Col[CandidateColumn_Num].pre_arcs_Num[add_route.nodes[j]] = Col[CandidateColumn_Num].pre_arcs_Num[add_route.nodes[j]] + 1;
#if Frameworks == 1
					//�ҵ�cycle���׽ڵ��λ��
					if (Col[CandidateColumn_Num].Customer_indicator[add_route.nodes[j]] >= 2)
					{
						Col[CandidateColumn_Num].cus_cyclePosi[add_route.nodes[j]][Col[CandidateColumn_Num].cus_cyclePosi_num[add_route.nodes[j]]] = j;
						Col[CandidateColumn_Num].cus_cyclePosi_num[add_route.nodes[j]] = Col[CandidateColumn_Num].cus_cyclePosi_num[add_route.nodes[j]] + 1;
						//һ���ǳ���
						Col[CandidateColumn_Num].elementary_ornot = false;
					}
#endif
				}
				Col[CandidateColumn_Num].pre_arcs[add_route.nodes[add_route.nodesNum - 1]][Col[CandidateColumn_Num].pre_arcs_Num[add_route.nodes[add_route.nodesNum - 1]]]= add_route.nodes[add_route.nodesNum - 2];
				Col[CandidateColumn_Num].pre_arcs_Num[add_route.nodes[add_route.nodesNum - 1]] = Col[CandidateColumn_Num].pre_arcs_Num[add_route.nodes[add_route.nodesNum - 1]] + 1;
				//�ĸ���վ
				Col[CandidateColumn_Num].depot_indicator[Col[CandidateColumn_Num].Depot-p.Customer_Num] = 1;
			}
			//��δ��ݳɱ�
			Col[CandidateColumn_Num].Duration = add_route.Duration;
			Col[CandidateColumn_Num].Load = add_route.Load;
			if (0 == Conf::cost_type)
			{
				Col[CandidateColumn_Num].Travelcost = add_route.Travelcost;
				Col[CandidateColumn_Num].Totalcost = Col[CandidateColumn_Num].Travelcost;
			}
			else
			{
				Col[CandidateColumn_Num].Travelcost = add_route.Travelcost;
				Col[CandidateColumn_Num].Decaycost = add_route.Decaycost;
				Col[CandidateColumn_Num].Energycost = add_route.Energycost;
				Col[CandidateColumn_Num].Totalcost = Col[CandidateColumn_Num].Travelcost+ Col[CandidateColumn_Num].Decaycost + Col[CandidateColumn_Num].Energycost;
			}
			BB.branch[BB.cur_node].used_column[BB.branch[BB.cur_node].used_column_num] = CandidateColumn_Num;
			BB.branch[BB.cur_node].used_column_num = BB.branch[BB.cur_node].used_column_num + 1;
			CandidateColumn_Num = CandidateColumn_Num + 1;
		}
		else //�״δﵽMAX_COLUMN_IN_RMP��Ҫȫ����һ��
		{
			CandidateColumn_Num = CandidateColumn_Num + 1;
			//������
			//�����������ܣ���ɾ�������������
			cout << "�����г����� " << endl;
			cin.get();
		}
	}
	else //�ﵽMAX_COLUMN_IN_RMP֮�󣬼����·����Ҫ���ս�������Ȼ���ٴ������ϼ���Column_pool
	{
		CandidateColumn_Num = CandidateColumn_Num;
		//�ȶ�Add_route���н�������

		//�Ӻ����������滻routeset�е�·��
		//�����������ܣ���ɾ�������������
		cout << "�����г����� " << endl;
		cin.get();
	}
}


bool ColumnPool::Objective_function(BranchABound & BB, Columns & route, Problem & p)
{
	float temp_Totalcost = 0, temp_Travelcost = 0, temp_Decaycost = 0, temp_Energycost = 0, temp_Duration = 0, temp_Load = 0, temp_arrivalTime = 0;

	if (route.Depot>=0)
	{
		//һ������ĳɱ���ʱ�䴰Լ��
		for (int i = 0; i<route.nodesNum - 1; i++)
		{
			temp_arrivalTime = fmax(temp_arrivalTime+ p.Allnode[route.nodes[i]].servicetime +BB.branch[BB.cur_node].CostNetwork_Branch[route.nodes[i]][route.nodes[i + 1]], p.Allnode[route.nodes[i+1]].startTW);
			if (temp_arrivalTime>p.Allnode[route.nodes[i+1]].endTW+ MINDOUBLE)
			{
				cout << "ERROR:����в�����ʱ�䴰" << endl;
				cin.get();
			}
			temp_Travelcost = temp_Travelcost + BB.branch[BB.cur_node].CostNetwork_Branch[route.nodes[i]][route.nodes[i + 1]];
			temp_Duration = temp_Duration + BB.branch[BB.cur_node].CostNetwork_Branch[route.nodes[i]][route.nodes[i + 1]]+ p.Allnode[route.nodes[i]].servicetime;
			temp_Load = temp_Load + p.Allnode[route.nodes[i]].demand;
		}
		//���Ǹ��ܳɱ�
		if (1 == Conf::cost_type)
		{
			float temp_decay = 0, temp_energy = 0, temp_TolST = 0, temp_TolTT = 0, temp_lamda = 1,temp_AT=0, temp_deliveryTime;
			for (int i = 0; i < route.nodesNum - 1; i++)
			{
				temp_deliveryTime = fmax(temp_AT + p.Allnode[route.nodes[i]].servicetime + BB.branch[BB.cur_node].CostNetwork_Branch[route.nodes[i]][route.nodes[i + 1]], p.Allnode[route.nodes[i+1]].startTW)- temp_AT - p.Allnode[route.nodes[i]].servicetime;
				temp_AT = temp_AT + p.Allnode[route.nodes[i]].servicetime + temp_deliveryTime;
				temp_TolST = temp_TolST + p.Allnode[route.nodes[i+1]].servicetime;
				temp_TolTT = temp_TolTT + temp_deliveryTime;

				temp_lamda = temp_lamda*exp(-Conf::decay_rate_travel*temp_deliveryTime -Conf::decay_rate_load*p.Allnode[route.nodes[i+1]].servicetime);
				temp_decay = temp_decay + p.Allnode[route.nodes[i]].demand/ temp_lamda;
			}
			temp_energy = Conf::thermal_load_travel*temp_TolTT + Conf::thermal_load_load*temp_TolST;

			temp_Decaycost = Conf::unit_cargoCost*temp_decay;
			temp_Energycost= Conf::unit_kcalCost*temp_energy;
			temp_Load = temp_Load + temp_decay;
		}

	}

	route.Travelcost = temp_Travelcost;
	route.Decaycost = temp_Decaycost;
	route.Energycost = temp_Energycost;
	route.Totalcost = temp_Travelcost+ temp_Decaycost+ temp_Energycost;
	route.Duration = temp_Duration;
	route.Load = temp_Load;

	if (route.Load>p.Veh.Veh_Capacity+ MINDOUBLE)
	{
		cout << "ERROR:����в���������" << endl;
		cin.get();
	}
#if DURATIONORNOT == 1
	if (route.Duration>p.Veh.Duration+ MINDOUBLE)
	{
		cout << "ERROR:����в�����duration" << endl;
		cin.get();
	}
#endif

	if (route.Totalcost > MAXNUM - 1)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void ColumnPool::Add_auxi_columns(BranchABound & BB, Problem & p)
{
	int obj_depot = Col[CandidateColumn_Num - 1].Depot;
	for (int i = p.Customer_Num; i<p.Customer_Num+p.Depot_Num; i++)
	{
		if (i != obj_depot)
		{
			Copy_toAuxiCol(Col[CandidateColumn_Num - 1]);
			Auxi_column.Depot = i;
			//�����Ƿ����
			if (true==Objective_function(BB, Auxi_column, p))
			{
				Add_columns(false, BB, Auxi_column, p);
			}
		}
	}
}

void ColumnPool::Copy_toAuxiCol(Columns & obj_route)
{
	Auxi_column.nodesNum = obj_route.nodesNum;
	for (int i = 0; i<obj_route.nodesNum; i++)
	{
		Auxi_column.nodes[i] = obj_route.nodes[i];
	}
}


