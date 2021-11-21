#include "stdafx.h"
#include "Initialization.h"
#include "Conf.h"

Initialization::Initialization()
{
	//��ʵ�ͻ�
	Ini_column.nodes = new int[Conf::MAX_NODES];
	for (int j = 0; j < Conf::MAX_NODES; j++)
	{
		Ini_column.nodes[j] = -1;
	}
	Ini_column.nodesNum = 0;
}


Initialization::~Initialization()
{
}

void Initialization::Ini_clear(Problem &p)
{
	Ini_column.Depot = -1;
	Ini_column.nodesNum = -1;
	Ini_column.Load = -1;
	Ini_column.Totalcost = -1;
	Ini_column.Reducedcost = -1;
	Ini_column.Duration = -1;

	for (int i = 0; i<p.Customer_Num+ p.Depot_Num; i++)
	{
		Ini_column.Customer_indicator[i] = 0;
	}
	for (int i = 0; i<p.Depot_Num; i++)
	{
		Ini_column.depot_indicator[i] = 0;
	}
}

void Initialization::InitialColumns(const string & file_name, BranchABound &BB, ColumnPool & pool, Problem & p)
{
	//���ٿռ�
	Ini_column.Customer_indicator = new int[p.Customer_Num + p.Depot_Num];
	Ini_column.depot_indicator = new int[p.Depot_Num];

	//���ɳ������ɳ��У�����ʡ�ԣ����ɸı�˳��
	Insert_slack_column_vehicle(BB, pool, p);

	//�������г���Ϊ1��·���������гأ���ʼ���к��ٻ������Щ��·���������гغ��������ٶȣ�
	Insert_simple_route(BB, pool, p);

	//��CSV������һ����ʼ���н�
	Iniroute_ReadfromCSV(file_name, BB, pool, p);

	//����ÿ���ͻ���Ӧ���ɳ��У�����ʡ��
	Insert_slack_column_node(BB, pool, p);
}

void Initialization::Insert_slack_column_vehicle(BranchABound &BB, ColumnPool & pool, Problem & p)
{
	//�Գ�վ�ɳڱ���
	//�ӳ�վֱ�ӻص���վ
	for (int i = 0; i < p.Depot_Num; i++)
	{
		Ini_clear(p);
		Ini_column.Depot = p.Customer_Num+i;
		Ini_column.nodesNum = 2;
		Ini_column.nodes[0] = p.Customer_Num + i;
		Ini_column.nodes[1] = p.Customer_Num + i;

		Ini_column.Travelcost = 0;
		Ini_column.Decaycost = 0;
		Ini_column.Energycost = 0;
		Ini_column.Totalcost = 0;
		Ini_column.Duration = 0;
		Ini_column.Load = 0;

		pool.Add_columns(false, BB, Ini_column, p);
	}
}

void Initialization::Insert_slack_column_node(BranchABound & BB, ColumnPool & pool, Problem & p)
{
	//�Խڵ��ɳڱ���
	for (int i = 0; i<p.Customer_Num; i++)
	{
		Ini_clear(p);
		Ini_column.Depot = -1;

		Ini_column.nodesNum = 3;
		Ini_column.nodes[0] = -1;
		Ini_column.nodes[1] = i;
		Ini_column.nodes[2] = -1;

		Ini_column.Travelcost = 999;
		Ini_column.Decaycost = 0;
		Ini_column.Energycost = 0;
		Ini_column.Totalcost = 999;
		Ini_column.Duration = 999;
		Ini_column.Load = 999;

		pool.Add_columns(false, BB, Ini_column, p);
	}
}

void Initialization::Insert_simple_route(BranchABound &BB, ColumnPool & pool, Problem & p)
{
	//ÿ��-ÿ����վ-ÿ���ͻ�
	for (int i = 0; i < p.Depot_Num; i++)
	{
		for (int j = 0; j<p.Customer_Num; j++)
		{
			Ini_clear(p);
			Ini_column.Depot = p.Customer_Num + i;
			//��ʼ��
			Ini_column.nodes[0] = p.Customer_Num + i;
			Ini_column.nodesNum = 1;
			//�м��
			Ini_column.nodes[1] = j;
			Ini_column.nodesNum = 2;
			//�յ�
			Ini_column.nodes[2] = p.Customer_Num + i;
			Ini_column.nodesNum = 3;

			Ini_column.Travelcost = 0;
			Ini_column.Decaycost = 0;
			Ini_column.Energycost = 0;
			Ini_column.Totalcost = 0;
			Ini_column.Duration = 0;
			Ini_column.Load = 0;

			pool.Add_columns(true, BB, Ini_column, p);
		}
	}
}

void Initialization::Iniroute_ReadfromCSV(const string & file_name, BranchABound &BB, ColumnPool & pool, Problem & p)
{
	string number;
	ifstream fin(file_name.c_str());
	int ini_solution_num = 0;

	if (!fin.is_open())
	{
		cerr << "Can't open" << file_name << "for input.\n";
		cin.get();
	}


	//��ʼ����·������
	getline(fin, number, ',');
	ini_solution_num = atoi(number.c_str());
	getline(fin, number);
	for (int i = 0; i < ini_solution_num; i++)
	{
		Ini_clear(p);
		//��·��������Щ��
		//��һ����Ϊ��վ
		getline(fin, number, ',');
		getline(fin, number, ',');
		Ini_column.Depot = atoi(number.c_str())-1;
		Ini_column.nodesNum = 0;
		Ini_column.nodes[Ini_column.nodesNum] = Ini_column.Depot;
		Ini_column.nodesNum = Ini_column.nodesNum + 1;
		
		int temp_customerid;
		while (1)
		{
			getline(fin, number, ',');
			if (atoi(number.c_str())-1 == Ini_column.Depot)
			{
				getline(fin, number);
				Ini_column.nodes[Ini_column.nodesNum] = Ini_column.Depot;
				Ini_column.nodesNum = Ini_column.nodesNum + 1;
				break;
			}
			else
			{
				temp_customerid = atoi(number.c_str()) - 1;
				Ini_column.nodes[Ini_column.nodesNum] = temp_customerid;
				Ini_column.nodesNum = Ini_column.nodesNum + 1;
			}
		}

		Ini_column.Travelcost = 0;
		Ini_column.Decaycost = 0;
		Ini_column.Energycost = 0;
		Ini_column.Totalcost = 0;
		Ini_column.Duration = 0;
		Ini_column.Load = 0;

		pool.Add_columns(true, BB, Ini_column, p);
#if ADDCOL == 1
		pool.Add_auxi_columns(BB, p);
#endif
	}
	fin.close();

}

