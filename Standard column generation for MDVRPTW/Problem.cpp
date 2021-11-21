#include "stdafx.h"
#include "Problem.h"
#include "Conf.h"

Problem::Problem()
{
	Vehicle_Num = 0;
	Customer_Num = 0;
	Depot_Num = 0;
	ColumnPool_num = 0;
}


Problem::~Problem()
{
}

void Problem::ReadfromCSV(const string & file_name)
{
	string number;
	ifstream fin(file_name.c_str());

	if (!fin.is_open())
	{
		cerr << "Can't open" << file_name << "for input.\n";
		cin.get();
	}

	//��һ�в���
	getline(fin, number);
	getline(fin, number, ',');//�ڶ��е�һ�в���
	//Vehicle_Num
	getline(fin, number, ',');
	Vehicle_Num = atoi(number.c_str());
	//Customer_Num
	getline(fin, number, ',');
	Customer_Num = atoi(number.c_str());
	//Veh.Duration
	getline(fin, number, ',');
	Veh.Duration = atof(number.c_str());
	//Veh.Veh_Capacity
	getline(fin, number, ',');
	Veh.Veh_Capacity = atof(number.c_str());
	//Depot_Num
	getline(fin, number, ',');
	Depot_Num = atoi(number.c_str());

	//��������
	Cus = new Customer[Customer_Num];
	Depot = new Customer[Depot_Num];
	//

	//Cus
	//�����в���
	getline(fin, number);
	getline(fin, number);
	for (int i = 0; i < Customer_Num; i++)
	{
		//node_id
		getline(fin, number, ',');
		Cus[i].node_id = atoi(number.c_str())-1;
		//x
		getline(fin, number, ',');
		Cus[i].x = atof(number.c_str());
		//y
		getline(fin, number, ',');
		Cus[i].y = atof(number.c_str());
		//servicetime
		getline(fin, number, ',');
		Cus[i].servicetime = atof(number.c_str());
		//demand
		getline(fin, number, ',');
		Cus[i].demand = atof(number.c_str());
		//startTW
		getline(fin, number, ',');
		Cus[i].startTW = atof(number.c_str());
		//endTW
		getline(fin, number);
		Cus[i].endTW = atof(number.c_str());
		
	}

	//Depot
	for (int i = 0; i < Depot_Num; i++)
	{
		//node_id
		getline(fin, number, ',');
		Depot[i].node_id = atoi(number.c_str()) - 1;		//depot��Ӧ��node_id��Customer_Num��ʼ
		//x
		getline(fin, number, ',');
		Depot[i].x = atof(number.c_str());
		//y
		getline(fin, number, ',');
		Depot[i].y = atof(number.c_str());
		//servicetime
		getline(fin, number, ',');
		Depot[i].servicetime = atof(number.c_str());
		//demand
		getline(fin, number, ',');
		Depot[i].demand = atof(number.c_str());
		//startTW
		getline(fin, number, ',');
		Depot[i].startTW = atof(number.c_str());
		//endTW
		getline(fin, number);
		Depot[i].endTW = atof(number.c_str());
	}
	fin.close();

	//����Allnode
	Allnode = new Customer[Customer_Num+ Depot_Num];
	for (int i = 0; i < Customer_Num; i++)
	{
		Allnode[i].Copy(Cus[i]);
	}
	for (int i = 0; i < Depot_Num; i++)
	{
		Allnode[i+ Customer_Num].Copy(Depot[i]);
	}
	//����cycle
	cycle_node = new int[Customer_Num +1];
	cycle_node_num = 0;

	Bulid_matrix();
	Bulid_feasibleArc();
	Bulid_ngset();
	Bulid_network();
	Sort_customer();
	Set_dssr_ngset();
}

void Problem::Bulid_matrix(void) 
{
	//����
	Distance_matrix = new float*[Customer_Num + Depot_Num];
	for (int i = 0; i<Customer_Num + Depot_Num; i++)
	{
		Distance_matrix[i] = new float[Customer_Num + Depot_Num];
		for (int j = 0; j < Customer_Num + Depot_Num; j++)
		{
			Distance_matrix[i][j] = 0;
		}
	}
	//
	double distance;
	for (int i = 0; i<Customer_Num + Depot_Num; i++)
	{
		for (int j = i; j < Customer_Num + Depot_Num; j++)
		{
			distance = Calculate_distance_byCustomerId(i, j);
			Distance_matrix[i][j] = Distance_matrix[j][i] = distance;
		}
	}
}

void Problem::Bulid_feasibleArc(void)
{
	for (int i = 0; i < Customer_Num+ Depot_Num; i++)
	{
		Allnode[i].for_feasible_extensions = new int[Customer_Num + Depot_Num];
		Allnode[i].back_feasible_extensions = new int[Customer_Num + Depot_Num];
	}

	//�ȳ�ʼ��
	for (int i = 0; i<Customer_Num + Depot_Num; i++)
	{
		for (int j = 0; j < Customer_Num + Depot_Num; j++)
		{
			Allnode[i].for_feasible_extensions[j] = 0;
			Allnode[i].back_feasible_extensions[j] = 0;
		}
	}

	//�����ٸ���ʱ�䴰�ҵ��ܹ���չ�ĵ㣬��ֵΪ1 
	for (int i = 0; i<Customer_Num + Depot_Num; i++)
	{
		for (int j = 0; j < Customer_Num + Depot_Num; j++)
		{
			if ((Allnode[i].startTW + Allnode[i].servicetime + Calculate_traveltime(i, j))<Allnode[j].endTW)
			{
				Allnode[i].for_feasible_extensions[j] = 1;
			}
		}
	}
	//�����ٸ���ʱ�䴰�ҵ��ܹ���չ�ĵ㣬��ֵΪ1 
	for (int i = 0; i<Customer_Num + Depot_Num; i++)
	{
		for (int j = 0; j < Customer_Num + Depot_Num; j++)
		{
			if ((Allnode[j].startTW + Allnode[j].servicetime + Calculate_traveltime(j, i))<Allnode[i].endTW)
			{
				Allnode[i].back_feasible_extensions[j] = 1;
			}
		}
	}
}

void Problem::Bulid_ngset(void)
{
	//������
	powerlist = new long[Conf::EACH_LONG_NUM];
	powerlist[0] = 1;
	for (int i = 1; i < Conf::EACH_LONG_NUM; i++)
	{
		powerlist[i] = powerlist[i - 1] * 2;
	}
	passnode_length= int((Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1;
	for (int i = 0; i < Customer_Num; i++)
	{
		Allnode[i].ngSet = new int[Customer_Num];
		Allnode[i].negSet_passnode = new long[int((Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		Allnode[i].ngMem_passnode = new long[int((Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];

		Allnode[i].dssr_ngSet = new int[Customer_Num];
		Allnode[i].dssr_negSet_passnode = new long[int((Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		Allnode[i].dssr_ngMem_passnode = new long[int((Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];

		Allnode[i].Ang_ngSet = new int[Customer_Num];
		Allnode[i].Ang_negSet_passnode = new long[int((Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
		Allnode[i].Ang_ngMem_passnode = new long[int((Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1];
	}
	//��ʼ��
	for (int i = 0; i < Customer_Num; i++)
	{
		for (int j = 0; j < Customer_Num; j++)
		{
			Allnode[i].ngSet[j] = -1;
		}
		int tempnum = int((float)(Customer_Num - MINDOUBLE) / (float)Conf::EACH_LONG_NUM) + 1;
		for (int j = 0; j < tempnum; j++)
		{
			Allnode[i].negSet_passnode[j] = 0;
			Allnode[i].ngMem_passnode[j] = 0;
		}
	}
	//��ֵ
	//ȷ��ng�Ĵ�С
	int temp_ngnum;
	if (Conf::MAX_NEIGHBOURHOOD_NUM>Customer_Num)
	{
		temp_ngnum = Customer_Num;
	}
	else
	{
		temp_ngnum = Conf::MAX_NEIGHBOURHOOD_NUM;
	}
	//����Distance_matrix��������
	int i, j, k;
	int temp_posi,ordered_num;	//neighbourhood���Ѿ�����õ�����
	bool check;
	for (i = 0; i < Customer_Num; i++)
	{
		Allnode[i].ngSet[0] = 0;
		ordered_num = 1;
		for (j = 1; j <Customer_Num; j++)
		{
			for (k = ordered_num; k >= 1; k--)
			{
				if (Calculate_travelcost(i,j)<Calculate_travelcost(i, Allnode[i].ngSet[k - 1]))
				{
					continue;
				}
				else
				{
					break;
				}
			}
			//
			temp_posi = k;
			if (temp_posi<temp_ngnum)
			{
				if (ordered_num<temp_ngnum)
				{
					ordered_num = ordered_num + 1;
				}
				//ǰ����������
				for (k = ordered_num - 1; k > temp_posi; k--)
				{
					Allnode[i].ngSet[k] = Allnode[i].ngSet[k - 1];
				}
				//�ٸ�ֵ
				Allnode[i].ngSet[temp_posi] = j;
			}
		}
		Allnode[i].ngSet_num = temp_ngnum;
	}
	//���i�ڵ��Ƿ���i�ڵ�������У�������������ĩβλ��
	for (i = 0; i < Customer_Num; i++)
	{
		check = true;
		for (j = 0; j < temp_ngnum; j++)
		{
			if (i == Allnode[i].ngSet[j])
			{
				check = false;
				break;
			}
		}
		if (true == check)//i�ڵ㲻��i�ڵ��������
		{
			Allnode[i].ngSet[temp_ngnum - 1] = i;
		}
	}
	//תΪ�����ƣ�����neighbourhood_passnode��
	int lineid, rowid, nodeno;
	for (i = 0; i < Customer_Num; i++)
	{
		for (j = 0; j < temp_ngnum; j++)
		{
			nodeno = Allnode[i].ngSet[j];
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			Allnode[i].negSet_passnode[rowid] += powerlist[lineid];
		}

	}
	//ת��neighbourhood�õ�ng_memory_passnode
	for (i = 0; i < Customer_Num; i++)
	{
		for (j = 0; j < temp_ngnum; j++)
		{
			nodeno = i;
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			Allnode[Allnode[i].ngSet[j]].ngMem_passnode[rowid] += powerlist[lineid];
		}
	}
}

void Problem::Bulid_network(void)
{
	Cost_network = new float*[Customer_Num + Depot_Num];
	Time_network = new float*[Customer_Num + Depot_Num];
	for (int j = 0; j < Customer_Num + Depot_Num; j++)
	{
		Cost_network[j] = new float[Customer_Num + Depot_Num];
		Time_network[j] = new float[Customer_Num + Depot_Num];
		for (int k = 0; k < Customer_Num + Depot_Num; k++)
		{
			Cost_network[j][k] = 0;
			Time_network[j][k] = 0;
		}
	}
	
	//��ʼ�����������磺Cost_network��Time_network
	for (int i = 0; i < Customer_Num + Depot_Num; i++)
	{
		for (int j = 0; j < Customer_Num + Depot_Num; j++)
		{
			Cost_network[i][j] = Calculate_travelcost(i, j);
			Time_network[i][j] = Calculate_traveltime(i, j);
		}
	}

	//����ֵMax_arrivaltime
	Max_arrivaltime=new float[Depot_Num];

	for (int i = 0; i < Depot_Num; i++)
	{
		Max_arrivaltime[i] = -1;
		for (int j = 0; j<Customer_Num; j++)
		{
			if ((Allnode[j].endTW + Allnode[j].servicetime + Time_network[j][Customer_Num+i])>Max_arrivaltime[i])
			{
				Max_arrivaltime[i] = Allnode[j].endTW + Allnode[j].servicetime + Time_network[j][Customer_Num + i];
			}
		}
	}
}

void Problem::Sort_customer(void)
{
	int i, j, k;
	int temp_posi,ordered_num;
	int temp_ngnum = Customer_Num;
	//������
	Cus_ascend_startTW = new int[Customer_Num];
	for (i = 0; i < Customer_Num; i++)
	{
		Cus_ascend_startTW[i] = -1;
	}

	//����startTW��������
	Cus_ascend_startTW[0] = 0;
	ordered_num = 1;
	for (j = 1; j <Customer_Num; j++)
	{
		for (k = ordered_num; k >= 1; k--)
		{
			if (Allnode[j].startTW<Allnode[Cus_ascend_startTW[k-1]].startTW	)
			{
				continue;
			}
			else
			{
				break;
			}
		}
		//
		temp_posi = k;
		if (temp_posi<temp_ngnum)
		{
			if (ordered_num<temp_ngnum)
			{
				ordered_num = ordered_num + 1;
			}
			//ǰ����������
			for (k = ordered_num - 1; k > temp_posi; k--)
			{
				Cus_ascend_startTW[k] = Cus_ascend_startTW[k - 1];
			}
			//�ٸ�ֵ
			Cus_ascend_startTW[temp_posi] = j;
		}
	}
}

void Problem::Set_dssr_ngset(void)
{
	int tempnum = int((float)(Customer_Num - MINDOUBLE) / (float)Conf::EACH_LONG_NUM) + 1;

#if INHERIT+Frameworks == 0
	//��ʼ��
	for (int i = 0; i < Customer_Num; i++)
	{
		for (int j = 0; j < Customer_Num; j++)
		{
			Allnode[i].dssr_ngSet[j] = -1;
		}
		for (int j = 0; j < tempnum; j++)
		{
			Allnode[i].dssr_negSet_passnode[j] = 0;
			Allnode[i].dssr_ngMem_passnode[j] = 0;
		}
	}
	for (int i = 0; i < Customer_Num; i++)
	{
		Allnode[i].dssr_ngSet_num = 1;
		Allnode[i].dssr_ngSet[0] = i;
	}

	//תΪ�����ƣ�����neighbourhood_passnode��
	int lineid, rowid, nodeno;
	for (int i = 0; i < Customer_Num; i++)
	{
		for (int j = 0; j < 1; j++)
		{
			nodeno = Allnode[i].dssr_ngSet[j];
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			Allnode[i].dssr_negSet_passnode[rowid] += powerlist[lineid];
		}

	}
	//ת��neighbourhood�õ�ng_memory_passnode
	for (int i = 0; i < Customer_Num; i++)
	{
		for (int j = 0; j < 1; j++)
		{
			nodeno = i;
			rowid = int((nodeno + MINDOUBLE) / Conf::EACH_LONG_NUM);
			lineid = nodeno - rowid*Conf::EACH_LONG_NUM;
			Allnode[Allnode[i].dssr_ngSet[j]].dssr_ngMem_passnode[rowid] += powerlist[lineid];
		}
	}
#else
	//�ȳ�ʼ��dssr_ngSet
	for (int i = 0; i < Customer_Num; i++)
	{
		for (int j = 0; j < Customer_Num; j++)
		{
			Allnode[i].dssr_ngSet[j] = -1;
		}
		for (int j = 0; j < tempnum; j++)
		{
			Allnode[i].dssr_negSet_passnode[j] = 0;
			Allnode[i].dssr_ngMem_passnode[j] = 0;
		}
	}
	//ֱ�ӽ�ngSet���Ƹ�dssr_ngSet
	for (int i = 0; i < Customer_Num; i++)
	{
		Allnode[i].dssr_ngSet_num = 0;
		for (int j = 0; j < Customer_Num; j++)
		{
			if (Allnode[i].ngSet[j] >=0)
			{
				Allnode[i].dssr_ngSet[Allnode[i].dssr_ngSet_num] = Allnode[i].ngSet[j];
				Allnode[i].dssr_ngSet_num = Allnode[i].dssr_ngSet_num + 1;
			}
			else
			{
				break;
			}
		}
		for (int j = 0; j < tempnum; j++)
		{
			Allnode[i].dssr_negSet_passnode[j] = Allnode[i].negSet_passnode[j];
			Allnode[i].dssr_ngMem_passnode[j] = Allnode[i].ngMem_passnode[j];
		}
	}
#endif

#if SDCTYPE == 1
	//�ȳ�ʼ��Ang_ngSet
	for (int i = 0; i < Customer_Num; i++)
	{
		for (int j = 0; j < Customer_Num; j++)
		{
			Allnode[i].Ang_ngSet[j] = -1;
		}
		for (int j = 0; j < tempnum; j++)
		{
			Allnode[i].Ang_negSet_passnode[j] = 0;
			Allnode[i].Ang_ngMem_passnode[j] = 0;
		}
	}
	//ֱ�ӽ�ngSet���Ƹ�Ang_ngSet
	for (int i = 0; i < Customer_Num; i++)
	{
		Allnode[i].Ang_ngSet_num = 0;
		for (int j = 0; j < Customer_Num; j++)
		{
			if (Allnode[i].ngSet[j] >= 0)
			{
				Allnode[i].Ang_ngSet[Allnode[i].Ang_ngSet_num] = Allnode[i].ngSet[j];
				Allnode[i].Ang_ngSet_num = Allnode[i].Ang_ngSet_num + 1;
			}
			else
			{
				break;
			}
		}
		for (int j = 0; j < tempnum; j++)
		{
			Allnode[i].Ang_negSet_passnode[j] = Allnode[i].negSet_passnode[j];
			Allnode[i].Ang_ngMem_passnode[j] = Allnode[i].ngMem_passnode[j];
		}
	}
#endif
}

float Problem::Calculate_distance_byCustomerId(int startid, int endid)
{
	float start_x, start_y, end_x, end_y , d_x, d_y, distance;
	
	if (startid<Customer_Num)
	{
		start_x = Allnode[startid].x;
		start_y = Allnode[startid].y;
	}
	else
	{
		start_x = Depot[startid- Customer_Num].x;
		start_y = Depot[startid - Customer_Num].y;
	}

	if (endid<Customer_Num)
	{
		end_x = Allnode[endid].x;
		end_y = Allnode[endid].y;
	}
	else
	{
		end_x = Depot[endid - Customer_Num].x;
		end_y = Depot[endid - Customer_Num].y;
	}


	d_x = start_x - end_x;
	d_y = start_y - end_y;
	distance = std::sqrt(d_x * d_x + d_y * d_y);
	distance = (int)(distance * 100) / 100.0;

	return distance;
}

float Problem::Calculate_travelcost(int startnode, int endnode)
{
	float travelcost = 0;
	travelcost= Conf::unit_cost*Distance_matrix[startnode][endnode];

	return travelcost;
}

float Problem::Calculate_traveltime(int startnode, int endnode)
{
	float traveltime = 0;
	traveltime = Conf::unit_time*Distance_matrix[startnode][endnode];

	return traveltime;
}

void Problem::Generate_cycle(int path_lenght, int * path, int startposi)
{
	int obj_Node, next_Node;

	obj_Node = path[startposi];
	cycle_node[0] = obj_Node;
	cycle_node_num = 1;

	for (int i = startposi+1; i <path_lenght; i++)
	{
		next_Node = path[i];
		cycle_node[cycle_node_num] = next_Node;
		cycle_node_num = cycle_node_num + 1;
		if (next_Node == obj_Node)
		{
			break;
		}
	}
}

void Problem::Generate_cycle_back(int path_lenght, int * path, int endposi)
{
	int obj_Node, next_Node;

	obj_Node = path[endposi];
	cycle_node[0] = obj_Node;
	cycle_node_num = 1;

	for (int i = endposi - 1; i >= 0; i--)
	{
		next_Node = path[i];
		cycle_node[cycle_node_num] = next_Node;
		cycle_node_num = cycle_node_num + 1;
		if (next_Node == obj_Node)
		{
			break;
		}
	}
}

bool Problem::Add_ngset_byCycle(void)
{
	int add_Node = cycle_node[0];	//��add_Node�����cycle�е�����ngset��
	int check_Node;
	bool valid_ornot = false;

	for (int i = 1; i<cycle_node_num - 1; i++)
	{
		check_Node = cycle_node[i];

		//�����ж�Allnode[check_Node]��dssr_ngSet���Ƿ��Ѿ�����add_Node
		//����Ѿ�������������ӵ�dssr_ngSet
		if (false == belong_toset(add_Node, Allnode[check_Node].dssr_negSet_passnode))
		{
			//�������Allnode[check_Node]��dssr_ngSet�����λ�ü���add_Node
			//dssr_ngSet
			Allnode[check_Node].dssr_ngSet[Allnode[check_Node].dssr_ngSet_num] = add_Node;
			Allnode[check_Node].dssr_ngSet_num = Allnode[check_Node].dssr_ngSet_num + 1;
			//dssr_negSet_passnode
			Insert_toPassnode(add_Node, Allnode[check_Node].dssr_negSet_passnode);
			//dssr_ngMem_passnode
			Insert_toPassnode(check_Node, Allnode[add_Node].dssr_ngMem_passnode);

			valid_ornot = true;
		}
	}

	return valid_ornot;
}

bool Problem::Add_Angngset_byCycle(void)
{
	int add_Node = cycle_node[0];	//��add_Node�����cycle�е�����Ang_ngSet��
	int check_Node;
	bool valid_ornot = false;

	for (int i = 1; i<cycle_node_num - 1; i++)
	{
		check_Node = cycle_node[i];

		//�����ж�Allnode[check_Node]��Ang_ngSet���Ƿ��Ѿ�����add_Node
		//����Ѿ�������������ӵ�Ang_ngSet
		if (false == belong_toset(add_Node, Allnode[check_Node].Ang_negSet_passnode))
		{
			//�������Allnode[check_Node]��Ang_ngSet�����λ�ü���add_Node
			//dssr_ngSet
			Allnode[check_Node].Ang_ngSet[Allnode[check_Node].Ang_ngSet_num] = add_Node;
			Allnode[check_Node].Ang_ngSet_num = Allnode[check_Node].Ang_ngSet_num + 1;
			//Ang_negSet_passnode
			Insert_toPassnode(add_Node, Allnode[check_Node].Ang_negSet_passnode);
			//Ang_ngMem_passnode
			Insert_toPassnode(check_Node, Allnode[add_Node].Ang_ngMem_passnode);

			valid_ornot = true;
		}
	}

	return valid_ornot;
}

bool Problem::belong_toset(int node, long * set)
{
	int lineid, rowid;
	rowid = int((node + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = node - rowid*Conf::EACH_LONG_NUM;
	if (0 == (set[rowid] & powerlist[lineid]))
	{
		//node����set��
		return false;
	}
	else
	{
		//node������set��
		return true;
	}
}

bool Problem::check_samesubset(long * S1, long * S2)
{
	for (int i = 0; i < passnode_length; i++)
	{
		if (S1[i] != S2[i])
		{
			return false;
		}
	}
	return true;
}

void Problem::Insert_toPassnode(int insert_node, long * temp_passnode)
{
	int lineid, rowid;
	rowid = int((insert_node + MINDOUBLE) / Conf::EACH_LONG_NUM);
	lineid = insert_node - rowid*Conf::EACH_LONG_NUM;
	temp_passnode[rowid] += powerlist[lineid];
}

bool Problem::Check_Augng_Feasible(int cyc_posi, Columns obj_col)
{
	//Augng_feasible��������һ��������
	//Augng_infeasible��ȫ������
	bool Feasi_ornot = false;
	int temp_cus = obj_col.nodes[cyc_posi];
	for (int i = cyc_posi - 1; i >= 0; i--)
	{
		if (temp_cus == obj_col.nodes[i])
		{
			break;
		}
		if (false == belong_toset(temp_cus, Allnode[obj_col.nodes[i]].Ang_negSet_passnode))
		{
			Feasi_ornot = true;
			break;
		}
	}

	return Feasi_ornot;
}


