#include "stdafx.h"
#include "Cpelx_object.h"


////����ı���������Ϊȫ�ֱ�������ΪCPLEX�е��಻���ڲ�ͬ��֮������/////////////////
IloModel Rmp_model;
IloNumVarArray Rmp_routevar_x;
IloObjective Rmp_obj;
IloRangeArray Rmp_ctall;				//��������ֻ��һ������Լ����
										//0��p.Customer_Num-1Ϊ���Լ����
										//p.Customer_Num��p.Customer_Num+p.Depot_Num-1Ϊ��վ��Լ��
										//�����ٸ�SRC����KPATH cuts
IloCplex Rmp_cplex;
IloCplex::BasisStatusArray var_stat;
IloCplex::BasisStatusArray allcon_stat;
////////////////////////////////////////////////////////////////////////////////////



Cpelx_object::Cpelx_object()
{
}

Cpelx_object::~Cpelx_object()
{
}

void Cpelx_object::ini_parameter(void)
{
	Rmp_model =IloModel(env);
	Rmp_routevar_x =IloNumVarArray(env);
	Rmp_obj =IloObjective(env);
	Rmp_ctall = IloRangeArray(env);
	var_stat=IloCplex::BasisStatusArray(env);
	allcon_stat =IloCplex::BasisStatusArray(env);
	Rmp_cplex = IloCplex(Rmp_model);
}

void Cpelx_object::ini_cplex(void)
{
	//���¿���һ��CPLEX����
	env=IloEnv();
}



