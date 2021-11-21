#include "stdafx.h"
#include "Cpelx_object.h"


////这里的变量必须作为全局变量，因为CPLEX中的类不能在不同类之间生存/////////////////
IloModel Rmp_model;
IloNumVarArray Rmp_routevar_x;
IloObjective Rmp_obj;
IloRangeArray Rmp_ctall;				//我们这里只用一个整体约束，
										//0到p.Customer_Num-1为点的约束，
										//p.Customer_Num到p.Customer_Num+p.Depot_Num-1为场站的约束
										//后面再给SRC或者KPATH cuts
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
	//重新开辟一个CPLEX环境
	env=IloEnv();
}



