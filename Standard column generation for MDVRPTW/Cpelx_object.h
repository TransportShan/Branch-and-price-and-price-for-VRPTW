#ifndef CPLEX_OBJECT_H_
#define CPLEX_OBJECT_H_

#include <ilcplex/ilocplex.h>
#include "Problem.h"

////����ı���������Ϊȫ�ֱ�������ΪCPLEX�е��಻���ڲ�ͬ��֮������/////////////////
extern IloModel Rmp_model;
extern IloNumVarArray Rmp_routevar_x;
extern IloObjective Rmp_obj;
extern IloRangeArray Rmp_ctall;
extern IloCplex Rmp_cplex;
extern IloCplex::BasisStatusArray var_stat;
extern IloCplex::BasisStatusArray allcon_stat;
////////////////////////////////////////////////////////////////////////////////////

class Cpelx_object
{
public:
	Cpelx_object();
	virtual ~Cpelx_object();

	void ini_parameter(void);
	void ini_cplex(void);
public:
	IloEnv env;	//����һ��CPLEX����
};

#endif