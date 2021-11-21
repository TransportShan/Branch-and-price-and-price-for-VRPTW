#ifndef CPLEX_OBJECT_H_
#define CPLEX_OBJECT_H_

#include <ilcplex/ilocplex.h>
#include "Problem.h"

////这里的变量必须作为全局变量，因为CPLEX中的类不能在不同类之间生存/////////////////
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
	IloEnv env;	//开辟一个CPLEX环境
};

#endif