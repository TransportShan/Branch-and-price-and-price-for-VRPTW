#ifndef COLUMNPOOL_H_
#define COLUMNPOOL_H_

#include <math.h>

#include "Columns.h"
#include "Problem.h"
#include "BranchABound.h"

using namespace std;

class ColumnPool
{
public:
	ColumnPool();
	virtual ~ColumnPool();

	void Ini(Problem &p);
	void Add_columns(bool calculatecost_or_not, BranchABound & BB, Columns &add_route, Problem &p);	//向列池中添加列，列池不允许在ColumnPool类外进行修改，所有修改列池的操作都要通过函数进行,
																					//参数calculatecost_or_not表示重新计算该路径的实际成本（因为有时候传入的列的cost值不是实际成本，例如RC值）
																					//规定：向列池添加的列一定是类Columns的一个对象（一列）
	bool Objective_function(BranchABound & BB,Columns &route, Problem & p);
	void Add_auxi_columns(BranchABound & BB, Problem &p);
	void Copy_toAuxiCol(Columns &obj_route);

public:
	Columns *Col;					//所有列，预设大小为[Conf::MAX_COLUMN_IN_RMP]
	Columns Auxi_column;			//中间列
	int CandidateColumn_Num;		//当前已经向列池中加入的列的数量
};

#endif