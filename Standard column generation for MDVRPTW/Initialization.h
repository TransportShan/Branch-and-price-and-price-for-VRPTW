#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "ColumnPool.h"
#include "BranchABound.h"

class Initialization
{
public:
	Initialization();
	virtual ~Initialization();

	void Ini_clear(Problem &p);

	void InitialColumns(const string & file_name, BranchABound &BB, ColumnPool &pool, Problem &p);
	void Insert_slack_column_vehicle(BranchABound &BB, ColumnPool &pool, Problem &p); //生成每天的的松弛变量（Day_Num个），并添加到列池中
	void Insert_slack_column_node(BranchABound &BB, ColumnPool &pool, Problem &p); //生成每天的的松弛变量（Day_Num个），并添加到列池中
	void Insert_simple_route(BranchABound &BB, ColumnPool &pool, Problem &p); //生成所有基本路径（只经过一个客户节点的路径），并添加到列池中
	void Iniroute_ReadfromCSV(const string & file_name, BranchABound &BB, ColumnPool &pool, Problem &p);	//从CSV直接读取一个可行的初始解


public:
	Columns Ini_column;
};



#endif
