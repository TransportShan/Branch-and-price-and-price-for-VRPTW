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
	void Insert_slack_column_vehicle(BranchABound &BB, ColumnPool &pool, Problem &p); //����ÿ��ĵ��ɳڱ�����Day_Num����������ӵ��г���
	void Insert_slack_column_node(BranchABound &BB, ColumnPool &pool, Problem &p); //����ÿ��ĵ��ɳڱ�����Day_Num����������ӵ��г���
	void Insert_simple_route(BranchABound &BB, ColumnPool &pool, Problem &p); //�������л���·����ֻ����һ���ͻ��ڵ��·����������ӵ��г���
	void Iniroute_ReadfromCSV(const string & file_name, BranchABound &BB, ColumnPool &pool, Problem &p);	//��CSVֱ�Ӷ�ȡһ�����еĳ�ʼ��


public:
	Columns Ini_column;
};



#endif
