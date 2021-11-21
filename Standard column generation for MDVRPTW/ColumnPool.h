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
	void Add_columns(bool calculatecost_or_not, BranchABound & BB, Columns &add_route, Problem &p);	//���г�������У��гز�������ColumnPool��������޸ģ������޸��гصĲ�����Ҫͨ����������,
																					//����calculatecost_or_not��ʾ���¼����·����ʵ�ʳɱ�����Ϊ��ʱ������е�costֵ����ʵ�ʳɱ�������RCֵ��
																					//�涨�����г���ӵ���һ������Columns��һ������һ�У�
	bool Objective_function(BranchABound & BB,Columns &route, Problem & p);
	void Add_auxi_columns(BranchABound & BB, Problem &p);
	void Copy_toAuxiCol(Columns &obj_route);

public:
	Columns *Col;					//�����У�Ԥ���СΪ[Conf::MAX_COLUMN_IN_RMP]
	Columns Auxi_column;			//�м���
	int CandidateColumn_Num;		//��ǰ�Ѿ����г��м�����е�����
};

#endif