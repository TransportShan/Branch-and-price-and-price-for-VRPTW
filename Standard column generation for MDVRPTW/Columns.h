#ifndef COLUMNS_H_
#define COLUMNS_H_

#include "Conf.h"

class Columns
{
public:
	Columns();
	virtual ~Columns();

	void Update_cus_cyclePosi(int obj_cus);
public:
	//·����������
	int Depot;							//����·ʼ���ĳ�վ����node_idΪ׼������һ����վΪVRP.Customer_Num
	int nodesNum;						//����·�����ͻ�������
	int *nodes;							//����·�����Ŀͻ����ϣ���СΪ[MAX_NODES]
	float Load;							//����·��װ����
	float Totalcost;					//����·���ܳɱ�=����ɱ�+���ܳɱ�+��Դ�ɱ�
	float Travelcost;					//����·������ɱ������ȣ�
	float Decaycost;					//����·�ĸ��ܳɱ�
	float Energycost;					//����·����Դ�ɱ�
	float Duration;						//��·���ĵ�ǰ�Ѿ��ۻ�������ʱ�䣬=��;����ʱ��+����ʱ�䣬ע�ⲻ�����ȴ�ʱ��
	float Reducedcost;					//����·�ڵ�ǰ���ж�Ӧ��reduced cost

	int *Customer_indicator;			//��·��������Щ�ͻ���������λ��Ϊ1������Ϊ0����СΪ[p.Customer_Num + p.Depot_Num]
										//ע�⣺��·���ǳ���ʱ����ֵ���Դ���1
	int *depot_indicator;				//��·�����ĸ���վ�������Ӹó�վ����Ϊ1������Ϊ0����СΪ[p.Depot_Num]

#if Frameworks == 1
	//��ng-cycle-SDC���
	bool elementary_ornot;				//��·�����Ƿ�Ϊ���ȵģ�trueΪ����·����false��������һ��cycle
	int **cus_cyclePosi;				//��·����ÿ���ͻ��γɵ�ÿ��cycle�����cycle�����ǵ�ǰng-feasible�ģ�ĩβ����·����λ�ã���ʼ��Ϊ0����СΪ[p.Customer_Num]*[int(Conf::MAX_NODES/3)]
	int *cus_cyclePosi_num;				//cus_cyclePosi�Ĵ�С����СΪ[p.Customer_Num]����ʼ��Ϊ0
	int *SDC_cof;						//��·������ÿ���ͻ����Ӧ��SDC�ϵ�ϵ������ʼ��Ϊ0����СΪ[p.Customer_Num]
										//�����ϵ����Ϊ����RMP�з����޸�CPLEX�е�ϵ��
#endif
	//��branch�йص�
	int **pre_arcs;						//��ʾ·����nodes[i]֮ǰ�����ڵ����ż��ϣ���СΪ[p.Customer_Num + p.Depot_Num]*[Conf::MAX_PASS_NUM]���洢������ţ�-1��ʾû�о�����
	int *pre_arcs_Num;					//pre_arcs��ÿ�����ϵļ��ϴ�С����СΪ[p.Customer_Num + p.Depot_Num]����ʼ��Ϊ0
	int **succ_arcs;					//��ʾ·����nodes[i]֮�󾭹��ڵ����ż��ϣ���СΪ[p.Customer_Num + p.Depot_Num]*[Conf::MAX_PASS_NUM]���洢������ţ�-1��ʾû�о�����
	int *succ_arcs_Num;					//succ_arcs��ÿ�����ϵļ��ϴ�С����СΪ[p.Customer_Num + p.Depot_Num]����ʼ��Ϊ0

	//��RMP�йصģ����࣬��Ϊ����RMP�ᶨ����������
	float solution_value;				//��·����RMP���Ž��ж�Ӧ��ֵ
	int baseornot;						//�Ƿ�Ϊ���⣬Ϊ����Ϊ1������Ϊ0
	int posi_baseornot;					//�Ƿ�Ϊ������,Ϊ������Ϊ1������Ϊ0
	int compatibleornot;				//�Ƿ��뵱ǰ��(��)��S���ݣ����Ƿ����ڼ���Cs.=1����ݣ�����Cs��=0�򲻼��ݣ�����Is��
	float incompatibility;				//���������(compatibleornot==0),��ʾ�뵱ǰ��(��)��S�Ĳ����ݶȡ���incompatibility==0ʱ����ʾ���ݡ�

	//PSP�и���
	int pre_label;
	int succ_label;
};

#endif
