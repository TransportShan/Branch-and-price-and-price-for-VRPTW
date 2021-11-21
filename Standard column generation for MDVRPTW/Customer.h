#ifndef CUSTOMER_H_
#define CUSTOMER_H_

class Customer
{
public:
	Customer();
	virtual ~Customer();

	void Copy(Customer fromCus);

public:
	//��׼����
	float demand;				//����
	float x;					//������
	float y;					//������
	float startTW;				//��ʼʱ�䴰
	float endTW;				//����ʱ�䴰
	float servicetime;			//����ʱ��
	int node_id;				//��ţ���0��ʼ

	//int Nearest_depot_of_node;				//��ʾ������ÿ���ͻ��ڵ�����ĳ�վ
	//int *Nearest_nodes_of_depot;				//һά��̬���飬����һ����վ�����ó�վ������С�Ŀͻ�����
	//int NearestnodesNum_eachdepot;			//һά��̬���飬��ÿһ����վ������С�Ŀͻ�����

	//������չ
	int *for_feasible_extensions;				//��ʾǰ�򣺴Ӹÿͻ������ܹ���Լ���µ��������Ŀͻ��ڵ㣬����ܵ���ýڵ�Ϊ1������Ϊ0����СΪ[VRP.Customer_Num+Depot_Num]
	int *back_feasible_extensions;				//��ʾ���򣺴Ӹÿͻ������ܹ���Լ���µ��������Ŀͻ��ڵ㣬����ܵ���ýڵ�Ϊ1������Ϊ0����СΪ[VRP.Customer_Num+Depot_Num]
	//ng-set,�洢��ʼֵ�����㷨�в����
	int *ngSet;									//�����,�ڵ�i�ǵõ����нڵ㣬��СΪ[Customer_Num]
	int ngSet_num;							//��¼dssr_ngSet���ϵĴ�С
	long *negSet_passnode;						//����ĵ�,�ö����ƴ洢����СΪ[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long *ngMem_passnode;						//����㣬�ǵ�i�ڵ�����нڵ㣬��СΪ[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	//ng-set�ĸ���������ng-set��ʼֵ������dssr���������չ
	int *dssr_ngSet;							//��ӦngSet����СΪ[Customer_Num]
	int dssr_ngSet_num;							//��¼dssr_ngSet���ϵĴ�С
	long *dssr_negSet_passnode;					//��ӦnegSet_passnode����СΪ[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long *dssr_ngMem_passnode;					//��ӦngMem_passnode����СΪ[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	//CPA����£�ng-cycle-SDC��Ҫ����չngset
	int *Ang_ngSet;								//��ӦngSet����СΪ[Customer_Num]
	int Ang_ngSet_num;							//��¼Ang_ngSet���ϵĴ�С
	long *Ang_negSet_passnode;					//��ӦnegSet_passnode����СΪ[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long *Ang_ngMem_passnode;					//��ӦngMem_passnode����СΪ[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
};

#endif