#ifndef CUT_H_
#define CUT_H_

#include <algorithm>

#include "Problem.h"
#include "Utility.h"

class Repetition	//�м��������ǰRMP����ÿ����ľ���������������RMP�����ҵ��µ�SDC
{
public:
	Repetition();
	virtual ~Repetition();

public:
	int Customer_id;		//�ͻ�������
	float Indicator;		//ÿ���ͻ��������Ĵ���
	int Prohibit_ornot;		//=1��ʾ�����ڸõ��ϼ���SDC��=0��ʾ
};

class SDC
{
public:
	SDC();
	virtual ~SDC();

	void Ini_SDC(Problem &p);									//��ʼ��SDC
	int Find_SDC(Utility & Results, Problem &p);				//��Rep_customers����Indicator�����������ҵ����ظ�������������customer
	void Reset_RepCus(Problem &p);								//����Rep_customers
	void Cpoy_Rep_customers(Problem &p);						//��Rep_customers_copy��ֵ
public:
	//��¼�Ѿ����ɵ�SDC
	int *SDC_nodes;							//��¼Ӧ��SDC��node���������ϣ���СΪ[p.Customer_Num]
	int *SDC_indicator;						//����ÿͻ���Ӧ��SDC����Ϊ1������Ϊ0����СΪ[p.Customer_Num]
	int processed_num;						//SDC_nodes���Ѿ���������д��Լ����������Ҳ���Կ���SDC_nodes����Ч��С
	int added_num;							//�¼��룬�������Լ��������
											//��¼ÿ��SDC��ӦCPLEX������Rmp_ctall�����
	int *SDC_cons_index;					//ÿ��SDC_nodes��Ӧ��SDCcuts��Rmp_ctall�е���ţ���СΪ[p.Customer_Num]

	Repetition *Rep_customers;				//�����洢RMP��ǰ��ÿ���ͻ��������Ĵ�������СΪ[p.Customer_Num]
	Repetition *Rep_customers_copy;			//Rep_customers�ĸ�������CustomerToSDC==1ʱ����

											//ng-cycle-SDC���
	bool ngset_change;						//����RMP����LP�⣬�Ƿ��нڵ��ngset����չ������չΪtrue������Ϊfalse����ʼ��Ϊfalse
	int *ngset_addNum;						//CPA����£�һ�����Ż���ÿ���ڵ��ngset�Ƿ���չ������չΪ1������Ϊ0����ʼ��Ϊ0����СΪ[p.Customer_Num]
};


class KPATH
{
public:
	KPATH();
	virtual ~KPATH();

	void Ini_KPATH(Problem &p);										//��ʼ��KPATH
	bool Already_inKpath(int *obj_subset, int obj_subset_num);		//�ж�obj_subset�Ƿ��뵱ǰ�¼��루���µ�added_num������Kpath�غ�
	void Add_Kpath(int *obj_subset, int obj_subset_num);			//��obj_subset���뵱ǰKPATH��
public:
	//��¼�Ѿ����ɵ�Kpath
	int **Kpath_subset;						//��¼ÿ��Kpath��Ч����ʽ��Ӧ�Ľڵ㼯�ϣ���СΪ[Conf::MAX_KPATH_NUM]*[Conf::MAX_SUBSET_KPATH]
	int *Kpath_subset_len;					//��¼ÿ��Kpath��Ч����ʽ��Ӧ�Ľڵ㼯�ϵĴ�С����СΪ[Conf::MAX_KPATH_NUM]
	int **Kpath_indicator;					//���һ��Kpath_subset�����ÿͻ�����Ϊ1������Ϊ0����СΪ[Conf::MAX_KPATH_NUM]*[p.Depot_num+p.Customer_Num]
	int processed_num;						//Kpath_subset���Ѿ���������д��Լ��������
	int added_num;							//�¼��룬�������Լ��������
	//��¼ÿ��Kpath��ӦCPLEX������Rmp_ctall�����
	int *Kpath_cons_index;					//ÿ��Kpath_subset��Ӧ��Kpath��Rmp_ctall�е���ţ���СΪ[Conf::MAX_KPATH_NUM]
};

class RCC
{
public:
	RCC();
	virtual ~RCC();

	void Ini_RCC(Problem &p);																							//��ʼ��RCC
	bool Already_inRCC(long* obj_subset_passnode, Problem &p);															//�ж�obj_subset�Ƿ��뵱ǰ�¼��루���µ�added_num������RCC�غ�
	void Add_RCC(int obj_RHS, int *obj_subset, int obj_subset_num, long* obj_subset_passnode, Problem &p);				//��obj_subset���뵱ǰRCC��
public:
	//��¼�Ѿ����ɵ�Kpath
	int **RCC_subset;						//��¼ÿ��RCC��Ч����ʽ��Ӧ�Ľڵ㼯�ϣ���СΪ[Conf::MAX_RCC_NUM]*[Conf::MAX_SUBSET_RCC]
	int *RCC_subset_len;					//��¼ÿ��RCC��Ч����ʽ��Ӧ�Ľڵ㼯�ϵĴ�С����СΪ[Conf::MAX_RCC_NUM]
	int *RCC_RHS;							//��¼ÿ��RCC��Ч����ʽ���Ҳೣ������СΪ[Conf::MAX_RCC_NUM]
	long **RCC_subset_passnode;				//��¼RCC_subset�Ķ����ƣ���СΪ[Conf::MAX_RCC_NUM]*[int((p.Customer_Num - MINDOUBLE) / Conf::EACH_LONG_NUM) + 1]
	int **RCC_indicator;					//���һ��RCC_subset�����ÿͻ�����Ϊ1������Ϊ0����СΪ[Conf::MAX_RCC_NUM]*[p.Depot_num+p.Customer_Num]
	int processed_num;						//RCC_subset���Ѿ���������д��Լ��������
	int added_num;							//�¼��룬�������Լ��������
											//��¼ÿ��RCC��ӦCPLEX������Rmp_ctall�����
	int *RCC_cons_index;					//ÿ��RCC_subset��Ӧ��RCC_subset��Rmp_ctall�е���ţ���СΪ[Conf::MAX_RCC_NUM]
};

//subset-row cuts
class SRC
{
public:
	SRC();
	virtual ~SRC();

	void Ini_SRC(Problem &p);																							//��ʼ��SRC
	float add_state(int SRC_length);																					//���ݵ�SRC�ļ���C�Ĵ�С��ȷ��ÿ��state����������
	int Get_RHS(int SRC_length);																						//���ݵ�SRC�ļ���C�Ĵ�С������SRC�Ҳ�ϵ��
	int Get_insert_no(float cur_violation);																				//��ȡviolation�еĲ���λ��
	void Insert_VioandRoute(int posi,float cur_violation);																//��cur_violation��temp_2D�ֱ����violation��subset_route�ĵ�posi��λ��
	bool Check_SameSubset(int S1, int S2);																				//���SRC_subset[S1]��SRC_subset[S1]��ͬ�򷵻�true�����򷵻�false
public:
	//��¼�Ѿ����ɵ�SRC
	int **SRC_subset;						//��¼ÿ��SRC��Ч����ʽ�нڵ㼯��C����СΪ[Conf::MAX_SR_NUM]*[Conf::MAX_SR_SET]
	int *SRC_subset_len;					//��¼ÿ��SRC��Ч����ʽ�м���C�Ĵ�С����СΪ[Conf::MAX_SR_NUM]
	int *SRC_RHS;							//��¼ÿ��SRC��Ч����ʽ���Ҳೣ������СΪ[Conf::MAX_SR_NUM]
	int **SRC_subset_indicator;				//���һ��SRC_subset�����ÿͻ�����Ϊ1������Ϊ0����СΪ[Conf::MAX_SR_NUM]*[p.Depot_num+p.Customer_Num]

	int **SRC_LimitVertexSet;				//��¼ÿ��SRC��Ч����ʽ��limit_memory�нڵ�ļ��ϣ���СΪ[Conf::MAX_SR_NUM]*[p.Customer_Num]
	int *SRC_LimitVertexSet_num;			//��¼ÿ��SRC��Ч����ʽ��limit_memory�нڵ����������СΪ[Conf::MAX_SR_NUM]
	int **SRC_LimitVertexSet_indicator;		//��¼ÿ��SRC��Ч����ʽ��limit_memory�нڵ�ļ��ϣ���������õ���Ϊ1������Ϊ0����СΪ[Conf::MAX_SR_NUM]*[p.Depot_num+p.Customer_Num]

	int ***SRC_LimitArcSet_indicator;		//��¼ÿ��SRC��Ч����ʽ��limit_memory�л��ļ��ϣ���������û���Ϊ1������Ϊ0����СΪ[Conf::MAX_SR_NUM]*[p.Depot_num+p.Customer_Num]*[p.Depot_num+p.Customer_Num]

	int processed_num;						//SRC_subset���Ѿ���������д��Լ��������
	int added_num;							//�¼��룬�������Լ��������
	//��¼ÿ��SRC��ӦCPLEX������Rmp_ctall�����
	int *SRC_cons_index;					//ÿ��SRC��Ӧ��SRC_subset��Rmp_ctall�е���ţ���СΪ[Conf::MAX_SR_NUM]
	//����separation algorithm�ı���
	vector<float> violation;				//��������м�¼ǰConf::MAX_ADD_SR��Υ��Լ���̶�����SRC�ĳͷ�ֵ����СΪ[Conf::MAX_ADD_SR+1]
	vector<vector<int>> subset_route;		//��������м�¼ǰConf::MAX_ADD_SR��Υ��Լ���̶�����SRC��subset����Щ·������СΪ[Conf::MAX_ADD_SR+1]*[Conf::MAX_SR_SET+100]
											//��0��Ϊsubset����;��1-Conf::MAX_SR_SET��Ϊsubset�Ľڵ����;��Conf::MAX_SR_SET+1��Ϊ�ü���C����·���ĸ���;��Conf::MAX_SR_SET+2-Conf::MAX_SR_SET+99Ϊ������·�������
	vector<int> temp_2D;					//��subset_route����һ�еĹ����У���ʱ�洢�����￪����Ϊ�˼��ٺ����з�����̬���٣���СΪ[Conf::MAX_SR_SET+100]
	int violation_num;						//��������У��ҵ���SRC����
};


#endif