#ifndef BRANCHABOUND_H_
#define BRANCHABOUND_H_

#include "Problem.h"
#include "Solution.h"

using namespace std;


//�����������������������
//���ö�̬�ڴ�������
//ÿ��һ��BranchNodeǰ���ȿ��ٶ�̬���飻ÿ�ζ�BranchNode�����������ӽڵ��֧���ͷŶ�̬����
class NewConstraint
{
public:
	NewConstraint();
	virtual ~NewConstraint();

public:
	int type;									//��ʼ��Ϊ-1��=1��ʾ�뻡���
	//��һ�ַ�֧����
	int startnode;								//��֧�������
	int endnode;								//��֧�������
	bool arc_state;								//��ʾ����ĳ����ĳ���ͻ��ķ�֧״̬��
												//=true��ʾ��(startnode,endnode)��branch_day��һ����������=false��ʾ��(startnode,endnode)��branch_day��һ������������
};


class BranchNode
{
public:
	BranchNode();
	virtual ~BranchNode();

	void Copy_Constraints(BranchNode preNode);

public:
	//�ڵ���CG�������״̬
	int node_id;								//�ڵ�ı�ţ�0��Ӧ���ڵ�

	int depth;									//�÷�֧�ڵ��ϵķ�֧��ȣ���0��ʼ
	int branch_state;							//�÷�֧�ڵ��ϵ�״̬��
												//��ʼ��Ϊ-1��=0ʱ��ʾδ���LP��=1ʱ��ʾ�����δ��֧��=2ʱ��ʾ������ҷ�֧
	int pre_nodeid;								//ĸ�ڵ��node_id
	int Son_node[2];

	bool branch_ornot;							//=true��ʾ�Ѿ���֧������=false��ʾ��δ��֧
	int branched_num;							//�Ѿ������֧����������ʼ��Ϊ0��һ���ﵽ2�����ͷŸõ��ϵĶ�̬����
	int state;									//��node����CG����������������ʼ��Ϊ-1��ʾ��û�����,0��ʾ�޽�(��֦)��1��ʾ������(����)��2��ʾʵ����(��֧)
	float LB_value;								//��ʾ�ýڵ���CG�������Ŀ�꺯��ֵ
	float UB_value;								//��ʾCG�������Ŀ�꺯��ֵ

	int *used_column;							//��node�£�CG����ʱ�����ϸ�nodeԼ���������г��е������������СΪ[MAX_COLUMN_IN_RMP]
	int used_column_num;						//��node�£�CG����ʱ��used_column������
	int total_column_num;						//��node�£�CG����ʱ�г�����

	int *LBsol_index;							//��node�£�CG����ʱRMP���Ž���>0�ı������г��е�λ�ã���СΪ[2*Customer_Num]
	float *LBsol_value;							//��node�£�CG����ʱRMP���Ž���>0�ı�����ֵ����СΪ[2*Customer_Num]
	int LBsol_num;								//��node�£�CG����ʱRMP���Ž���>0�ı����ĸ���

	//�÷�֧�ϵ�Ҫ��
	//�복������ص�
	int *vehicleNum_upper;						//��ʾÿ����վ�����ĳ���������,��СΪ[p.Depot_Num]
	int *vehicleNum_lower;						//��ʾÿ����վ�����ĳ���������,��СΪ[p.Depot_Num]
	//��ͻ����߻���ص�
	NewConstraint *addCons_New;					//�ڸ�node���¼����Լ������СΪ[MAX_ADD]
	int addCons_New_num;						//�ڸ�node���¼����Լ��������
	NewConstraint *addCons_Total;				//�����¼����Լ�����ڣ��ڸ�node�����п��ǵ�Լ������СΪ[int(MAX_BRANCH_NODES/2)]
	int addCons_Total_num;						//�ڸ�node�����п��ǵ�Լ��������

	//��ÿ��BranchNode�����磬��ÿ����֧�ڵ�����Ҫ����
	float **CostNetwork_Branch;					//ÿ����֦�ڵ��ϵķ������磬������֮��û��·����������ô����ΪMAXNUM��[Customer_Num+Depot_Num]*[Customer_Num+Depot_Num]
	int *valid_Cus;								//��Ч��customer���ϣ���ʾ��֧�����У���Щ�㻹���Ը��ݶ�ż����ֵ���������б����ǣ���СΪ[Customer_Num]
												//1��ʾ��customer�ڸ�����Ч������Ϊ0

	//CG����
	//���ӽڵ��Ҫ�����ڹ���addCons_New
	int branch_strategy;						//��֧���ԣ���ʼ��Ϊ-1����
												//0����ʾ�Գ�������֧��1����ʾ�Ի���֧
	int branch_depot;							//ָʾ�Դ��ĸ���վ�����ĳ�������֧
	int branch_vehicleNum;						//ָʾ�������ķֽ�㣨��֦�Ͻ磬��֦�½磩
												//��֦<= floor(BB.vehicle_num[i]),��֦>=floor(BB.vehicle_num[i])+1
	int branchcons_num;							//branch_Startnode����branch_Endnode�Ĵ�С
	int *branch_Startnode;						//ָʾ��֧������㣬��СΪ[Conf::MAX_ADD]
	int *branch_Endnode;						//ָʾ��֧�����յ㣬��СΪ[Conf::MAX_ADD]
};


class BranchABound
{
public:
	BranchABound();
	virtual ~BranchABound();

	void Claim_Branch(Problem &p);
	void Choose_solveNode(void);					//ѡ����һ�����Ľڵ�
	void releaseNode(Problem &p);

	void Update_bestLB(SolINFOR &SolInfo);
public:
	BranchNode *branch;								//Ѱ��BranchNode�������������СΪ[Conf::MAX_BRANCH_NODES]
	int exist_nodeNum;								//branch���Ѿ����ڵ�node����
	int best_node_upper;							//�Ͻ�ֵ��õķ�֧������
	int best_node_lower;							//�½�ֵ��õķ�֧������
	int cur_node;									//��ǰ��֦�Ľڵ�����
	float best_upper;								//��֧�����Ѿ��ҵ�������Ͻ�
	float best_lower;								//��֧�����Ѿ��ҵ�������½�

	//��һ�δ��Ǹ�ĸ�ڵ㿪ʼ��֧
	int mother_node;

	//�������
	int Toslove_index[2];							//��СΪ[2]���ֱ�
	int Toslove_num;								//Toslove_index������

	//������
	float *vehicle_num;								//��ÿ����վ�����ĳ���������СΪ[p.Depot_Num]
	float **flow_onarc;								//ÿ�����ϵ��ۻ���������СΪ[p.Customer_Num]*[p.Customer_Num]����������ֻ�Կͻ���֮��Ļ����з�֧�������Ǵ�ÿ����վ����������
	int *customer_branch;							//����ͻ����Է�֧Ϊ1������Ϊ0����СΪ[p.Customer_Num]
};

#endif