#ifndef SUBPROBLEM_H_
#define SUBPROBLEM_H_

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//				         	Exact-DSSR-NG-Dynamic-Programming for ESPPRC								  //
//���ܣ�1��˫�������																					  //
//		2���ֽ��Ķ�̬ȷ��(����ֽ���Ե���ʱ�����)													  //
//		3��k-cycle����������																			  //
//		4��״̬�ռ��˥���ɳڼ���																		  //
//		5��ng-path����������																			  //
//											2020-12-17													  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iterator>
#include <vector>

#include "Problem.h"
#include "RMP.h"
#include "Cut.h"
#include "Columns.h"
#include "BranchABound.h"

//��matlab�йص�
#include "mclmcr.h"
#include "matrix.h"
#include "mclcppclass.h"
#include "qiu_ni.h"

using namespace std;

const int BACKWARD = -1;			//��������ʾ���򣬷���
const int FORWARD = 1;				//��������ʾ��������

//���ñ���
extern int *temp_Index;				//�����õ���ʱ���������ⷴ�����ٿռ�
extern long *temppass;				//��ʱ�жϼ��ϵ��м���������ⷴ�����ٿռ�
extern int passnode_length;			//passnode�����ά��
extern long *passnodelist;			//passnode���ֵ�
extern vector<int> elimi_nodes;		//�Ƚ�����label֧���ϵʱ����������SDC��RC��gapֵ�Ľڵ㼯��
//

class Path	//label
{
public:
	Path();
	virtual ~Path();

	void get_ComplementarySet_toFeasibleExtensionsbyNgpath(long * SetA, long * SetB);
	bool get_IntersectionSet_toFeasibleExtensionsbyNgpath(long * SetA);
	void Copy_Customer(Path & fromlabel);
	void Copy_SDCnode(Path & fromlabel, SDC &Cuts_sdc);
	void update_passnode_kcycle(int cycle_num,int next_modifynode);
	void copy_passnode_ngpath(Path & fromlabel);
	void copy_passnode(Path & fromlabel);
	void update_passnode_ngpath(long* nextcustomer_neighbourhood_passnode, int next_modifynode,Path & fromlabel);
	bool belong_toset(int node, long *set);
	bool update_elementary_passnode(int next_modifynode);
	void copy_passnode_2cycle(long* frompassnode_ngpath);
	void Union_2cycle(int obj_modifynode);

	void copy_passnode_simple(Path & fromlabel);
	void update_passnode_kcycle_simple(int cycle_num, int next_modifynode);
public:
	//״̬
	int exist_state;						//��·����container�е�״̬��0��ʾ��ɾ����1��ʾ����Ч
	int extend_state;						//��·���Ƿ�����չ��0��ʾ���ܣ�1��ʾ�Կ�����չ
	//·����Ϣ������˳��
	int *customeList;						//����·���ξ�����customer����ţ���СΪ[MAX_NODES]
											//�����Ǵ�nodes[0]��nodes[nodenum-1]
											//�����Ǵ�nodes[nodenum-1]��nodes[0]
	int customeList_num;					//����·������customer����
	//int pre_label;						//ǰ��label�����
	//��Դ��Ϣ
	float RC_value;							//��·����Ӧ��RCֵ
	float usedcapacity;						//����·��β���Ѿ�ʹ�õĳ�������ǰ��
	float availablecapacity;				//����·��ǰʣ�೵����������
	float accu_duration;					//����·�ۼƵ�duration
	//float auxi_duration;					//��������accu_duration
	float arrivaltime;						//����·��ǰ�㣨���㼯���һ���㣩�ĵ���ʱ�䣨����ģ���ǰ��
	float surplus_time;						//����·��ǰ�㷴����չ��ʣ��ʱ�䣨���򣩣���consumed_time���������������ࣩ
	float consumed_time;					//����·�ӵ�ǰ����������յ�t����Сʱ�䣨���򣩣��������ࣩ

	//cut-�˴��޸�
#if SRCUT == 1
	float *state_src;						//����·ÿ��src��Ӧ�ĵ�ǰϵ��������ÿ���ӵ�1��ı�һ��RC_value������Ϊ0,��СΪ[MAX_SR_NUM]
#endif

	//������Ϣ
	float OBJ_value;						//��·����Ӧ��Ŀ�꺯��ֵ
	//float AugRC_value;					//����·�ڿ���stableʱ������RCֵ
	//int cycle_num;						//����·����Ȧ���������ظ������Ľڵ��������
	//int repetition_posi[MAX_CYCLE];		//��¼nodes�ĸ�λ���Ǳ��ظ������Ľڵ�
	//bool complementornot;					//����·��passnode�Ƿ��¼ȫ�������ڵ�

	//��������й�
	long *passnode;							//�жϾ���������顣������Ϊ�����͡���СΪ[int((CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
											//���磬������2��4�������Ʊ��ʽ��1010��������Ϊ10����������passnode���������10.Ȼ��ͨ��һ��������ת������powerlist��ʵ��ת����ͨ��&��|λ��������ʵ���ж�
											//passnode����¼�ظ������Ľڵ�
											//��EPO��������elementatyΪtrue��һֱ��¼�����elementatyΪfalse��ֹͣ��¼
											//��CPA�����һֱ��¼�����Ľڵ�
	bool elementaty;						//��·���Ƿ���ȣ�=true�������·����=false���л�
	
	//ng-cycle-SDC�йص�
	vector<int> SDCnode;					//��·������Augng_set���ƵĲ��ܾ����Ľڵ����ʵ��ţ��ݶ�Ϊ[Conf::MAX_NODES]
	long *passnode_SDCnode;					//�洢SDCnode�Ķ�������Ϣ����СΪ[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
	int SDCnode_num;						//SDCnode�д洢�Ľڵ�ĸ���

	//2-cycle�й�
	long *passnode_2cycle;					//�����ڶ����ڵ�+ngpath�е����������ڵ㵽���һ���ڵ㣬��СΪ[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
	int prenode;							//ǰһ��������modifynode��Ҳ���ǵ����ڶ���modifynode
											//Ĭ�ϸ�·�������������ڵ�,���нڵ㶼��p.Customer_Num����
	//ng-path�йص�
	long *passnode_ngpath;					//�洢��ng-path���ƵĲ��ܾ����Ľڵ�,�ö����Ʊ�ʾ����СΪ[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
	int *ngpath;							//�洢��ng-path���ƵĲ��ܾ����Ľڵ�,��С�������ݶ�Ϊ[Conf::MAX_NODES]
	int ngpath_num;							//ngpath�д洢�Ľڵ�ĸ���
	bool augment_ornot;						//��·���Ƿ�����(������·��֧�䲿��·��)�����Ϊtrue��feasible_extensions_byngpath��Ч�����Ϊfalse�����ж�feasible_extensions_byngpath
	long *feasible_extensions_byngpath;		//����һ������ng-path���Ƹ�·������չʱֻ�ܾ����Ľڵ㣬�洢�ڶ������У���СΪ[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]

	//��stable�йص�
	//float feasible_curfitness;		//��·����Ŀ�꺯������ֵ(�����Ŷ�)
	//float unfeasible_curfitness;		//��·���Ĳ�����ʱ��Ŀ�꺯��(�����Ŷ�)
	//float *y_byinverse;				//��·����Ӧ��ϵ������pr���Ե�ǰ���������invB�õ��ж�����yr,��СΪ[temp_op.CustomerNum+temp_op.DepotNum]

};

class NodeBucket
{
public:
	NodeBucket();
	virtual ~NodeBucket();
public:
	//��label���
	int *Path_index;							//�洢���ڵ�ǰ�ͻ�(����ڵ�)�ϵ�label��ath_container��λ��,��ʼ����СΪ[MAX_PATH_NUM]
	int Path_Num;								//��ǰ�ͻ�(����ڵ�)�ϰ�����label(path)������PSP����ʱ��ʼ��Ϊ0

	//��completion bound���
	float *Completion_bounds;					//ǰ��label��Completion_bounds[i]��ʾ�Ӹõ����������depot����consumed_time������iʱ����СRCֵ��RCֵ�½磩����С�ݶ�Ϊ[��վ��ʱ�䴰����]
												//����label��
	int Bucket_max;								//�ڵ�ǰ�ڵ��У���Ҫ������������
												//ǰ��label��
												//����label��
	int Bucket_min;								//�ڵ�ǰ�ڵ��У���Ҫ������������
												//ǰ��label��
												//����label��
};

class Subproblem
{
public:
	Subproblem();
	virtual ~Subproblem();

	void Claim_PSP(Problem &p);
	void Get_dual_info(int obj_depot, float remain_arc, BranchABound & BB, RMP &lp, Problem &p, SDC &Cuts_sdc);	//remain_arc��ʾ��ÿ���ڵ���������Ļ�����Ŀ���������ģ�ϴ�ʱ��ȡ0.3-0.6֮�����ټ���ͬʱ���ᶪʧ̫��⣬ȡ1ʱ�ܹ�������Ž�
	void Label_clear(Problem &p);
	void Feasible_Extension(BranchABound & BB, RMP &lp, Problem &p);
	void Build_Mapping(BranchABound & BB, RMP & lp, Problem &p);
	void Reset_Maxtime(BranchABound & BB, RMP & lp, Problem &p);
	bool Check_validCus(int check_node,BranchABound & BB, RMP & lp);
	void Ini_compleBound_bucket(Problem &p);

	void Initial_Hpoints(Problem &p);
	void Ini_ngset(BranchABound & BB, RMP & lp, Problem &p);
	void ng_clear(void);
	void Initial_lables(BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);												//���ɳ�ʼlabel
	int Seek_containerIndex(int direction);																										//�ҵ���ǰ���Բ���Path_container��λ��
	int Get_FoundIndex(void);																													//�ҵ���ǰ���Բ���Path_container��λ��
	void Add_container(int direction,int order,int incre);																						//�����޸ĵ�λ�ã����Path_container��Delelted_index�е����
	void Add_found(int order);
	void Delete_container(int direction,int container_index);																					//�����޸ĵ�λ�ã�ɾ��PDelelted_index�е����
	void Delete_pathindex(int direction, int customer, int pathindex);																			//�����޸ĵ�λ�ã�ɾ��Path_index���
	void Insert_toPassnode(int insert_modifynode, long *temp_passnode);																			//��insert_modifynode���뵽temp_passnode���γɶ�����
	void remove_fromPassnode(int remove_modifynode, long *temp_passnode);
	bool if_union_domination(bool iniornot, long * newroute);
	bool domination_2cycle_ngpath_cuts(int direction,  int nextcustomer, RMP & lp, Path & nextV,  SRC & Cuts_src);								//֧������ص㺯��
	bool belong_toset(int node, long *set);																										//�����node�Ƿ��ڼ���set��
	bool check_subset(long* S1, long* S2);																										//���S1�Ƿ���S2���Ӽ������S2֧�䷵��true�����S2��֧��S1��ע�ⲻ��˵S2��S1������֧���ϵ���п���S1֧��S2���򷵻�false��
	bool DP_Labeling(int direction, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);										//��������Ѱ��·�������庯��
	int Search_Nextcustomer(int itor, Problem & p);																								//DP_Labeling��Ѱ����һ����չ��customer���ĸ���
	bool ExtendLable_onCustomer(int direction,int cur_customer, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);			//ָ��һ��customer����չlabel
	bool Extend_Onestep(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);			//�ɵ�ǰ��currentnode��nextnode����չһ�������������չ����true�����򷵻�false
	bool satisfying_resource_constrains(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, Problem & p);
	bool if_2cycle(Path & curlabel, int next_modifynode);
	bool if_ng_path(int next_modifynode, Path & curlabel);
	float Add_RC(int direction, int currentCus, Path & curlabel, int nextCus, BranchABound & BB, RMP & lp, Problem &p);
	bool combinebidirection_kcycle_ngpath_cuts_stable(BranchABound & BB, RMP & lp, Problem & p, SRC & Cuts_src);									//�ϲ����������label
																																				//����trueʱ����elementary��RC>=0,���������ҵ�һ��RC<0��elementary��·��
																																				//����falseʱ����elementary��RC<0
	bool check_route_samenode(long* pass1, long * pass2);																						//�ж�pass1��pass2�Ƿ�����ͬ�㣬=true������ͬ�㣬=false��û��
	bool Update_FoundPath(float tempRC, int forLabel_index, Path forlabel, int backLabel_index, Path backlabel, Problem &p);
	void Update_ngset(Problem &p);																												//�ҵ�Shortest_path��ÿ��cycle��������cycle�нڵ��ngset
	void Update_compleBound(Problem &p);
	void Generate_cycle(int direction, int endposi,Problem & p);																				//����Ѱ��Shortest_path��һ��cycle
	void Add_ngset_byCycle(Problem &p);																											//��Cycle�нڵ��е�ngset����ظ��ڵ�
	void Extend_ngset(int direction, Problem & p);
	void Label_update(Problem &p);
	void generate_ngpath_Byroute(Path & objlabel);
	bool Check_ngpath_Byroute(Path & objlabel);
	void Construct_FoundPath(Problem &p);
	void Delete_duplicate(Problem &p);
	void Set_dssr_ngset(Problem &p);
	bool Copy_Augngset_SDC(int next_cus, Path &nextlabel, Path & curlabel);																		//��curlabel��SDC��Ϣ��ֵ��nextlabel�����ж��Ƿ����next_cus,��������򷵻�true������Ϊfalse
	bool In_SDCcycle(int customer_no, Path & check_label);																						//���customer_no�Ƿ���check_label.SDCnode��
	float Insert_toSDC(int customer_no, Path & objlabel, RMP & lp);																				//һ�����մ�С��������ģ����ı�size�Ĳ������
	void Insert_intoVector(int insert_posi, int customer_no, Path & objlabel);
	float Calculate_RCincrement(Path & better_label, Path & dominated_label, RMP & lp);															//����dominace rule�µ�RC����Сgapֵ
	float Calculate_Joinmodify(Path & forlabel, Path & backlabel, RMP & lp);																	//����ǰ��label�ϲ�������label��RC��Ҫ������ֵ
	float Calculate_SRCdual_Gap(Path & better_label, Path & dominated_label, RMP & lp, SRC & Cuts_src);											//����better_label��state_src��dominated_label��state_src�и���ʱ������SRC��Ӧ��ż�������ܺͣ�����һ������ֵ

	bool DP_Ng_Cuts_Dssr_Stable(BranchABound & BB, RMP &lp, Problem &p, SRC &Cuts_src, SDC &Cuts_sdc);											//����trueʱ����С��RCΪ�������򷵻�false
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool TSPTW_exist(int start_depot, int *subset, int subset_num,BranchABound & BB, RMP &lp, Problem &p);										//ֻ�����жϸ����ĵ㼯subset���Ƿ����һ��TSPTW���start_depot������������ڷ���true�����򷵻�false
																																				//ȥ��DP_Ng_Cuts_Dssr_Stable�����й�cut�Ĳ���
	void Build_Mapping_simple(BranchABound & BB, RMP & lp, Problem &p);
	void Feasible_Extension_simple(BranchABound & BB, RMP &lp, Problem &p);
	void Ini_ngset_simple(BranchABound & BB, RMP & lp, Problem &p);
	void Initial_lables_simple(BranchABound & BB, RMP & lp, Problem & p);
	bool DP_Labeling_simple(int direction, BranchABound & BB, RMP & lp, Problem & p);
	int ExtendLable_onCustomer_simple(int direction, int cur_customer, BranchABound & BB, RMP & lp, Problem & p);
	int Extend_Onestep_simple(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, RMP & lp, Problem & p);
	bool domination_2cycle_ngpath_cuts_simple(int direction, int nextcustomer, RMP & lp, Path & nextV);
	bool combinebidirection_kcycle_ngpath_cuts_stable_simple(BranchABound & BB, RMP & lp, Problem & p);
public:
	//�ؼ����������������Ƿ�ȷ���
	bool exact_ornot;							//=trueʱ����ȷ��⣬������2cycle��=falseʱ������ʽ���
	bool restricted_extend;						//PSP��չ�Ƿ�ֻ��ʹRC���ٵĻ�
												//=trueʱ������ʽ��չ��=falseʱ����ȷ��չ

	//��PSPָ���йص�
	int all_label_num;							//�����ж�����src�᲻�ᵼ��label���Ӻܶ࣬��������Ӻܶ࣬�͵��ûع�
	int init_label_num;							//�����ж�����src�᲻�ᵼ��label���Ӻܶ࣬��������Ӻܶ࣬�͵��ûع�
	float PSP_time;								//����PSP���ʱ��

	//�����ҵ������йص�
	float reduce_cost;							//PSP���ҵ�����С��RCֵ
	float improve_reduced_cost;					//�Ľ�PSP�е�Ŀ�꺯��ֵ�������˻�ʱʹ��
	float feasible_descent;						//PSP���ҵ�����С��Yֵ��С��0��ȷ���½�
	float unfeasible_descent;					//PSP���ҵ�����С�������Ŷ���Ŀ�꺯��������

	//������ʹ�õ�����
	int current_depot;							//��ǰ�������еĳ�����վ,��0��ʼ��Depot_Num
	float fix_cost;								//��ʾ���������Ĺ̶��ɱ�(��ż����)
	float arc_remain_proportion;				//��ʾ��ÿ���ڵ���������Ļ�����Ŀ���������ģ�ϴ�ʱ��ȡ0.3-0.6֮�����ټ���ͬʱ���ᶪʧ̫��⣬ȡ1ʱ�ܹ�������Ž�

	//����Դbound�йص�
	float Hpoint_For;							//��̬�ֽ����½磨��ǰ�������йأ�
	float Hpoint_Back;							//��̬�ֽ����Ͻ磨�뷴�������йأ�

	//���е���չ��
	int **ForfeasiExten_Index;					//ǰ�򣺴洢������չ��customer����ţ���СΪ[Customer_Num+1]*[Customer_Num+1],0��Customer_Num-1Ϊ�ͻ���Customer_NumΪ���������еĳ�վ
	int *ForfeasiExten_Index_num;				//�洢ForfeasiExten_Index��ÿ�����������СΪ[Customer_Num+1]
	int **BackfeasiExten_Index;					//���򣺴洢������չ��customer����ţ���СΪ[Customer_Num+1]*[Customer_Num+1]��0��Customer_Num-1Ϊ�ͻ���Customer_NumΪ���������еĳ�վ
	int *BackfeasiExten_Index_num;				//�洢BackfeasiExten_Index��ÿ�����������СΪ[Customer_Num+1]

	//ӳ���ϵ
	int *Mapping_Reduced;						//��Realnetworkӳ�䵽Modifiednetwork�Ķ�Ӧ��ϵ��Ԫ��Ϊ���,Ŀ���Ǽ���passnode����,��СΪ[Customer_Num]
												//��ʵ��customer�����
												//Mapping_Reduced[i]��ʾModifiednetwork�е�i���㣨��0��ʼ����ӦRealnetwork��Customer�����
	int *Mapping_fromRN_toMN;					//Mapping_Reduced�ķ���ӳ��,Ԫ��Ϊ-1������ţ���СΪ[Customer_Num]
												//ӳ����Modifiednetwork�����
												//Mapping_fromRN_toMN[i]��ʾRealnetwork��i��Customer��ӦModifiednetwork�е����
	int Modifiednetwork_Nodes_num;				//��Mapping_Reduced�п��ǵĵ�ĸ���


	//��ngset�йص�,����洢�Ķ���modifynode
	int **neighbourhood;						//�����,�ڵ�i�ǵõ����нڵ㣬��СΪ[Customer_Num]*[Conf::MAX_NODES]
	int *neighbourhood_num;						//neighbourhood����Ч��С����СΪ[Customer_Num]
	long **neighbourhood_passnode;				//����ĵ�,�ö����ƴ洢����СΪ[Customer_Num]*[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long **ng_memory_passnode;					//����㣬�ǵ�i�ڵ�����нڵ㣬��СΪ[Customer_Num]*[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]

	//��ng-cycle-SDC�йص�
	long **Aug_neighbourhood_passnode;		//��չngset������ĵ�,�ö����ƴ洢����СΪ[Customer_Num]*[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	
	//��Complebound

	//������
	int *Delelted_Forwardindex;					//��¼ForwardPath_container�Ѿ���ɾ����label��������֮�����λ����ӣ���СΪ[int(MAX_PATH_CONTAINER/10)]
	int Delelted_Forwardindex_num;				//����Delelted_Forwardindex�����������ʱ�������

	int *Delelted_Backwardindex;				//��¼BackwardPath_container�Ѿ���ɾ����label��������֮�����λ����ӣ���СΪ[int(MAX_PATH_CONTAINER/10)]
	int Delelted_Backwardindex_num;				//����Delelted_Backwardindex�����������ʱ�������

	//Bucket
	NodeBucket *ForwardBucket;					//�����洢PSP���ҵ�������ǰ��·������СΪ[p.Customer_Num+1]
	NodeBucket *BackwardBucket;					//�����洢PSP���ҵ�������ǰ��·������СΪ[p.Customer_Num+1]

	//�洢�ҵ����ܹ�����PSP��label
	//��Ҫ�洢ΪColumns��
	Columns *FoundPath_container;				//�����洢�ҵ����ܹ�����PSP���ҵ�������·������СΪ[p.Customer_Num+1]
	int FoundPath_container_num;				//����FoundPath_container�Ѿ�ʹ�õ�����
	int *Delelted_Foundindex;					//��¼FoundPath_container�Ѿ���ɾ����label��������֮�����λ����ӣ���СΪ[p.Customer_Num+1]
	int Delelted_Foundindex_num;				//����Delelted_Foundindex�����������ʱ�������

	//��elementary��·��
	Path Shortest_path;							//�����洢PSP���ҵ���RCֵ��С��·���������Ƿ����
	int Shortest_forlabel;						//��¼���Shortest_path��ǰ��label�����
	int Shortest_backlabel;						//��¼���Shortest_path�ķ���label�����

	//Shortest_path�е�cycle
	int *cycle_Modifynode;						//��¼Shortest_path��һ��cycle������customer��Modifiednetwork����ţ���СΪ[p.Customer_Num+1]
	int cycle_Modifynode_num;					//cycle_Modifynode�Ĵ�С

	//TSPTW�йص�
	float *temp_dual;							//��TSPTW_exist������ÿ���ڵ��ϵĶ�żֵ��ѡ�еĶ���999�������Ķ���-999����СΪ[p.Customer_Num]
	float temp_Max_arrivaltime;					//��ʱ�洢ȫ��Max_arrivaltime
	long temp_passnodevalue;					//�����ж��Ƿ񾭹�subset��ÿ����
};


#endif