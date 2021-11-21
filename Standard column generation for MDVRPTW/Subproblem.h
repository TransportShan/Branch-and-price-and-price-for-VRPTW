#ifndef SUBPROBLEM_H_
#define SUBPROBLEM_H_

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//				         	Exact-DSSR-NG-Dynamic-Programming for ESPPRC								  //
//功能：1，双向的搜索																					  //
//		2，分界点的动态确定(这里分界点以到达时间衡量)													  //
//		3，k-cycle的消减技术																			  //
//		4，状态空间的衰减松弛技术																		  //
//		5，ng-path的消减技术																			  //
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

//与matlab有关的
#include "mclmcr.h"
#include "matrix.h"
#include "mclcppclass.h"
#include "qiu_ni.h"

using namespace std;

const int BACKWARD = -1;			//常数，表示方向，反向
const int FORWARD = 1;				//常数，表示方向，正向

//公用变量
extern int *temp_Index;				//排序用的临时变量，避免反复开辟空间
extern long *temppass;				//临时判断集合的中间变量，避免反复开辟空间
extern int passnode_length;			//passnode的最大维度
extern long *passnodelist;			//passnode的字典
extern vector<int> elimi_nodes;		//比较两个label支配关系时，用来修正SDC的RC的gap值的节点集合
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
	//状态
	int exist_state;						//该路径在container中的状态，0表示被删除，1表示仍有效
	int extend_state;						//该路径是否还能拓展，0表示不能，1表示仍可以拓展
	//路径信息，包括顺序
	int *customeList;						//该线路依次经过的customer的序号，大小为[MAX_NODES]
											//正向是从nodes[0]到nodes[nodenum-1]
											//反向是从nodes[nodenum-1]到nodes[0]
	int customeList_num;					//该线路经过的customer个数
	//int pre_label;						//前向label的序号
	//资源信息
	float RC_value;							//该路径对应的RC值
	float usedcapacity;						//该线路到尾点已经使用的车容量（前向）
	float availablecapacity;				//该线路当前剩余车容量（反向）
	float accu_duration;					//该线路累计的duration
	//float auxi_duration;					//辅助计算accu_duration
	float arrivaltime;						//该线路当前点（即点集最后一个点）的到达时间（最早的）（前向）
	float surplus_time;						//该线路当前点反向拓展的剩余时间（反向）（与consumed_time互补）（待用冗余）
	float consumed_time;					//该线路从当前点出发到达终点t的最小时间（反向）（待用冗余）

	//cut-此处修改
#if SRCUT == 1
	float *state_src;						//该线路每个src对应的当前系数余量，每增加到1则改变一次RC_value并重置为0,大小为[MAX_SR_NUM]
#endif

	//辅助信息
	float OBJ_value;						//该路径对应的目标函数值
	//float AugRC_value;					//该线路在考虑stable时的增广RC值
	//int cycle_num;						//该线路的子圈的数量（重复经过的节点的数量）
	//int repetition_posi[MAX_CYCLE];		//记录nodes哪个位置是被重复经过的节点
	//bool complementornot;					//该线路的passnode是否记录全部经过节点

	//与初等性有关
	long *passnode;							//判断经过点的数组。该数组为长整型。大小为[int((CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
											//例如，经过点2和4，二进制表达式：1010，长整数为10，这样我在passnode存的数字是10.然后，通过一个二进制转换数组powerlist来实现转换。通过&，|位操作运算实现判断
											//passnode不记录重复经过的节点
											//在EPO框架下如果elementaty为true就一直记录；如果elementaty为false就停止记录
											//在CPA框架下一直记录经过的节点
	bool elementaty;						//该路径是否初等，=true，则初等路径；=false则有环
	
	//ng-cycle-SDC有关的
	vector<int> SDCnode;					//该路径中因Augng_set限制的不能经过的节点的真实序号，暂定为[Conf::MAX_NODES]
	long *passnode_SDCnode;					//存储SDCnode的二进制信息，大小为[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
	int SDCnode_num;						//SDCnode中存储的节点的个数

	//2-cycle有关
	long *passnode_2cycle;					//倒数第二个节点+ngpath中倒数第三个节点到最后一个节点，大小为[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
	int prenode;							//前一个经过的modifynode，也就是倒数第二个modifynode
											//默认该路径至少有两个节点,所有节点都从p.Customer_Num发出
	//ng-path有关的
	long *passnode_ngpath;					//存储因ng-path限制的不能经过的节点,用二进制表示，大小为[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]
	int *ngpath;							//存储因ng-path限制的不能经过的节点,大小不定，暂定为[Conf::MAX_NODES]
	int ngpath_num;							//ngpath中存储的节点的个数
	bool augment_ornot;						//该路径是否增广(被其他路径支配部分路径)，如果为true则feasible_extensions_byngpath生效，如果为false则不用判断feasible_extensions_byngpath
	long *feasible_extensions_byngpath;		//由于一个或多个ng-path限制该路径在拓展时只能经过的节点，存储在二进制中，大小为[int((temp_op.CustomerNum-0.1)/ EACH_LONG_NUM) + 1 + 1]

	//与stable有关的
	//float feasible_curfitness;		//该路径的目标函数可行值(向上扰动)
	//float unfeasible_curfitness;		//该路径的不可行时的目标函数(向下扰动)
	//float *y_byinverse;				//该路径对应的系数向量pr乘以当前基的逆矩阵invB得到判断向量yr,大小为[temp_op.CustomerNum+temp_op.DepotNum]

};

class NodeBucket
{
public:
	NodeBucket();
	virtual ~NodeBucket();
public:
	//与label相关
	int *Path_index;							//存储，在当前客户(进入节点)上的label在ath_container的位置,初始化大小为[MAX_PATH_NUM]
	int Path_Num;								//当前客户(进入节点)上包含的label(path)数量，PSP结束时初始化为0

	//与completion bound相关
	float *Completion_bounds;					//前向label：Completion_bounds[i]表示从该点出发，到达depot，且consumed_time不超过i时的最小RC值（RC值下界），大小暂定为[场站的时间窗上限]
												//反向label：
	int Bucket_max;								//在当前节点中，需要检测的区域上限
												//前向label：
												//反向label：
	int Bucket_min;								//在当前节点中，需要检测的区域下限
												//前向label：
												//反向label：
};

class Subproblem
{
public:
	Subproblem();
	virtual ~Subproblem();

	void Claim_PSP(Problem &p);
	void Get_dual_info(int obj_depot, float remain_arc, BranchABound & BB, RMP &lp, Problem &p, SDC &Cuts_sdc);	//remain_arc表示从每个节点出发保留的弧的数目，当问题规模较大时，取0.3-0.6之间会加速计算同时不会丢失太多解，取1时能够求解最优解
	void Label_clear(Problem &p);
	void Feasible_Extension(BranchABound & BB, RMP &lp, Problem &p);
	void Build_Mapping(BranchABound & BB, RMP & lp, Problem &p);
	void Reset_Maxtime(BranchABound & BB, RMP & lp, Problem &p);
	bool Check_validCus(int check_node,BranchABound & BB, RMP & lp);
	void Ini_compleBound_bucket(Problem &p);

	void Initial_Hpoints(Problem &p);
	void Ini_ngset(BranchABound & BB, RMP & lp, Problem &p);
	void ng_clear(void);
	void Initial_lables(BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);												//生成初始label
	int Seek_containerIndex(int direction);																										//找到当前可以插入Path_container的位置
	int Get_FoundIndex(void);																													//找到当前可以插入Path_container的位置
	void Add_container(int direction,int order,int incre);																						//根据修改的位置，添加Path_container和Delelted_index中的序号
	void Add_found(int order);
	void Delete_container(int direction,int container_index);																					//根据修改的位置，删除PDelelted_index中的序号
	void Delete_pathindex(int direction, int customer, int pathindex);																			//根据修改的位置，删除Path_index序号
	void Insert_toPassnode(int insert_modifynode, long *temp_passnode);																			//将insert_modifynode插入到temp_passnode并形成二进制
	void remove_fromPassnode(int remove_modifynode, long *temp_passnode);
	bool if_union_domination(bool iniornot, long * newroute);
	bool domination_2cycle_ngpath_cuts(int direction,  int nextcustomer, RMP & lp, Path & nextV,  SRC & Cuts_src);								//支配规则，重点函数
	bool belong_toset(int node, long *set);																										//检查是node是否在集合set中
	bool check_subset(long* S1, long* S2);																										//检查S1是否是S2的子集，如果S2支配返回true，如何S2不支配S1（注意不是说S2和S1不存在支配关系，有可能S1支配S2）则返回false。
	bool DP_Labeling(int direction, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);										//在网络中寻找路径的主体函数
	int Search_Nextcustomer(int itor, Problem & p);																								//DP_Labeling中寻找下一个拓展的customer是哪个？
	bool ExtendLable_onCustomer(int direction,int cur_customer, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);			//指定一个customer后，拓展label
	bool Extend_Onestep(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, RMP & lp, Problem & p, SRC &Cuts_src, SDC &Cuts_sdc);			//由当前点currentnode向nextnode点拓展一步，如果可以拓展返回true，否则返回false
	bool satisfying_resource_constrains(int direction, int currentCus, Path & curlabel, int nextCus, Path & nextlabel, BranchABound & BB, Problem & p);
	bool if_2cycle(Path & curlabel, int next_modifynode);
	bool if_ng_path(int next_modifynode, Path & curlabel);
	float Add_RC(int direction, int currentCus, Path & curlabel, int nextCus, BranchABound & BB, RMP & lp, Problem &p);
	bool combinebidirection_kcycle_ngpath_cuts_stable(BranchABound & BB, RMP & lp, Problem & p, SRC & Cuts_src);									//合并正反方向的label
																																				//返回true时：非elementary的RC>=0,或者至少找到一个RC<0的elementary的路径
																																				//返回false时：非elementary的RC<0
	bool check_route_samenode(long* pass1, long * pass2);																						//判断pass1与pass2是否有相同点，=true则有相同点，=false则没有
	bool Update_FoundPath(float tempRC, int forLabel_index, Path forlabel, int backLabel_index, Path backlabel, Problem &p);
	void Update_ngset(Problem &p);																												//找到Shortest_path中每个cycle，并更新cycle中节点的ngset
	void Update_compleBound(Problem &p);
	void Generate_cycle(int direction, int endposi,Problem & p);																				//反向寻找Shortest_path中一个cycle
	void Add_ngset_byCycle(Problem &p);																											//向Cycle中节点中的ngset添加重复节点
	void Extend_ngset(int direction, Problem & p);
	void Label_update(Problem &p);
	void generate_ngpath_Byroute(Path & objlabel);
	bool Check_ngpath_Byroute(Path & objlabel);
	void Construct_FoundPath(Problem &p);
	void Delete_duplicate(Problem &p);
	void Set_dssr_ngset(Problem &p);
	bool Copy_Augngset_SDC(int next_cus, Path &nextlabel, Path & curlabel);																		//将curlabel的SDC信息赋值给nextlabel，并判断是否插入next_cus,如果插入则返回true，否则为false
	bool In_SDCcycle(int customer_no, Path & check_label);																						//检查customer_no是否在check_label.SDCnode中
	float Insert_toSDC(int customer_no, Path & objlabel, RMP & lp);																				//一个按照从小到大排序的，不改变size的插入操作
	void Insert_intoVector(int insert_posi, int customer_no, Path & objlabel);
	float Calculate_RCincrement(Path & better_label, Path & dominated_label, RMP & lp);															//计算dominace rule下的RC的最小gap值
	float Calculate_Joinmodify(Path & forlabel, Path & backlabel, RMP & lp);																	//计算前后label合并后，整体label的RC需要修正的值
	float Calculate_SRCdual_Gap(Path & better_label, Path & dominated_label, RMP & lp, SRC & Cuts_src);											//计算better_label中state_src比dominated_label中state_src中更大时的所有SRC对应对偶变量的总和，返回一个非正值

	bool DP_Ng_Cuts_Dssr_Stable(BranchABound & BB, RMP &lp, Problem &p, SRC &Cuts_src, SDC &Cuts_sdc);											//返回true时：最小的RC为负，否则返回false
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool TSPTW_exist(int start_depot, int *subset, int subset_num,BranchABound & BB, RMP &lp, Problem &p);										//只用来判断给定的点集subset中是否存在一个TSPTW解从start_depot出发，如果存在返回true，否则返回false
																																				//去掉DP_Ng_Cuts_Dssr_Stable所有有关cut的操作
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
	//关键参数，用来控制是否精确求解
	bool exact_ornot;							//=true时，精确求解，不调用2cycle，=false时，启发式求解
	bool restricted_extend;						//PSP拓展是否只在使RC减少的弧
												//=true时，启发式拓展，=false时，精确拓展

	//与PSP指标有关的
	int all_label_num;							//用来判断增加src会不会导致label增加很多，如果会增加很多，就调用回滚
	int init_label_num;							//用来判断增加src会不会导致label增加很多，如果会增加很多，就调用回滚
	float PSP_time;								//本次PSP求解时间

	//与新找到的列有关的
	float reduce_cost;							//PSP中找到的最小的RC值
	float improve_reduced_cost;					//改进PSP中的目标函数值，考虑退化时使用
	float feasible_descent;						//PSP中找到的最小的Y值，小于0则确定下降
	float unfeasible_descent;					//PSP中找到的最小的向下扰动后，目标函数减少量

	//子问题使用的网络
	int current_depot;							//当前子问题中的出发场站,从0开始到Depot_Num
	float fix_cost;								//表示从起点出发的固定成本(对偶变量)
	float arc_remain_proportion;				//表示从每个节点出发保留的弧的数目，当问题规模较大时，取0.3-0.6之间会加速计算同时不会丢失太多解，取1时能够求解最优解

	//与资源bound有关的
	float Hpoint_For;							//动态分界点的下界（与前向搜索有关）
	float Hpoint_Back;							//动态分界点的上界（与反向搜索有关）

	//可行的拓展点
	int **ForfeasiExten_Index;					//前向：存储可以拓展的customer的序号，大小为[Customer_Num+1]*[Customer_Num+1],0到Customer_Num-1为客户，Customer_Num为该子问题中的场站
	int *ForfeasiExten_Index_num;				//存储ForfeasiExten_Index在每天的数量，大小为[Customer_Num+1]
	int **BackfeasiExten_Index;					//反向：存储可以拓展的customer的序号，大小为[Customer_Num+1]*[Customer_Num+1]，0到Customer_Num-1为客户，Customer_Num为该子问题中的场站
	int *BackfeasiExten_Index_num;				//存储BackfeasiExten_Index在每天的数量，大小为[Customer_Num+1]

	//映射关系
	int *Mapping_Reduced;						//由Realnetwork映射到Modifiednetwork的对应关系，元素为序号,目的是减少passnode长度,大小为[Customer_Num]
												//真实的customer的序号
												//Mapping_Reduced[i]表示Modifiednetwork中第i个点（从0开始）对应Realnetwork中Customer的序号
	int *Mapping_fromRN_toMN;					//Mapping_Reduced的反向映射,元素为-1或者序号，大小为[Customer_Num]
												//映射在Modifiednetwork的序号
												//Mapping_fromRN_toMN[i]表示Realnetwork第i个Customer对应Modifiednetwork中的序号
	int Modifiednetwork_Nodes_num;				//在Mapping_Reduced中考虑的点的个数


	//与ngset有关的,这里存储的都是modifynode
	int **neighbourhood;						//邻域点,节点i记得的所有节点，大小为[Customer_Num]*[Conf::MAX_NODES]
	int *neighbourhood_num;						//neighbourhood的有效大小，大小为[Customer_Num]
	long **neighbourhood_passnode;				//邻域的点,用二进制存储，大小为[Customer_Num]*[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long **ng_memory_passnode;					//领域点，记得i节点的所有节点，大小为[Customer_Num]*[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]

	//与ng-cycle-SDC有关的
	long **Aug_neighbourhood_passnode;		//拓展ngset下邻域的点,用二进制存储，大小为[Customer_Num]*[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	
	//与Complebound

	//大容器
	int *Delelted_Forwardindex;					//记录ForwardPath_container已经被删除的label的数量，之后向该位置添加，大小为[int(MAX_PATH_CONTAINER/10)]
	int Delelted_Forwardindex_num;				//跟踪Delelted_Forwardindex的数量，添加时逆序添加

	int *Delelted_Backwardindex;				//记录BackwardPath_container已经被删除的label的数量，之后向该位置添加，大小为[int(MAX_PATH_CONTAINER/10)]
	int Delelted_Backwardindex_num;				//跟踪Delelted_Backwardindex的数量，添加时逆序添加

	//Bucket
	NodeBucket *ForwardBucket;					//用来存储PSP中找到的所有前向路径，大小为[p.Customer_Num+1]
	NodeBucket *BackwardBucket;					//用来存储PSP中找到的所有前向路径，大小为[p.Customer_Num+1]

	//存储找到的能够传给PSP的label
	//需要存储为Columns类
	Columns *FoundPath_container;				//用来存储找到的能够传给PSP中找到的所有路径，大小为[p.Customer_Num+1]
	int FoundPath_container_num;				//跟踪FoundPath_container已经使用的数量
	int *Delelted_Foundindex;					//记录FoundPath_container已经被删除的label的数量，之后向该位置添加，大小为[p.Customer_Num+1]
	int Delelted_Foundindex_num;				//跟踪Delelted_Foundindex的数量，添加时逆序添加

	//非elementary的路径
	Path Shortest_path;							//用来存储PSP中找到的RC值最小的路径，无论是否初等
	int Shortest_forlabel;						//记录组成Shortest_path的前向label的序号
	int Shortest_backlabel;						//记录组成Shortest_path的反向label的序号

	//Shortest_path中的cycle
	int *cycle_Modifynode;						//记录Shortest_path中一个cycle包含的customer在Modifiednetwork的序号，大小为[p.Customer_Num+1]
	int cycle_Modifynode_num;					//cycle_Modifynode的大小

	//TSPTW有关的
	float *temp_dual;							//在TSPTW_exist函数中每个节点上的对偶值，选中的都是999，其他的都是-999，大小为[p.Customer_Num]
	float temp_Max_arrivaltime;					//暂时存储全局Max_arrivaltime
	long temp_passnodevalue;					//用于判断是否经过subset中每个点
};


#endif