#ifndef CUSTOMER_H_
#define CUSTOMER_H_

class Customer
{
public:
	Customer();
	virtual ~Customer();

	void Copy(Customer fromCus);

public:
	//标准属性
	float demand;				//需求
	float x;					//横坐标
	float y;					//纵坐标
	float startTW;				//开始时间窗
	float endTW;				//结束时间窗
	float servicetime;			//服务时间
	int node_id;				//序号，从0开始

	//int Nearest_depot_of_node;				//表示网络中每个客户节点最近的场站
	//int *Nearest_nodes_of_depot;				//一维动态数组，给定一个场站，到该场站距离最小的客户集合
	//int NearestnodesNum_eachdepot;			//一维动态数组，到每一个场站距离最小的客户数量

	//可行拓展
	int *for_feasible_extensions;				//表示前向：从该客户出发能够在约束下到达其他的客户节点，如果能到达该节点为1，否则为0，大小为[VRP.Customer_Num+Depot_Num]
	int *back_feasible_extensions;				//表示反向：从该客户出发能够在约束下到达其他的客户节点，如果能到达该节点为1，否则为0，大小为[VRP.Customer_Num+Depot_Num]
	//ng-set,存储初始值，在算法中不变的
	int *ngSet;									//邻域点,节点i记得的所有节点，大小为[Customer_Num]
	int ngSet_num;							//记录dssr_ngSet集合的大小
	long *negSet_passnode;						//邻域的点,用二进制存储，大小为[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long *ngMem_passnode;						//领域点，记得i节点的所有节点，大小为[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	//ng-set的副本，接收ng-set初始值，并在dssr框架下逐步拓展
	int *dssr_ngSet;							//对应ngSet，大小为[Customer_Num]
	int dssr_ngSet_num;							//记录dssr_ngSet集合的大小
	long *dssr_negSet_passnode;					//对应negSet_passnode，大小为[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long *dssr_ngMem_passnode;					//对应ngMem_passnode，大小为[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	//CPA框架下，ng-cycle-SDC需要的拓展ngset
	int *Ang_ngSet;								//对应ngSet，大小为[Customer_Num]
	int Ang_ngSet_num;							//记录Ang_ngSet集合的大小
	long *Ang_negSet_passnode;					//对应negSet_passnode，大小为[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
	long *Ang_ngMem_passnode;					//对应ngMem_passnode，大小为[int((Customer_Num - 0.1) / Conf::EACH_LONG_NUM) + 1]
};

#endif