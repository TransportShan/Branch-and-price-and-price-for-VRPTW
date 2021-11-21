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
	//路径本身属性
	int Depot;							//该线路始发的场站，以node_id为准，即第一个场站为VRP.Customer_Num
	int nodesNum;						//该线路经过客户的数量
	int *nodes;							//该线路经过的客户集合，大小为[MAX_NODES]
	float Load;							//该线路的装载量
	float Totalcost;					//该线路的总成本=运输成本+腐败成本+能源成本
	float Travelcost;					//该线路的运输成本（长度）
	float Decaycost;					//该线路的腐败成本
	float Energycost;					//该线路的能源成本
	float Duration;						//该路径的当前已经累积的运输时间，=在途运输时间+服务时间，注意不包括等待时间
	float Reducedcost;					//该线路在当前解中对应的reduced cost

	int *Customer_indicator;			//该路径经过哪些客户，经过该位置为1，否则为0，大小为[p.Customer_Num + p.Depot_Num]
										//注意：当路径非初等时，该值可以大于1
	int *depot_indicator;				//该路径从哪个场站出发，从该场站出发为1，否则为0，大小为[p.Depot_Num]

#if Frameworks == 1
	//与ng-cycle-SDC相关
	bool elementary_ornot;				//该路径上是否为初等的，true为初等路径，false则至少有一个cycle
	int **cus_cyclePosi;				//该路径上每个客户形成的每个cycle（这个cycle必须是当前ng-feasible的）末尾点在路径的位置，初始化为0，大小为[p.Customer_Num]*[int(Conf::MAX_NODES/3)]
	int *cus_cyclePosi_num;				//cus_cyclePosi的大小，大小为[p.Customer_Num]，初始化为0
	int *SDC_cof;						//该路径上在每个客户点对应的SDC上的系数，初始化为0，大小为[p.Customer_Num]
										//保存该系数是为了在RMP中方便修改CPLEX中的系数
#endif
	//与branch有关的
	int **pre_arcs;						//表示路径上nodes[i]之前经过节点的序号集合，大小为[p.Customer_Num + p.Depot_Num]*[Conf::MAX_PASS_NUM]，存储的是序号，-1表示没有经过点
	int *pre_arcs_Num;					//pre_arcs在每个点上的集合大小，大小为[p.Customer_Num + p.Depot_Num]，初始化为0
	int **succ_arcs;					//表示路径上nodes[i]之后经过节点的序号集合，大小为[p.Customer_Num + p.Depot_Num]*[Conf::MAX_PASS_NUM]，存储的是序号，-1表示没有经过点
	int *succ_arcs_Num;					//succ_arcs在每个点上的集合大小，大小为[p.Customer_Num + p.Depot_Num]，初始化为0

	//与RMP有关的，冗余，因为在类RMP会定义类似数组
	float solution_value;				//该路径在RMP最优解中对应的值
	int baseornot;						//是否为基解，为基解为1，否则为0
	int posi_baseornot;					//是否为正基解,为正基解为1，否则为0
	int compatibleornot;				//是否与当前正(基)解S兼容，即是否属于集合Cs.=1则兼容，属于Cs；=0则不兼容，属于Is。
	float incompatibility;				//如果不兼容(compatibleornot==0),表示与当前正(基)解S的不兼容度。当incompatibility==0时，表示兼容。

	//PSP中辅助
	int pre_label;
	int succ_label;
};

#endif
