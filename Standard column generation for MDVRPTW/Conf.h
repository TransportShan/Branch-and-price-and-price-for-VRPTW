#ifndef CONF_H_
#define CONF_H_

#include <string>


#define MAXCELLLEN  102400			//读取CSV所需要的常数，当CSV文件的数据没有特别多时，该常数不用改
#define MAXNUM  9999999				//定义一个无穷大量
#define MINNUM	-9999999			//定义一个无穷小量
#define MINDOUBLE 0.0001			//定义一个等价于0的数量

#define DURATIONORNOT 0				//在约束中是否考虑duration
									//0：不考虑duration
									//1：考虑duration
#define Frameworks	0				//应用EPO还是CPA框架
									//0：EPO框架
									//1：CPA框架，当Frameworks=1时，EXACTPSP只能为0（不能调用DSSR框架）
#define AUGMENT 1					//控制CPA框架下如何扩大elementary constraints
									//0：通过扩大ngset
									//1：通过添加SDC约束
#define ClearColumn 1				//在AUGMENT==0时，扩大ngset后，是否根据cycle删除列池中的ng-infeasible列
									//0：不删除
									//1：删除
#define SRCUT 1						//0: 不考虑subset row-cut, 1: 考虑
#define KPATHCUT 0					//0: 不考虑kpath-cut, 1: 考虑
#define RCCUT 0						//0: 不考虑rounded capacaity-cut, 1: 考虑
#define NGINI 1						//在PSP中初始化ngset策略
									//0：静态初始化，只在每个customer的ngset中选择
									//1：动态初始化，在当前网络中选出最小的MAX_NEIGHBOURHOOD_NUM邻近点
#define FEASIBLE 1					//如何拓展网络
									//0：只考虑对偶值
									//1：对偶值基础上，考虑可行拓展点
#define NGDOI 1						//支配过程中的ngpath使用策略
									//0：以ng完全支配为主
									//1：以multi-ng支配为主
#define ORDERLABEL 1				//0: 不排序, 1: 在PSP合并阶段重新排序
#define FOUND 1						//在PSP合并label阶段的不同策略：
									//0：只留下前MAX_PSP_SIZE个找到的RC值小于0的elementary路径
									//1：在每个customer上都找到前后向关键资源gap值最小的路径
									//2：找到Pareto前沿
									//3：找到RC值最小的elementary路径
#define SEARCH 1					//在PSP拓展阶段中，搜索策略
									//0：按照customer顺序搜索
									//1：按照customer开始时间窗的升序顺序搜索
#define TCYCLE 2					//2-cycle支配的策略
									//0：不考虑2-cycle消减
									//1：精确策略，2-cycle必须在ngpath基础上判断
									//2：启发式策略，2-cycle与ngpath独立运行
#define NEWNG 1						//DSSR-ng框架下的更新neset后，label的ngpath的更新规则
									//0：表示不更新ngpath，feasible_extensions_byngpath和augment_ornot不变
									//1：表示更新ngpath，ngpath_num和passnode_ngpath，重置feasible_extensions_byngpath并将augment_ornot重置为false
#define UPBOUND 0					//在分支树上对上界的操作
									//0：在每个分支上CG结束后不调用CPLEX求一个整数解
									//1：在每个分支上CG结束后调用CPLEX求一个整数解
#define PRICING 0					//对子问题的求解策略
									//0：每次迭代依次对Depot_Num个子问题求解，直到每个depot的子问题都不返回列
									//1：优先对一个depot的子问题求解，直到该depot的子问题不返回列，再转到下一个depot求解
#define BRANCHVEH 0					//是否对车辆数分支
									//0：表示分支过程不考虑车辆数分支
									//1：表示分支过程考虑对车辆数分支
#define COMPLETION 0				//completion bound更新策略
									//0：顺次更新
									//1：间隔更新
#define ADDCOL 0					//向列池中添加列的策略
									//0：只添加一个
									//1：同时添加所有场站出发的可行路径
#define EXACTPSP 0					//运算时间影响重要因素：PSP是否求解到理论最优
									//0：非最优
									//1：理论最优
#define DSSRNG 1					//EPO-dssr框架下的ngset初始化策略
									//0：不初始化
									//1：根据上一次迭代，初始化ngset
#define INHERIT 1					//分支过程中，是否继承上一代的ngset
									//0：不继承
									//1：继承
#define SDCindicator 1				//在RMP的线性松弛解中，通过什么指标寻找SDC
									//0：通过节点重复次数
									//1：通过SDC约束违反程度
#define CustomerToSDC 0				//在寻找哪些客户点能够加入SDC时，采取的寻找策略
									//0：不做限制
									//1：一次迭代中，一个加入SDC的客户点ng-neighbourhood的客户点都被禁止加入SDC
#define SDCTYPE 1					//Frameworks==1时，SDC的种类
									//0：SDC限制任何cycle
									//1：SDC限制拓展ngset下的ng-infeasible cycle
#define QUICKBranch 1				//深度分支时的策略
									//0：最优两支中选择LB_value最小的支
									//1：直接选左支
#define EXACTEXTEND 1				//PSP中的拓展策略
									//0：启发式拓展：只有c_ij+pi_j<0时，才从i拓展到j
									//1：精确拓展

class Conf
{
public:
	//成本相关的
	static float unit_cost;					//每单位距离的成本
	//时间相关的
	static float unit_time;					//每单位距离的行驶时间
	//易腐货物相关的
	static float decay_rate_travel;			//Decay rate during traveling (item/(hour×item))
	static float decay_rate_load;			//Decay rate during serving customers (item/(hour×item))
	static float unit_cargoCost;			//单位货物价值 ($/item)
	//冷藏车相关的
	static float thermal_load_travel;		//Thermal load during traveling (kcal /h)
	static float thermal_load_load;			//Thermal load during serving customers (kcal /h)
	static float unit_kcalCost;				//单位kcal能源成本 ($/kcal)


	//问题类型相关的
	static int cost_type;					//0表示经典的路径成本；1表示考虑客户满意度的惩罚成本
	static int descent_strategy;			//0表示加入最快下降路径，1表示加入可行最快下降路径，2表示0和1两者均加入，3表示所有非支配路径均加入

	//列池相关的
	static int MAX_COLUMN_IN_RMP;			//列池的最大容量(在RMP问题中最多存在的列数),多了报错
	static int MAX_PASS_NUM;				//pre_arcs或者succ_arcs存储节点的最大数量
	static int MAX_PSP_SIZE;				//在PSP中，能够存储找到的路径的最多数量，在允许PSP中一起加入很多列时有用
	static int MAX_TEMPORARY_inPSP;			//常数，为了找出非退化解，提前存储路径的最多数量
	static int MAX_M3_CPLEX;                //常数，为求解M3对应的基础解系，设置的一个额外约束
	static int MAX_SHOW;					//常数，代表最多存储的500次迭代的计算结果
	static int MAX_NODES;					//常数，VRP问题路径最多经过客户数（包括起终点depot）
	static int MAX_VECTOR;					//常数，子问题中SDCnode的最大长度
	static int EACH_LONG_NUM;				//常数，EACH_LONG_NUM:长整可以容纳的位数


	//定价子问题相关的
	static int MAX_PATH_CONTAINER;			//常数，path_container中label的最大数量
	static int MAX_PATH_NUM;				//常数，每个节点的线路集数量。每个节点的状态集合，当计算过程中，超过这个值，会有提示要求增加该值
	static bool Not_DynamicBound;			//表示是否采用动态资源定界，如果为true为动态，否则给一个常数0.58	
	static int MAX_NEIGHBOURHOOD_NUM;		//常数，ng-route中领域的大小
	static int CYCLE_NUM;					//考虑的k-cycle的k大小
	static float Exact_threshold;			//PSP中遍历定义域的终止阈值
	static float Remain_proIni;				//启发式DP的消减网络初始规模，0-1之间

	//分支相关的
	static int MAX_BRANCH_NODES;			//常数，能够分支的最大数量
	static int MAX_ADD;						//常数，在分支树上每个节点上每次最多加入的约束数量
	static float MIN_GAP;					//常数，branch-and-bound终止时的最小gap值
	static float PRE_BOUND;
	static float heuristic_BOUND;

	//与cut相关的
	static int MAX_ADD_SDC;					//常数，每次RMP求解后，最多能添加的SDCcuts的数量

	static int MAX_SR_SET;					//SR-cut的集合C的最大势，例如3,4,5,；最大不超过5
											//|C|=3时，遍历；|C|>=4时，C中任意两点距离不超多定长MAX_SR_DISTANCE
	static float MAX_SR_DISTANCE;			//|C|>=4时，C中任意两点的最长距离
	static float MIN_SR_THRESHOLD;			//超过最小违反程度的SR-path才会被加入RMP
	static int MAX_ADD_SR;					//常数，每次重新优化，最多能添加的SR-cut的数量
	static int MAX_SR_NUM;					//常数，RMP中最多能够添加的SR-cut的数量
	static float SR_MULTIPY_3;				//|C|=3时,SR-cut不等式左边的系数
	static float SR_MULTIPY_4;				//|C|=4时,SR-cut不等式左边的系数
	static float SR_MULTIPY_5;				//|C|=5时,SR-cut不等式左边的系数

	static int MAX_ROBUST_DEPTH;			//表示RMP中加入robust-cut的分支节点最大序号，一般为2两层，即7

	static int MAX_KPATH_NUM;				//常数，RMP中最多能够添加的KPATH的数量
	static int MAX_ADD_KPATH;				//常数，每次重新优化，最多能添加的KPATH的数量
	static int MAX_SUBSET_KPATH;			//k-path cut中subset的最大包含节点数量
	static float MIN_KPATH_THRESHOLD;		//超过最小违反程度的k-path才会被加入RMP

	static int MAX_RCC_NUM;					//常数，RMP中最多能够添加的RCC的数量
	static int MAX_ADD_RCC;					//常数，每次重新优化，最多能添加的RCC的数量
	static int MAX_SUBSET_RCC;				//RCC中subset的最大包含节点数量
	static float MIN_RCC_THRESHOLD;			//超过最小违反程度的RCC才会被加入RMP
	static float MAX_RCC_RHS;				//RCC有效不等式右侧常数值的最大值
};


#endif /* CONF_H_ */