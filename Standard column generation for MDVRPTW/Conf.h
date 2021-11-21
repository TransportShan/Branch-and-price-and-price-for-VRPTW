#ifndef CONF_H_
#define CONF_H_

#include <string>


#define MAXCELLLEN  102400			//��ȡCSV����Ҫ�ĳ�������CSV�ļ�������û���ر��ʱ���ó������ø�
#define MAXNUM  9999999				//����һ���������
#define MINNUM	-9999999			//����һ������С��
#define MINDOUBLE 0.0001			//����һ���ȼ���0������

#define DURATIONORNOT 0				//��Լ�����Ƿ���duration
									//0��������duration
									//1������duration
#define Frameworks	0				//Ӧ��EPO����CPA���
									//0��EPO���
									//1��CPA��ܣ���Frameworks=1ʱ��EXACTPSPֻ��Ϊ0�����ܵ���DSSR��ܣ�
#define AUGMENT 1					//����CPA������������elementary constraints
									//0��ͨ������ngset
									//1��ͨ�����SDCԼ��
#define ClearColumn 1				//��AUGMENT==0ʱ������ngset���Ƿ����cycleɾ���г��е�ng-infeasible��
									//0����ɾ��
									//1��ɾ��
#define SRCUT 1						//0: ������subset row-cut, 1: ����
#define KPATHCUT 0					//0: ������kpath-cut, 1: ����
#define RCCUT 0						//0: ������rounded capacaity-cut, 1: ����
#define NGINI 1						//��PSP�г�ʼ��ngset����
									//0����̬��ʼ����ֻ��ÿ��customer��ngset��ѡ��
									//1����̬��ʼ�����ڵ�ǰ������ѡ����С��MAX_NEIGHBOURHOOD_NUM�ڽ���
#define FEASIBLE 1					//�����չ����
									//0��ֻ���Ƕ�żֵ
									//1����żֵ�����ϣ����ǿ�����չ��
#define NGDOI 1						//֧������е�ngpathʹ�ò���
									//0����ng��ȫ֧��Ϊ��
									//1����multi-ng֧��Ϊ��
#define ORDERLABEL 1				//0: ������, 1: ��PSP�ϲ��׶���������
#define FOUND 1						//��PSP�ϲ�label�׶εĲ�ͬ���ԣ�
									//0��ֻ����ǰMAX_PSP_SIZE���ҵ���RCֵС��0��elementary·��
									//1����ÿ��customer�϶��ҵ�ǰ����ؼ���Դgapֵ��С��·��
									//2���ҵ�Paretoǰ��
									//3���ҵ�RCֵ��С��elementary·��
#define SEARCH 1					//��PSP��չ�׶��У���������
									//0������customer˳������
									//1������customer��ʼʱ�䴰������˳������
#define TCYCLE 2					//2-cycle֧��Ĳ���
									//0��������2-cycle����
									//1����ȷ���ԣ�2-cycle������ngpath�������ж�
									//2������ʽ���ԣ�2-cycle��ngpath��������
#define NEWNG 1						//DSSR-ng����µĸ���neset��label��ngpath�ĸ��¹���
									//0����ʾ������ngpath��feasible_extensions_byngpath��augment_ornot����
									//1����ʾ����ngpath��ngpath_num��passnode_ngpath������feasible_extensions_byngpath����augment_ornot����Ϊfalse
#define UPBOUND 0					//�ڷ�֧���϶��Ͻ�Ĳ���
									//0����ÿ����֧��CG�����󲻵���CPLEX��һ��������
									//1����ÿ����֧��CG���������CPLEX��һ��������
#define PRICING 0					//���������������
									//0��ÿ�ε������ζ�Depot_Num����������⣬ֱ��ÿ��depot�������ⶼ��������
									//1�����ȶ�һ��depot����������⣬ֱ����depot�������ⲻ�����У���ת����һ��depot���
#define BRANCHVEH 0					//�Ƿ�Գ�������֧
									//0����ʾ��֧���̲����ǳ�������֧
									//1����ʾ��֧���̿��ǶԳ�������֧
#define COMPLETION 0				//completion bound���²���
									//0��˳�θ���
									//1���������
#define ADDCOL 0					//���г�������еĲ���
									//0��ֻ���һ��
									//1��ͬʱ������г�վ�����Ŀ���·��
#define EXACTPSP 0					//����ʱ��Ӱ����Ҫ���أ�PSP�Ƿ���⵽��������
									//0��������
									//1����������
#define DSSRNG 1					//EPO-dssr����µ�ngset��ʼ������
									//0������ʼ��
									//1��������һ�ε�������ʼ��ngset
#define INHERIT 1					//��֧�����У��Ƿ�̳���һ����ngset
									//0�����̳�
									//1���̳�
#define SDCindicator 1				//��RMP�������ɳڽ��У�ͨ��ʲôָ��Ѱ��SDC
									//0��ͨ���ڵ��ظ�����
									//1��ͨ��SDCԼ��Υ���̶�
#define CustomerToSDC 0				//��Ѱ����Щ�ͻ����ܹ�����SDCʱ����ȡ��Ѱ�Ҳ���
									//0����������
									//1��һ�ε����У�һ������SDC�Ŀͻ���ng-neighbourhood�Ŀͻ��㶼����ֹ����SDC
#define SDCTYPE 1					//Frameworks==1ʱ��SDC������
									//0��SDC�����κ�cycle
									//1��SDC������չngset�µ�ng-infeasible cycle
#define QUICKBranch 1				//��ȷ�֧ʱ�Ĳ���
									//0��������֧��ѡ��LB_value��С��֧
									//1��ֱ��ѡ��֧
#define EXACTEXTEND 1				//PSP�е���չ����
									//0������ʽ��չ��ֻ��c_ij+pi_j<0ʱ���Ŵ�i��չ��j
									//1����ȷ��չ

class Conf
{
public:
	//�ɱ���ص�
	static float unit_cost;					//ÿ��λ����ĳɱ�
	//ʱ����ص�
	static float unit_time;					//ÿ��λ�������ʻʱ��
	//�׸�������ص�
	static float decay_rate_travel;			//Decay rate during traveling (item/(hour��item))
	static float decay_rate_load;			//Decay rate during serving customers (item/(hour��item))
	static float unit_cargoCost;			//��λ�����ֵ ($/item)
	//��س���ص�
	static float thermal_load_travel;		//Thermal load during traveling (kcal /h)
	static float thermal_load_load;			//Thermal load during serving customers (kcal /h)
	static float unit_kcalCost;				//��λkcal��Դ�ɱ� ($/kcal)


	//����������ص�
	static int cost_type;					//0��ʾ�����·���ɱ���1��ʾ���ǿͻ�����ȵĳͷ��ɱ�
	static int descent_strategy;			//0��ʾ��������½�·����1��ʾ�����������½�·����2��ʾ0��1���߾����룬3��ʾ���з�֧��·��������

	//�г���ص�
	static int MAX_COLUMN_IN_RMP;			//�гص��������(��RMP�����������ڵ�����),���˱���
	static int MAX_PASS_NUM;				//pre_arcs����succ_arcs�洢�ڵ���������
	static int MAX_PSP_SIZE;				//��PSP�У��ܹ��洢�ҵ���·�������������������PSP��һ�����ܶ���ʱ����
	static int MAX_TEMPORARY_inPSP;			//������Ϊ���ҳ����˻��⣬��ǰ�洢·�����������
	static int MAX_M3_CPLEX;                //������Ϊ���M3��Ӧ�Ļ�����ϵ�����õ�һ������Լ��
	static int MAX_SHOW;					//�������������洢��500�ε����ļ�����
	static int MAX_NODES;					//������VRP����·����ྭ���ͻ������������յ�depot��
	static int MAX_VECTOR;					//��������������SDCnode����󳤶�
	static int EACH_LONG_NUM;				//������EACH_LONG_NUM:�����������ɵ�λ��


	//������������ص�
	static int MAX_PATH_CONTAINER;			//������path_container��label���������
	static int MAX_PATH_NUM;				//������ÿ���ڵ����·��������ÿ���ڵ��״̬���ϣ�����������У��������ֵ��������ʾҪ�����Ӹ�ֵ
	static bool Not_DynamicBound;			//��ʾ�Ƿ���ö�̬��Դ���磬���ΪtrueΪ��̬�������һ������0.58	
	static int MAX_NEIGHBOURHOOD_NUM;		//������ng-route������Ĵ�С
	static int CYCLE_NUM;					//���ǵ�k-cycle��k��С
	static float Exact_threshold;			//PSP�б������������ֹ��ֵ
	static float Remain_proIni;				//����ʽDP�����������ʼ��ģ��0-1֮��

	//��֧��ص�
	static int MAX_BRANCH_NODES;			//�������ܹ���֧���������
	static int MAX_ADD;						//�������ڷ�֧����ÿ���ڵ���ÿ���������Լ������
	static float MIN_GAP;					//������branch-and-bound��ֹʱ����Сgapֵ
	static float PRE_BOUND;
	static float heuristic_BOUND;

	//��cut��ص�
	static int MAX_ADD_SDC;					//������ÿ��RMP�����������ӵ�SDCcuts������

	static int MAX_SR_SET;					//SR-cut�ļ���C������ƣ�����3,4,5,����󲻳���5
											//|C|=3ʱ��������|C|>=4ʱ��C������������벻���ඨ��MAX_SR_DISTANCE
	static float MAX_SR_DISTANCE;			//|C|>=4ʱ��C����������������
	static float MIN_SR_THRESHOLD;			//������СΥ���̶ȵ�SR-path�Żᱻ����RMP
	static int MAX_ADD_SR;					//������ÿ�������Ż����������ӵ�SR-cut������
	static int MAX_SR_NUM;					//������RMP������ܹ���ӵ�SR-cut������
	static float SR_MULTIPY_3;				//|C|=3ʱ,SR-cut����ʽ��ߵ�ϵ��
	static float SR_MULTIPY_4;				//|C|=4ʱ,SR-cut����ʽ��ߵ�ϵ��
	static float SR_MULTIPY_5;				//|C|=5ʱ,SR-cut����ʽ��ߵ�ϵ��

	static int MAX_ROBUST_DEPTH;			//��ʾRMP�м���robust-cut�ķ�֧�ڵ������ţ�һ��Ϊ2���㣬��7

	static int MAX_KPATH_NUM;				//������RMP������ܹ���ӵ�KPATH������
	static int MAX_ADD_KPATH;				//������ÿ�������Ż����������ӵ�KPATH������
	static int MAX_SUBSET_KPATH;			//k-path cut��subset���������ڵ�����
	static float MIN_KPATH_THRESHOLD;		//������СΥ���̶ȵ�k-path�Żᱻ����RMP

	static int MAX_RCC_NUM;					//������RMP������ܹ���ӵ�RCC������
	static int MAX_ADD_RCC;					//������ÿ�������Ż����������ӵ�RCC������
	static int MAX_SUBSET_RCC;				//RCC��subset���������ڵ�����
	static float MIN_RCC_THRESHOLD;			//������СΥ���̶ȵ�RCC�Żᱻ����RMP
	static float MAX_RCC_RHS;				//RCC��Ч����ʽ�Ҳೣ��ֵ�����ֵ
};


#endif /* CONF_H_ */