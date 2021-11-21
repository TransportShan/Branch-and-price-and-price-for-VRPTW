#ifndef UTILITY_H_
#define UTILITY_H_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

//输出各种参数指标放在这里，由于无法提前知道CG的迭代次数，因此用vector数组存储，动态增加数组空间
class Utility
{
public:
	Utility();
	virtual ~Utility();

	void Ini_UTILITY(void);
	void Reset_UTILITY(void);			//清空vector，但不回收空间

	int Show_RootNode_results(void);
public:
	int Reopt_num;						//rootnode中，CPA框架下重新优化（添加一组SDC）的次数
	int CGitr_num;						//rootnode中，CG的总迭代次数，每计算一次RMP计入CGitr_num
	float Total_time;					//rootnode计算完成的总时间
	int SDC_num;						//rootnode中，CPA框架下一共添加的SDC数量

	vector<int> PSP_label_num;			//rootnode中，每个PSP中生成的label总数量=前向+反向，预留空间为2000
	vector<float> PSP_time;				//rootnode中，每个PSP的计算时间，预留空间为2000
	vector<vector<int>> SDC_eachitr;	//rootnode中，CPA框架下每次重新优化添加的SDC所在客户点的序号，每次添加的SDC的数量就是每行的长度，预留空间为100*100

	//输出数据时，文件名参数
	string RootNode_file_name;
};

#endif