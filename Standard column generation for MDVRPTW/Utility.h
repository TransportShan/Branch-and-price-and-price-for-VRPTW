#ifndef UTILITY_H_
#define UTILITY_H_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

//������ֲ���ָ�������������޷���ǰ֪��CG�ĵ��������������vector����洢����̬��������ռ�
class Utility
{
public:
	Utility();
	virtual ~Utility();

	void Ini_UTILITY(void);
	void Reset_UTILITY(void);			//���vector���������տռ�

	int Show_RootNode_results(void);
public:
	int Reopt_num;						//rootnode�У�CPA����������Ż������һ��SDC���Ĵ���
	int CGitr_num;						//rootnode�У�CG���ܵ���������ÿ����һ��RMP����CGitr_num
	float Total_time;					//rootnode������ɵ���ʱ��
	int SDC_num;						//rootnode�У�CPA�����һ����ӵ�SDC����

	vector<int> PSP_label_num;			//rootnode�У�ÿ��PSP�����ɵ�label������=ǰ��+����Ԥ���ռ�Ϊ2000
	vector<float> PSP_time;				//rootnode�У�ÿ��PSP�ļ���ʱ�䣬Ԥ���ռ�Ϊ2000
	vector<vector<int>> SDC_eachitr;	//rootnode�У�CPA�����ÿ�������Ż���ӵ�SDC���ڿͻ������ţ�ÿ����ӵ�SDC����������ÿ�еĳ��ȣ�Ԥ���ռ�Ϊ100*100

	//�������ʱ���ļ�������
	string RootNode_file_name;
};

#endif