#include "stdafx.h"
#include "Utility.h"

Utility::Utility()
{
}

Utility::~Utility()
{
}

void Utility::Ini_UTILITY(void)
{
	Reopt_num = 0;
	CGitr_num = 0;
	Total_time = 0;
	SDC_num = 0;

	PSP_label_num.reserve(2000);
	PSP_time.reserve(2000);
	SDC_eachitr.reserve(100);

	RootNode_file_name = "./result./RootNode_performance.csv";
}

void Utility::Reset_UTILITY(void)
{
	Reopt_num = 0;
	CGitr_num = 0;
	Total_time = 0;
	SDC_num = 0;

	PSP_label_num.clear();
	PSP_time.clear();

	for (int i = 0; i < SDC_eachitr.size(); i++)
	{
		SDC_eachitr[i].clear();
	}
	SDC_eachitr.clear();
}

int Utility::Show_RootNode_results(void)
{
	ofstream oFile;
	int i, j;

	oFile.open(RootNode_file_name.c_str(), ios::out | ios::trunc);

	if (!oFile.is_open())
	{
		cerr << "Can't open" << RootNode_file_name << "for write.\n";
		return 0;
	}
	else
	{
		//CPA框架下重新优化（添加一组SDC）的次数
		oFile << "CPA重新优化次数：" << ',' << Reopt_num << endl;
		//CG的总迭代次数
		oFile << "CG的总迭代次数：" << ',' << CGitr_num << endl;
		//计算完成的总时间
		oFile << "计算完成的总时间：" << ',' << Total_time << endl;
		//CPA框架下一共添加的SDC数量
		oFile << "添加的SDC总数量：" << ',' << SDC_num << endl;

		//每次计算PSP中生成的label数量
		oFile << "每次计算PSP中生成的label数量：" << endl;
		oFile << "迭代次数：" << ',';
		for (i = 0; i<PSP_label_num.size(); i++)
		{
			oFile << i+1 << ',';
		}
		oFile << endl;
		oFile << "数值：" << ',';
		for (i = 0; i < PSP_label_num.size(); i++)
		{
			oFile << PSP_label_num[i] << ',';
		}
		oFile << endl;

		//每次计算PSP的计算时间
		oFile << "每次计算PSP的计算时间：" << endl;
		oFile << "迭代次数：" << ',';
		for (i = 0; i<PSP_time.size(); i++)
		{
			oFile << i + 1 << ',';
		}
		oFile << endl;
		oFile << "数值：" << ',';
		for (i = 0; i < PSP_time.size(); i++)
		{
			oFile << PSP_time[i] << ',';
		}
		oFile << endl;

		//CPA框架下每次重新优化添加的SDC所在客户
		oFile << "CPA框架下每次重新优化添加的SDC所在客户：" << endl;
		for (i = 0; i<SDC_eachitr.size(); i++)
		{
			oFile << "第" << ',' << i + 1 << "次：" << ',';
			for (j = 0; j < SDC_eachitr[i].size(); j++)
			{
				oFile << SDC_eachitr[i][j]+1 << ',';
			}
			oFile << endl;
		}

		oFile.close();
		return 1;
	}
}
