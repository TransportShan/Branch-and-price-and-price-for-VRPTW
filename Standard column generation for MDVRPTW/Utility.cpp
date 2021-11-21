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
		//CPA����������Ż������һ��SDC���Ĵ���
		oFile << "CPA�����Ż�������" << ',' << Reopt_num << endl;
		//CG���ܵ�������
		oFile << "CG���ܵ���������" << ',' << CGitr_num << endl;
		//������ɵ���ʱ��
		oFile << "������ɵ���ʱ�䣺" << ',' << Total_time << endl;
		//CPA�����һ����ӵ�SDC����
		oFile << "��ӵ�SDC��������" << ',' << SDC_num << endl;

		//ÿ�μ���PSP�����ɵ�label����
		oFile << "ÿ�μ���PSP�����ɵ�label������" << endl;
		oFile << "����������" << ',';
		for (i = 0; i<PSP_label_num.size(); i++)
		{
			oFile << i+1 << ',';
		}
		oFile << endl;
		oFile << "��ֵ��" << ',';
		for (i = 0; i < PSP_label_num.size(); i++)
		{
			oFile << PSP_label_num[i] << ',';
		}
		oFile << endl;

		//ÿ�μ���PSP�ļ���ʱ��
		oFile << "ÿ�μ���PSP�ļ���ʱ�䣺" << endl;
		oFile << "����������" << ',';
		for (i = 0; i<PSP_time.size(); i++)
		{
			oFile << i + 1 << ',';
		}
		oFile << endl;
		oFile << "��ֵ��" << ',';
		for (i = 0; i < PSP_time.size(); i++)
		{
			oFile << PSP_time[i] << ',';
		}
		oFile << endl;

		//CPA�����ÿ�������Ż���ӵ�SDC���ڿͻ�
		oFile << "CPA�����ÿ�������Ż���ӵ�SDC���ڿͻ���" << endl;
		for (i = 0; i<SDC_eachitr.size(); i++)
		{
			oFile << "��" << ',' << i + 1 << "�Σ�" << ',';
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
