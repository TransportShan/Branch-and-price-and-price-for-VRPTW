// VRP_CG.cpp : �������̨Ӧ�ó������ڵ㡣
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//				         	Branch-and-cut-and-price-and-stable for MDVRPTW								  //
//ʵ���������ܣ�1��Cyclic paths allowed (CPA) framework��2��elementary paths only (EPO) framework		  //
//Cuts��1, Strong Degree Constraints																	  //
//��Ҫ���ף�Reaching the Elementary Lower Bound in the Vehicle Routing Problem with Time Windows		  //
//																										  //
//											2021-11-01													  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include <iostream>
#include <ctime>

#include "Conf.h"
#include "Columns.h"
#include "RMP.h"
#include "Problem.h"
#include "ColumnPool.h"
#include "Initialization.h"
#include "Column_Generation.h"
#include "Subproblem.h"
#include "BranchABound.h"
#include "Solution.h"
#include "Utility.h"
#include "Cut.h"


using namespace std;



int _tmain(int argc, _TCHAR* argv[])
{
	//����ʱ�����ӣ���������������������������Ҫʹ���������ֻ��Ҫ��main������һ������
	srand((unsigned)time(NULL));
	//���ȼ���matlab��dll�ļ��Ƿ�ɹ�����
	//if (!qiu_niInitialize())
	//{
	//	cout << "Couldnot initialize lib:inverse!" << endl;
	//	cin.get();
	//}
	//����㷨��ʱ�� 
	clock_t start;							//��¼�㷨�Ŀ�ʼʱ��
	clock_t	finish_read;					//��¼���ݶ���Ľ���ʱ��
	clock_t	finish_ini_column_pool;			//��¼��ʼ���гصĽ���ʱ��
	clock_t	iter_start;						//��֦����ÿ��node����⿪ʼʱ��
	clock_t iter_end;						//��֦����ÿ��node��������ʱ��
	clock_t end;							//��¼�㷨�Ľ���ʱ��
	//��¼�����Ĵ���
	int iter_num_node = 1;						//��¼��������
	//��������ʱ���ļ�������
	string data_file_name;					//��������ļ�
	string iniSolution_file_name;			//�����ʼ����ļ�
	start = clock();
	data_file_name = "./pr_data./mdvrptw02.csv";
	iniSolution_file_name = "./pr_data./ini_mdvrptw02.csv"; 

	//������
	Problem VRP;
	ColumnPool Pool;
	Initialization IniCol;
	RMP LP;
	Column_Generation CG;
	Subproblem PSP;
	BranchABound BB;
	SolINFOR SolI;
	Utility Results;

	SDC Cuts_sdc;
	SRC Cuts_src;
	KPATH Cuts_kpath;
	RCC Cuts_rcc;
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//�������ݹ���
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	VRP.ReadfromCSV(data_file_name);	//��������
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//�������ݽ���
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��¼��ȡ��������ʱ��
	finish_read = clock();
	cout << "��ȡ��������ʱ��: " << (float)(finish_read - start) / CLOCKS_PER_SEC << "S" << endl;
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��ʼ���г�
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	CG.Claim_CG();
	Pool.Ini(VRP);
	BB.Claim_Branch(VRP);
	IniCol.InitialColumns(iniSolution_file_name, BB,Pool,VRP);
	LP.Ini_RMP(VRP);
	PSP.Claim_PSP(VRP);
	SolI.Claim_Solution(VRP);
	Results.Ini_UTILITY();

#if Frameworks+AUGMENT == 2
	Cuts_sdc.Ini_SDC(VRP);
#endif
#if KPATHCUT == 1
	Cuts_kpath.Ini_KPATH(VRP);
#endif
#if RCCUT == 1
	Cuts_rcc.Ini_RCC(VRP);
#endif
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//�гس�ʼ������
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��¼��ʼ���г�����ʱ��
	finish_ini_column_pool = clock();
	cout << "��ʼ���г�����ʱ��: " << (float)(finish_ini_column_pool - finish_read) / CLOCKS_PER_SEC << "S" << endl;
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��ѭ����ʼ
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	do
	{
		//��¼ÿ�ε����Ŀ�ʼʱ��
		iter_start = clock();
		//ʹ��column generation���ڵ��ϵ�RMP
		CG.SolveNode_ByCG(Results, SolI,BB,IniCol,Pool, VRP, LP, PSP, Cuts_src, Cuts_kpath, Cuts_rcc, Cuts_sdc);
		//////////////////////////////////���¿�ʼ��֧////////////////////////////////
		//̽���Ƿ�õ�һ�������⣬������������ҳ����ĸ��������з�֧
		CG.Detect_branch(SolI, BB, Pool, VRP);
		//ȷ���÷�֧���״̬,���ж��Ƿ���ֹ�㷨
		if (true == CG.Check_state(SolI, BB, Pool, VRP))
		{
			break;
		}
		/////////////////ѡ���´ζ��ĸ�ĸ�ڵ���з�֧/////////////////////////
		//���������Ϊ��ʱ��ѡ����Է�֧��ĸ�ڵ�
		if (0 == BB.Toslove_num)
		{
			if (true == CG.Choose_branchNode(SolI, BB, Pool, VRP))
			{
				break;
			}
			//����������֧��,������������֧�������������
			if (true == CG.Branching(BB, VRP))
			{
				break;
			}
		}
		//ѡ����һ�����ķ�֧��
		BB.Choose_solveNode();
		//////////////////////////////////////////////////////////////////////

		//����ѡ�������֧�㣬�����г�
		CG.Update_used_column(BB,Pool, VRP);
		//��������
		CG.Update_network(BB,VRP);
		//��Ҫ�õ���ǰnode�ϵ�cut����Щ
#if SRCUT == 1
		//update_src(iter_num_node, branchnodes, temp_src);
#endif
		//���һ�����ͷ�����
		BB.releaseNode(VRP);

		//��¼һ�ε�������ʱ��
		iter_end = clock();
		cout << "��" << iter_num_node << "�η�֧����ʱ��: " << (float)(iter_end - iter_start) / CLOCKS_PER_SEC << "S" << endl;

		//��¼��������
		iter_num_node++;

	} while (1);

	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��ѭ������
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//��¼�㷨����ʱ��
	qiu_niTerminate();
	end = clock();
	cout << "�����㷨����ʱ��: " << (float)(end - start) / CLOCKS_PER_SEC << "S" << endl;
	cin.get();
	return 0;

}

