// VRP_CG.cpp : 定义控制台应用程序的入口点。
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//				         	Branch-and-cut-and-price-and-stable for MDVRPTW								  //
//实现两个功能：1，Cyclic paths allowed (CPA) framework；2，elementary paths only (EPO) framework		  //
//Cuts：1, Strong Degree Constraints																	  //
//重要文献：Reaching the Elementary Lower Bound in the Vehicle Routing Problem with Time Windows		  //
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
	//设置时间种子，无论是主函数还是其他函数体要使用随机数，只需要在main中设置一次种子
	srand((unsigned)time(NULL));
	//首先检验matlab的dll文件是否成功生成
	//if (!qiu_niInitialize())
	//{
	//	cout << "Couldnot initialize lib:inverse!" << endl;
	//	cin.get();
	//}
	//监控算法的时间 
	clock_t start;							//记录算法的开始时间
	clock_t	finish_read;					//记录数据读入的结束时间
	clock_t	finish_ini_column_pool;			//记录初始化列池的结束时间
	clock_t	iter_start;						//分枝数上每个node的求解开始时间
	clock_t iter_end;						//分枝数上每个node的求解结束时间
	clock_t end;							//记录算法的结束时间
	//记录迭代的次数
	int iter_num_node = 1;						//记录迭代次数
	//输入数据时，文件名参数
	string data_file_name;					//输入参数文件
	string iniSolution_file_name;			//输入初始解的文件
	start = clock();
	data_file_name = "./pr_data./mdvrptw02.csv";
	iniSolution_file_name = "./pr_data./ini_mdvrptw02.csv"; 

	//声明类
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
	//读入数据过程
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	VRP.ReadfromCSV(data_file_name);	//构建网络
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//读入数据结束
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//记录读取数据所用时间
	finish_read = clock();
	cout << "读取数据所用时间: " << (float)(finish_read - start) / CLOCKS_PER_SEC << "S" << endl;
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//初始化列池
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
	//列池初始化结束
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//记录初始化列池所用时间
	finish_ini_column_pool = clock();
	cout << "初始化列池所用时间: " << (float)(finish_ini_column_pool - finish_read) / CLOCKS_PER_SEC << "S" << endl;
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//主循环开始
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	do
	{
		//记录每次迭代的开始时间
		iter_start = clock();
		//使用column generation求解节点上的RMP
		CG.SolveNode_ByCG(Results, SolI,BB,IniCol,Pool, VRP, LP, PSP, Cuts_src, Cuts_kpath, Cuts_rcc, Cuts_sdc);
		//////////////////////////////////以下开始分支////////////////////////////////
		//探查是否得到一个整数解，如果非整数解找出对哪个变量进行分支
		CG.Detect_branch(SolI, BB, Pool, VRP);
		//确定该分支点的状态,并判断是否终止算法
		if (true == CG.Check_state(SolI, BB, Pool, VRP))
		{
			break;
		}
		/////////////////选择下次对哪个母节点进行分支/////////////////////////
		//待求解序列为空时，选择可以分支的母节点
		if (0 == BB.Toslove_num)
		{
			if (true == CG.Choose_branchNode(SolI, BB, Pool, VRP))
			{
				break;
			}
			//产生两个分支点,并将这两个分支点加入待求解序列
			if (true == CG.Branching(BB, VRP))
			{
				break;
			}
		}
		//选择下一个求解的分支点
		BB.Choose_solveNode();
		//////////////////////////////////////////////////////////////////////

		//根据选择的求解分支点，更新列池
		CG.Update_used_column(BB,Pool, VRP);
		//更新网络
		CG.Update_network(BB,VRP);
		//需要得到当前node上的cut是哪些
#if SRCUT == 1
		//update_src(iter_num_node, branchnodes, temp_src);
#endif
		//最后一步是释放网络
		BB.releaseNode(VRP);

		//记录一次迭代结束时间
		iter_end = clock();
		cout << "第" << iter_num_node << "次分支所用时间: " << (float)(iter_end - iter_start) / CLOCKS_PER_SEC << "S" << endl;

		//记录迭代次数
		iter_num_node++;

	} while (1);

	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//主循环结束
	//----------------------------------------------------------------------------------------------------------------------------------------------------------
	//记录算法结束时间
	qiu_niTerminate();
	end = clock();
	cout << "整个算法所用时间: " << (float)(end - start) / CLOCKS_PER_SEC << "S" << endl;
	cin.get();
	return 0;

}

