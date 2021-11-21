#include "stdafx.h"
#include "Columns.h"


Columns::Columns()
{	
}


Columns::~Columns()
{
}

void Columns::Update_cus_cyclePosi(int obj_cus)
{
#if Frameworks == 1
	//°Ñcus_cyclePosi[obj_cus][i]Îª-1µÄÌÞ³ý
	int temp_num = cus_cyclePosi_num[obj_cus] - 1;
	for (int i = temp_num; i >= 0; i--)
	{
		if (cus_cyclePosi[obj_cus][i]<0)
		{
			for (int j = i; j<cus_cyclePosi_num[obj_cus] - 1; j++)
			{
				cus_cyclePosi[obj_cus][j] = cus_cyclePosi[obj_cus][j + 1];
			}
			cus_cyclePosi_num[obj_cus] = cus_cyclePosi_num[obj_cus] - 1;
		}
	}
#endif
}
