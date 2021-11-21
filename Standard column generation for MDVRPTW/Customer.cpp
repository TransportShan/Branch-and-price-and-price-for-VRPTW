#include "stdafx.h"
#include "Customer.h"


Customer::Customer()
{
}


Customer::~Customer()
{
}

void Customer::Copy(Customer fromCus)
{
	node_id=fromCus.node_id;
	x = fromCus.x;
	y = fromCus.y;
	servicetime = fromCus.servicetime;
	demand = fromCus.demand;
	startTW = fromCus.startTW;
	endTW = fromCus.endTW;
}
