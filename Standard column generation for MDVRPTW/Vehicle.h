#ifndef VEHICLE_H_
#define VEHICLE_H_
class Vehicle
{
public:
	Vehicle();
	virtual ~Vehicle();

	float Veh_Capacity; //VRPTW问题的车辆最大载货能力
	float Duration; //一条路径上车辆的最长服务时间
};

#endif 