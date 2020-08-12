#pragma once
#include "View.h"
class DistanceView :
	public View
{
	double distances[16];
public:
	DistanceView();
	DistanceView(View*);
	~DistanceView();
	void SetDistance(unsigned char, double);
	double GetDistance(unsigned char);
};

