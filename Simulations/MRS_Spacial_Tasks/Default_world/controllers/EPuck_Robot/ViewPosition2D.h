#pragma once
#include "View.h"
class ViewPosition2D :
	public View
{
	float x,y,a;
public:
	ViewPosition2D();
	ViewPosition2D(float, float);
	ViewPosition2D(float,float,float);
	~ViewPosition2D();
	int GetAspect(int, TypeDefinitions::ViewType, char[]);
	void SetX(float);
	void SetY(float);
	void SetA(float);
	float GetX();
	float GetY();
	float GetA();
};

