#include "ViewPosition2D.h"



ViewPosition2D::ViewPosition2D() : View(TypeDefinitions::VIEW_POSITION_2D)
{
	x = 0.0f;
	y = 0.0f;
	a = 0.0f;
}

ViewPosition2D::ViewPosition2D(float x, float y) : ViewPosition2D(x,y,0.0f)
{
}


ViewPosition2D::ViewPosition2D(float x, float y, float a) : View(TypeDefinitions::VIEW_POSITION_2D)
{
	SetX(x);
	SetY(y);
	SetA(a);
}

ViewPosition2D::~ViewPosition2D()
{
}

int ViewPosition2D::GetAspect(int, TypeDefinitions::ViewType, char[])
{
	return 0;
}

void ViewPosition2D::SetX(float x)
{
	this->x = x;
}

void ViewPosition2D::SetY(float y)
{
	this->y = y;
}

void ViewPosition2D::SetA(float a)
{
	this->a = a;
}

float ViewPosition2D::GetX()
{
	return x;
}

float ViewPosition2D::GetY()
{
	return y;
}

float ViewPosition2D::GetA()
{
	return a;
}
