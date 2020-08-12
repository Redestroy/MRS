#include "View.h"



void View::SetType(TypeDefinitions::ViewType type)
{
	this->type = type;
}

View::View()
{
}

View::View(TypeDefinitions::ViewType type)
{
	this->type = type;
}


View::~View()
{
}

void View::SetData(unsigned char pos, unsigned char data)
{
	if (pos < 256) this->data[pos] = data;
}

char View::GetData(unsigned char pos)
{
	if (pos < 256) return data[pos];
	return -1;
}

void View::onCreate(long zeroTime)
{
	//timeCreated = millis() - zeroTime;
}
