#pragma once
#ifndef VIEW
#define VIEW

#include "TypeDefinitions.h"
class View
{
private:
	TypeDefinitions::ViewType type;
	long timeCreated;
	unsigned char data[256];

protected:
	void SetType(TypeDefinitions::ViewType);

public:
	View();
	View(TypeDefinitions::ViewType);
	~View();

	TypeDefinitions::ViewType GetType() { 
		return type;
	};

	void SetData(unsigned char, unsigned char);
	char GetData(unsigned char);

	virtual void onCreate(long);
	
	//virtual int GetAspect(int, TypeDefinitions::ViewType, char[]) = 0; // Writes the data value of the view to the buffer, returns buffer size in bytes/, args: aspect id, aspect type, buffer in which aspect is written
 
};
#endif
