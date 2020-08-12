#pragma once
#include "TypeDefinitions.h"
#include <string>
#include <iostream>
class MessageTranslator
{
public:
	MessageTranslator();
	~MessageTranslator();
	static bool isValid(std::string *, int);
	static bool isValidCBAA(std::string *, int);
	static TypeDefinitions::MessageType getMessageType(char);
	static TypeDefinitions::MessageType_CBAA getMessageTypeCBAA(char);
	static int getTaskIndex(std::string *);
	static int getRobotId(std::string *);
	//static int getTaskIndex(std::string *);
};

