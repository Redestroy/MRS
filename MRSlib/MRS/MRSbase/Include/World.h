#pragma once
#include <map>
#include "Agent.h"
class World
{
	int agent_count;
	std::map<int, Agent*> agents;
protected:
public:
	World();
	void Init();
	void Add(Agent*);
	void Affect(Action);
};

