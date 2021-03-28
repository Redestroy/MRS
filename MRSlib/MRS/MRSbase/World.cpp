#include "pch.h"
#include "World.h"

World::World()
{
}

void World::Init()
{
}

void World::Add(Agent* agent)
{
	agents.emplace(agent_count++, agent);
}
/*
void World::Affect(Action)
{
}
*/
