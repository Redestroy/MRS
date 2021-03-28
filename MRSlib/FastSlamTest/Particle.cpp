#include "Particle.h"
#include <cstdio>
#include <iostream>
namespace MRS{

	int Particle::number_of_particles = 0;

	Particle::Particle() : position(),  objectIds(), objects()
	{
		id = number_of_particles;
		number_of_particles++;
		me_size = 0;
		worth = 0.1;
	}

	Particle::Particle(Particle* p)
	{
		this->Copy(p);
		id = number_of_particles;
		number_of_particles++;
	}

	Particle::~Particle()
	{
		//std::cout << "Particle deleted "<< id <<"\n";
	}

	void Particle::SetPosition(MRS::Device::View2DPosition * pos)
	{
		//printf("Position of p arg: x %f y %f a %f \n", pos->GetX(), pos->GetY(), pos->GetA());
		this->position = *pos;
		//printf("Position of p res: x %f y %f a %f \n", position.GetX(), position.GetY(), position.GetA());
		if (objects.find(0) == objects.end()) {
			me_size++;
			objectIds.push_back(0);
			auto pair = objects.emplace(0, position);
			//printf("Is inserted: %c \n", pair.second ? 'Y' : 'N');
		}
		else {
			objects[0] = position;
		}
	}

	MRS::Device::View2DPosition * Particle::GetPosition()
	{
		return &position;
	}

	std::deque<int> * Particle::GetObjectIds()
	{
		return &objectIds;
	}

	std::map<int, MRS::Device::View2DPosition> * Particle::GetObjects()
	{
		return &objects;
	}

	int Particle::AddObject(int id, MRS::Device::View2DPosition* obj_pos)
	{
		if (id == 0) return 0;
		else if (id < 0) return -1;
		else {
			if (objects.find(id)->first != id) {
				//printf("placeing object\n");
				auto pair = objects.emplace(id, *obj_pos);
				//printf("Is inserted: %c \n", pair.second ? 'Y' : 'N');
				objectIds.push_back(id);
				//me_size++;
				me_size = objects.size();
				/*
				printf("-------------------------%d \n",me_size);
				for (auto obj : objects) {
					printf("Obj info: id %d posX %f posY %f posZ %f \n", obj.first, obj.second.GetX(), obj.second.GetY(), obj.second.GetA());
				}
				printf("------------------------- \n");
				*/
				return (int)objects.size();
			}
			else {
				return -2;
			}
		}
	}

	int Particle::RemoveObject(int id)
	{
		if (id == 0) return 0;
		else if (id < 0) return -1;
		else {
			if (FindByValue(id) != objectIds.end()) {
				printf("removing object\n");
				objects.erase(id);
				objectIds.erase(FindByValue(id));
				me_size = objects.size();
				return (int)objects.size();
			}
			else {
				return -2;
			}
		}
	}

	int Particle::Clear()
	{
		me_size = 0;
		objects.clear();
		return (int)objects.size();
	}

	double Particle::GetBestFit(MRS::Device::DistanceView* v)
	{
		//Returns minimum object distance
		printf("X distance : %f", v->ToCartesianGetX());
		MRS::Device::View2DPosition obj_position = position + MRS::Device::View2DPosition(v->ToCartesianGetX(), v->ToCartesianGetY());
		double min_distance = __DBL_MAX__;
		double temp = 0;
		//printf("Checking objects:");
		for (auto obj : objects) {
			temp = abs(obj.second.GetX() - obj_position.GetX());
			//printf("Current: %f vs Best %f", temp, min_distance);
			if (temp < min_distance && obj.first != 0) {
				min_distance = temp;
			}
		}
		return min_distance;
	}

	void Particle::UpdateObjectPositions()
	{
		// Not neccesary
		/*
		for (auto obj : objects) {
			if (obj.first != 0) {
				obj.second = 
			}
		}*/
	}

	MRS::Device::View2DPosition* Particle::GetBestFitObject(MRS::Device::DistanceView* v)
	{
		MRS::Device::View2DPosition obj_position = position + MRS::Device::View2DPosition(v->ToCartesianGetX(), v->ToCartesianGetY());
		double min_distance = __DBL_MAX__;
		double temp = 0;
		MRS::Device::View2DPosition * bestObject = nullptr;
		for (auto obj : objects) {
			temp = obj.second.GetX() - obj_position.GetX();
			if (temp < min_distance && obj.first != 0) {
				min_distance = temp;
				bestObject = &(obj.second);
			}
		}
		return bestObject;
	}

	std::deque<int>::iterator Particle::FindByValue(int id)
	{
		for (auto obj = objectIds.begin(); obj != objectIds.end(); obj++) {
			if (*obj == id) {
				return obj;
			}
		}
		return objectIds.end();
	}

	int Particle::GetSize()
	{
		return me_size;
	}

	void Particle::SetWorth(double worth)
	{
		this->worth = worth;
	}
	double Particle::GetWorth()
	{
		return worth;
	}

	double Particle::CalculateWorth(double S, double Y)
	{
		double s = S;
		double y = Y;
		double y_t = y;
		printf("S %f Y %f W", s, y);
		worth = 1 / sqrt(fabs(2 * PI * s)) / sqrt(exp(y * y_t / s));
		printf("%f\n", worth);
		return worth;
	}
	void Particle::CopyObjectIds(std::deque<int>* ids)
	{
		/*
		objectIds.clear();
		for (auto id : *ids)
		{

		}*/
	}

	void Particle::CopyObjects(std::map<int, MRS::Device::View2DPosition>* objs)
	{
		Clear();
		for (auto obj : *objs)
		{
			AddObject(obj.first, &obj.second);		
		}
		me_size = objects.size();
	}
	std::string Particle::OutputParticleInfo()
	{
		std::string particle_info = "";
		//(particle id (pos x p_x) (obj size (o id_o x p_x) ...))
		particle_info.append("(particle ");
		particle_info.append(std::to_string(id));
		particle_info.append(" (pos ");
		double d = position.GetX();
		particle_info.append(std::to_string(d));
		particle_info.append(" ");
		d = position.GetP_X();
		particle_info.append(std::to_string(d));
		particle_info.append(") (obj ");
		particle_info.append(std::to_string(me_size));
		for (auto obj : objects) {
			particle_info.append(" (o ");
			particle_info.append(std::to_string(obj.first));
			particle_info.append(" ");
			d = obj.second.GetX();
			particle_info.append(std::to_string(d));
			particle_info.append(" ");
			d = obj.second.GetP_X();
			particle_info.append(std::to_string(d));
			particle_info.append(")");
		}
		particle_info.append("))\n");
		return particle_info;
	}
}