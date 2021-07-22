#pragma once
#ifndef ATASK
#define ATASK
#include "Task.hpp"
#include "Action.hpp"
#include <vector>
#include <iostream>
namespace MRS {
	namespace Task {
		class ATask :
			public Task
		{
		private:
			std::vector<Device::Action> commands; // Need to think about choosing a data structure - can use an array
			std::vector<Device::Action>::iterator taskSize;
			void SetCondition(Condition*);
		public:
			ATask();
			ATask(size_t);
			ATask(size_t, unsigned char, float);
			ATask(size_t, unsigned char, float, Condition*);
			ATask(size_t, unsigned char, float, Condition*, Condition*);
			ATask(std::string*);
			~ATask();
			void Add(Device::Action action);
			void SetAttributesFromString(std::string*, size_t);
			Device::Action GetNextCommand();
			std::string* toString();
		};
	}
}
#endif

