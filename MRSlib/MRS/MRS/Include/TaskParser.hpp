#pragma once
#include "Task.hpp"
#include "ATask.hpp"
#include "Behaviour.hpp"
#include "ParametricATask.hpp"
#include "ProtocolParser.hpp"
namespace MRS {
	namespace Task {
		class TaskParser {
		public:
			static void SetAttributesFromString(std::string, std::size_t, Task*);
			static void SetAttributesFromString(std::string, std::size_t, ATask*);
			static void SetAttributesFromString(std::string, std::size_t, Behaviour*);
			static void SetAttributesFromString(std::string, std::size_t, ParametricATask*);

		};
	}
}