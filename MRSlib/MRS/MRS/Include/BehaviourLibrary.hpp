#pragma once
#include "Behaviour.hpp"
#include "ProtocolParser.hpp"
#include <map>
#include <fstream>

namespace MRS {
	namespace Task {
		class BehaviourLibrary {
		private:
			std::map<std::string, Behaviour*> library;
		public:
			BehaviourLibrary();
			~BehaviourLibrary();

			void Add(std::string name, Behaviour* behaviour);
			void Add(std::string line);
			void Populate(std::string block);
			void PopulateFromFile(std::string filename);

			std::map<std::string, Behaviour*>::iterator end();
			std::map<std::string, Behaviour*>::iterator begin();
			void emplace(std::string name, Behaviour* behaviour);
			void emplace(std::string line);
		};
	}
}