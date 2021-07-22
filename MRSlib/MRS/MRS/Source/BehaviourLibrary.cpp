#include "../Include/BehaviourLibrary.hpp"

namespace MRS {
	namespace Task {
		
		BehaviourLibrary::BehaviourLibrary() : library()
		{
		}

		BehaviourLibrary::~BehaviourLibrary()
		{
			//delete all behaviours
			for (auto b : library) {
				if(b.second != nullptr) delete b.second;
			}
			library.clear();
		}

		void BehaviourLibrary::Add(std::string name, Behaviour* behaviour)
		{
			library.emplace(name, behaviour);
		}

		void BehaviourLibrary::Add(std::string line)
		{
			//Add(ProtocolParser::GetKey(line), new Behaviour(ProtocolParser::GetValue(line)));
		}

		void BehaviourLibrary::Populate(std::string block)
		{
			// tokanize block in lines
			// Add each line to library
			std::string token;
			//while (token = ProtocolParser::GetLine() != "") {
			//	Add(token);
			//}
		}

		void BehaviourLibrary::PopulateFromFile(std::string filename)
		{
			std::fstream f;
			f.open(filename, std::ios::in);
			std::string token;
			while (getline(f, token)) {
				Add(token);
			}
			f.close();
		}

		std::map<std::string, Behaviour*>::iterator BehaviourLibrary::end()
		{
			return library.end();
		}

		std::map<std::string, Behaviour*>::iterator BehaviourLibrary::begin()
		{
			return library.begin();
		}

		void BehaviourLibrary::emplace(std::string name, Behaviour* behaviour)
		{
			Add(name, behaviour);
		}

		void BehaviourLibrary::emplace(std::string line)
		{
			Add(line);
		}

	}
}