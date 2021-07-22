#include "../Include/Robot.hpp"

namespace MRS {
	namespace Task {
		Robot::Robot() {
			devices = new Device::DeviceBlock();
			inbuilt_behaviours = new BehaviourLibrary();
			std::cout << "Robot created " << std::endl;
		}

		Robot::~Robot() {
			delete devices;
			delete inbuilt_behaviours;
			std::cout << "Robot destroyed: " << std::endl;
		}

		void Robot::Init() {
			std::cout << "Initializing: " << std::endl;
			OnInit();
		}

		void Robot::Exit()
		{
			OnExit();
		}

		void Robot::Update() {
			std::cout << "Updating sensors: " << std::endl;
		}

		void MRS::Task::Robot::CreateBehavioursFromFile(std::string filename)
		{
			inbuilt_behaviours->PopulateFromFile(filename);
			//std::fstream f;
			//TODO: add try catch
			
			/*
			f.open(filename, std::ios::in);
			std::string line;
			while (getline(f, line)) {
				// verify header
				if (line[0] = 'H' && line[2] == 'E') continue;
				else {
					// parse lines as behaviours and tasks
					inbuilt_behaviours->Add(line);
					// add tasks to inbuilt behaviour library
				}
			}
			f.close();
			*/
		}

		void MRS::Task::Robot::AddBehaviour(Behaviour* behaviour)
		{
			inbuilt_behaviours->Add(Behaviour::GetTypename(behaviour->GetBehaviourType()), behaviour);
		}

		void MRS::Task::Robot::AddBehaviour(std::string name,Behaviour* behaviour)
		{
			inbuilt_behaviours->Add(name, behaviour);
		}

		void Robot::SetDevices(Device::DeviceBlock* devices)
		{
			this->devices = devices;
		}

		Device::DeviceBlock* MRS::Task::Robot::GetDevices()
		{
			return devices;
		}

		BehaviourLibrary* MRS::Task::Robot::GetBehaviourLibrary()
		{
			return inbuilt_behaviours;
		}

	}
}