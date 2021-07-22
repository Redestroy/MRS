#pragma once
#include "Agent.hpp"
#include "DeviceBlock.hpp"
#include "Task.hpp"
#include "BehaviourLibrary.hpp"
#include <fstream>
#include <iostream>

namespace MRS {
	namespace Task {
		class
#ifdef BUILD_DLL
			__declspec(dllexport)
#endif // BUILD_DLL
		Robot : public Agent {
		private:
			Device::DeviceBlock * devices;
			BehaviourLibrary * inbuilt_behaviours;
		public:
			//Flow
			Robot();
			virtual ~Robot();
			void Init();
			void Update();
			void Exit();
			virtual void DoAction(Device::Action) = 0; //TODO: get rid of void pointer
			
			virtual void execute(Task*) = 0;
			virtual void OnInit() = 0;
			virtual void OnUpdate() = 0;
			virtual void OnExit() = 0;

			//Container operators
			void CreateBehavioursFromFile(std::string filename);
			void AddBehaviour(Behaviour*);
			void AddBehaviour(std::string name, Behaviour* behaviour);

			//Setters
			void SetDevices(Device::DeviceBlock*);
			//Getters
			Device::DeviceBlock* GetDevices();		 // TODO: replace with shared pointer
			BehaviourLibrary* GetBehaviourLibrary(); // TODO: replace with shared pointer

		};

		template <class T>
		class RobotInfo {
		protected:
			std::map<std::string, std::string> robot_information;
			std::string filename = "robot.info";
		public:
			RobotInfo() : robot_information() {}

			~RobotInfo() {
				robot_information.clear();
			}

			std::string GetInformation(std::string parameter) {
				return robot_information.at(parameter);
			}

			virtual void GenerateInformation(T*) = 0;
			void OutputInformationToFile() {
				//TODO: add try and catch
				std::fstream f;
				f.open(filename, std::ios::out);
				//TODO: add header
				//TODO: add sorted key format file for better formatting
				for(auto info : robot_information) f << info.first << " : " << info.second << std::endl;
				f.close();
			}
			void OutputInformationToFile(std::string file) {
				filename = file;
				OutputInformationToFile();
			}
			void OutputInformationToFile(std::string file, std::string path) {
				filename = path + file;
				OutputInformationToFile();
			}
		};


		class DummyRobot : public Robot {
		private:
			class DummyRobotInfo : public RobotInfo<DummyRobot> {
			private:
			public:
				void GenerateInformation(DummyRobot* robot) {
					filename = "dummy_robot.info"; // TODO: add seperate method that adds extension automatically
					//populate map with robot information
					//TODO: optimize so the entire information does not need to be regenerated each time
					robot_information.clear();
					robot_information.emplace("Test data ", "Data values");
					robot_information.emplace("Key", "val");
					robot_information.emplace("Empty val line ", "");
				}

			};
			DummyRobotInfo robot_information;
		public:
			DummyRobot() : robot_information() {
				std::cout << "Dummy robot created " << std::endl;
			}
			virtual ~DummyRobot() {
				std::cout << "Dummy robot destroyed " << std::endl;
			}
			
			virtual void OnInit() {
				robot_information.GenerateInformation(this);
				std::cout << "Initializing " << std::endl;
			}
			virtual void OnUpdate() {
				std::cout << "Updating " << std::endl;
			}

			void DoAction(Device::Action action) {
				std::cout << "Doing action: " << action.toString() << " " << std::endl;
			}

			void execute(Task* t) {
				if (t != nullptr) {
					std::cout << "Excecuting task: " << t->GetID() << " with priority " << t->GetPriority() << " " << std::endl;
				}
				else {
					std::cout << "Nullptr task received" << std::endl;
				}
			}

			void OnExit() {}

		};

		

		
		
	}
}