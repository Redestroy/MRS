#pragma once
#include "Device.hpp"
#include <deque>			//might need to implament seperatly
//#include "tree.h"

//TODO: create and iterator and fix insertion of new objects
namespace MRS {
	namespace Device {
		enum class DeviceTopology
		{
			LEAF,
			JOINT,
			HEAD
		};
		class
#ifdef BUILD_DLL
			__declspec(dllexport)
#endif // BUILD_DLL
			 DeviceTree //:
	//		public tree<Device>
		{
			//class Device;
		private:
			DeviceTree* root;
			int device_count;	//size of supertree
			int id;				//id in supertree
			DeviceTree* parent;
			//DeviceTree* current_tree; is basically this
			Device* current;
			int child_count;
			std::deque<DeviceTree*> children;
		public:
			DeviceTree();
			DeviceTree(DeviceTree* parent, DeviceTree* root, MRS::Device::Device* device);
			~DeviceTree();

			void Insert(MRS::Device::Device*, long deviceId, long parentId);
			Device* GetDevice( long deviceId); //Get device in the current subtree with local index
			DeviceTree* GetDeviceBranch(long deviceId);
			Device* GetDevice(); //TODO: Rename Device to GetDevice and GetDevice(int) as an overload
			int GetIndex();
			DeviceTree* GetRoot();

			void operator=(DeviceTree* tree) {
				this->current = tree->GetDevice();
			}

			void operator=(DeviceTree tree) {
				this->current = tree.GetDevice();
			}

			void operator=(MRS::Device::Device* dev) {
				this->current = dev;
			}

			void operator=(MRS::Device::Device dev) {
				this->current = &dev;
			}

			bool operator==(DeviceTree tree) {
				return this->id == tree.GetIndex() && this->root == tree.GetRoot();
			}

			bool operator==(DeviceTree* tree) {
				return this->id == tree->GetIndex() && this->root == tree->GetRoot();
			}
		};
	}
}

