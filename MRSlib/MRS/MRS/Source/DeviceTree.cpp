//#include "pch.h"
//#include "DeviceTree.hpp"
#include "../Include/DeviceTree.hpp"
//#include "..\Include\DeviceTr.hpp"

namespace MRS{
	namespace Device {
		
		DeviceTree::DeviceTree()
		{
		}

		DeviceTree::DeviceTree(DeviceTree * parent, DeviceTree * root, MRS::Device::Device* device) : children()
		{
			this->parent = parent;
			this->root = root;
			this->current = device;
		}

		DeviceTree::~DeviceTree()
		{
		}

		// using Insert with deviceId that exists in sub branch will replace the nodes device
		void DeviceTree::Insert(MRS::Device::Device* device, long deviceId, long parentId) //device id is in subtree
		{
			if (deviceId == 0) { //Replacing current device
				current = device;
			}
			else if(parentId == 0){ // If is a direct child
				//children.insert(new DeviceTree(this, root, device));
			}
		}

		DeviceTree* DeviceTree::GetDeviceBranch(long deviceId)
		{
			//do tree search
			if (deviceId == root->GetIndex()) return root;
			if (deviceId == this->GetIndex()) return this;
			DeviceTree* lastChild = nullptr;
			for (auto ch : children) {
				if (deviceId == ch->GetIndex()) return ch;
				if (deviceId < ch->GetIndex())
				{
					if (lastChild != nullptr) return lastChild->GetDeviceBranch(deviceId);
				}
				lastChild = ch;
			}
			return nullptr;
		}

		Device* DeviceTree::GetDevice()
		{
			return current;
		}

		int DeviceTree::GetIndex()
		{
			return id;
		}

		DeviceTree* DeviceTree::GetRoot()
		{
			return root;
		}

		Device* DeviceTree::GetDevice(long deviceId)
		{
			//do tree search
			if (deviceId == root->GetIndex()) return root->GetDevice();
			if (deviceId == this->GetIndex()) return this->GetDevice();
			DeviceTree* lastChild = nullptr;
			for (auto ch : children) {
				if (deviceId == ch->GetIndex()) return ch->GetDevice();
				if (deviceId < ch->GetIndex())
				{
					if(lastChild != nullptr) return lastChild->GetDevice(deviceId);
				}
				lastChild = ch;
			}
			return nullptr;
		}
	}
}