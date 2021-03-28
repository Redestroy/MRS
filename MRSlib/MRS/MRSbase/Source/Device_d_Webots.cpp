#pragma once
#include "../Include/pch.h"
#include "../Include/Device_d_Webots.hpp"

namespace MRS {
	namespace Drivers {
		namespace Webots {
			Device_d_Webots::Device_d_Webots()
			{
				Device::Device();
				wb_device_ptr = nullptr;
			}

			Device_d_Webots::Device_d_Webots(webots::Device* ptr)
			{
				Device::Device();
				wb_device_ptr = ptr;
				name = new std::string(ptr->getName());
				model = new std::string(ptr->getModel());
				// copy relevant info
				if (ptr != nullptr) {
					this->SetDeviceType(GetDeviceTypeFromNodeType(ptr->getNodeType()));
				}
			}

			Device_d_Webots::~Device_d_Webots()
			{
				delete name;
				delete model;
			}

			MRS::Device::DeviceType Device_d_Webots::GetDeviceTypeFromNodeType(int type)
			{
				switch (type)
				{
				case webots::Node::ACCELEROMETER :
				case webots::Node::DISTANCE_SENSOR :
				case webots::Node::GPS :
				case webots::Node::GYRO :
				case webots::Node::INERTIAL_UNIT :
				case webots::Node::POSITION_SENSOR :
				case webots::Node::RADAR :
				case webots::Node::RANGE_FINDER :
				case webots::Node::TOUCH_SENSOR :
				case webots::Node::LIDAR :
				case webots::Node::LIGHT_SENSOR :
				case webots::Node::MICROPHONE :
					return MRS::Device::DeviceType::SENSOR;
				case webots::Node::BALL_JOINT :
				case webots::Node::BRAKE :
				case webots::Node::CONNECTOR :
				case webots::Node::DIFFERENTIAL_WHEELS :
				//case webots::Node::DIRECTIONAL_LIGHT :
				case webots::Node::DISPLAY :
				case webots::Node::HINGE_2_JOINT :
				case webots::Node::HINGE_JOINT :
				case webots::Node::LED :
				case webots::Node::LINEAR_MOTOR :
				case webots::Node::MUSCLE :
				case webots::Node::PEN :
				//case webots::Node::POINT_LIGHT:
				case webots::Node::PROPELLER :
				case webots::Node::ROTATIONAL_MOTOR :
				case webots::Node::SLIDER_JOINT :
				case webots::Node::SPEAKER :
				//case webots::Node::SPOT_LIGHT :
					return MRS::Device::DeviceType::ACTUATOR;
				case webots::Node::BOX: // all shapes are complex
					return MRS::Device::DeviceType::COMPLEX;
				//case 4:
				//	return MRS::Device::DeviceType::LINK;
				case webots::Node::RECEIVER :
					return MRS::Device::DeviceType::RECIEVER;
				case webots::Node::EMITTER :
					return MRS::Device::DeviceType::SENDER;
				case webots::Node::RADIO :
					return MRS::Device::DeviceType::COMUNICATOR;
				default:
					return MRS::Device::DeviceType::Null;
				}
			}

			


		}
	}
}