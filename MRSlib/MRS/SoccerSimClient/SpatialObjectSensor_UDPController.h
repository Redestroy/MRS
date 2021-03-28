#pragma once
#include <MRS/Include/SpatialObjectSensor.hpp>
#include <MRS/Include/ObjectViewList.hpp>
#include <memory>
#include "UDPSensor.h"
#include "SoccerServerCommandDict.h"

namespace MRS {
	namespace Device {
		class SpatialObjectSensor_UDPController : public SpatialObjectSensor, UDPSensor
		{
		private:
			std::string command_template;
			std::string args_template;
			std::string current_payload;
			ObjectViewList obj_view;
			// object list view objView
			//
			std::shared_ptr<DummyObjectSensor<SpatialObjectSensor_UDPController>> obj_sensor;
			std::shared_ptr<DummyDistanceSensor<SpatialObjectSensor_UDPController>> dist_sensor;
		protected:
			void ParsePacket(std::string);
		public:
			SpatialObjectSensor_UDPController();
			SpatialObjectSensor_UDPController(UDPController*);
			SpatialObjectSensor_UDPController(UDPController*, std::string);
			~SpatialObjectSensor_UDPController();
			virtual void OnUpdate(long delta);
			virtual void OnPacketReceived(std::string);
			int GetLastObjectID();
			DistanceView* GetLastObjectDistView();
		};
	}
}

