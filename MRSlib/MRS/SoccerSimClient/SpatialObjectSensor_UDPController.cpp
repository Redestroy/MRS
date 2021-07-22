#include "SpatialObjectSensor_UDPController.h"
namespace MRS {
	namespace Device {
		
		void SpatialObjectSensor_UDPController::ParsePacket(std::string packet)
		{
			std::string subpacket = "";
			std::size_t currentParentesiesOpen = 0, next = 0;
			double timestamp = -1;
			std::string temp, dataSubstring, firstSubstring, secondSubstring;
			currentParentesiesOpen = (int)packet.find_first_of('(', next);
			next = SoccerServerCommandDict::FindEndOfNextOpeningParentesies(temp, "()");
			//this gives whole string without carrege returns and unneccessary symbols and gives string npos if string is improperly formated
			if (next == std::string::npos) {
				std::cerr << "Faulty packet formatting\n";
				return;
			}
			temp = packet.substr(currentParentesiesOpen, next - currentParentesiesOpen + 1);
			//now, get see
			next = temp.find_first_of(' ');
			dataSubstring = temp.substr(1, next);
			if (dataSubstring.compare("see") != 0) {
				std::cerr << "Not see packet - error in packet routing\n";
				return;
			}
			// here get timestamp
			currentParentesiesOpen = next;
			next = temp.find_first_of(' ', next + 1);
			timestamp = SoccerServerCommandDict::ParseTimestamp(packet); //proper way
			//improper way, but used to keep track of count
			dataSubstring = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
			try {
				timestamp = std::stod(dataSubstring);
			}
			catch (...) {
				std::cerr << "Timestamp is not a double\n";
				return;
			}
			// here get object info
			currentParentesiesOpen = temp.find_first_of('(', next);
			next = SoccerServerCommandDict::FindEndOfNextOpeningParentesies(temp, "()", currentParentesiesOpen);
			obj_view.Clear();
			while (next != std::string::npos) {
				subpacket = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
				//firstSubstring = dataSubstring.substr(0, dataSubstring.find_first_of(' '));
				//secondSubstring = dataSubstring.substr(dataSubstring.find_first_of(' ') + 1);
				//response_map.emplace(firstSubstring, secondSubstring);
				currentParentesiesOpen = temp.find_first_of('(', next);
				next = SoccerServerCommandDict::FindEndOfNextOpeningParentesies(temp, "()", currentParentesiesOpen);
				std::cout << "Parameter: <" << subpacket << "> -> <" << firstSubstring << "> : <" << secondSubstring << ">\n";
				// TODO: use better constructor for ObjectView - one that takes in timestamp, name and Coords
				std::unique_ptr<ObjectView> obj(new ObjectView());
				obj->onCreate((long)timestamp); //TODO: change to double in onCreate
				obj->SetName(SoccerServerCommandDict::ParseObjectParameter("ObjName", subpacket));
				DistanceView obj_distance_view = (SoccerServerCommandDict::ToPolRelCoords(SoccerServerCommandDict::ParseObjectParameter("Coords", subpacket))); //TODO: add a copy function and an = operator to distance view
				std::unique_ptr<DistanceView> obj_dist_view_ptr(&obj_distance_view); //TODO: fix pointer situation - change 3 lines into single line
				obj->SetCoords(&obj_distance_view);
				//ObjectView(timestamp, SoccerServerCommandDict::ParseObjectParameter("ObjName", subpacket), SoccerServerCommandDict::ToPolRelCoords(SoccerServerCommandDict::ParseObjectParameter("Coords", subpacket)))
				obj_view.Add(std::move(obj));
			}
		}

		SpatialObjectSensor_UDPController::SpatialObjectSensor_UDPController() : SpatialObjectSensor_UDPController(nullptr)
		{

		}
		SpatialObjectSensor_UDPController::SpatialObjectSensor_UDPController(UDPController* ptr) : SpatialObjectSensor(obj_sensor.get(), dist_sensor.get()), UDPSensor(ptr), obj_view()
		{
			obj_sensor->SetDetector(&SpatialObjectSensor_UDPController::GetLastObjectID);
			obj_sensor->SetOvertaker(this);
			dist_sensor->SetDetector(&SpatialObjectSensor_UDPController::GetLastObjectDistView);
			dist_sensor->SetOvertaker(this);
		}

		SpatialObjectSensor_UDPController::SpatialObjectSensor_UDPController(UDPController* ptr, std::string cmd_format) : SpatialObjectSensor_UDPController(ptr)
		{
				//TODO: replace with ProtocolParser method in the future
				command_template = cmd_format.substr(0, cmd_format.find(';'));
				args_template = cmd_format.substr(cmd_format.find(';') + 1, cmd_format.find('\r') - cmd_format.find(';') - 1);
		}

		SpatialObjectSensor_UDPController::~SpatialObjectSensor_UDPController()
		{

		}

		void SpatialObjectSensor_UDPController::OnUpdate(long delta)
		{
			//GetUDPController()->SendCommand("",""); //replace with command dict
			// On update only update views according to the last received packet, that was of the correct sensor type
			std::cout << "On update: spatial object sensor \n";
			SpatialObjectSensor::OnUpdate(delta);
			UDPSensor::OnUpdate(delta);
#ifdef ONLINE_PARSE
			//bonus of online parsing - aware of all information
			//minus of online parsing - might be resource heavy
			//Call world update
#endif // ONLINE_PARSE
#ifdef UPDATE_PARSE
			ParsePacket(current_packet);
			//Call world update
#endif // UPDATE_PARSE
		}

		void SpatialObjectSensor_UDPController::OnPacketReceived(std::string payload)
		{
			//if (GetPacketIdentifier(payload).compare("see") == 0) {
				//parse packet or save as current packet
#define ONLINE_PARSE
#ifdef ONLINE_PARSE
				//bonus of online parsing - aware of all information
				//minus of online parsing - might be resource heavy
				ParsePacket(payload); //reads in info to sensor view
				//Call for world to save view info (if needed)
#endif // ONLINE_PARSE
#ifdef UPDATE_PARSE
				current_packet.Clear();
				current_packet = payload;
#endif // UPDATE_PARSE
			//}
		}

		int SpatialObjectSensor_UDPController::GetLastObjectID()
		{
			return 0;
		}

		DistanceView* SpatialObjectSensor_UDPController::GetLastObjectDistView()
		{
			return nullptr;
		}
	 }
}