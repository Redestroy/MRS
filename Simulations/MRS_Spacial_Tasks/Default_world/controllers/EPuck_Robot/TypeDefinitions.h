#pragma once
#ifndef TYPEDEFINITIONS
#define TYPEDEFINITIONS
class TypeDefinitions
{
public:
	enum ActuatorType {
		DIFFERENTIAL_DRIVE = 'D',
		ACTION_ACTUATOR,
		VIRTUAL_DIGITAL = 'L',
		VIRTUAL_PWM,
		TEXT_OUTPUT,
		FORWARD,
		BACKWARD,
		CLOCKWISE,
		COUNTER_CLOCKWISE,
		ARC_DRIVE,
		RAW_DRIVE,
		NULL_ACTUATOR = 'N',
		DELAY = 'W',
		ACTION_IMPOSSIBLE
	};

	enum SensorType {
		SENSOR_DISTANCE_POINT,
		SENSOR_DISTANCE_ARC,
		SENSOR_DETECTOR_SECTOR,
		SENSOR_GRADIENT_FLOOR,
		SENSOR_GRADIENT,
		SENSOR_POSITION_GLOBAL,
		SENSOR_POSITION_RELATIVE,
		SENSOR_ENCODER,
		SENSOR_TEXT_INPUT,
		SENSOR_COMPLEX,
	};

	enum ViewType {
		VIEW_DISTANCE_POINT,
		VIEW_DISTANCE_ARC,
		VIEW_DETECTOR_SECTOR,
		VIEW_GRADIENT,
		VIEW_POSITION_2D,
		VIEW_VELOCITY_2D,
		VIEW_ACCELERATION_2D,
		VIEW_POSITION_3D,
		VIEW_VELOCITY_3D,
		VIEW_ACCELERATION_3D,
		VIEW_TEXT,
		VIEW_COMPLEX,
	};

	enum ConditionType {
		CONDITION_POSITION = 'P',
		CONDITION_IN_RANGE = 'R',
		CONDITION_WAIT = 'W',
		CONDITION_INPUT_TEXT_EQUALS = 'I',
		CONDITION_UNDEFINED = 'U',
		CONDITION_SUBTASK = 'S',
		CONDITION_COMPLEX = 'C',
		CONDITION_NULL = 'N'
	};

	enum ExecutableBehaviour {
		BEHAVIOUR_DRIVE,
		BEHAVIOUR_SAFE_DRIVE,
		BEHAVIOUR_SEARCH,
		BEHAVIOUR_STANDBY,
		BEHAVIOUR_WAIT,
		BEHAVIOUR_SAFE_WAIT,
		BEHAVIOUR_ASK_FOR_INPUT,
		BEHAVIOUR_ADJUST
	};

	enum TaskState {
		TASK_STATE_UNKNOWN,
		TASK_IN_QUEUE,
		TASK_PREPARING,
		TASK_EXECUTING,
		TASK_COMPLETE,
		TASK_FAILED,
		TASK_IMPOSSIBLE
	};

	enum ReturnCode {
		SUCCESS,
		IN_PROGRESS,
		ERROR,
		FAIL,
		UNEXPECTED
	};

	enum UpdateType {
		QUIETLY,
		ADD,
		EDIT,
		REBUILD,
		MASTER
	};

	enum Pin {
		RX,
		TX,
		D2,
		D3,
		D4,
		D5,
		D6,
		D7,
		D8,
		D9,
		D10,
		D12,
		D13,
		
		A0,
		A1,
		A2,
		A3,
		A4,
		A5,
		A6,
		A7
	};

	enum Type {
		Action = 'A',
		Behaviour = 'B',
		Condition = 'C',
		Device = 'D',
		Task = 'T',
		Sensor = 'S',
		Representation = 'R',
		View = 'V',
		Pin_T = 'P',
		Null = 'N'
	};

	public:
		static void TypeToString(char * buffer, TypeDefinitions::Type type, char subtype) {
			buffer[0] = (char)type;
			buffer[1] = '_';
			buffer[2] = subtype;
		}

		static TypeDefinitions::Type CharToType(char t) {
			switch (t) {
			case (char)Action: return TypeDefinitions::Action;
			case (char)Behaviour: return TypeDefinitions::Behaviour;
			case (char)Condition: return TypeDefinitions::Condition;
			case (char)Device: return TypeDefinitions::Device;
			case (char)Task: return TypeDefinitions::Task;
			case (char)Sensor: return TypeDefinitions::Sensor;
			case (char)Representation: return TypeDefinitions::Representation;
			case (char)View: return TypeDefinitions::View;
			case (char)Pin_T: return TypeDefinitions::Pin_T;
			default: return TypeDefinitions::Null;
			}
		}

		static TypeDefinitions::ConditionType CharToCondition(char t) {
			switch (t) {
			case (char)CONDITION_COMPLEX: return TypeDefinitions::CONDITION_COMPLEX;
			case (char)CONDITION_POSITION: return TypeDefinitions::CONDITION_POSITION;
			case (char)CONDITION_INPUT_TEXT_EQUALS: return TypeDefinitions::CONDITION_INPUT_TEXT_EQUALS;
			case (char)CONDITION_NULL: return TypeDefinitions::CONDITION_NULL;
			case (char)CONDITION_SUBTASK: return TypeDefinitions::CONDITION_SUBTASK;
			case (char)CONDITION_UNDEFINED: return TypeDefinitions::CONDITION_UNDEFINED;
			case (char)CONDITION_WAIT: return TypeDefinitions::CONDITION_WAIT;
			default: return TypeDefinitions::CONDITION_UNDEFINED;
			}
		}

		static TypeDefinitions::ActuatorType CharToAction(char a) {
			switch (a) {
			case (char)VIRTUAL_DIGITAL: return TypeDefinitions::VIRTUAL_DIGITAL;
			case (char)DIFFERENTIAL_DRIVE: return TypeDefinitions::DIFFERENTIAL_DRIVE;
			case (char)DELAY: return TypeDefinitions::DELAY;
			case (char)NULL_ACTUATOR: return NULL_ACTUATOR;
			default: return TypeDefinitions::ACTION_IMPOSSIBLE;
			}
		}

		enum MessageType {
			PING = 'P',
			TASK_COMPLETE_MESSAGE = 'E',
			TASK_MESSAGE = 'T',
			GPS_REQUEST = 'G',
			INVALID_MESSAGE = '0'
		};

		enum MessageType_CBAA {
			CBAA_PING = 'P',
			CBAA_TASK_COMPLETE_MESSAGE = 'E',
			CBAA_BIDS_MESSAGE = 'B',
			CBAA_TASK_MESSAGE = 'T',
			CBAA_GPS_REQUEST = 'G',
			CBAA_HANDSHAKE = 'H',
			CBAA_INVALID_MESSAGE = '0'
		};

		enum TaskType {
			TASK_ATOMIC = 'A',
			TASK_BEHAVIOUR = 'B',
			TASK_COMPLEX = 'C'
		};

		static TypeDefinitions::TaskState GetTaskState(char a) {
			switch (a) {
			case (int)TASK_FAILED: return TypeDefinitions::TASK_FAILED;
			case (int)TASK_COMPLETE: return TypeDefinitions::TASK_COMPLETE;
			default: return TypeDefinitions::TASK_STATE_UNKNOWN;
			}
		}

};
#endif
