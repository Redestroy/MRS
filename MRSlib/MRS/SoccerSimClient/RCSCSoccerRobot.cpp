#include "RCSCSoccerRobot.h"

namespace MRS {
	RCSCSoccerRobot::RCSCSoccerRobot() : PlayerAgent(), Robot()
	{
		this->behaviourLibrary = new Task::BehaviourLibrary();
		//this->worldview = new Environment::Worldview();
		/*
		this->layer =
#ifdef MRS_TTA
			new Algorithms::MRS_TTA();
#elif MRS_STA
			new Algorithms::MRS_STA();
#elif MRS_RTA
			new Algorithms::MRS_RTA();
#elif COMPARISON
			new Algorithms::SamplePlayer();
#elif SIMPLE_LAYER
			new Algorithms::SimpleLayer();
#endif // 
		*/

			
	}

	RCSCSoccerRobot::~RCSCSoccerRobot()
	{
		delete behaviourLibrary;
	}

	void RCSCSoccerRobot::OnInit()
	{
		//Init for robot class
			// this can also be done in main before connecting if it takes two long
			// Create behaviour library from file
			// Create device, sensor and actuator blocks for robot
			// Populate these blocks
			// Create Worldview
			// Add current robot information to worldview
			// initialize control layer - with communication and team information
		std::cout << "Initializing" << std::endl;
		this->behaviourLibrary->PopulateFromFile("rcsc_behaviour_library.behaviours");
		//this->GetDevices();
		//this->SetActuatorBlock();
		//this->SetSensorBlock();

		//this->worldview->GenerateDeviceBlockInformation(this->GetDevices);
		//this->worldview->GenerateActuatorInformation(this->GetActuatorBlock());
		//this->worldview->InitializeSensorViews(this->GetSensorBlock());
		//this->worldview->GenerateSelfModel();

		//this->layer->Init();
	}

	void RCSCSoccerRobot::OnUpdate()
	{
		std::cout << "Updating" << std::endl;
		//World.Update
		//Layer.Update
		//Update parametric behaviours?
	}

	void RCSCSoccerRobot::OnExit()
	{
		// Clear important stuff
		// Output log file
	}

	void RCSCSoccerRobot::execute(MRS::Task::Task* task)
	{
		std::cout << "Executing task" << std::endl;
		switch (task->GetType()) {
		case Task::TaskType::ATOMIC:
			std::cout << "Doing atomic task " << std::endl;
			// get next action
			// DoAction()
			// update state
			break;
		case Task::TaskType::PARAMETRIC:
			std::cout << "Doing parametric task " << std::endl;
			// hidden - update action parameters
			// get next action
			// DoAction()
			// update state
			break;
		case Task::TaskType::BEHAVIOUR:
			std::cout << "Doing behaviour task " << std::endl;
			//if(Task::TaskState)
			// check base task
			// if completed, check behaviour condition
			//		if not fullfilled reset task and get next action
			//		else set state and exit
			// else get next action for the behaviour base task
			// DoAction
			// Update task state
			break;
		case Task::TaskType::COMPLEX:
			std::cout << "Doing complex task " << std::endl;
			// get current task
			// check if done
			// if not done
			// get next command
			// if action is inf go down
			// if action is null go up
			// if done, exit
			break;
		case Task::TaskType::UNDEFINED:
			std::cout << "Doing RCSC task " << std::endl;
			//validate task as RCSC
			//Action = library->GetTask((RCSCTask)(task)->GetKey());
			break;
		default:
			std::cout << "Nonexistent task type - future version? " << std::endl;
		}
	}

	void RCSCSoccerRobot::DoAction(Device::Action action)
	{
		std::cout << "Do action is overridden by rcsc agent methods" << std::endl;
		double power = 0, direction = 0, moment = 0, x = 0, y = 0;
		switch (action.GetActionType()) {
		case Device::ActionType::DASH:
			power = Device::Action::FirstHalfAsDouble(action.GetActionParameter());
			direction = Device::Action::SecondHalfAsDouble(action.GetActionParameter());
			this->doDash(power, direction); //Real
			break;
		case Device::ActionType::TURN:
			moment = (double)(action.GetActionParameter());
			this->doTurn(moment); //Real
			break;
		case Device::ActionType::KICK:
			power = Device::Action::FirstHalfAsDouble(action.GetActionParameter());
			direction = Device::Action::SecondHalfAsDouble(action.GetActionParameter());
			this->doKick(power, direction); //Real
			break;
		case Device::ActionType::TACKLE:
			power = (double)(action.GetActionParameter());
			this->doTackle(power); //Real
			break;
		case Device::ActionType::MOVE:
			x = Device::Action::FirstHalfAsDouble(action.GetActionParameter()); // TODO check, maybe int is good enough
			y = Device::Action::SecondHalfAsDouble(action.GetActionParameter()); // TODO check, maybe int is good enough
			this->doMove(x, y); // Real restricted
			break;
		case Device::ActionType::CATCH:
			this->doCatch(); //Real, restricted
			break;
		case Device::ActionType::TURN_VIEWPORT:
			this->doTurnNeck(moment); //Configurative, viewport change
			break;
			/*
			this->doIntention(); // Executes stored action. Probobly not needed
			this->doPointto(x, y); // Configurative, Implicit (broadcast intent) Communication?
			this->doPointtoOff(x, y); // Configurative, Implicit (broadcast intent) Communication?
			
			this->doChangeView(width); //Configurative
			this->doAttentionto(side, unam); //Configurative
			this->doAttentiontoOff(side, unam); //Configurative
			*/
		default:
			return;
		}
		//this->GetActuatorBlock()->GetActuator(action->GetType())->DoAction(action);
	}


	bool RCSCSoccerRobot::initImpl(rcsc::CmdLineParser& cmd_parser)
	{
		// Init for robot class
		// this can also be done in main before connecting if it takes two long
		// Create behaviour library from file
		// Create device, sensor and actuator blocks for robot
		// Populate these blocks
		// Create Worldview
		// Add current robot information to worldview
		// initialize control layer - with communication and team information
		// 
		Init();
		return rcsc::PlayerAgent::initImpl(cmd_parser);
	}

	void RCSCSoccerRobot::actionImpl()
	{
		//layer->Process()
		//Execute behaviour
		//execute(layer->GetBehavior)
		this->doDash(5);
		std::cout << "Setting action" << std::endl;
	}

	void RCSCSoccerRobot::communicationImpl()
	{
		//Communicate
		//if(communication_neccessary)
		//layer->CreateMessage();
		//GetComDevice()->SendMessage(Message)
		std::cout << "Communication implamentation" << std::endl;
	}

	void RCSCSoccerRobot::handleActionStart()
	{
		// Update
		// 
		//layer->PreUpdate()
		//layer->Update
		Update();
		//worldview->Update();
		std::cout << "Action started" << std::endl;
	}

	void RCSCSoccerRobot::handleActionEnd()
	{
		//layer->PostUpdate();
		std::cout << "Action ended" << std::endl;
	}

	void RCSCSoccerRobot::handleServerParam()
	{
		//TODO add server parameters to world
		std::cout << "Parsing server params" << std::endl;
		// PlayerTypeInfo params
		// worldview->SetServerParameters(params);
	}

	void RCSCSoccerRobot::handlePlayerParam()
	{
		//TODO add player parameters to world
		std::cout << "Parsing player params" << std::endl;
		// PlayerTypeInfo params
		// worldview->SetPlayerParameters(params->self, params);
	}

	void RCSCSoccerRobot::handlePlayerType()
	{
		//TODO:: add player type parameters to world
		std::cout << "Parsing player type params" << std::endl;
		// PlayerTypeInfo params
		// worldview->SetPlayerTypeParameters(params->type_id, params);
	}



}
