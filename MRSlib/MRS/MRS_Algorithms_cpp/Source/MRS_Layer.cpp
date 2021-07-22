#include "../Include/MRS_Layer.hpp"

namespace MRS {
	namespace Algorithms {
		void Layer::SetLayerState(LayerState state)
		{
			this->state = state;
			if (debug) {
				PrintState(state);
			}
		}

		Layer::Layer() : Layer(false)  {
			
		}

		Layer::Layer(bool debug)
		{
			this->debug = debug;
			SetLayerState(LayerState::CREATEING);
			Create();
			SetLayerState(LayerState::CREATED);
		}

		Layer::~Layer()
		{
			Destroy();
		}

		void Layer::Create()
		{
			OnCreate();
		}

		void Layer::Init()
		{
			SetLayerState(LayerState::INITIALIZING);
			OnInit();
			SetLayerState(LayerState::INITIALIZED);
		}

		void Layer::Start()
		{
			SetLayerState(LayerState::STARTING);
			OnStart();
			SetLayerState(LayerState::STARTED);
			Run();
		}

		void Layer::Run()
		{
			is_working = true;
			while (is_working) {
				SetLayerState(LayerState::PROCESSING);
				Process();
				SetLayerState(LayerState::PROCESSED);
				PreUpdate();
				SetLayerState(LayerState::UPDATING);
				Update();
				SetLayerState(LayerState::UPDATED);
				PostUpdate();
			}
		}

		void Layer::Stop()
		{
			is_working = false;
			OnStop();
			SetLayerState(LayerState::STOPPED);
		}

		void Layer::Pause()
		{
			is_working = false;
			OnPause();
			SetLayerState(LayerState::PAUSED);
		}

		void Layer::PreUpdate()
		{
			OnPreUpdate();
		}

		void Layer::Update()
		{
			OnUpdate();
		}

		void Layer::PostUpdate()
		{
			OnPostUpdate();
		}

		void Layer::Exit()
		{
			SetLayerState(LayerState::EXITING);
			OnExit();
			SetLayerState(LayerState::EXITED);
		}

		void Layer::Destroy()
		{
			OnDestroy();
		}

		LayerState Layer::GetLayerState()
		{
			return state;
		}

		bool Layer::GetDebug()
		{
			return debug;
		}

		void Layer::PrintState(LayerState state)
		{
			std::cout << "State: ";
			switch (state) {
			case LayerState::NEW:
				std::cout << "NEW";
				break;
			case LayerState::CREATEING:
				std::cout << "CREATEING";
				break;
			case LayerState::CREATED:
				std::cout << "CREATED";
				break;
			case LayerState::INITIALIZING:
				std::cout << "INITIALIZING";
				break;
			case LayerState::INITIALIZED:
				std::cout << "INITIALIZED";
				break;
			case LayerState::STARTING:
				std::cout << "STARTING";
				break;
			case LayerState::STARTED:
				std::cout << "STARTED";
				break;
			case LayerState::PROCESSING:
				std::cout << "PROCESSING";
				break;
			case LayerState::PROCESSED:
				std::cout << "PROCESSED";
				break;
			case LayerState::UPDATING:
				std::cout << "UPDATING";
				break;
			case LayerState::UPDATED:
				std::cout << "UPDATED";
				break;
			case LayerState::PAUSED:
				std::cout << "PAUSED";
				break;
			case LayerState::STOPPED:
				std::cout << "STOPPED";
				break;
			case LayerState::EXITING:
				std::cout << "EXITING";
				break;
			case LayerState::EXITED:
				std::cout << "EXITED";
				break;
			default:
				std::cout << "Error: No such state exists";
			}
			std::cout << "\n";
		}

	
	}
}