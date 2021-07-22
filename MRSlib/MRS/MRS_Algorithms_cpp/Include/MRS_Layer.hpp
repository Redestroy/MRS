#pragma once
#include <string>
#include <iostream>
#include <boost/timer/timer.hpp>

namespace MRS {
	namespace Algorithms {
		enum class LayerState {
			NEW,
			CREATEING,
			CREATED,
			INITIALIZING,
			INITIALIZED,
			STARTING,
			STARTED,
			PROCESSING,
			PROCESSED,
			UPDATING,
			UPDATED,
			PAUSED,
			STOPPED,
			EXITING,
			EXITED
		};

		class Layer {
		private:
			LayerState state = LayerState::NEW;
			bool debug = false;
			bool is_working = false;
		protected:
			void SetLayerState(LayerState);
		public:
			Layer();
			Layer(bool debug);
			virtual ~Layer();

			virtual void Create();
			virtual void OnCreate() = 0;
			
			virtual void Init();
			virtual void OnInit() = 0;

			virtual void Start();
			virtual void Run();
			virtual void Stop();
			virtual void Pause();

			virtual void OnStart() = 0;
			virtual void Process() = 0;
			virtual void OnStop() = 0;
			virtual void OnPause() = 0;

			virtual void PreUpdate();
			virtual void Update();
			virtual void PostUpdate();

			virtual void OnPreUpdate() = 0;
			virtual void OnUpdate() = 0;
			virtual void OnPostUpdate() = 0;

			virtual void Exit();
			virtual void OnExit() = 0;

			virtual void Destroy();
			virtual void OnDestroy() = 0;

			LayerState GetLayerState();
			bool GetDebug();

			void PrintState(LayerState state);

		};


		class SimpleLayer : public Layer {
		private:
			boost::timer::cpu_timer my_timer;
			boost::timer::nanosecond_type time_one = 0, time_two = 0, time_three = 0;
			boost::timer::nanosecond_type MeasureTimePassed() {
				time_three = my_timer.elapsed().system;
				boost::timer::nanosecond_type delta = time_three - time_two;
				time_two = time_three;
				return delta;
			}

			void PrintTimePassed() {
				if (GetDebug()) std::cout << "Time passed:: " << MeasureTimePassed() << "\n";
			}

		public:
			SimpleLayer() : SimpleLayer(false) {}
			SimpleLayer(bool debug) : Layer(debug)
			{
				my_timer.start();
				time_one = my_timer.elapsed().system;
				time_two = time_one;
			}

			virtual ~SimpleLayer(){
				PrintTimePassed();
			}

			virtual void OnCreate() {
				PrintTimePassed();
			}

			virtual void OnInit() {
				PrintTimePassed();
			}

			virtual void OnStart() {
				PrintTimePassed();
			}

			virtual void Process() {
				PrintTimePassed();
			}

			virtual void OnStop() {
				PrintTimePassed();
			}

			virtual void OnPause() {
				PrintTimePassed();
			}

			virtual void OnPreUpdate() {
				PrintTimePassed();
			}

			virtual void OnUpdate() {
				PrintTimePassed();
			}

			virtual void OnPostUpdate() {
				PrintTimePassed();
			}

			virtual void OnExit() {
				PrintTimePassed();
			}

			virtual void OnDestroy() {
				PrintTimePassed();
			}

		};
	}
}