#pragma once
#include <rcsc/player/player_agent.h>
#include <MRS/Include/Robot.hpp>
//#include <MRS/Include/>
namespace MRS {
	class RCSCSoccerRobot : public rcsc::PlayerAgent, public Task::Robot
	{
	private:
		Task::BehaviourLibrary * behaviourLibrary;
        Environment::Worldview* worldview;
        // Add Worldview
        // TODO implament MRS::Algorithms as a library and integrate it into Robot
        // MRS::Algorithms::Layer* controlLayer;
	public:
		RCSCSoccerRobot();
		virtual ~RCSCSoccerRobot();
		
		// RobotInterface
		void OnInit();
		void OnUpdate();
        void OnExit();
		void execute(MRS::Task::Task*);
		void DoAction(Device::Action);

		// RCSC agentInterface
        //But you must call PlayerAgent::initImpl() in this method.
        virtual bool initImpl(rcsc::CmdLineParser& cmd_parser);

        //This method is used to set player's body action.
        virtual void actionImpl();

        /*!
        register say action to ActionEffector
        This method is called just after turn_neck action.
        */
        virtual void communicationImpl();

        //This method is called at the top of action().
        virtual void handleActionStart();

        //This method is called at the end of action() but before the debug output.
        virtual void handleActionEnd();

        //this method is called just after analyzing server_param message.
        virtual void handleServerParam();

        // this method is called just after analyzing player_param message.
        virtual void handlePlayerParam();

        // this method is called just after analyzing player_type message.
        virtual void handlePlayerType();
	};
}

