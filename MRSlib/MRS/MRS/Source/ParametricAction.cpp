#pragma once
#include "../Include/ParametricAction.hpp"
namespace MRS {
	namespace Device {
		ParametricAction::ParametricAction() : ParametricAction(nullptr, nullptr, nullptr, 0, 1)
		{

		}

		ParametricAction::ParametricAction( Math::MathFunction* function, Environment::Worldview* world, std::string* parameter_names, std::size_t number_of_parameters, long long int scale)
		{
			parameter_count = number_of_parameters;
			this->scale = scale;
			this->function = function;
			this->world = world;
			SetParameters(parameter_names, parameter_count);
		}

		ParametricAction::ParametricAction(std::string description) : ParametricAction()
		{

		}

		ParametricAction::~ParametricAction()
		{
		}

		long long int ParametricAction::GetActionParameter()
		{
			UpdateParameters();
			Recalculate();
			return actionParameter;
		}

		void ParametricAction::SetParameterCount(std::size_t number_of_parameters)
		{
			parameter_count = number_of_parameters;
		}

		void ParametricAction::SetFunction(Math::MathFunction* f)
		{
			function = f;
		}

		void ParametricAction::SetParameters(std::string* names, std::size_t number)
		{
			if (number == 0) return;
			parameters.resize(number);
			for (int i = 0; i < number; i++) {
				parameters[i].name = "" + names[i];
			}
			UpdateParameters();
		}

		void ParametricAction::UpdateParameters()
		{
			//Updates all parameters from world
			for (auto p : parameters) {
				p.value = world->GetParameterValue(p.name);
			}
		}

		void ParametricAction::Recalculate()
		{
			// get array from vector
			// call calculate function
			// assign to action parameter
			actionParameter2 = function->Calculate(parameters.data(),parameters.size());
			actionParameter = (long long int)(actionParameter2 * scale);
		}
	}
}