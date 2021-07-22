#include "Action.hpp"
#include "MathFunction.hpp"
#include "Worldview.hpp"
#include <vector>

namespace MRS {
	namespace Device {
		class ParametricAction : public Action
		{
		public:
			ParametricAction();
			ParametricAction(Math::MathFunction * function, Environment::Worldview* world, std::string* parameter_names, std::size_t number_of_parameters = 1, long long int scale = 1);
			ParametricAction(std::string);
			~ParametricAction();
			virtual long long int GetActionParameter();
			void SetParameterCount(std::size_t);
			void SetFunction(Math::MathFunction*);
			void SetParameters(std::string* names, std::size_t number);
			void UpdateParameters();
			void Recalculate();
		private:
			std::size_t parameter_count;
			Math::MathFunction * function; // TODO: replace with unique ptr
			std::vector<Math::Parameter> parameters;
			Environment::Worldview * world; // TODO: replace with shared ptr
			long long int scale;
		};

	}
}