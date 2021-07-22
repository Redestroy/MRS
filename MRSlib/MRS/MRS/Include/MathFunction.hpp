#pragma once
#include <deque>
#include <cstdio>

namespace MRS {
	namespace Math {
		struct Parameter {
			std::string name;
			double value;
		};

		class MathFunction {
		private:
		public:
			virtual double Calculate(Parameter* parameters, std::size_t number_of_parameters) = 0;
			virtual std::size_t GetNumberOfNecessaryParameters() = 0;
		};

		class ConstantFunction : public MathFunction {
		private:
			double value;
		public:
			ConstantFunction(double value) {
				this->value = value;
			}

			~ConstantFunction() {}

			double Calculate(Parameter* parameters, std::size_t number_of_parameters) {
				return value;
			}

			std::size_t GetNumberOfNecessaryParameters() {
				return 0;
			}
		};

		class LinearFunction : public MathFunction {
		private:
			double koeficient;
			double constant;
		public:
			LinearFunction(double koeficient = 1, double constant = 0) {
				this->koeficient = koeficient;
				this->constant = constant;
			}
			~LinearFunction() {}

			double Calculate(Parameter* parameters, std::size_t number_of_parameters) {
				if (number_of_parameters == 1) {
					return koeficient * (parameters[0]).value + constant;
				}
				//else if (number_of_parameters > 1) {
				//	for(int i = 0)
				//}
				else {
					//TODO: Throw exception
					return -1;
				}
			}

			std::size_t GetNumberOfNecessaryParameters() {
				return 1;
			}

		};

		class MultiDimensionalLinearFunction : public MathFunction {
		private:
			std::deque<double> coefficients;
			double constant;
		public:
			MultiDimensionalLinearFunction(std::size_t number_of_coefficients, double default_coefficient = 1, double constant = 0) : coefficients(number_of_coefficients, default_coefficient) {
				this->constant = constant;
			}

			~MultiDimensionalLinearFunction() {}

			void SetCoefficient(std::size_t index, double value) {
				coefficients[index] = value;
			}

			void SetCoeficients(double* values, std::size_t number) {
				for (int i = 0; i < number; i++) {
					coefficients[i] = values[i];
				}
			}

			void SetRangeOfCoeficients(double* values, std::size_t start_number, std::size_t number) {
				for (int i = start_number; i < start_number + number; i++) {
					coefficients[i] = values[i - start_number];
				}
			}

			double Calculate(Parameter* parameters, std::size_t number_of_parameters) {
				if (number_of_parameters == 1) {
					return coefficients[0] * parameters[0].value + constant;
				}
				else if (number_of_parameters > 1 && number_of_parameters <= coefficients.size()) {
					double result = 0;
					for (std::size_t i = 0; i < number_of_parameters; i++) {
						result += parameters[i].value * coefficients[i];
					}
					result += constant;
					return result;
				}
				else if (number_of_parameters > coefficients.size()) {
					double result = 0;
					for (std::size_t i = 0; i < coefficients.size(); i++) {
						result += parameters[i].value * coefficients[i];
					}
					for (std::size_t i = coefficients.size(); i < number_of_parameters; i++) {
						result += parameters[i].value;
					}
					result += constant;
					return result;
				}
				else {
					//TODO: Throw exception
					return 0;
				}

			}

			std::size_t GetNumberOfNecessaryParameters() {
				return coefficients.size();
			}
		};

		class TriganometricFunction : public MathFunction {};
		class ExponentialFunction : public MathFunction {};
		
		template <class T>
		class ExternalFunction : public MathFunction {
		private:
			typedef double (T::* external_function)(Parameter[], std::size_t);
			external_function function;
			T* calculator_object;
			std::size_t necessary_parameters;
		public:
			ExternalFunction(T* calculator_object, external_function function, std::size_t necessary_parameters = 1) {
				this->calculator_object = calculator_object;
				this->function = function;
				this->necessary_parameters = necessary_parameters;
			}
			~ExternalFunction() {}

			double Calculate(Parameter* parameters, std::size_t number_of_parameters) {
				return calculator_object->function(parameters, number_of_parameters);
			}
			std::size_t GetNumberOfNecessaryParameters() {
				return necessary_parameters;
			}
		};

		class MultiDimensionalTriganometricFunction : public MathFunction {};
		class MultiDimensionalExponentialFunction : public MathFunction {};
		class MultiDimensionalExternalFunction : public MathFunction {};

		class ChainedFunctionSum : public MathFunction {
		private:
			std::deque<MathFunction*> functions;
		public:
			double Calculate(Parameter* parameters, std::size_t number_of_parameters) {
				if (number_of_parameters < GetNumberOfNecessaryParameters()) return 0; // TODO: throw exception
				double result = 0;
				std::size_t range_size = 0;
				Parameter* parameter_subrange = parameters;
				for (auto f : functions) {
					range_size = f->GetNumberOfNecessaryParameters();
					result += f->Calculate(parameter_subrange, range_size);
					parameter_subrange += range_size;
					if (parameter_subrange > parameters+number_of_parameters) break; // TODO: throw exception
				}
				return result;
			}
			std::size_t GetNumberOfNecessaryParameters() {
				std::size_t n = 0;
				for (auto f : functions) n += f->GetNumberOfNecessaryParameters();
				return n;
			}
		};
	}
}