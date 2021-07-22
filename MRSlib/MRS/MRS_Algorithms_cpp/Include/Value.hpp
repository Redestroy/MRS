#pragma once
#include <math.h>
#include <complex>
#include "../../MRS/Include/Behaviour.hpp"

namespace MRS {
	namespace Algorithms {
		class Value {
		private:
			double scalar_value = 0;
			double imaginary_value = 0;
			//Function impact function;
		public:
			Value(double scalar = 0, double imaginary = 0) {
				scalar_value = scalar;
				imaginary_value = imaginary;
			}

			~Value() {

			}

			double Scalar() {
				return scalar_value;
			}

			double Imaginary() {
				return imaginary_value;
			}

			double Module() {
				return sqrt(scalar_value * scalar_value + imaginary_value * imaginary_value);
			}

			double Angle() {
				return atan2(imaginary_value, scalar_value);
			}

			Value operator+(Value v) {
				//TODO: use impact function
				return Value(scalar_value+v.Scalar(), imaginary_value + v.Imaginary());
			}

			Value operator+(double d) {
				//TODO: use impact function
				return Value(scalar_value + d, imaginary_value);
			}

			Value operator+(std::complex<double> d) {
				//TODO: use impact function
				return Value(scalar_value + d.real(), imaginary_value + d.imag());
			}

			void operator=(double d) {
				//TODO: use impact function
				scalar_value = d;
			}

			void operator=(std::complex<double> d) {
				//TODO: use impact function
				scalar_value = d.real();
				imaginary_value = d.imag();
			}

		};

		namespace ValueOperations {
			struct CompareValues {
				bool operator()(std::pair<MRS::Algorithms::Value, Task::Behaviour*> a, std::pair<MRS::Algorithms::Value, Task::Behaviour*> b) {
					return a.first.Scalar() > b.first.Scalar();
				}
			};
		}
	}
}