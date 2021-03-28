#pragma once
#include "View.hpp"
namespace MRS {
	namespace Device {
		class View2DPosition :
			public View
		{
			double x, y, a;
			double p_x, p_y, p_a;
		public:
			View2DPosition();
			View2DPosition(double, double);
			View2DPosition(double, double, double);
			~View2DPosition();
			//int GetAspect(int, TypeDefinitions::ViewType, char[]);
			void SetX(double);
			void SetY(double);
			void SetA(double);
			double GetX();
			double GetY();
			double GetA();
			void SetP_X(double);
			void SetP_Y(double);
			void SetP_A(double);
			double GetP_X();
			double GetP_Y();
			double GetP_A();
			void operator=(View2DPosition v) {
				x = v.GetX();
				y = v.GetY();
				a = v.GetA();
				p_x = v.GetP_X();
				p_y = v.GetP_Y();
				p_a = v.GetP_A();
			}

			void operator+=(View2DPosition v) {
				x += v.GetX();
				y += v.GetY();
				a += v.GetA();
				p_x += v.GetP_X();
				p_y += v.GetP_Y();
				p_a += v.GetP_A();
			}

			View2DPosition operator+(View2DPosition v) {
				View2DPosition v2(x + v.GetX(), y + v.GetY(),a + v.GetA());
				v2.SetP_X(p_x + v.GetP_X());
				v2.SetP_Y(p_y + v.GetP_Y());
				v2.SetP_A(p_a + v.GetP_A());
				return v2;
			}

			// scaling
			void operator*=(View2DPosition v) {
				x *= v.GetX();
				y *= v.GetY();
				a *= v.GetA();
				p_x *= v.GetP_X();
				p_y *= v.GetP_Y();
				p_a *= v.GetP_A();
			}
		};
	}
}
