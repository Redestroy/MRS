#include "../Include/View2DPosition.hpp"

namespace MRS {
	namespace Device {
		View2DPosition::View2DPosition() : View(MRS::Device::ViewType::VIEW_POSITION_2D)
		{
			x = 0.0f;
			y = 0.0f;
			a = 0.0f;
			p_x = 0.1f;
			p_y = 0.1f;
			p_a = 0.1f;
		}

		View2DPosition::View2DPosition(double x, double y) : View2DPosition(x, y, 0.0f)
		{
		}


		View2DPosition::View2DPosition(double x, double y, double a) : View(MRS::Device::ViewType::VIEW_POSITION_2D)
		{
			SetX(x);
			SetY(y);
			SetA(a);
			p_x = 0.1f;
			p_y = 0.1f;
			p_a = 0.1f;
		}

		View2DPosition::~View2DPosition()
		{
		}

		void View2DPosition::SetX(double x)
		{
			this->x = x;
		}

		void View2DPosition::SetY(double y)
		{
			this->y = y;
		}

		void View2DPosition::SetA(double a)
		{
			this->a = a;
		}

		double View2DPosition::GetX()
		{
			return x;
		}

		double View2DPosition::GetY()
		{
			return y;
		}

		double View2DPosition::GetA()
		{
			return a;
		}
		void View2DPosition::SetP_X(double p_x)
		{
			this->p_x = p_x;
		}
		void View2DPosition::SetP_Y(double p_y)
		{
			this->p_y = p_y;
		}
		void View2DPosition::SetP_A(double p_a)
		{
			this->p_a = p_a;
		}
		double View2DPosition::GetP_X()
		{
			return p_x;
		}
		double View2DPosition::GetP_Y()
		{
			return p_y;
		}
		double View2DPosition::GetP_A()
		{
			return p_a;
		}
	}
}