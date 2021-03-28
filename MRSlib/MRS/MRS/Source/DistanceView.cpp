#include "../Include/DistanceView.hpp"

namespace MRS {
	namespace Device {
		
		DistanceView::DistanceView()
		{
			SetType(MRS::Device::ViewType::VIEW_DISTANCE_POINT);
			angle = (double *)(this->GetDataPtr() + ANGLE_PTR_OFFSET);
			distance = (double *)(this->GetDataPtr() + DISTANCE_PTR_OFFSET);
			precision_angle = (double *)(this->GetDataPtr() + PRECISION_A_PTR_OFFSET);
			precision_distance = (double *)(this->GetDataPtr() + PRECISION_D_PTR_OFFSET);
		}

		DistanceView::DistanceView(double distance, double direction)
		{
		}

		DistanceView::~DistanceView()
		{
		}

		void DistanceView::SetDistance(double dist) {
			*distance = dist;
		}

		double DistanceView::GetDistance()
		{
			return *distance;
		}

		void DistanceView::SetAngle(double angle)
		{
			*(this->angle) = angle;
		}

		double DistanceView::GetAngle()
		{
			return *angle;
		}

		void DistanceView::SetPrecisionAngle(double p_a)
		{
			*precision_angle = p_a;
		}

		double DistanceView::GetPrecisionAngle()
		{
			return *precision_angle;
		}

		void DistanceView::SetPrecisionDistance(double p_d)
		{
			*precision_distance = p_d;
		}

		double DistanceView::GetPrecisionDistance()
		{
			return *precision_distance;
		}

		double DistanceView::ToCartesianGetX()
		{
			return *distance * cos(*angle);
		}

		double DistanceView::ToCartesianGetY()
		{
			return *distance * sin(*angle);
		}

		double DistanceView::ToDegreesGetAngle()
		{
			return *angle*180/PI;
		}
		
	}
}