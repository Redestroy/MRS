#include "../Include/ObjectView.hpp"

namespace MRS {
	namespace Device {
		ObjectView::ObjectView() : ObjectView("")
		{
			SetUnnamed();
		}

		ObjectView::ObjectView(std::string obj_name) : ObjectView(obj_name, 0, 0, false)
		{
		}

		ObjectView::ObjectView(std::string obj_name, double c1, double c2, bool UseCartesian) : ObjectView(obj_name, c1, c2, 0, 0, UseCartesian)
		{
		}

		ObjectView::ObjectView(std::string obj_name, double c1, double c2, double d_c1, double d_c2, bool UseCartesian) : View(MRS::Device::ViewType::VIEW_OBJECT_2D), view()
		{
			SetName(obj_name);
			if (use_cartesian) {
			 // figure out later
				x = c1;
				y = c2;
				vx = d_c1;
				vy = d_c2;
			}
			else {
				view.SetDistance(c1);
				view.SetAngle(c2);
				vx = d_c1;
				vy = d_c2;
			}
			use_cartesian = UseCartesian;
		}

		ObjectView::~ObjectView()
		{
		}

		void ObjectView::SetCoords(DistanceView* coords)
		{
			view.SetDistance(coords->GetDistance());
			view.SetAngle(coords->GetAngle());
		}

		DistanceView* ObjectView::GetCoords()
		{
			return &view;
		}

		void ObjectView::SetUnnamed()
		{
			name.clear();
			name = "OBJ_NF";
		}

		bool ObjectView::IsUnnamed()
		{
			return name.compare("OBJ_NF") == 0;
		}

		void ObjectView::SetName(std::string new_name)
		{
			name.clear();
			name = "" + new_name;
		}

		std::string ObjectView::GetName()
		{
			return name;
		}

		bool ObjectView::IsUsingCartesian()
		{
			return use_cartesian;
		}

		void ObjectView::Move(double c1, double c2)
		{

		}
	}
}