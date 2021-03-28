#pragma once
#include "View.hpp"
#include "DistanceView.hpp"
#include <string>
namespace MRS {
	namespace Device {
		class ObjectView :
			public View
		{
			std::string name;
			DistanceView view;
			double x;
			double y;
			double vx;
			double vy;
			bool use_cartesian;
		public:
			ObjectView();
			ObjectView(std::string);
			ObjectView(std::string, double c1, double c2, bool UseCartesian = false);
			ObjectView(std::string, double c1, double c2, double d_c1, double d_c2, bool UseCartesian = false);
			~ObjectView();
			//int GetAspect(int, TypeDefinitions::ViewType, char[]);
			void SetCoords(DistanceView *);
			DistanceView * GetCoords();
			void SetUnnamed();
			bool IsUnnamed();
			void SetName(std::string);
			std::string GetName();
			bool IsUsingCartesian();

			void Move(double c1, double c2);

			void operator=(ObjectView v) {
				name = "";
				name += v.GetName();
				SetCoords(v.GetCoords());
				use_cartesian = v.IsUsingCartesian();
			}
		};
	}
}
