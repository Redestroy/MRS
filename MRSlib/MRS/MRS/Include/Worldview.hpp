#pragma once
#include "View.hpp"
#include <map>
#include <string>
namespace MRS {
	namespace Environment {
		enum class WorldviewType
		{
			Spatial1D,	// line world - operates with 1d line position
			Spatial2D,	// 2d world - operates on a 2d plane
			Spatial3D,	// 3D world	- operates in 3d space
			Reactive,	// pure sensor input defines action output, lookup table
			SymbolicLogic, // world defined by logic expressions
			ComplexWorld, // worldview composed of multiple different worldviews
			Internet,	// world for representing web agents worldview
			Topological // World represented by a graph

		};

		class Worldview
		{
		private:
			std::multimap<MRS::Device::ViewType, std::map<std::string, Device::View*>, std::greater_equal<MRS::Device::ViewType>>* all_views;
			WorldviewType world_type;
			//LinkedList<View> views;
			//std::vector<View*> attributes;
			//unsigned int attributeCount;
			//unsigned int fullView[5];
			//char* dataString;
			//char toDecimal(char);
			//View* frontView;
			//View* floorView;
			//DistanceView* obstacleView;
			//ViewPosition2D* relativePosition;
			//ViewPosition2D* absolutePosition;
			//ViewPosition2D* origin;
			///float distances[8];
			//float radValues[8];
			//float sinValues[8];
			//float cosValues[8];
			//float sinTotal, cosTotal;
		public:
			Worldview();
			~Worldview();

			void AddView(std::string, Device::View*, Device::ViewType);
			void RemoveView();
			Device::View* GetView(std::string);
			Device::ViewType GetViewType(std::string);
			
			void SetWorldviewType(WorldviewType);
			WorldviewType GetWorldviewType();
			//bool hasAttribute(TypeDefinitions::ViewType);
			//char GetAttributePlace(TypeDefinitions::ViewType);
			//ViewPosition2D * GetPosition2D();
			//ViewText * GetTextString();
			//void ModifyAttribute(TypeDefinitions::ViewType, View*);
			//void Moved(float, float);
			//void Moved(int, int);
			//void Update();
			//void SetPositionView(View*);
			//void SetLocalPositionView(View*);
			//void SetOrigin(View*);
			//void SetSectorView(View*);
			//View* GetLocalPositionView();
			//View* GetOrigin();
			//View* GetPositionView();
			//View* GetSectorView();
			//void SetFloorView(View*);
			//View* GetFloorView();
			//char* GetFullViewString();
			//float GetAverageDistanceSin();
			//float GetAverageDistanceCos();
			// Some method templates
			//void GenerateDetectorOutput(char [][] detectorsOut, long timeStart, long timeEnd);
			//void GenerateGradientOutput(char [][] detectorsOut, long timeStart, long timeEnd);
			//void GenerateDistanceOutput(char [][] distancesOut, long timeStart, long timeEnd);
			//void GenerateTrajectory(char [][][] trajectoryOut, long timeStart, long timeEnd);
			//void GenerateObjectMap(char [][][][] mapOut, long timeStart,long timeEnd); // This generates map from trajectory + Distance output
			//void GenerateMarkerMap(char [][][][] mapOut, long timeStart,long timeEnd); // This generates map from trajectory + Detector output
		};
	}
}