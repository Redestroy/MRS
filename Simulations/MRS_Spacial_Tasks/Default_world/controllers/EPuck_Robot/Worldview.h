
#ifndef WORLDVIEW
#define WORLDVIEW
#include "View.h"
//#include <vector>
#include "ViewPosition2D.h"
#include "DistanceView.h"
#include <math.h>
//#include "ViewText.h"
class Worldview
{
private:
	//LinkedList<View> views;
	//std::vector<View*> attributes;
	unsigned int attributeCount;
	unsigned int fullView[5];
	char * dataString;
	char toDecimal(char);
	View * frontView;
	View * floorView;
	DistanceView * obstacleView;
	ViewPosition2D * relativePosition;
	ViewPosition2D * absolutePosition;
	ViewPosition2D * origin;
	float distances[8];
	float radValues[8];
	float sinValues[8];
	float cosValues[8];
	float sinTotal, cosTotal;
public:
	Worldview();
	~Worldview();

	bool hasAttribute(TypeDefinitions::ViewType);
	char GetAttributePlace(TypeDefinitions::ViewType);
	//ViewPosition2D * GetPosition2D();
	//ViewText * GetTextString();
	void ModifyAttribute(TypeDefinitions::ViewType, View *);
	void Moved(float, float);
	void Moved(int, int);
	void Update();
	void SetPositionView(View *);
	void SetLocalPositionView(View *);
	void SetOrigin(View *);
	void SetSectorView(View *);
	View * GetLocalPositionView();
	View * GetOrigin();
	View * GetPositionView();
	View * GetSectorView();
	void SetFloorView(View *);
	View * GetFloorView();
	char * GetFullViewString();
	float GetAverageDistanceSin();
	float GetAverageDistanceCos();
	// Some method templates
	//void GenerateDetectorOutput(char [][] detectorsOut, long timeStart, long timeEnd);
	//void GenerateGradientOutput(char [][] detectorsOut, long timeStart, long timeEnd);
	//void GenerateDistanceOutput(char [][] distancesOut, long timeStart, long timeEnd);
	//void GenerateTrajectory(char [][][] trajectoryOut, long timeStart, long timeEnd);
	//void GenerateObjectMap(char [][][][] mapOut, long timeStart,long timeEnd); // This generates map from trajectory + Distance output
	//void GenerateMarkerMap(char [][][][] mapOut, long timeStart,long timeEnd); // This generates map from trajectory + Detector output
};
#endif

