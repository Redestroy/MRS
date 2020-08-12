#include "Worldview.h"



Worldview::Worldview()//:attributes()
{
	//attributes.resize(8);
	//attributes[0] = new ViewPosition2D();
	attributeCount = 1;
	dataString = new char[29];
	absolutePosition = new ViewPosition2D(0.0f, 0.0f, 0.0f);
	obstacleView = new DistanceView();
	radValues[0] = 1.27f;
	radValues[1] = 0.77f;
	radValues[2] = 0.00f;
	radValues[3] = 5.21f;
	radValues[4] = 4.21f;
	radValues[5] = 3.14159f;
	radValues[6] = 2.37f;
	radValues[7] = 1.87f;
	for (int i = 0; i < 8; i++) {
		distances[i] = 0;
		sinValues[i] = (float)(sin(radValues[i]));
		cosValues[i] = (float)(cos(radValues[i]));
	}
}


Worldview::~Worldview()
{
	delete [] dataString;
}

bool Worldview::hasAttribute(TypeDefinitions::ViewType type)
{
	//for (View * v : attributes) {
	//	if (v->GetType() == type) return true;
	//}
	return false;
}

char Worldview::GetAttributePlace(TypeDefinitions::ViewType type)
{
	char i = 0;
	return -1;
}

void Worldview::ModifyAttribute(TypeDefinitions::ViewType type, View * nextAttribute)
{
	char i;
	if (( i = GetAttributePlace(type)) != (char)-1) {
	//	attributes[i] = nextAttribute;
	}
	else {
		//attributes[attributeCount] = nextAttribute;
		attributeCount++;
	}
}

void Worldview::Moved(float x, float y)
{
	//((ViewPosition2D *)attributes[0])->SetX(((ViewPosition2D *)attributes[0])->GetX() + x);
	//((ViewPosition2D *)attributes[0])->SetY(((ViewPosition2D *)attributes[0])->GetY() + y);
}

void Worldview::Moved(int x, int y)
{
	Moved((float)x, (float) y);
}

void Worldview::Update()
{
	//fullView[0] = frontView->GetData(0);
	//fullView[1] = floorView->GetData(0) | (0b00000011&(floorView->GetData(1)<<8));
	//fullView[2] = floorView->GetData(2) | (0b00000011 & (floorView->GetData(3) << 8));
	//fullView[3] = floorView->GetData(4) | (0b00000011 & (floorView->GetData(5) << 8));
	if (relativePosition != nullptr && origin != nullptr) {
		absolutePosition->SetX(origin->GetX() + relativePosition->GetX());
		absolutePosition->SetY(origin->GetY() + relativePosition->GetY());
		absolutePosition->SetA(origin->GetA() + relativePosition->GetA());
	}

	if (floorView != nullptr && obstacleView != nullptr) {
		sinTotal = 0;
		cosTotal = 0;
		for (int i = 0; i < 8; i++) {
			obstacleView->SetDistance(i, ((DistanceView*)floorView)->GetDistance(i));
			distances[i] = float(0.1f*obstacleView->GetDistance(i));
			sinTotal += sinValues[i] * distances[i];
			cosTotal += cosValues[i] * distances[i];
		}
	}
	//fullView[0] = frontView->GetData(0);
}

void Worldview::SetPositionView(View * pos)
{
	absolutePosition = (ViewPosition2D *)pos;
}

void Worldview::SetLocalPositionView(View * pos)
{
	relativePosition = (ViewPosition2D *)pos;
}

void Worldview::SetOrigin(View * pos)
{
	origin = (ViewPosition2D *)pos;
}

void Worldview::SetSectorView(View * sectorView)
{
	frontView = sectorView;
}

View * Worldview::GetLocalPositionView()
{
	return (View *)relativePosition;
}

View * Worldview::GetOrigin()
{
	return (View *)origin;
}

View * Worldview::GetPositionView()
{
	return (View *)absolutePosition;
}

View * Worldview::GetSectorView()
{
	return frontView;
}

void Worldview::SetFloorView(View * view)
{
	floorView = view;
	if (obstacleView == nullptr) obstacleView = new DistanceView(view);
}

View * Worldview::GetFloorView()
{
	return floorView;
}

char * Worldview::GetFullViewString()
{
	char * temp = dataString;
	temp[0] = 'V';
	temp[1] = ':';
	temp[2] = ' ';
	for (int i = 0; i < 5; i++) {
		temp[3 + i * 5] = toDecimal(fullView[i] / 1000);
		temp[3 + i * 5 + 1] = toDecimal((fullView[i] % 1000) / 100);
		temp[3 + i * 5 + 2] = toDecimal((fullView[i]%100) / 10);
		temp[3 + i * 5 + 3] = toDecimal(fullView[i] % 10);
		temp[3 + i * 5 + 4] = ' ';
	}
	temp[28] = '\n';
	temp[29] = '\0';
	return temp;
}

float Worldview::GetAverageDistanceSin()
{
	return sinTotal;
}

float Worldview::GetAverageDistanceCos()
{
	return cosTotal;
}

char Worldview::toDecimal(char a) {
	if (a >= 0 && a <=9) return (a+48);
	return -1;
}
