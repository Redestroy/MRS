#include "EpuckEncoderSet.h"
#include <corecrt_math.h>



EpuckEncoderSet::EpuckEncoderSet() : Sensor()
{
	SetType(TypeDefinitions::SENSOR_ENCODER);
	ENCODER_CPM = 0.025;
	countLeft = 0; 
	countRight = 0;
	velLeft = 0.0;
	velRight = 0.0;
	TIME_STEP = 8;
}

EpuckEncoderSet::EpuckEncoderSet(webots::Robot * robot) : Sensor()
{
	SetType(TypeDefinitions::SENSOR_ENCODER);
	leftEncoder = robot->getPositionSensor("left wheel sensor");
	rightEncoder = robot->getPositionSensor("right wheel sensor");
	ENCODER_CPM = 0.025;
	countLeft = 0;
	countRight = 0;
	velLeft = 0.0;
	velRight = 0.0;
	TIME_STEP = 8;
	leftEncoder->enable(TIME_STEP);
	rightEncoder->enable(TIME_STEP);
}

EpuckEncoderSet::EpuckEncoderSet(webots::PositionSensor * left, webots::PositionSensor * right) : Sensor()
{
	SetType(TypeDefinitions::SENSOR_ENCODER);
	leftEncoder = left;
	rightEncoder = right;
	ENCODER_CPM = 0.025;
	countLeft = 0;
	countRight = 0;
	velLeft = 0.0;
	velRight = 0.0;
	TIME_STEP = 8;
	leftEncoder->enable(TIME_STEP);
	rightEncoder->enable(TIME_STEP);
}


EpuckEncoderSet::~EpuckEncoderSet()
{
}

void EpuckEncoderSet::CreateView(Worldview * world)
{
	View * pos = new ViewPosition2D();
	((ViewPosition2D *)pos)->SetX(0);
	((ViewPosition2D *)pos)->SetY(0);
	((ViewPosition2D *)pos)->SetA(0);
	SetView(pos);
	world->SetLocalPositionView(pos);
}

void EpuckEncoderSet::Update()
{
	double x, y, a;
	double deltaCountLeft, deltaCountRight;
	double deltaLeft, deltaRight;
	double deltaLin, deltaAng, deltaAngDeg, Ang, AngDeg;
	deltaCountLeft = leftEncoder->getValue();
	deltaCountRight = rightEncoder->getValue();
	
	//add fancy math here
	// fancy math should probably done in a world class method;
	deltaLeft = (deltaCountLeft - countLeft)/ENCODER_CPM/2;//deltaCountLeft * 2 * 3.14159 * 0.0205/ENCODER_CPR;//replace consts with motor consts later
	deltaRight = (deltaCountRight - countRight)/ENCODER_CPM/2;//deltaCountRight * 2 * 3.14159 * 0.0205 / ENCODER_CPR;
	countLeft = deltaCountLeft;
	countRight = deltaCountRight;

	deltaLin = (deltaRight + deltaLeft) / 2;
	deltaAng = (deltaRight - deltaLeft) / 52;
	deltaAngDeg = deltaAng * 180 / 3.14159;
	Ang = ((ViewPosition2D *)(this->GetView()))->GetA();
	AngDeg = Ang * 180 / 3.14159;
	//printf("dL: %f\t dA %f", deltaLin, deltaAng);

#ifdef DEG_TRIG
	if (deltaAng > 0.0005 || deltaAng < -0.0005) {
		x = ((ViewPosition2D *)(this->GetView()))->GetX() + (sin(0.5*deltaAngDeg) / (0.5*deltaAngDeg))*deltaLin*cos(0.5*deltaAngDeg + AngDeg);
		y = ((ViewPosition2D *)(this->GetView()))->GetY() + (sin(0.5*deltaAngDeg) / (0.5*deltaAngDeg))*deltaLin*sin(0.5*deltaAngDeg + AngDeg);
		a = ((ViewPosition2D *)(this->GetView()))->GetA() + deltaAng;
	}
	else {
		x = ((ViewPosition2D *)(this->GetView()))->GetX() + deltaLin*cos(0.5*deltaAngDeg + AngDeg);
		y = ((ViewPosition2D *)(this->GetView()))->GetY() + deltaLin*sin(0.5*deltaAngDeg + AngDeg);
		a = ((ViewPosition2D *)(this->GetView()))->GetA() + deltaAng;
	}
#else
	if (deltaAng > 0.0005 || deltaAng < -0.0005) {
		x = ((ViewPosition2D *)(this->GetView()))->GetX() + (sin(0.5*deltaAng) / (0.5*deltaAng))*deltaLin*cos(0.5*deltaAng + Ang);
		y = ((ViewPosition2D *)(this->GetView()))->GetY() + (sin(0.5*deltaAng) / (0.5*deltaAng))*deltaLin*sin(0.5*deltaAng + Ang);
		a = ((ViewPosition2D *)(this->GetView()))->GetA() + deltaAng;
	}
	else {
		x = ((ViewPosition2D *)(this->GetView()))->GetX() + deltaLin * cos(0.5*deltaAng + Ang);
		y = ((ViewPosition2D *)(this->GetView()))->GetY() + deltaLin * sin(0.5*deltaAng + Ang);
		a = ((ViewPosition2D *)(this->GetView()))->GetA() + deltaAng;
	}
#endif
	a = std::fmod(a, 6.28318);
	if (a > 3.14159) a = (a - 6.28318);
	else if (a < -3.14159) a = (a + 6.28318);
	//a = std::fmod(a, 3.14159);// -3.14159;

	((ViewPosition2D *)(this->GetView()))->SetX((float)x);//use fancy math
	((ViewPosition2D *)(this->GetView()))->SetY((float)y);//use fancy math
	((ViewPosition2D *)(this->GetView()))->SetA((float)a);//use fancy math
}
