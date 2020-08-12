#include "DistanceView.h"



DistanceView::DistanceView() : View(TypeDefinitions::VIEW_DISTANCE_POINT)
{
}

DistanceView::DistanceView(View * view)
{
	DistanceView * v = (DistanceView*)(view);
	for (int i = 0; i < 16; i++) {
		distances[i] = v->GetDistance(i);
	}
}


DistanceView::~DistanceView()
{
}

void DistanceView::SetDistance(unsigned char pos, double value)
{
	if (pos < 16) distances[pos] = value;
}

double DistanceView::GetDistance(unsigned char pos)
{
	if (pos < 16) return distances[pos];
	return -1.0;
}
