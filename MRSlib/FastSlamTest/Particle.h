#pragma once
#include <map>
#include <deque>
#include <MRS/Include/View2DPosition.hpp>
#include <MRS/Include/DistanceView.hpp>
namespace MRS {

	class Particle
	{
	private:
		MRS::Device::View2DPosition position;
		std::deque<int> objectIds;
		std::map<int, MRS::Device::View2DPosition> objects;
		int me_size;
		static int number_of_particles;
		double worth;
		int id;
	public:
		Particle();
		Particle(Particle*);
		~Particle();

		void SetPosition(MRS::Device::View2DPosition* );
		MRS::Device::View2DPosition* GetPosition();

		std::deque<int>* GetObjectIds();
		std::map<int, MRS::Device::View2DPosition>* GetObjects();

		int AddObject(int, MRS::Device::View2DPosition *);
		int RemoveObject(int);
		int Clear();
		double GetBestFit(MRS::Device::DistanceView *);
		void UpdateObjectPositions();
		MRS::Device::View2DPosition * GetBestFitObject(MRS::Device::DistanceView*);
		std::deque<int>::iterator FindByValue(int id);
		int GetSize();

		void SetWorth(double);
		double GetWorth();
		double CalculateWorth(double S, double Y);

		void CopyObjectIds(std::deque<int>*);
		void CopyObjects(std::map<int, MRS::Device::View2DPosition>*);

		std::string OutputParticleInfo();

		void operator=(Particle p) {
			me_size = p.GetSize();
			worth = p.GetWorth();
			CopyObjects(p.GetObjects());
			SetPosition(p.GetPosition());
			CopyObjectIds(p.GetObjectIds());	
		}

		void operator=(Particle * p) {
			me_size = p->GetSize();
			worth = p->GetWorth();
			CopyObjects(p->GetObjects());
			SetPosition(p->GetPosition());
			CopyObjectIds(p->GetObjectIds());

		}

		void Copy(Particle* p) {
			me_size = p->GetSize();
			worth = p->GetWorth();
			CopyObjects(p->GetObjects());
			SetPosition(p->GetPosition());
			CopyObjectIds(p->GetObjectIds());
		}
	};
}
