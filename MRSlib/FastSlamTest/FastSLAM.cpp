#include "FastSLAM.h"
#include <MRS/Include/View2DPosition.hpp>
#include <iostream>
#include <cstdlib>

#define ACCURACY 1000
#define DIVIDER 1000.0
#define MIN_DISTANCE 40.0
#define READING_ERROR 0.2
#define MOVE_ERROR 0.3
namespace MRS {
	FastSLAM::FastSLAM() : particles(), origin(3.45, 0.2, 5)
	{
		current_time = 0;
		srand((unsigned int)current_time);
		particles.push_back((new Particle()));
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		particles.push_back(new Particle());
		for (auto p : particles) {
			//do shenanigans
			//printf("Size before: %d \n", p->GetSize());
			double rand_double = (rand() % ACCURACY) * abs(5) * MOVE_ERROR;
			double offset1 = rand_double / DIVIDER - (abs(5) * MOVE_ERROR * ACCURACY / DIVIDER / 2);
			offset1 /= 2;
			origin.SetX(origin.GetX()+offset1);
			p->SetPosition(&origin); //add randomness
			p->GetPosition()->SetP_X(MOVE_ERROR);
			printf("Position of p: x %f y %f p %f \n", p->GetPosition()->GetX(), p->GetPosition()->GetY(), p->GetPosition()->GetP_X());
			//printf("Size after: %d \n", p->GetSize());
		}
	}

	FastSLAM::~FastSLAM()
	{
	}

	void FastSLAM::Update(double delta_time, MRS::Device::View * view) // change to view
	{
		
		current_time += delta_time;
		srand((unsigned int)current_time);
		
		switch (view->GetType()) {
		case MRS::Device::ViewType::VIEW_POSITION_2D :
				// calculate movement
			{
			//std::cout << "Robot moved\n";
			double offset1 = 0;
			double offset2 = 0;
			double offset3 = 0;
			
			MRS::Device::View2DPosition * moved_view = (MRS::Device::View2DPosition*)view; //= *(p.GetPosition()) + MRS::Device::View2DPosition(1, 1, 3);
		
			for (auto p : particles) {
				try {
					double rand_double = (rand() % ACCURACY) * abs(moved_view->GetX()) * MOVE_ERROR;
					offset1 = (rand_double / DIVIDER - (abs(moved_view->GetX()) * MOVE_ERROR * ACCURACY / DIVIDER)/2);
					std::cout << "Changed by: " << offset1 << "\n";
						rand_double = (rand() % ACCURACY) * abs(moved_view->GetY()) * MOVE_ERROR;
					offset2 = rand_double / DIVIDER - (abs(moved_view->GetX()) * MOVE_ERROR * ACCURACY / DIVIDER);
					rand_double = (rand() % ACCURACY) * abs(moved_view->GetA()) * MOVE_ERROR;
					offset3 = rand_double / DIVIDER - (abs(moved_view->GetX()) * MOVE_ERROR * ACCURACY / DIVIDER); //ADD gausian distribution formula
					offset1 /= 2.0;
					offset2 /= 2.0;
					offset3 /= 2.0;
				}
				catch (...) {
					std::cout << "Numeric Error occurred \n";
				}
				//std::cout << "Robot moved\n";
				//moved = *(p->GetPosition()) + MRS::Device::View2DPosition(offset1, offset2, offset3);
				p->GetPosition()->SetX(p->GetPosition()->GetX() + moved_view->GetX() + offset1);
				p->GetPosition()->SetP_X(MOVE_ERROR);
				//p.UpdateObjectPositions();
			}
			std::cout << "Robot moved \n";
			}
			break;
		case MRS::Device::ViewType::VIEW_DISTANCE_POINT :
			{
				MRS::Device::DistanceView* dist_view = (MRS::Device::DistanceView*) view;
				//std::cout << "View info: OBJ " << dist_view->GetAngle() << " rad " << dist_view->GetDistance() << " m \n";
				bool is_known = false;
			// 1st check whether to obj is known
				for (auto p : particles) {
					printf("Size: %d \n", p->GetSize());
					if (p->GetSize() > 1) {
						is_known = is_known || (p->GetBestFit(dist_view) <= MIN_DISTANCE);
					}
				}
				//std::cout << "Checked known objects\n";
				double pos = 0;
				double prob = 0;
				if (is_known) {
			// 3rd a) hard part
					//std::cout << "Object is known\n";
					// 1st - get object prognosis
					for (auto p : particles) {
						MRS::Device::View2DPosition obj;
						MRS::Device::View2DPosition * obj_ptr = p->GetBestFitObject(dist_view);
						if (obj_ptr != nullptr) {
							obj = *obj_ptr;
							// 2nd - Kalman filter
							// Args: Obj_pos Obj_prob Particle p 
							VKF(p, &obj, dist_view, &pos, &prob);
							obj.SetX(pos);
							obj.SetP_X(prob);
							p->GetPosition()->SetP_X(prob);
						}
						else {
							// somthing went wrong
							std::cout << "No best fit for object - bug detected \n";
						}
					}
					std::deque<Particle*> temporary_particles;
					for (auto p : particles) {
						temporary_particles.emplace_back(new Particle(p));
					}
					int k = 0;
					double* w = new double[10]; // probabilities of particle selection
					double* b = new double[10]; // borders of particle selection
					// Collect particle worth in a single array
					for (auto p : particles) {
						w[k++] = p->GetWorth();
					}
					k = 0;
					double step = Sum(10, w) / 10;
					// Calculate borders
					for (auto p : particles) {
						if (k == 0) b[k] = p->GetWorth();
						else b[k] = b[k - 1] + p->GetWorth();
						k++;
					}
					k = 0;
					printf("w \t\tb\n");
					for (int i = 0; i < 10; i++) {
						printf("%f \t%f\n", w[i], b[i]);
					}
					// Replace particles
					for (auto p : particles) {
						*p = temporary_particles.at(GetIndex(10, k * step, b));
						//p->GetPosition()->SetP_X(MOVE_ERROR);
					}
					//std::cout << "Deleted doubles 1: before\n";
					//std::cout << "Deleted doubles 2: after deleting 1st array\n";
					//delete w;
					//std::cout << "Deleted doubles 3: after deleting 1st array ptr\n";
					delete[] w;
					delete[] b;

					for (auto p : temporary_particles) {
						delete p;
					}
				}
				else {
					//std
			// 3rd b) add obj to map::cout << "Object is unknown\n";
					for (auto p : particles) {
						MRS::Device::View2DPosition new_pos = *p->GetPosition() + MRS::Device::View2DPosition(dist_view->ToCartesianGetX(), dist_view->ToCartesianGetY());
						new_pos.SetP_X(0.1);
						int g = p->AddObject((int)p->GetSize() + 4, &new_pos);
						//printf("Result from add object: %d\n ",g);
					}
				}
			// 4th create the new particles
				//std::cout << "Updating wheights\n";
				//std::cout << "Deleted doubles 4: after deleting 2nd array\n";
				//delete b;
				//std::cout << "Deleted doubles 4: after deleting 2nd array ptr\n";
				//std::cout << "Deleted doubles\n";
				
			}
			break;
		default:
			break;
		}
		
		// update map
	}

	void FastSLAM::OutputMap()
	{
		// update map
		// create file
		// output map to file
		std::cout << "Map\n";
		int i = 0;
		for (auto p : particles) {
			std::cout << "Particle " << i <<"\n";
			std::cout << "Nr: " << i++ << "\t";
			std::cout << "Pos: " << p->GetPosition()->GetX() << "\t";
			std::cout << "Prob: " << p->GetPosition()->GetP_X() << "\t";
			std::cout << "Worth: " << p->GetWorth() << "\t";
			std::cout << "Objects: " << p->GetSize() << "\n";
			std::cout << p->OutputParticleInfo();
		}
	}

	std::string FastSLAM::OutputMapString()
	{
		std::string output = "";
		output.append("Map\n");
		int i = 0;
		for (auto p : particles) {
			output.append("Particle ");
			output.append(std::to_string(i));
			output.append("\n");
			output.append("Nr: ");
			output.append(std::to_string(i++));
			output.append("\t");
			output.append("Pos: ");
			output.append(std::to_string(p->GetPosition()->GetX()));
			output.append("\t");
			output.append("Prob: ");
			output.append(std::to_string(p->GetPosition()->GetP_X()));
			output.append("\t");
			output.append("Worth: ");
			output.append(std::to_string(p->GetWorth()));
			output.append("\t");
			output.append("Objects: ");
			output.append(std::to_string(p->GetSize()));
			output.append( "\n");
			output.append(p->OutputParticleInfo());
		}
		return output;
	}

	int FastSLAM::GetIndex(int len, double value, double* borders)
	{
		for (int i = 0; i < len; i++) {
			if (value < borders[i]) return i;
		}
		return len-1;
	}

	double FastSLAM::Sum(int len, double* values)
	{
		double sum = 0;
		for (int i = 0; i < len; i++) {
			sum += values[i];
		}
		return sum;
	}

	void FastSLAM::VKF(Particle* p, MRS::Device::View2DPosition* object, MRS::Device::DistanceView * sensor_reading, double* pos, double* prob)
	{
		// Position prognosis
		double X_p = object->GetX();
		// Probability prognosis
		double P = p->GetPosition()->GetP_X();
		// Inovation
		double Z = p->GetPosition()->GetX() + sensor_reading->ToCartesianGetX();
		double H = 1;
		double Y = Z - H * X_p;
		// Inovation kovariance
		double Ht = 1 / H;
		double Q = sensor_reading->ToCartesianGetX() * READING_ERROR;
		double S = H * P * Ht + Q;
		// Kalman gain
		double Sinv = 1 / S;
		double K = P * Ht * Sinv;
		// new position
		p->SetWorth(p->CalculateWorth(S, Y));
		*pos = X_p + K * Y;
		// new probability
		
		double I = 1;
		*prob = (I - K * H) * P;
		printf("X_p = %f; P_p = %f; Z = %f; H = %f; Y = %f; H_t = %f;\n Q = %f; S = %f; Sinv = %f; K = %f; X = %f, P = %f \n",
			X_p, P, Z, H, Y, Ht, Q, S, Sinv, K, *pos, *prob
		);
	}

}