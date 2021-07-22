#include "../Include/Worldview.hpp"

namespace MRS{
	namespace Environment {

		Worldview::Worldview()
		{
			central_timer.start();
		}

		Worldview::~Worldview()
		{
		}

		void Worldview::AddView(std::string name, Device::View* view, Device::ViewType type)
		{
			//check if deck of type exists
			//if yes add to deck
			//if no create new deque
		}

		void Worldview::RemoveView()
		{
			//delete view from deck
			// if no views of type remain, delete deque
		}

		Device::View* Worldview::GetView(std::string name)
		{
			// get view pointer by string
			for (auto p : *all_views) {
				if (p.second.count(name) != 0) {
					return p.second.at(name);
				}
			}
			return nullptr;
		}

		Device::ViewType Worldview::GetViewType(std::string name)
		{
			return GetView(name)->GetType();
		}

		void Worldview::SetWorldviewType(WorldviewType type)
		{
			this->world_type = type;
		}

		WorldviewType Worldview::GetWorldviewType()
		{
			return world_type;
		}

		bool Worldview::GetPredicateValue(std::string name)
		{
			return predicates.at(name);
		}

		Predicate Worldview::GetPredicate(std::string name)
		{
			Predicate p;
			p.name = name;
			p.value = predicates.at(name);
			return p;
		}

		double Worldview::GetParameterValue(std::string name)
		{
			return parameters.at(name);
		}

		std::pair<std::string, double> Worldview::GetParameter(std::string name)
		{
			return std::pair<std::string, double>();
		}

		boost::timer::nanosecond_type Worldview::GetTime()
		{
			return time;
		}

		void Worldview::Update()
		{
			time = central_timer.elapsed().system;
			OnUpdate();
		}

	}
}
