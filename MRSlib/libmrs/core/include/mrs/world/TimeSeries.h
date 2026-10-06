#pragma once
// Fixed-capacity history of stamped values (spec 05 §5.4).
#include <cstddef>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace MRS {
	namespace Environment {
		struct Sample {
			double stamp = 0.0;
			double value = 0.0;
		};

		class TimeSeries {
		public:
			explicit TimeSeries(std::size_t capacity = 256) : capacity_(capacity == 0 ? 1 : capacity) {}

			// Older samples than the newest are ignored; an equal stamp replaces the newest.
			void Add(double stamp, double value) {
				if (!samples_.empty()) {
					if (stamp < samples_.back().stamp) return;
					if (stamp == samples_.back().stamp) {
						samples_.back().value = value;
						return;
					}
				}
				samples_.push_back({stamp, value});
				while (samples_.size() > capacity_) samples_.pop_front();
			}

			std::optional<Sample> Latest() const {
				if (samples_.empty()) return std::nullopt;
				return samples_.back();
			}

			// Linear interpolation between the samples around t; nothing outside them.
			std::optional<double> At(double t) const {
				if (samples_.empty() || t < samples_.front().stamp || t > samples_.back().stamp) return std::nullopt;
				for (std::size_t k = 1; k < samples_.size(); ++k) {
					const Sample& b = samples_[k];
					if (t > b.stamp) continue;
					const Sample& a = samples_[k - 1];
					const double span = b.stamp - a.stamp;
					return span <= 0.0 ? b.value : a.value + (b.value - a.value) * (t - a.stamp) / span;
				}
				return samples_.back().value;
			}

			// Samples with t0 <= stamp <= t1, oldest first.
			std::vector<Sample> Window(double t0, double t1) const {
				std::vector<Sample> out;
				for (const auto& s : samples_)
					if (s.stamp >= t0 && s.stamp <= t1) out.push_back(s);
				return out;
			}

			std::size_t Size() const { return samples_.size(); }
			std::size_t Capacity() const { return capacity_; }
			void SetCapacity(std::size_t capacity) {
				capacity_ = capacity == 0 ? 1 : capacity;
				while (samples_.size() > capacity_) samples_.pop_front();
			}

		private:
			std::size_t capacity_;
			std::deque<Sample> samples_;
		};
	}
}
