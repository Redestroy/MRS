#pragma once
#include "../../MRS/Include/Condition.hpp"
#include <bitset>
namespace MRS {
	namespace Algorithms {

		template <std::size_t bit_size>
		class State {
		private:
			std::bitset<bit_size> state_data;
		protected:
			std::bitset<bit_size>* GetData() {
				return &state_data;
			}

			void SetData(std::bitset<bit_size>* data) {
				for (std::size_t i = 0; i < state_data.size(); i++) {
					state_data[i] = data->test(i);
				}
			}

		public:
			State() {}
			~State() {}
			virtual bool GetBinaryState() = 0;
			virtual unsigned char GetTriState() = 0;
			virtual int GetStateAsInt() = 0;
			virtual unsigned char* GetStateAsBitArray() = 0;

			bool operator==(State* s) {
				return state_data == s->GetData();
			}

		};

		class BinaryState : public State<1> {
		private:
			bool binary_state;
			unsigned char tri_state;
			unsigned char integer;
			unsigned char character_array;
		public:
			bool GetBinaryState() {
				return binary_state;
			}
			unsigned char GetTriState() {
				return tri_state;
			}
			int GetStateAsInt() {
				return (int)integer;
			}
			unsigned char* GetStateAsBitArray(){
				return &character_array;
			}
			void SetState(bool b) {
				binary_state = b;
				if (b) {
					tri_state = 1;
					integer = 1;
					character_array = 1;
				}
				else {
					tri_state = -1;
					integer = 0;
					character_array = 0;
				}
				GetData()[0] = b;
			}
		};
	}
}