#pragma once
#ifndef CLASS_VIEW
#define CLASS_VIEW
#define DATA_SIZE 64
namespace MRS {
	namespace Device {
		enum class ViewType {
			VIEW_NULL,
			VIEW_DISTANCE_POINT,
			VIEW_OBJECT_2D,
			VIEW_OBJECTS_2D,
			VIEW_DISTANCE_ARC,
			VIEW_DETECTOR_SECTOR,
			VIEW_GRADIENT,
			VIEW_POSITION_2D,
			VIEW_VELOCITY_2D,
			VIEW_ACCELERATION_2D,
			VIEW_POSITION_3D,
			VIEW_VELOCITY_3D,
			VIEW_ACCELERATION_3D,
			VIEW_TEXT,
			VIEW_COMPLEX
		};

		class View
		{
		private:
			ViewType type;
			long timeCreated;
			unsigned char data[DATA_SIZE];

		protected:
			void SetType(ViewType);
			unsigned char * GetDataPtr();

		public:
			View();
			View(ViewType);
			~View();

			ViewType GetType() {
				return type;
			};

			void SetData(unsigned char, unsigned char);
			char GetData(unsigned char);

			virtual void onCreate(long);

			//virtual int GetAspect(int, TypeDefinitions::ViewType, char[]) = 0; // Writes the data value of the view to the buffer, returns buffer size in bytes/, args: aspect id, aspect type, buffer in which aspect is written

		};
	}
}
#endif
