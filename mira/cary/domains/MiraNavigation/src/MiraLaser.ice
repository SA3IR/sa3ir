#ifndef ROBOCOMPMIRALASER_ICE
#define ROBOCOMPMIRALASER_ICE

module RoboCompMiraLaser{

	["cpp:comparable"]
	struct PointT{
		float x;
		float y;
		float angle;
		float range;
	};

	sequence<PointT> LaserDataT;

	interface LaserReporter{
		void toggleLaserReport(bool enable);
	};

	interface Laser{
		void reportLaserData(LaserDataT data);
	};
};
  
#endif
