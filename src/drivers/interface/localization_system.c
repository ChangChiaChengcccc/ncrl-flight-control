#include "optitrack.h"
#include "localization_system.h"

bool is_localization_info_available(void)
{
	return optitrack_available();
}

void get_enu_position(float *pos)
{
	optitrack_read_pos(pos);
}

void get_wgs84_position(float *latitude, float *longtitude, float *height)
{
}

void get_enu_velocity(float *vel)
{
	optitrack_read_vel(vel);
}
