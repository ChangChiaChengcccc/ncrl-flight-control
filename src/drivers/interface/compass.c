#include <stdint.h>
#include <stdbool.h>
#include "proj_config.h"
#include "ist8310.h"
#include "debug_link.h"

void get_imu_compass_raw(float *mag_raw)
{
	ist8310_get_raw_mag(mag_raw);
}

bool is_compass_present(void)
{
#if (SELECT_LOCALIZATION == LOCALIZATION_USE_GPS_MAG) || \
    (SELECT_LOCALIZATION == LOCALIZATION_USE_OPTITRACK)
	return true;
#else
	return false;
#endif
}

float get_imu_compass_update_freq(void)
{
	return ist8310_get_update_freq();
}

float get_imu_compass_raw_strength(void)
{
	return ist8310_get_raw_mag_strength();
}

void send_compass_debug_message(debug_msg_t *payload)
{
	float mag_raw[3] = {0.0f};
	get_imu_compass_raw(mag_raw);

	float mag_strength = get_imu_compass_raw_strength();
	float update_freq = get_imu_compass_update_freq();

	pack_debug_debug_message_header(payload, MESSAGE_ID_COMPASS);
	pack_debug_debug_message_float(&mag_raw[0], payload);
	pack_debug_debug_message_float(&mag_raw[1], payload);
	pack_debug_debug_message_float(&mag_raw[2], payload);
	pack_debug_debug_message_float(&mag_strength, payload);
	pack_debug_debug_message_float(&update_freq, payload);
}