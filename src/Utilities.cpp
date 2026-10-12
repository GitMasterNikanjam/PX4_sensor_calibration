#include "Utilities.hpp"

#include <cmath>
#include <cstdio>

namespace calibration
{
namespace
{

constexpr int MAX_SENSOR_COUNT = 4;

bool MakeScalarKey(char *buffer, size_t size, const char *sensor_type, uint8_t instance, const char *cal_type)
{
	if (sensor_type == nullptr || cal_type == nullptr) {
		return false;
	}

	const int length = std::snprintf(buffer, size, "CAL_%s%u_%s", sensor_type, static_cast<unsigned>(instance), cal_type);
	return length >= 0 && static_cast<size_t>(length) < size;
}

bool MakeVectorKey(char *buffer, size_t size, const char *sensor_type, uint8_t instance, char axis, const char *cal_type)
{
	if (sensor_type == nullptr || cal_type == nullptr) {
		return false;
	}

	const int length = std::snprintf(buffer, size, "CAL_%s%u_%c%s", sensor_type, static_cast<unsigned>(instance), axis,
					 cal_type);
	return length >= 0 && static_cast<size_t>(length) < size;
}

} // namespace

int8_t FindCurrentCalibrationIndex(const CalibrationStore &store, const char *sensor_type, uint32_t device_id)
{
	if (device_id == 0 || sensor_type == nullptr) {
		return -1;
	}

	for (uint8_t index = 0; index < MAX_SENSOR_COUNT; ++index) {
		char key[64] {};
		int32_t stored_id = 0;

		if (MakeScalarKey(key, sizeof(key), sensor_type, index, "ID")
		    && store.get_int32(key, stored_id) && static_cast<uint32_t>(stored_id) == device_id) {
			return static_cast<int8_t>(index);
		}
	}

	return -1;
}

int8_t FindAvailableCalibrationIndex(const CalibrationStore &store, const char *sensor_type, uint32_t device_id,
				     int8_t preferred_index)
{
	const int8_t current = FindCurrentCalibrationIndex(store, sensor_type, device_id);

	if (current >= 0) {
		return current;
	}

	bool available[MAX_SENSOR_COUNT] {};

	for (uint8_t index = 0; index < MAX_SENSOR_COUNT; ++index) {
		char key[64] {};
		int32_t stored_id = 0;
		const bool found = MakeScalarKey(key, sizeof(key), sensor_type, index, "ID") && store.get_int32(key, stored_id);
		available[index] = !found || stored_id == 0;
	}

	if (preferred_index >= 0 && preferred_index < MAX_SENSOR_COUNT && available[preferred_index]) {
		return preferred_index;
	}

	for (int8_t index = 0; index < MAX_SENSOR_COUNT; ++index) {
		if (available[index]) {
			return index;
		}
	}

	return -1;
}

int32_t GetCalibrationParamInt32(const CalibrationStore &store, const char *sensor_type, const char *cal_type,
				 uint8_t instance)
{
	char key[64] {};
	int32_t value = 0;

	if (MakeScalarKey(key, sizeof(key), sensor_type, instance, cal_type)) {
		store.get_int32(key, value);
	}

	return value;
}

float GetCalibrationParamFloat(const CalibrationStore &store, const char *sensor_type, const char *cal_type,
			       uint8_t instance)
{
	char key[64] {};
	float value = NAN;

	if (MakeScalarKey(key, sizeof(key), sensor_type, instance, cal_type)) {
		store.get_float(key, value);
	}

	return value;
}

bool SetCalibrationParam(CalibrationStore &store, const char *sensor_type, const char *cal_type, uint8_t instance,
			 int32_t value)
{
	char key[64] {};
	return MakeScalarKey(key, sizeof(key), sensor_type, instance, cal_type) && store.set_int32(key, value);
}

bool SetCalibrationParam(CalibrationStore &store, const char *sensor_type, const char *cal_type, uint8_t instance,
			 float value)
{
	char key[64] {};
	return MakeScalarKey(key, sizeof(key), sensor_type, instance, cal_type) && store.set_float(key, value);
}

matrix::Vector3f GetCalibrationParamsVector3f(const CalibrationStore &store, const char *sensor_type,
		const char *cal_type, uint8_t instance)
{
	matrix::Vector3f values{};
	char key[64] {};
	static constexpr char axes[] {'X', 'Y', 'Z'};

	for (int axis = 0; axis < 3; ++axis) {
		if (MakeVectorKey(key, sizeof(key), sensor_type, instance, axes[axis], cal_type)) {
			store.get_float(key, values(axis));
		}
	}

	return values;
}

bool SetCalibrationParamsVector3f(CalibrationStore &store, const char *sensor_type, const char *cal_type,
				  uint8_t instance, const matrix::Vector3f &values)
{
	char key[64] {};
	static constexpr char axes[] {'X', 'Y', 'Z'};
	bool success = true;

	for (int axis = 0; axis < 3; ++axis) {
		if (!MakeVectorKey(key, sizeof(key), sensor_type, instance, axes[axis], cal_type)
		    || !store.set_float(key, values(axis))) {
			success = false;
		}
	}

	return success;
}

} // namespace calibration
