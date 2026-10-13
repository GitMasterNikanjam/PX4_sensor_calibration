#pragma once

#include <src/math.hpp>
#include <cstdint>

namespace calibration
{

/** Host-provided persistence interface for calibration values.
 * Implement these operations for the application's preferred storage backend.
 */
class CalibrationStore
{
public:
	virtual ~CalibrationStore() = default;
	virtual bool get_int32(const char *key, int32_t &value) const = 0;
	virtual bool get_float(const char *key, float &value) const = 0;
	virtual bool set_int32(const char *key, int32_t value) = 0;
	virtual bool set_float(const char *key, float value) = 0;
};

int8_t FindCurrentCalibrationIndex(const CalibrationStore &store, const char *sensor_type, uint32_t device_id);
int8_t FindAvailableCalibrationIndex(const CalibrationStore &store, const char *sensor_type, uint32_t device_id,
				     int8_t preferred_index = -1);

int32_t GetCalibrationParamInt32(const CalibrationStore &store, const char *sensor_type, const char *cal_type,
				 uint8_t instance);
float GetCalibrationParamFloat(const CalibrationStore &store, const char *sensor_type, const char *cal_type,
			       uint8_t instance);
bool SetCalibrationParam(CalibrationStore &store, const char *sensor_type, const char *cal_type, uint8_t instance,
			 int32_t value);
bool SetCalibrationParam(CalibrationStore &store, const char *sensor_type, const char *cal_type, uint8_t instance,
			 float value);

matrix::Vector3f GetCalibrationParamsVector3f(const CalibrationStore &store, const char *sensor_type,
		const char *cal_type, uint8_t instance);
bool SetCalibrationParamsVector3f(CalibrationStore &store, const char *sensor_type, const char *cal_type,
				  uint8_t instance, const matrix::Vector3f &values);

} // namespace calibration
