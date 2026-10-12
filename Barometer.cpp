#include "Barometer.hpp"

#include <cmath>

namespace calibration
{

Barometer::Barometer(uint32_t device_id, bool external) :
	_device_id(device_id),
	_external(external)
{
	Reset();
}

void Barometer::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		Reset();
	}
}

void Barometer::set_external(bool external)
{
	if (_external != external) {
		_external = external;
		Reset();
	}
}

void Barometer::set_thermal_offset(float offset)
{
	if (std::isfinite(offset)) {
		_thermal_offset = offset;
	}
}

bool Barometer::set_offset(float offset)
{
	if (std::isfinite(offset) && std::fabs(_offset - offset) > 0.01f) {
		_offset = offset;
		++_calibration_count;
		return true;
	}

	return false;
}

bool Barometer::set_calibration_index(int index)
{
	if (index >= 0 && index < MAX_SENSOR_COUNT) {
		_calibration_index = static_cast<int8_t>(index);
		return true;
	}

	return false;
}

void Barometer::Reset()
{
	_offset = 0.f;
	_thermal_offset = 0.f;
	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;
	_calibration_index = -1;
	_calibration_count = 0;
}

} // namespace calibration
