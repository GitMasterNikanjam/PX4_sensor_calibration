#include "Accelerometer.hpp"

namespace calibration
{

using matrix::Vector3f;

Accelerometer::Accelerometer(uint32_t device_id, bool external) :
	_device_id(device_id),
	_external(external)
{
	Reset();
}

void Accelerometer::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		Reset();
	}
}

void Accelerometer::set_external(bool external)
{
	if (_external != external) {
		_external = external;
		Reset();
	}
}

void Accelerometer::set_thermal_offset(const Vector3f &offset)
{
	if (offset.isAllFinite()) {
		_thermal_offset = offset;
	}
}

bool Accelerometer::set_offset(const Vector3f &offset)
{
	if (offset.isAllFinite() && Vector3f(_offset - offset).longerThan(0.01f)) {
		_offset = offset;
		++_calibration_count;
		return true;
	}

	return false;
}

bool Accelerometer::set_scale(const Vector3f &scale)
{
	if (scale.isAllFinite() && scale(0) > 0.f && scale(1) > 0.f && scale(2) > 0.f
	    && Vector3f(_scale - scale).longerThan(0.01f)) {
		_scale = scale;
		++_calibration_count;
		return true;
	}

	return false;
}

bool Accelerometer::set_calibration_index(int index)
{
	if (index >= 0 && index < MAX_SENSOR_COUNT) {
		_calibration_index = static_cast<int8_t>(index);
		return true;
	}

	return false;
}

void Accelerometer::Reset()
{
	_offset.zero();
	_scale = Vector3f{1.f, 1.f, 1.f};
	_thermal_offset.zero();
	_rotation.setIdentity();
	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;
	_calibration_index = -1;
	_calibration_count = 0;
}

Vector3f Accelerometer::Correct(const Vector3f &sample) const
{
	return _rotation * Vector3f{(sample - _thermal_offset - _offset).emult(_scale)};
}

Vector3f Accelerometer::BiasCorrectedSensorOffset(const Vector3f &bias) const
{
	return _offset + (_rotation.T() * bias).edivide(_scale);
}

} // namespace calibration
