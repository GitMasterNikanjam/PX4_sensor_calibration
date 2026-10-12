#include "Gyroscope.hpp"

namespace calibration
{

using matrix::Vector3f;

Gyroscope::Gyroscope(uint32_t device_id, bool external) :
	_device_id(device_id),
	_external(external)
{
	Reset();
}

void Gyroscope::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		Reset();
	}
}

void Gyroscope::set_external(bool external)
{
	if (_external != external) {
		_external = external;
		Reset();
	}
}

void Gyroscope::set_thermal_offset(const Vector3f &offset)
{
	if (offset.isAllFinite()) {
		_thermal_offset = offset;
	}
}

bool Gyroscope::set_offset(const Vector3f &offset)
{
	if (offset.isAllFinite() && (Vector3f(_offset - offset).longerThan(0.01f) || _calibration_count == 0)) {
		_offset = offset;
		++_calibration_count;
		return true;
	}

	return false;
}

bool Gyroscope::set_calibration_index(int index)
{
	if (index >= 0 && index < MAX_SENSOR_COUNT) {
		_calibration_index = static_cast<int8_t>(index);
		return true;
	}

	return false;
}

void Gyroscope::Reset()
{
	_rotation.setIdentity();
	_offset.zero();
	_thermal_offset.zero();
	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;
	_calibration_index = -1;
	_calibration_count = 0;
}

Vector3f Gyroscope::Correct(const Vector3f &data) const
{
	return _rotation * Vector3f{data - _thermal_offset - _offset};
}

Vector3f Gyroscope::Uncorrect(const Vector3f &corrected_data) const
{
	return (_rotation.T() * corrected_data) + _thermal_offset + _offset;
}

Vector3f Gyroscope::BiasCorrectedSensorOffset(const Vector3f &bias) const
{
	return _offset + (_rotation.T() * bias);
}

} // namespace calibration
