#include "Magnetometer.hpp"

#include <cmath>

namespace calibration
{

using matrix::Vector3f;

Magnetometer::Magnetometer(uint32_t device_id, bool external) :
	_device_id(device_id),
	_external(external)
{
	Reset();
}

void Magnetometer::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		Reset();
	}
}

void Magnetometer::set_external(bool external)
{
	if (_external != external) {
		_external = external;
		Reset();
	}
}

void Magnetometer::set_thermal_offset(const Vector3f &offset)
{
	if (offset.isAllFinite()) {
		_thermal_offset = offset;
	}
}

bool Magnetometer::set_offset(const Vector3f &offset)
{
	if (offset.isAllFinite() && Vector3f(_offset - offset).longerThan(0.005f)) {
		_offset = offset;
		++_calibration_count;
		return true;
	}

	return false;
}

bool Magnetometer::set_scale(const Vector3f &diagonal_scale)
{
	if (diagonal_scale.isAllFinite() && diagonal_scale(0) > 0.f && diagonal_scale(1) > 0.f
	    && diagonal_scale(2) > 0.f && Vector3f(_scale.diag() - diagonal_scale).longerThan(0.01f)) {
		_scale(0, 0) = diagonal_scale(0);
		_scale(1, 1) = diagonal_scale(1);
		_scale(2, 2) = diagonal_scale(2);
		++_calibration_count;
		return true;
	}

	return false;
}

bool Magnetometer::set_offdiagonal(const Vector3f &offdiagonal)
{
	const Vector3f current{_scale(0, 1), _scale(0, 2), _scale(1, 2)};

	if (offdiagonal.isAllFinite() && Vector3f(current - offdiagonal).longerThan(0.01f)) {
		_scale(0, 1) = offdiagonal(0);
		_scale(1, 0) = offdiagonal(0);
		_scale(0, 2) = offdiagonal(1);
		_scale(2, 0) = offdiagonal(1);
		_scale(1, 2) = offdiagonal(2);
		_scale(2, 1) = offdiagonal(2);
		++_calibration_count;
		return true;
	}

	return false;
}

void Magnetometer::set_power_compensation(const Vector3f &compensation)
{
	if (compensation.isAllFinite()) {
		_power_compensation = compensation;
	}
}

void Magnetometer::UpdatePower(float power)
{
	if (std::isfinite(power)) {
		_power = power;
	}
}

bool Magnetometer::set_calibration_index(int index)
{
	if (index >= 0 && index < MAX_SENSOR_COUNT) {
		_calibration_index = static_cast<int8_t>(index);
		return true;
	}

	return false;
}

void Magnetometer::Reset()
{
	_rotation.setIdentity();
	_offset.zero();
	_scale.setIdentity();
	_thermal_offset.zero();
	_power_compensation.zero();
	_power = 0.f;
	_priority = _external ? DEFAULT_EXTERNAL_PRIORITY : DEFAULT_PRIORITY;
	_calibration_index = -1;
	_calibration_count = 0;
}

Vector3f Magnetometer::Correct(const Vector3f &data) const
{
	const Vector3f power_corrected = data + _power * _power_compensation;
	return _rotation * (_scale * (power_corrected - _thermal_offset - _offset));
}

Vector3f Magnetometer::BiasCorrectedSensorOffset(const Vector3f &bias) const
{
	return _offset + (_scale.I() * _rotation.T() * bias);
}

} // namespace calibration
