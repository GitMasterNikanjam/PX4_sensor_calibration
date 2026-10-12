#pragma once

#include <matrix/math.hpp>
#include <cstdint>

namespace calibration
{

/** Gyroscope calibration state and measurement correction.
 * Device discovery, orientation, thermal compensation and persistence belong
 * to the host application.
 */
class Gyroscope
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;
	static constexpr uint8_t DEFAULT_PRIORITY = 50;
	static constexpr uint8_t DEFAULT_EXTERNAL_PRIORITY = 75;
	static constexpr const char *SensorString() { return "GYRO"; }

	explicit Gyroscope(uint32_t device_id = 0, bool external = false);

	void set_device_id(uint32_t device_id);
	void set_external(bool external);
	void set_rotation(const matrix::Dcmf &sensor_to_body) { _rotation = sensor_to_body; }
	void set_thermal_offset(const matrix::Vector3f &offset);
	bool set_offset(const matrix::Vector3f &offset);
	bool set_calibration_index(int index);
	void set_priority(int32_t priority) { _priority = priority; }
	void Reset();

	matrix::Vector3f Correct(const matrix::Vector3f &data) const;
	matrix::Vector3f Uncorrect(const matrix::Vector3f &corrected_data) const;
	matrix::Vector3f BiasCorrectedSensorOffset(const matrix::Vector3f &bias) const;

	bool calibrated() const { return _device_id != 0 && _calibration_index >= 0; }
	uint8_t calibration_count() const { return _calibration_count; }
	int8_t calibration_index() const { return _calibration_index; }
	uint32_t device_id() const { return _device_id; }
	bool enabled() const { return _priority > 0; }
	bool external() const { return _external; }
	const matrix::Vector3f &offset() const { return _offset; }
	int32_t priority() const { return _priority; }
	const matrix::Dcmf &rotation() const { return _rotation; }
	const matrix::Vector3f &thermal_offset() const { return _thermal_offset; }

private:
	matrix::Dcmf _rotation{};
	matrix::Vector3f _offset{};
	matrix::Vector3f _thermal_offset{};
	int8_t _calibration_index{-1};
	uint32_t _device_id{0};
	int32_t _priority{-1};
	bool _external{false};
	uint8_t _calibration_count{0};
};

} // namespace calibration
