#include <Accelerometer.hpp>
#include <Barometer.hpp>
#include <Gyroscope.hpp>
#include <Magnetometer.hpp>
#include <Utilities.hpp>

#include <cstdint>
#include <iostream>
#include <map>
#include <string>

namespace
{

class MemoryCalibrationStore : public calibration::CalibrationStore
{
public:
	bool get_int32(const char *key, int32_t &value) const override
	{
		const auto it = _integers.find(key);

		if (it == _integers.end()) {
			return false;
		}

		value = it->second;
		return true;
	}

	bool get_float(const char *key, float &value) const override
	{
		const auto it = _floats.find(key);

		if (it == _floats.end()) {
			return false;
		}

		value = it->second;
		return true;
	}

	bool set_int32(const char *key, int32_t value) override
	{
		_integers[key] = value;
		return true;
	}

	bool set_float(const char *key, float value) override
	{
		_floats[key] = value;
		return true;
	}

private:
	std::map<std::string, int32_t> _integers;
	std::map<std::string, float> _floats;
};

bool save_vector(calibration::CalibrationStore &store, const char *sensor_type, const char *cal_type, int8_t slot,
		 const matrix::Vector3f &value)
{
	return calibration::SetCalibrationParamsVector3f(store, sensor_type, cal_type, static_cast<uint8_t>(slot), value);
}

void print_vector(const char *label, const matrix::Vector3f &value)
{
	std::cout << label << " [" << value(0) << ", " << value(1) << ", " << value(2) << "]\n";
}

bool save_identity(calibration::CalibrationStore &store, const char *sensor_type, uint32_t device_id, int8_t slot,
		   int32_t priority)
{
	return calibration::SetCalibrationParam(store, sensor_type, "ID", static_cast<uint8_t>(slot),
						 static_cast<int32_t>(device_id))
	       && calibration::SetCalibrationParam(store, sensor_type, "PRIO", static_cast<uint8_t>(slot), priority);
}

} // namespace

int main()
{
	MemoryCalibrationStore store;
	constexpr uint32_t accel_id = 1001;
	constexpr uint32_t baro_id = 2001;
	constexpr uint32_t gyro_id = 3001;
	constexpr uint32_t mag_id = 4001;

	// The host application discovers devices and decides which ones are external.
	calibration::Accelerometer accel{accel_id, false};
	calibration::Barometer baro{baro_id, false};
	calibration::Gyroscope gyro{gyro_id, false};
	calibration::Magnetometer mag{mag_id, true};

	// Supply the sensor-to-body transforms. Identity is used here for illustration.
	matrix::Dcmf sensor_to_body;
	sensor_to_body.setIdentity();
	accel.set_rotation(sensor_to_body);
	gyro.set_rotation(sensor_to_body);
	mag.set_rotation(sensor_to_body);

	// Apply calibration values and host-generated thermal/power corrections.
	accel.set_thermal_offset(matrix::Vector3f{0.01f, -0.02f, 0.03f});
	accel.set_offset(matrix::Vector3f{0.10f, -0.20f, 0.05f});
	accel.set_scale(matrix::Vector3f{1.02f, 0.99f, 1.01f});

	baro.set_thermal_offset(1.5f);
	baro.set_offset(12.0f);

	gyro.set_thermal_offset(matrix::Vector3f{0.01f, 0.00f, -0.01f});
	gyro.set_offset(matrix::Vector3f{0.10f, -0.05f, 0.02f});

	mag.set_thermal_offset(matrix::Vector3f{0.02f, -0.01f, 0.005f});
	mag.set_offset(matrix::Vector3f{0.30f, -0.20f, 0.10f});
	mag.set_scale(matrix::Vector3f{1.02f, 0.98f, 1.01f});
	mag.set_offdiagonal(matrix::Vector3f{0.015f, -0.012f, 0.008f});
	mag.set_power_compensation(matrix::Vector3f{0.002f, -0.001f, 0.003f});
	mag.UpdatePower(5.0f);

	// Allocate one slot for each sensor type and mark each object calibrated.
	const int8_t accel_slot = calibration::FindAvailableCalibrationIndex(store, "ACC", accel_id);
	const int8_t baro_slot = calibration::FindAvailableCalibrationIndex(store, "BARO", baro_id);
	const int8_t gyro_slot = calibration::FindAvailableCalibrationIndex(store, "GYRO", gyro_id);
	const int8_t mag_slot = calibration::FindAvailableCalibrationIndex(store, "MAG", mag_id);

	if (accel_slot < 0 || baro_slot < 0 || gyro_slot < 0 || mag_slot < 0) {
		std::cerr << "No calibration slot available.\n";
		return 1;
	}

	accel.set_calibration_index(accel_slot);
	baro.set_calibration_index(baro_slot);
	gyro.set_calibration_index(gyro_slot);
	mag.set_calibration_index(mag_slot);

	// Persist host-owned values through the generic utility API.
	bool saved = true;
	saved &= save_identity(store, "ACC", accel_id, accel_slot, accel.priority());
	saved &= save_vector(store, "ACC", "OFF", accel_slot, accel.offset());
	saved &= save_vector(store, "ACC", "SCALE", accel_slot, accel.scale());
	saved &= save_identity(store, "BARO", baro_id, baro_slot, baro.priority());
	saved &= calibration::SetCalibrationParam(store, "BARO", "OFF", static_cast<uint8_t>(baro_slot), baro.offset());
	saved &= save_identity(store, "GYRO", gyro_id, gyro_slot, gyro.priority());
	saved &= save_vector(store, "GYRO", "OFF", gyro_slot, gyro.offset());
	saved &= save_identity(store, "MAG", mag_id, mag_slot, mag.priority());
	saved &= save_vector(store, "MAG", "OFF", mag_slot, mag.offset());
	saved &= save_vector(store, "MAG", "SCALE", mag_slot, mag.diagonal_scale());
	saved &= save_vector(store, "MAG", "ODIAG", mag_slot, mag.offdiagonal());
	saved &= save_vector(store, "MAG", "COMP", mag_slot, mag.power_compensation());

	if (!saved) {
		std::cerr << "Failed to save one or more calibration values.\n";
		return 1;
	}

	// Apply one raw sample per sensor.
	print_vector("Accelerometer corrected", accel.Correct(matrix::Vector3f{0.2f, -0.1f, -9.7f}));
	std::cout << "Barometer corrected: " << baro.Correct(101325.0f) << " Pa\n";
	print_vector("Gyroscope corrected", gyro.Correct(matrix::Vector3f{0.2f, 0.1f, -0.05f}));
	print_vector("Magnetometer corrected", mag.Correct(matrix::Vector3f{0.4f, 0.1f, -0.2f}));

	// Demonstrate looking up and restoring an accelerometer's stored values.
	calibration::Accelerometer restored_accel{accel_id, false};
	const int8_t restored_slot = calibration::FindCurrentCalibrationIndex(store, "ACC", accel_id);

	if (restored_slot >= 0) {
		const matrix::Vector3f stored_offset = calibration::GetCalibrationParamsVector3f(store, "ACC", "OFF",
										  static_cast<uint8_t>(restored_slot));
		const matrix::Vector3f stored_scale = calibration::GetCalibrationParamsVector3f(store, "ACC", "SCALE",
										  static_cast<uint8_t>(restored_slot));
		restored_accel.set_calibration_index(restored_slot);
		restored_accel.set_offset(stored_offset);
		restored_accel.set_scale(stored_scale);
		restored_accel.set_rotation(sensor_to_body);
		restored_accel.set_thermal_offset(accel.thermal_offset());
		std::cout << "Accelerometer calibration restored from slot " << static_cast<int>(restored_slot)
			  << "; calibrated=" << restored_accel.calibrated() << "\n";
	}

	return 0;
}
