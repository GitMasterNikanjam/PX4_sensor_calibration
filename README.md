# PX4 Sensor Calibration Library

`PX4_sensor_calibration` is a small C++ library for holding sensor calibration
state and applying calibration corrections to measurements. It currently
supports accelerometers, barometers, gyroscopes, and magnetometers.

The sensor classes implement the math and keep the active calibration values.
The host application supplies sensor discovery, device-bus classification,
board and sensor orientation, thermal corrections, power measurements, and
persistent storage. The library has no dependency on PX4 firmware, uORB, PX4
parameters, or PX4 logging. It does use the extracted `PX4::matrix` library for
vector and matrix operations.

## What the library does

- Holds a device ID, external/internal flag, calibration slot, priority, and
  sensor-specific offsets and scale factors.
- Applies correction to raw sensor data with `Correct()`.
- Converts corrected values back for the sensors that provide `Uncorrect()`.
- Computes a proposed sensor offset from an output-frame bias with
  `BiasCorrectedSensorOffset()`.
- Provides `CalibrationStore` and utility functions for key generation,
  reading/writing calibration values, and selecting one of four calibration
  slots.

## What the host application must do

- Discover sensors and decide whether each is external.
- Supply a sensor-to-body rotation matrix for accelerometer, gyroscope, and
  magnetometer.
- Calculate and update thermal offsets and magnetometer power compensation.
- Implement `CalibrationStore` for its storage backend.
- Load values into each object and decide when to save them.
- Validate priority values and coordinate concurrent calibration-slot use.

## Get started

In this workspace, configure and build from the root:

```powershell
cmake -S . -B build
cmake --build build --target px4_sensor_calibration
```

The target is named `PX4::sensor_calibration` and links to `PX4::matrix`. To
include this directory in another CMake project, first make `PX4::matrix`
available, then add and link the target:

```cmake
add_subdirectory(path/to/PX4_sensor_calibration)
target_link_libraries(my_app PRIVATE PX4::sensor_calibration)
```

The main public headers are `Accelerometer.hpp`, `Barometer.hpp`,
`Gyroscope.hpp`, `Magnetometer.hpp`, and `Utilities.hpp`. All classes and helper
functions are in the `calibration` namespace.

## Complete guide

The detailed usage, API reference, and algorithm notes are included below in this file.

## Important behavior

- There are four calibration slots (`0` through `3`) per sensor type.
- `Reset()` clears calibration values and returns the rotation to identity. It
  keeps the device ID and external/internal flag, and chooses the corresponding
  default priority (`50` internal, `75` external).
- Calling `set_device_id()` with a different ID or changing `set_external()`
  calls `Reset()`. Apply persisted settings and host-provided corrections after
  those calls.
- Sensor classes do not expose `ParametersLoad()` or `ParametersSave()`.
  Persistence is explicit and host-owned.
- Several calibration setters apply a change threshold. They can return
  `false` and leave the current value unchanged when the requested value is
  close to the existing value. See "Loading and saving a sensor object" and
  "Numerical guards and limits" below before using them to restore persisted
  values.
- The current slot-selection API treats a failed ID read like an empty slot.
  See "Calibration storage interface and helpers" below before using it with a
  backend that can fail reads.


## Usage and API Reference


This guide describes the public API in `PX4_sensor_calibration`. The code uses
C++11 and the extracted PX4 Matrix library. Unless stated otherwise, examples
assume `using` declarations are avoided and names are fully qualified.

## 1. Add the library to a project

The workspace root defines `PX4::matrix` and adds this library as a subdirectory.
Configure and build the sensor target from the workspace root:

```powershell
cmake -S . -B build
cmake --build build --target px4_sensor_calibration
```

The extracted matrix repository is header-only and does not define a CMake
target itself. For another CMake project, create an interface target for its
headers before adding this subdirectory, then link the sensor library:

```cmake
add_library(px4_matrix INTERFACE)
add_library(PX4::matrix ALIAS px4_matrix)
target_include_directories(px4_matrix INTERFACE path/to/PX4_matrix)
add_subdirectory(path/to/PX4_sensor_calibration)
target_link_libraries(my_app PRIVATE PX4::sensor_calibration)
```

`PX4_sensor_calibration/CMakeLists.txt` expects the target name `PX4::matrix` to
already exist. If your matrix project exports a different target name, provide
an alias or change the integration CMake accordingly.

Include the API headers as needed:

```cpp
#include <Accelerometer.hpp>
#include <Barometer.hpp>
#include <Gyroscope.hpp>
#include <Magnetometer.hpp>
#include <Utilities.hpp>
```

Everything is in namespace `calibration`.

## 2. Common sensor lifecycle

Each sensor class stores its active calibration values. A typical lifecycle is:

1. Discover the sensor in the host application.
2. Construct the matching class with the device ID and external/internal flag.
3. Find a matching saved calibration slot, or choose an available slot.
4. Read saved calibration values using the host's `CalibrationStore`.
5. Apply the values with the sensor setters and supply rotation/thermal inputs.
6. Call `Correct()` for each measurement.
7. When calibration changes, write the new values to the host store.

Example setup for an accelerometer:

```cpp
calibration::Accelerometer accel{123456u, false};

matrix::Dcmf sensor_to_body; // identity by default; replace for the sensor mount
accel.set_rotation(sensor_to_body);
accel.set_thermal_offset(matrix::Vector3f{0.01f, 0.0f, -0.02f});
accel.set_offset(matrix::Vector3f{0.1f, -0.2f, 0.05f});
accel.set_scale(matrix::Vector3f{1.0f, 1.01f, 0.99f});
accel.set_calibration_index(0);

const matrix::Vector3f raw{0.0f, 0.0f, -9.81f};
const matrix::Vector3f corrected_body_frame = accel.Correct(raw);
```

`external` affects the initial priority only. The library does not infer
external status from a device ID. It also does not construct a rotation from
sensor bus data or board parameters; the host supplies the finished matrix.

### Reset and reconfiguration

`Reset()` clears the calibration index, offsets, scales, thermal offset, power
compensation and current power. It sets rotation to identity and priority to
`50` for internal or `75` for external sensors. It preserves device ID and
external status.

`set_device_id(new_id)` calls `Reset()` when the ID changes. `set_external(value)`
does the same when the flag changes. Therefore, call these before restoring
calibration and host-provided correction values. Calling them after setup clears
that setup.

## 3. Sensor classes

### Accelerometer

Header: `Accelerometer.hpp`

State and inputs:

- `set_device_id(uint32_t)` and `set_external(bool)` configure identity.
- `set_rotation(const matrix::Dcmf&)` supplies sensor-to-body rotation.
- `set_thermal_offset(const matrix::Vector3f&)` supplies the host's thermal
  correction.
- `set_offset(const matrix::Vector3f&)` sets the three-axis hard-iron/bias
  offset. The setter rejects non-finite values and ignores changes whose
  difference-vector length is not greater than `0.01`.
- `set_scale(const matrix::Vector3f&)` sets positive, finite per-axis scale
  values. It ignores changes whose difference-vector length is not greater
  than `0.01`.
- `set_calibration_index(int)` accepts slot indices `0` to `3`.
- `set_priority(int32_t)` sets the priority without range validation.

`Correct(sample)` subtracts thermal and calibration offsets, applies
per-axis scale, then rotates to the body frame. `BiasCorrectedSensorOffset(bias)`
returns a proposed sensor-frame offset without mutating the object.

Getters: `device_id()`, `external()`, `calibrated()`, `enabled()`,
`calibration_index()`, `calibration_count()`, `priority()`, `offset()`,
`scale()`, `rotation()`, and `thermal_offset()`.

### Barometer

Header: `Barometer.hpp`

State and inputs:

- `set_device_id(uint32_t)` and `set_external(bool)` configure identity.
- `set_thermal_offset(float)` accepts finite values.
- `set_offset(float)` sets the calibration offset. It rejects non-finite values
  and ignores changes with absolute difference at or below `0.01`.
- `set_calibration_index(int)` accepts slot indices `0` to `3`.
- `set_priority(int32_t)` sets the priority without range validation.

`Correct(data)` subtracts the thermal and calibration offsets.
`Uncorrect(corrected_data)` adds both offsets back.
`BiasCorrectedSensorOffset(bias)` returns `bias + thermal_offset + offset`.

Getters: `device_id()`, `external()`, `calibrated()`, `enabled()`,
`calibration_index()`, `calibration_count()`, `priority()`, `offset()`, and
`thermal_offset()`.

### Gyroscope

Header: `Gyroscope.hpp`

State and inputs:

- `set_device_id(uint32_t)` and `set_external(bool)` configure identity.
- `set_rotation(const matrix::Dcmf&)` supplies sensor-to-body rotation.
- `set_thermal_offset(const matrix::Vector3f&)` supplies the host's thermal
  correction.
- `set_offset(const matrix::Vector3f&)` sets the three-axis bias. It rejects
  non-finite values. The first finite call is accepted even when it is zero;
  later changes need a difference-vector length greater than `0.01`.
- `set_calibration_index(int)` accepts slot indices `0` to `3`.
- `set_priority(int32_t)` sets the priority without range validation.

`Correct(data)` subtracts thermal and calibration offsets, then rotates to the
body frame. `Uncorrect(corrected_data)` applies the inverse rotation and adds
the offsets back. `BiasCorrectedSensorOffset(bias)` returns a proposed
sensor-frame offset without mutating the object.

Getters: `device_id()`, `external()`, `calibrated()`, `enabled()`,
`calibration_index()`, `calibration_count()`, `priority()`, `offset()`,
`rotation()`, and `thermal_offset()`.

### Magnetometer

Header: `Magnetometer.hpp`

State and inputs:

- `set_device_id(uint32_t)` and `set_external(bool)` configure identity.
- `set_rotation(const matrix::Dcmf&)` supplies sensor-to-body rotation.
- `set_thermal_offset(const matrix::Vector3f&)` supplies a finite thermal
  offset. The current `Correct()` implementation subtracts it.
- `set_offset(const matrix::Vector3f&)` sets the three-axis offset. It rejects
  non-finite values and ignores changes whose difference-vector length is not
  greater than `0.005`.
- `set_scale(const matrix::Vector3f&)` sets the diagonal of the scale matrix.
  Each diagonal value must be finite and positive. Changes below a difference
  length of `0.01` are ignored.
- `set_offdiagonal(const matrix::Vector3f&)` sets the symmetric entries
  `(0,1)/(1,0)`, `(0,2)/(2,0)`, and `(1,2)/(2,1)`. Values must be finite and
  the difference-vector length must exceed `0.01`.
- `set_power_compensation(const matrix::Vector3f&)` sets a finite vector.
- `UpdatePower(float)` supplies the current finite scalar power value used by
  power compensation.
- `set_calibration_index(int)` accepts slot indices `0` to `3`.
- `set_priority(int32_t)` sets priority without range validation;
  `disable()` sets priority to zero.

`Correct(data)` applies power compensation, subtracts thermal and calibration
offsets, applies the full 3x3 scale matrix, and rotates to the body frame.
`BiasCorrectedSensorOffset(bias)` returns a proposed sensor-frame offset using
the inverse scale matrix and inverse rotation.

Getters: `device_id()`, `external()`, `calibrated()`, `enabled()`,
`calibration_index()`, `calibration_count()`, `priority()`, `offset()`,
`rotation()`, `scale()`, `diagonal_scale()`, `offdiagonal()`,
`thermal_offset()`, and `power_compensation()`.

The scale matrix must be invertible for `BiasCorrectedSensorOffset()` to be
meaningful. The setters check positive diagonal entries and finite
off-diagonal entries, but do not verify invertibility or positive definiteness.

## 4. Calibration storage interface and helpers

### Implement a backend

Derive from `calibration::CalibrationStore`. A return value of `true` means the
requested key operation succeeded; `false` means it did not. A minimal in-memory
example is:

```cpp
#include <Utilities.hpp>

#include <cstdint>
#include <map>
#include <string>

class MemoryCalibrationStore : public calibration::CalibrationStore
{
public:
	bool get_int32(const char *key, int32_t &value) const override
	{
		auto it = ints.find(key);
		if (it == ints.end()) { return false; }
		value = it->second;
		return true;
	}

	bool get_float(const char *key, float &value) const override
	{
		auto it = floats.find(key);
		if (it == floats.end()) { return false; }
		value = it->second;
		return true;
	}

	bool set_int32(const char *key, int32_t value) override
	{
		ints[key] = value;
		return true;
	}

	bool set_float(const char *key, float value) override
	{
		floats[key] = value;
		return true;
	}

private:
	std::map<std::string, int32_t> ints;
	std::map<std::string, float> floats;
};
```

For a persistent backend, store integer and floating-point values in a format
that preserves their types and values. The utility methods do not define a file
format or provide synchronization.

### Key naming

The helpers construct scalar keys as:

```text
CAL_<sensor_type><slot>_<cal_type>
```

Vector keys insert the axis before `cal_type`:

```text
CAL_<sensor_type><slot>_X<cal_type>
CAL_<sensor_type><slot>_Y<cal_type>
CAL_<sensor_type><slot>_Z<cal_type>
```

Examples:

| Purpose | Key(s) |
|---|---|
| Accelerometer slot 0 ID | `CAL_ACC0_ID` |
| Accelerometer slot 0 priority | `CAL_ACC0_PRIO` |
| Accelerometer slot 0 offset | `CAL_ACC0_XOFF`, `CAL_ACC0_YOFF`, `CAL_ACC0_ZOFF` |
| Magnetometer slot 2 scale | `CAL_MAG2_XSCALE`, `CAL_MAG2_YSCALE`, `CAL_MAG2_ZSCALE` |

Typical sensor types are `ACC`, `BARO`, `GYRO`, and `MAG`. The helper accepts
other non-null type strings too, provided the generated key fits its internal
buffer.

### Slot functions

`FindCurrentCalibrationIndex(store, sensor_type, device_id)` scans slots `0`
through `3` and returns the first slot whose `ID` equals `device_id`. A zero
device ID or no matching entry returns `-1`.

`FindAvailableCalibrationIndex(store, sensor_type, device_id, preferred_index)`:

1. Returns the existing slot when the device ID is already present.
2. Otherwise returns `preferred_index` if it is in range and appears empty.
3. Otherwise returns the first slot that appears empty.
4. Returns `-1` if no slot is available.

The function does not reserve the slot or write the device ID. The caller must
save the ID after selecting a slot and serialize concurrent allocation if
multiple tasks can calibrate the same sensor type.

**Read-failure caveat:** the current `CalibrationStore` interface only reports
success or failure. `FindAvailableCalibrationIndex()` treats a failed ID read
as an empty slot. A storage outage could therefore lead to selection of an
occupied slot. Do not use slot allocation with a backend that can have transient
read failures unless the backend prevents that ambiguity (for example, by
locking and validating all IDs before allocation).

`FindAvailableCalibrationIndex()` does not reject `device_id == 0`; it can
return an empty slot for that value. Check for a nonzero device ID before
calling it. `FindCurrentCalibrationIndex()` does reject zero.

### Read/write helpers

- `GetCalibrationParamInt32(...)` returns `0` if the key cannot be read.
- `GetCalibrationParamFloat(...)` returns NaN if the key cannot be read.
- `GetCalibrationParamsVector3f(...)` initializes components to zero; a failed
  component read leaves that component at zero.
- `SetCalibrationParam(...)` writes an `int32_t` or `float` scalar and returns
  whether the write succeeded.
- `SetCalibrationParamsVector3f(...)` writes three float keys and returns true
  only if all three writes succeed. Writes are sequential, not transactional;
  on a partial failure, some components may already have been stored.

Example: allocate a slot and save an accelerometer identity and calibration:

```cpp
const int8_t slot = calibration::FindAvailableCalibrationIndex(store, "ACC", device_id);

if (slot >= 0) {
	bool saved = true;
	saved &= calibration::SetCalibrationParam(store, "ACC", "ID", slot,
							  static_cast<int32_t>(device_id));
	saved &= calibration::SetCalibrationParam(store, "ACC", "PRIO", slot, int32_t{50});
	saved &= calibration::SetCalibrationParamsVector3f(store, "ACC", "OFF", slot, accel.offset());
	saved &= calibration::SetCalibrationParamsVector3f(store, "ACC", "SCALE", slot, accel.scale());
}
```

The scalar ID helper uses `int32_t`, matching the historic parameter schema.
Ensure the device ID range used by the host is representable in the chosen
storage type.

## 5. Loading and saving a sensor object

The library intentionally does not own a persistence policy. The host combines
the utility functions with setters. A simplified read flow is:

```cpp
const int8_t slot = calibration::FindCurrentCalibrationIndex(store, "ACC", device_id);

if (slot >= 0) {
	const matrix::Vector3f offset = calibration::GetCalibrationParamsVector3f(store, "ACC", "OFF", slot);
	const matrix::Vector3f scale = calibration::GetCalibrationParamsVector3f(store, "ACC", "SCALE", slot);
	const int32_t priority = calibration::GetCalibrationParamInt32(store, "ACC", "PRIO", slot);

	accel.set_calibration_index(slot);
	accel.set_offset(offset);
	accel.set_scale(scale);
	accel.set_priority(priority);
}
```

**Important:** offset and scale setters are change-oriented. They may decline a
small change and return false. For example, loading a nonzero offset with a
magnitude below its threshold into a default-zero object may leave the offset
at zero. The current API has no force-load/import setter. Applications that
must restore arbitrarily small persisted values should account for this
limitation; do not assume a false return always means the value was invalid.

Store the host-provided rotation, thermal offset, and magnetometer power
compensation alongside the calibration if they must survive restart. The
utility key functions can store custom scalar/vector keys, but the classes do
not interpret rotation enums or serialize matrices automatically.

## 6. Shared getters and state semantics

All four sensor classes expose:

- `device_id()` and `external()` for configured identity.
- `calibration_index()` and `calibrated()`. Calibration is true only for a
  nonzero device ID and slot index `0` to `3`; it does not assess whether the
  calibration values are physically accurate.
- `priority()` and `enabled()`. Enabled means priority is greater than zero.
- `calibration_count()`, which increments for successful changes to selected
  calibration offsets/scales. It is not a count of all setter calls and is
  reset by `Reset()`.
- `Reset()`, with the behavior described above.

`set_priority()` accepts any integer. The old PX4 parameter-loading behavior
clamped invalid priorities, but the independent class leaves policy validation
to the host. If the host uses the conventional range, validate `0` through
`100` before setting it.


## Calibration and Correction Algorithms


This document explains the calculations implemented by the sensor classes.
The library stores calibration parameters and applies them to measurements. It
does **not** estimate those parameters from sample data. It contains no
accelerometer six-position solver, gyroscope bias estimator, magnetometer
ellipsoid fit, or barometer reference-pressure routine. The host application or
a separate calibration procedure must calculate offsets, scales, rotations,
thermal models, and power-compensation coefficients.

## Notation and coordinate frames

For vector sensors:

- `x`: raw measurement in the sensor frame.
- `b`: stored calibration offset, in sensor units and sensor frame.
- `t`: thermal offset supplied by the host, in sensor units and sensor frame.
- `s`: accelerometer's three per-axis scale values.
- `S`: magnetometer's full 3-by-3 scale matrix.
- `R`: host-supplied rotation from sensor frame to body frame.
- `p`: current scalar power value supplied by `Magnetometer::UpdatePower()`.
- `c`: power-compensation vector supplied by the host.
- `⊙`: element-wise vector multiplication.

The host is responsible for keeping all values in compatible units. For
example, an accelerometer offset and thermal offset must use the same
acceleration units as its raw input. The body-frame rotation should be a proper
orthonormal rotation matrix. Gyroscope `Uncorrect()` uses `Rᵀ` as the inverse,
which is valid for an orthonormal rotation matrix.

## Accelerometer

Implemented by `Accelerometer::Correct()`:

```text
y = R [ (x - t - b) ⊙ s ]
```

Operations occur in this order:

1. Subtract the host thermal offset `t`.
2. Subtract calibration offset `b`.
3. Multiply each axis by its scale factor `s`.
4. Rotate the result into the body frame with `R`.

All axes are treated independently by the scale step. The class does not model
cross-axis terms. `set_scale()` requires each component to be finite and
positive.

The proposed offset helper is:

```text
b_new = b + diag(s)⁻¹ Rᵀ bias
```

It converts an output/body-frame bias to sensor coordinates and undoes the
per-axis scale before adding it to the existing offset. It returns a value; it
does not update the object.

## Barometer

Implemented by `Barometer::Correct()`:

```text
y = x - t - b
```

The pressure value, thermal correction, and calibration offset must all use the
same pressure unit. `Uncorrect()` reverses this additive correction:

```text
x = y + t + b
```

The implemented `BiasCorrectedSensorOffset()` returns:

```text
bias + t + b
```

It returns the computed value without changing the object's offset.

## Gyroscope

Implemented by `Gyroscope::Correct()`:

```text
y = R (x - t - b)
```

Thermal and calibration offsets are subtracted in sensor coordinates before
rotation. The inverse operation is:

```text
x = Rᵀ y + t + b
```

This inverse assumes `RᵀR = I`. `BiasCorrectedSensorOffset()` computes:

```text
b_new = b + Rᵀ bias
```

Again, the helper returns a candidate offset without mutating the object.

## Magnetometer

Implemented by `Magnetometer::Correct()`:

```text
y = R S (x + p c - t - b)
```

The steps are:

1. Add the power-dependent compensation `p c` to the measurement.
2. Subtract the host thermal offset `t` and calibration offset `b`.
3. Apply the full 3-by-3 scale matrix `S`.
4. Rotate into the body frame with `R`.

The matrix is symmetric. `set_scale()` sets its three diagonal entries. Given
`q = (qxy, qxz, qyz)`, `set_offdiagonal(q)` sets:

```text
S = [ sxx  qxy  qxz ]
    [ qxy  syy  qyz ]
    [ qxz  qyz  szz ]
```

The inverse-offset helper is:

```text
b_new = b + S⁻¹ Rᵀ bias
```

`S` must be invertible for this calculation to be meaningful. Setters reject
non-finite values and non-positive diagonal scales, but they do not check the
determinant, condition number, or positive definiteness of `S`. A nearly
singular matrix can produce very large or non-finite bias corrections.

Thermal behavior is worth checking during integration: this implementation
subtracts `thermal_offset` in `Correct()`. Do not also subtract the same
thermal correction in an upstream pipeline, or the correction will be applied
twice.

## Calibration offsets, scale, and rotation

Offsets are subtractive terms in each `Correct()` function. To compensate a
constant sensor bias, the stored offset generally has the same sign as that
bias. Scales multiply after offset removal. Rotations are applied after
calibration in the sensor frame.

The library does not derive `R` from an enum or board setting. It accepts a
matrix from the host. Default and reset rotations are identity. Changing device
ID or external/internal status resets the object, so the host must restore the
desired rotation and calibration afterward.

## Numerical guards and limits

The offset/scale setters reject non-finite values. Several also ignore small
changes to avoid counting insignificant adjustments:

| Setter | Acceptance rule |
|---|---|
| Accelerometer offset | Finite and difference-vector length `> 0.01` |
| Accelerometer scale | Finite, all components `> 0`, and difference-vector length `> 0.01` |
| Barometer offset | Finite and absolute difference `> 0.01` |
| Gyroscope offset | Finite and difference-vector length `> 0.01`, except the first finite call is accepted |
| Magnetometer offset | Finite and difference-vector length `> 0.005` |
| Magnetometer diagonal scale | Finite, all components `> 0`, and difference-vector length `> 0.01` |
| Magnetometer off-diagonal scale | Finite and difference-vector length `> 0.01` |

These thresholds affect loading as well as interactive calibration. A persisted
nonzero value smaller than the threshold can be declined when applied to a
default-initialized object. The current API does not offer a force-load setter.
The `Correct()` methods also do not validate measurement vectors for finiteness.

## Calibration slot selection

The utility functions use four slots per sensor type and the device ID to find
the current slot. For a new device, the preferred slot is used only if it is
empty; otherwise the first empty slot is selected. The selection is not
reserved, and the library does not persist a calibration automatically.

