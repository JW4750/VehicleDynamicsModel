# VehicleDynamicsModel

This repository contains MATLAB/Simulink-oriented source code for a
continuous-time four-wheel vehicle dynamics model and a demonstration
simulation of a 30 km/h left-hand turn manoeuvre. The implementation is
compatible with MATLAB R2024b and uses only base MATLAB functionality, so
it can be embedded directly inside Simulink using MATLAB Function blocks.

## Contents

- `matlab/vehicleParameters.m`: Parameter set representing a mid-size
  passenger car.
- `matlab/driverModel.m`: Simple PI-based longitudinal controller and
  Ackermann steering driver profile for the manoeuvre.
- `matlab/fourWheelVehicleDynamics.m`: Nonlinear four-wheel planar vehicle
  dynamics model providing translational and yaw responses.
- `matlab/simulate_left_turn.m`: Example script that integrates the model,
  produces diagnostic plots, and writes summary metrics to the MATLAB base
  workspace (`vehicleResults`).

## Running the example

From MATLAB R2024b (or later), change the working directory to the repo
root and execute:

```matlab
run(fullfile('matlab', 'simulate_left_turn.m'));
```

The script will generate six diagnostic plots and print summary statistics
for the final position, peak lateral acceleration, and peak yaw rate. The
struct `vehicleResults` is available in the base workspace for further
post-processing or visualisation.

## Integration into Simulink

- Use `vehicleParameters` to populate a `Simulink.Parameter` structure or
  MATLAB function input.
- Invoke `fourWheelVehicleDynamics` from a MATLAB Function block with the
  state vector and control inputs provided by Simulink subsystems.
- `driverModel` can be replaced with custom steering and powertrain
  controllers or connected to Simulink steering/throttle actuators.

## References

- Rajamani, R., *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012.
- Wong, J. Y., *Theory of Ground Vehicles*, 4th ed., Wiley, 2001.
- UWVehicleDynamics GitHub project (University of Waterloo Alternative
  Fuels Team), 2023.

These works informed the parameter selection, modelling approach, and
validation strategy adopted in this repository.
