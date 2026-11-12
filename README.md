# MSLFaults

## Name and purpose

Bounded fault effects inside components with a named Modelica 4.0.0 counterpart.

## Scope and non-goals

Physical and signal faults inside components with a named MSL counterpart. Each mode describes a bounded component mechanism.

## Requirements

- Modelica Standard Library 4.0.0.
- Tested with OpenModelica 1.25.5 (64-bit), DASSL and relative tolerance 1e-6. 10/10 default examples completed in the recorded runs.
- Other Modelica tools are compatibility targets pending separate verification.

## Installation

Extract the source tree, load Modelica 4.0.0, then open `MSLFaults/package.mo`. Confirm the Modelica version in your tool. Open an example from the package tree and use its declared experiment settings.

## Package structure

```text
MSLFaults/
    UsersGuide/
    Blocks/
    Electrical/
    Mechanics/
    Thermal/
    Fluid/
    Magnetic/
    Utilities/
    Examples/
```

The model catalog lists each class, its interfaces, an example and its recorded validation level.

## Core concepts and usage

- Use the corresponding MSL class as the constraining type and preserve host modifiers. The ResistorReplacement example shows the exact redeclare syntax.
- Select the component’s local FaultMode. Normal is the default. A reversible mode uses [faultStartTime, faultEndTime), or the optional trigger when useExternalTrigger=true.
- Plot active, effectLevel and activationCount with the actual physical quantities. Keep native initial conditions consistent across comparison branches.

## Examples

| Example | Stop time (s) | Purpose | Suggested observables |
|---|---:|---|---|
| ResistorReplacement | 5 | Compare two native-constrained electrical replacements through a reversible fault | Six currents and the fault branches’ active, effectLevel, and actual resistance or conductance. |
| CapacitorDegradation | 5 | Compare RC charging with a native capacitor and two replacements | Three voltages plus faulted.device.q, storedEnergy, lossPower and parameterPower. |
| InductorDegradation | 0.2 | Compare RL current with a native inductor and two replacements | Three currents plus faulted.device.psi, storedEnergy, lossPower and parameterPower. |
| RotationalDamping | 5 | Compare rotational damping, inertia and spring fault mechanisms | Speeds, damper.lossPower, faulted spring torque, spring.storedEnergy and parameterPower. |
| TranslationalStiffness | 10 | Compare two linear second-order stiffness responses and independent mass and damping faults | Position in each branch, spring force and energy, damper loss power and mass momentum. |
| ThermalDegradation | 60 | Compare heated capacities through conductive and convective paths | Temperatures, capacity.storedEnergy and heatPath.Q_flow. |
| FluidValveDegradation | 5 | Compare native flow, coefficient loss, and pressure sensing in separate paths | Flow and pressure outputs, inlet and outlet mass balance and actualStream enthalpy. reverseFlow and freezeCommand select alternate cases. |
| MagneticReluctance | 2 | Compare native reluctance and finite reluctance growth under 100 ampere-turns | Native, normal and faulted flux and faulted.device.R_m_effective. |
| SensorAndSignalFaults | 4 | Compare freezing, fixed output and bias against native paths with common input | Gain, lag, limit, voltage, current and actual load current in each branch. |
| ElectromechanicalFaultEffects | 8 | Compare circuit, motion and heat while separately exercising magnetic conversion | Motor winding current, rotor speed and heat capacity temperature. Separate converter flux, current and parameterPower. |

## Limitations and numerical assumptions

- Open, short, seizure and fracture targets use finite approximations where declared. Permanent modes document their latch.
- Use a finite ramp to resolve in-run parameter power for a storage coefficient change. The nominal relation recovers according to the declared law.
- The model catalog lists evidence for the actual example or class. A component fault study alone cannot establish system safety.

## Version and license

Version 0.6.0. The input archive has no standalone license for its original additions. NOTICE.md records the upstream license for MSL-derived source.
