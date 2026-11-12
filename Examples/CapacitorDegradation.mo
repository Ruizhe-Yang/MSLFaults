within MSLFaults.Examples;
model CapacitorDegradation "Compare RC charging with a native capacitor and two replacements"
  extends Modelica.Icons.Example;
  parameter Boolean dynamicLoss=false "Ramp the capacity loss during the run";
  parameter MSLFaults.Electrical.Analog.Basic.FaultableCapacitor.FaultMode mode=MSLFaults.Electrical.Analog.Basic.FaultableCapacitor.FaultMode.CapacitanceLoss;
  MSLFaults.Utilities.Internal.CapacitorCircuit native annotation(Placement(transformation(origin={-145,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeValue annotation(Placement(transformation(origin={-91,-36},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.CapacitorCircuit normal(redeclare MSLFaults.Electrical.Analog.Basic.FaultableCapacitor device) annotation(Placement(transformation(origin={0,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalValue annotation(Placement(transformation(origin={54,-36},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.CapacitorCircuit faulted(redeclare MSLFaults.Electrical.Analog.Basic.FaultableCapacitor device(faultMode=mode,faultStartTime=if dynamicLoss then 1 else 0,faultEndTime=6,transitionTime=if dynamicLoss then 0.2 else 0)) annotation(Placement(transformation(origin={145,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultedValue annotation(Placement(transformation(origin={199,-36},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(native.readout,nativeValue) annotation(Line(points={{-119,13},{-91,13},{-91,-36}},color={0,0,127}));
  connect(normal.readout,normalValue) annotation(Line(points={{26,13},{54,13},{54,-36}},color={0,0,127}));
  connect(faulted.readout,faultedValue) annotation(Line(points={{171,13},{199,13},{199,-36}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=5,Interval=0.01,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true),graphics={Text(extent={{-209,108},{209,94}},textString="Native, fault model at nominal settings, fault model with selected mode",textColor={34,61,73})}),
    Documentation(info="<html><p><b>Purpose</b> Compare RC charging with a native capacitor and two replacements</p><p><b>Interfaces</b> Instances and connections are shown in the Diagram layer.</p><p><b>Parameters</b> Default values are shown in the component modifiers.</p><p><b>Behavior</b> At 1.2 s the nominal branch is near 6.3212 V and the loss branch near 8.6466 V. dynamicLoss enables a finite ramp.</p><p><b>Initialization</b> All initial conditions are explicit in the source and component modifiers.</p><p><b>Activation and recovery</b> The given time windows follow the simulation time axis.</p><p><b>Limitations</b> This demonstration uses the declared inputs, initial conditions and idealized components.</p><p><b>Observables</b> Three voltages plus faulted.device.q, storedEnergy, lossPower and parameterPower.</p></html>"));
end CapacitorDegradation;
