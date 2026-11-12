within MSLFaults.Examples;
model InductorDegradation "Compare RL current with a native inductor and two replacements"
  extends Modelica.Icons.Example;
  MSLFaults.Utilities.Internal.InductorCircuit native annotation(Placement(transformation(origin={-145,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeValue annotation(Placement(transformation(origin={-91,-36},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.InductorCircuit normal(redeclare MSLFaults.Electrical.Analog.Basic.FaultableInductor device) annotation(Placement(transformation(origin={0,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalValue annotation(Placement(transformation(origin={54,-36},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.InductorCircuit faulted(redeclare MSLFaults.Electrical.Analog.Basic.FaultableInductor device(faultMode=MSLFaults.Electrical.Analog.Basic.FaultableInductor.FaultMode.InductanceLoss,faultStartTime=0,faultEndTime=1,transitionTime=0)) annotation(Placement(transformation(origin={145,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultedValue annotation(Placement(transformation(origin={199,-36},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(native.readout,nativeValue) annotation(Line(points={{-119,13},{-91,13},{-91,-36}},color={0,0,127}));
  connect(normal.readout,normalValue) annotation(Line(points={{26,13},{54,13},{54,-36}},color={0,0,127}));
  connect(faulted.readout,faultedValue) annotation(Line(points={{171,13},{199,13},{199,-36}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=0.2,Interval=0.0004,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true),graphics={Text(extent={{-209,108},{209,94}},textString="Native, fault model at nominal settings, fault model with selected mode",textColor={34,61,73})}),
    Documentation(info="<html><p><b>Purpose</b> Compare RL current with a native inductor and two replacements</p><p><b>Interfaces</b> Instances and connections are shown in the Diagram layer.</p><p><b>Parameters</b> Default values are shown in the component modifiers.</p><p><b>Behavior</b> At 0.02 s the nominal current is 0.7585 A and the loss case 1.0376 A. All branches approach 1.2 A.</p><p><b>Initialization</b> All initial conditions are explicit in the source and component modifiers.</p><p><b>Activation and recovery</b> The given time windows follow the simulation time axis.</p><p><b>Limitations</b> This demonstration uses the declared inputs, initial conditions and idealized components.</p><p><b>Observables</b> Three currents plus faulted.device.psi, storedEnergy, lossPower and parameterPower.</p></html>"));
end InductorDegradation;
