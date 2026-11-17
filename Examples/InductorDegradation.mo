within MSLFaults.Examples;
model InductorDegradation "Compare RL current and inductance loss"
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
    Documentation(info="<html><p>Compare native, normal and faulted inductors in identical RL circuits. The inductance-loss mode is active from initialisation; the 10 Ω resistor sets the eventual 1.2 A current while inductance changes the transient.</p><p>Plot the three Value outputs and compare the current rise following the voltage step. Use the stored experiment settings.</p></html>"));
end InductorDegradation;
