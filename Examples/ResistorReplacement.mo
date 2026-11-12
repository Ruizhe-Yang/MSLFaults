within MSLFaults.Examples;
model ResistorReplacement "Compare two native-constrained electrical replacements through a reversible fault"
  extends Modelica.Icons.Example;
  parameter MSLFaults.Electrical.Analog.Basic.FaultableResistor.FaultMode resistorMode=MSLFaults.Electrical.Analog.Basic.FaultableResistor.FaultMode.ResistanceStep;
  parameter MSLFaults.Electrical.Analog.Basic.FaultableConductor.FaultMode conductorMode=MSLFaults.Electrical.Analog.Basic.FaultableConductor.FaultMode.ConductanceStep;
  MSLFaults.Utilities.Internal.ResistorCircuit resistorNative annotation(Placement(transformation(origin={-234,116},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput resistorNativeCurrent annotation(Placement(transformation(origin={-168,69},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ResistorCircuit resistorNormal(redeclare MSLFaults.Electrical.Analog.Basic.FaultableResistor device) annotation(Placement(transformation(origin={-48,116},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput resistorNormalCurrent annotation(Placement(transformation(origin={18,69},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ResistorCircuit resistorFault(redeclare MSLFaults.Electrical.Analog.Basic.FaultableResistor device(faultMode=resistorMode,R_step=20)) annotation(Placement(transformation(origin={138,116},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput resistorFaultCurrent annotation(Placement(transformation(origin={204,69},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConductorCircuit conductorNative annotation(Placement(transformation(origin={-234,-85},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput conductorNativeCurrent annotation(Placement(transformation(origin={-168,-132},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConductorCircuit conductorNormal(redeclare MSLFaults.Electrical.Analog.Basic.FaultableConductor device) annotation(Placement(transformation(origin={-48,-85},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput conductorNormalCurrent annotation(Placement(transformation(origin={18,-132},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConductorCircuit conductorFault(redeclare MSLFaults.Electrical.Analog.Basic.FaultableConductor device(faultMode=conductorMode,G_fault=0.05)) annotation(Placement(transformation(origin={138,-85},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput conductorFaultCurrent annotation(Placement(transformation(origin={204,-132},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(resistorNative.readout,resistorNativeCurrent) annotation(Line(points={{-209,116},{-168,116},{-168,69}},color={0,0,127}));
  connect(resistorNormal.readout,resistorNormalCurrent) annotation(Line(points={{-23,116},{18,116},{18,69}},color={0,0,127}));
  connect(resistorFault.readout,resistorFaultCurrent) annotation(Line(points={{163,116},{204,116},{204,69}},color={0,0,127}));
  connect(conductorNative.readout,conductorNativeCurrent) annotation(Line(points={{-209,-85},{-168,-85},{-168,-132}},color={0,0,127}));
  connect(conductorNormal.readout,conductorNormalCurrent) annotation(Line(points={{-23,-85},{18,-85},{18,-132}},color={0,0,127}));
  connect(conductorFault.readout,conductorFaultCurrent) annotation(Line(points={{163,-85},{204,-85},{204,-132}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=5,Interval=0.01,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true),graphics={Text(extent={{-220,198},{220,184}},textString="Resistor comparison at constant 12 V",textColor={34,61,73}),Text(extent={{-220,-2},{220,-16}},textString="Conductor comparison at constant 12 V",textColor={34,61,73})}),
    Documentation(info="<html><p><b>Purpose</b> Compare two native-constrained electrical replacements through a reversible fault</p><p><b>Interfaces</b> Instances and connections are shown in the Diagram layer.</p><p><b>Parameters</b> Default values are shown in the component modifiers.</p><p><b>Behavior</b> Native and Normal current is 1.2 A. Fault branches fall to 0.6 A by 1.05 s and recover after 3.05 s.</p><p><b>Initialization</b> All initial conditions are explicit in the source and component modifiers.</p><p><b>Activation and recovery</b> The given time windows follow the simulation time axis.</p><p><b>Limitations</b> This demonstration uses the declared inputs, initial conditions and idealized components.</p><p><b>Observables</b> Six currents and the fault branches’ active, effectLevel, and actual resistance or conductance.</p></html>"));
end ResistorReplacement;
