within MSLFaults.Examples;
model MagneticReluctance "Compare native reluctance and finite reluctance growth under 100 ampere-turns"
  extends Modelica.Icons.Example;
  MSLFaults.Utilities.Internal.ReluctanceCircuit native annotation(Placement(transformation(origin={-145,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeValue annotation(Placement(transformation(origin={-91,-36},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ReluctanceCircuit normal(redeclare MSLFaults.Magnetic.FluxTubes.Basic.FaultableConstantReluctance device) annotation(Placement(transformation(origin={0,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalValue annotation(Placement(transformation(origin={54,-36},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ReluctanceCircuit faulted(redeclare MSLFaults.Magnetic.FluxTubes.Basic.FaultableConstantReluctance device(faultMode=MSLFaults.Magnetic.FluxTubes.Basic.FaultableConstantReluctance.FaultMode.ReluctanceIncrease,R_mFault=2000,faultStartTime=0,faultEndTime=3,transitionTime=0)) annotation(Placement(transformation(origin={145,13},extent={{-26,-26},{26,26}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultedValue annotation(Placement(transformation(origin={199,-36},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(native.flux,nativeValue) annotation(Line(points={{-119,13},{-91,13},{-91,-36}},color={0,0,127}));
  connect(normal.flux,normalValue) annotation(Line(points={{26,13},{54,13},{54,-36}},color={0,0,127}));
  connect(faulted.flux,faultedValue) annotation(Line(points={{171,13},{199,13},{199,-36}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=2,Interval=0.004,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-240,-160},{240,160}},preserveAspectRatio=true),graphics={Text(extent={{-209,108},{209,94}},textString="Native, fault model at nominal settings, fault model with selected mode",textColor={34,61,73})}),
    Documentation(info="<html><p><b>Purpose</b> Compare native reluctance and finite reluctance growth under 100 ampere-turns</p><p><b>Interfaces</b> Instances and connections are shown in the Diagram layer.</p><p><b>Parameters</b> Default values are shown in the component modifiers.</p><p><b>Behavior</b> The native and Normal flux is 0.1 Wb and the increased-reluctance flux is 0.05 Wb.</p><p><b>Initialization</b> All initial conditions are explicit in the source and component modifiers.</p><p><b>Activation and recovery</b> The given time windows follow the simulation time axis.</p><p><b>Limitations</b> This demonstration uses the declared inputs, initial conditions and idealized components.</p><p><b>Observables</b> Native, normal and faulted flux and faulted.device.R_m_effective.</p></html>"));
end MagneticReluctance;
