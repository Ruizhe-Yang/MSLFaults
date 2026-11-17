within MSLFaults.Examples;
model MagneticReluctance "Compare magnetic flux with increased reluctance"
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
    Documentation(info="<html><p>Apply 100 A magnetic potential difference to native, normal and faulted reluctances. The faulted value is 2000 A/Wb instead of 1000 A/Wb, giving a configured flux of 0.05 Wb compared with 0.1 Wb.</p><p>Plot nativeValue, normalValue and faultedValue. Use the stored experiment settings.</p></html>"));
end MagneticReluctance;
