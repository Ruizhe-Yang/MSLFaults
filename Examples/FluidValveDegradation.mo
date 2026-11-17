within MSLFaults.Examples;
model FluidValveDegradation "Compare valve actuation and pressure-measurement faults"
  extends Modelica.Icons.Example;
  parameter Boolean reverseFlow=false "Exchange the prescribed fluid boundary pressures";
  parameter Boolean freezeCommand=false "Capture the varying valve-opening command when true";
  parameter MSLFaults.Fluid.Valves.FaultableValveLinear.FaultMode mode=if freezeCommand then MSLFaults.Fluid.Valves.FaultableValveLinear.FaultMode.FreezeOpening else MSLFaults.Fluid.Valves.FaultableValveLinear.FaultMode.ActuatorGainLoss "Fault mode used in the valve comparison";
  MSLFaults.Utilities.Internal.ValveCircuit native(reverse=reverseFlow,varyOpening=freezeCommand) annotation(Placement(transformation(origin={-237,53},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeFlow annotation(Placement(transformation(origin={-169,18},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativePressure annotation(Placement(transformation(origin={-169,-56},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ValveCircuit normal(reverse=reverseFlow,varyOpening=freezeCommand,redeclare MSLFaults.Fluid.Valves.FaultableValveLinear valve,redeclare MSLFaults.Fluid.Sensors.FaultablePressure sensor) annotation(Placement(transformation(origin={-63,53},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalFlow annotation(Placement(transformation(origin={5,18},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalPressure annotation(Placement(transformation(origin={5,-56},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ValveCircuit faulted(reverse=reverseFlow,varyOpening=freezeCommand,redeclare MSLFaults.Fluid.Valves.FaultableValveLinear valve(faultMode=mode,actuatorGainFault=0.5,faultStartTime=if freezeCommand then 1 else 0,faultEndTime=6,transitionTime=0),redeclare MSLFaults.Fluid.Sensors.FaultablePressure sensor(faultMode=MSLFaults.Fluid.Sensors.FaultablePressure.FaultMode.Bias,biasFault=1000,faultStartTime=0,faultEndTime=6,transitionTime=0)) annotation(Placement(transformation(origin={111,53},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultedFlow annotation(Placement(transformation(origin={179,18},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultedPressure annotation(Placement(transformation(origin={179,-56},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(native.massFlow,nativeFlow) annotation(Line(points={{-213,59.24},{-169,59.24},{-169,18}},color={0,0,127}));
  connect(native.pressure,nativePressure) annotation(Line(points={{-213,46.76},{-188,46.76},{-188,-56},{-169,-56}},color={0,0,127}));
  connect(normal.massFlow,normalFlow) annotation(Line(points={{-39,59.24},{5,59.24},{5,18}},color={0,0,127}));
  connect(normal.pressure,normalPressure) annotation(Line(points={{-39,46.76},{-14,46.76},{-14,-56},{5,-56}},color={0,0,127}));
  connect(faulted.massFlow,faultedFlow) annotation(Line(points={{135,59.24},{179,59.24},{179,18}},color={0,0,127}));
  connect(faulted.pressure,faultedPressure) annotation(Line(points={{135,46.76},{160,46.76},{160,-56},{179,-56}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=5,Interval=0.01,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true),graphics={Text(extent={{-275,152},{275,138}},textString="Shared boundary conditions and command, independent hydraulic paths",textColor={34,61,73})}),
    Documentation(info="<html><p>Compare native, normal and faulted valve paths between fixed pressure boundaries. The default valve target halves its actuation gain; the pressure sensor adds 1000 Pa. reverseFlow exchanges boundary pressures and freezeCommand selects capture of a varying opening command.</p><p>Plot the three flow and three pressure outputs; rerun reverseFlow and freezeCommand separately. Use the stored experiment settings.</p></html>"));
end FluidValveDegradation;
