within MSLFaults.Examples;
model ThermalDegradation "Compare thermal capacity, conduction and convection faults"
  extends Modelica.Icons.Example;
  MSLFaults.Utilities.Internal.ConductiveCircuit native annotation(Placement(transformation(origin={-265,121},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeTemperature annotation(Placement(transformation(origin={-205,73},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConductiveCircuit normal(redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableHeatCapacitor capacity,redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableThermalConductor heatPath) annotation(Placement(transformation(origin={-109,121},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalTemperature annotation(Placement(transformation(origin={-49,73},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConductiveCircuit conductanceFault(redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableHeatCapacitor capacity,redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableThermalConductor heatPath(faultMode=MSLFaults.Thermal.HeatTransfer.Components.FaultableThermalConductor.FaultMode.ConductanceLoss,GFault=5,faultStartTime=0,faultEndTime=61,transitionTime=0)) annotation(Placement(transformation(origin={47,121},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput conductanceFaultTemperature annotation(Placement(transformation(origin={107,73},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConductiveCircuit capacityFault(redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableHeatCapacitor capacity(faultMode=MSLFaults.Thermal.HeatTransfer.Components.FaultableHeatCapacitor.FaultMode.CapacityLoss,CFault=50,faultStartTime=0,faultEndTime=61,transitionTime=0),redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableThermalConductor heatPath) annotation(Placement(transformation(origin={203,121},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput capacityFaultTemperature annotation(Placement(transformation(origin={263,73},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConvectiveCircuit nativeConvection annotation(Placement(transformation(origin={-220,-79},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeConvectionTemperature annotation(Placement(transformation(origin={-152,-128},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConvectiveCircuit normalConvection(redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableConvection heatPath) annotation(Placement(transformation(origin={-38,-79},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalConvectionTemperature annotation(Placement(transformation(origin={30,-128},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ConvectiveCircuit faultConvection(redeclare MSLFaults.Thermal.HeatTransfer.Components.FaultableConvection heatPath(faultMode=MSLFaults.Thermal.HeatTransfer.Components.FaultableConvection.FaultMode.Fouling,conductanceScaleFault=0.5,faultStartTime=0,faultEndTime=61,transitionTime=0)) annotation(Placement(transformation(origin={144,-79},extent={{-24,-24},{24,24}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultConvectionTemperature annotation(Placement(transformation(origin={212,-128},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(native.temperature,nativeTemperature) annotation(Line(points={{-241,121},{-205,121},{-205,73}},color={0,0,127}));
  connect(normal.temperature,normalTemperature) annotation(Line(points={{-85,121},{-49,121},{-49,73}},color={0,0,127}));
  connect(conductanceFault.temperature,conductanceFaultTemperature) annotation(Line(points={{71,121},{107,121},{107,73}},color={0,0,127}));
  connect(capacityFault.temperature,capacityFaultTemperature) annotation(Line(points={{227,121},{263,121},{263,73}},color={0,0,127}));
  connect(nativeConvection.temperature,nativeConvectionTemperature) annotation(Line(points={{-196,-79},{-152,-79},{-152,-128}},color={0,0,127}));
  connect(normalConvection.temperature,normalConvectionTemperature) annotation(Line(points={{-14,-79},{30,-79},{30,-128}},color={0,0,127}));
  connect(faultConvection.temperature,faultConvectionTemperature) annotation(Line(points={{168,-79},{212,-79},{212,-128}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=60,Interval=0.01,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true),graphics={Text(extent={{-277,213},{287,199}},textString="100 W input, 300 K ambient, thermal conductance comparison",textColor={34,61,73}),Text(extent={{-277,7},{277,-7}},textString="Separate comparison using the same convective Gc input",textColor={34,61,73})}),
    Documentation(info="<html><p>Heat independent thermal assemblies with 100 W against a 300 K environment. Reduced conduction or convection changes the final temperature; reduced capacity primarily changes the transient for the same thermal path. Fault targets are applied from initialisation.</p><p>Plot the seven temperature outputs and compare transient rate and final temperature separately. Use the stored experiment settings.</p></html>"));
end ThermalDegradation;
