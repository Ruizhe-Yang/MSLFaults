within MSLFaults.Examples;
model SensorAndSignalFaults "Compare captured outputs, fixed values and measurement bias"
  extends Modelica.Icons.Example;
  MSLFaults.Utilities.Internal.SignalCircuit nativeSignal annotation(Placement(transformation(origin={-245,118},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeSignalGain annotation(Placement(transformation(origin={-172,79},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeSignalLag annotation(Placement(transformation(origin={-172,42},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.SignalCircuit normalSignal(redeclare MSLFaults.Blocks.Math.FaultableGain gain,redeclare MSLFaults.Blocks.Continuous.FaultableFirstOrder lag,redeclare MSLFaults.Blocks.Nonlinear.FaultableLimiter limit) annotation(Placement(transformation(origin={-75,118},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalSignalGain annotation(Placement(transformation(origin={-2,79},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalSignalLag annotation(Placement(transformation(origin={-2,42},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.SignalCircuit faultSignal(redeclare MSLFaults.Blocks.Math.FaultableGain gain(faultMode=MSLFaults.Blocks.Math.FaultableGain.FaultMode.Freeze,faultStartTime=1,faultEndTime=2,transitionTime=0),redeclare MSLFaults.Blocks.Continuous.FaultableFirstOrder lag(faultMode=MSLFaults.Blocks.Continuous.FaultableFirstOrder.FaultMode.Freeze,faultStartTime=1,faultEndTime=2,transitionTime=0),redeclare MSLFaults.Blocks.Nonlinear.FaultableLimiter limit(faultMode=MSLFaults.Blocks.Nonlinear.FaultableLimiter.FaultMode.Stuck,stuckValue=0.25,faultStartTime=1,faultEndTime=2,transitionTime=0)) annotation(Placement(transformation(origin={95,118},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultSignalGain annotation(Placement(transformation(origin={168,79},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultSignalLag annotation(Placement(transformation(origin={168,42},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.SensorCircuit nativeSensing annotation(Placement(transformation(origin={-245,-98},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeSensingVoltage annotation(Placement(transformation(origin={-176,-137},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeSensingCurrent annotation(Placement(transformation(origin={-176,-179},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.SensorCircuit normalSensing(redeclare MSLFaults.Electrical.Analog.Sensors.FaultableCurrentSensor currentSensor,redeclare MSLFaults.Electrical.Analog.Sensors.FaultableVoltageSensor voltageSensor) annotation(Placement(transformation(origin={-75,-98},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalSensingVoltage annotation(Placement(transformation(origin={-6,-137},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalSensingCurrent annotation(Placement(transformation(origin={-6,-179},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.SensorCircuit faultSensing(redeclare MSLFaults.Electrical.Analog.Sensors.FaultableCurrentSensor currentSensor(faultMode=MSLFaults.Electrical.Analog.Sensors.FaultableCurrentSensor.FaultMode.Stuck,faultStartTime=1,faultEndTime=2,transitionTime=0),redeclare MSLFaults.Electrical.Analog.Sensors.FaultableVoltageSensor voltageSensor(faultMode=MSLFaults.Electrical.Analog.Sensors.FaultableVoltageSensor.FaultMode.Bias,biasFault=1,faultStartTime=1,faultEndTime=2,transitionTime=0)) annotation(Placement(transformation(origin={95,-98},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultSensingVoltage annotation(Placement(transformation(origin={164,-137},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultSensingCurrent annotation(Placement(transformation(origin={164,-179},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(nativeSignal.gainValue,nativeSignalGain) annotation(Line(points={{-220,129.25},{-172,129.25},{-172,79}},color={0,0,127}));
  connect(nativeSignal.lagValue,nativeSignalLag) annotation(Line(points={{-220,118},{-193,118},{-193,42},{-172,42}},color={0,0,127}));
  connect(normalSignal.gainValue,normalSignalGain) annotation(Line(points={{-50,129.25},{-2,129.25},{-2,79}},color={0,0,127}));
  connect(normalSignal.lagValue,normalSignalLag) annotation(Line(points={{-50,118},{-23,118},{-23,42},{-2,42}},color={0,0,127}));
  connect(faultSignal.gainValue,faultSignalGain) annotation(Line(points={{120,129.25},{168,129.25},{168,79}},color={0,0,127}));
  connect(faultSignal.lagValue,faultSignalLag) annotation(Line(points={{120,118},{147,118},{147,42},{168,42}},color={0,0,127}));
  connect(nativeSensing.readVoltage,nativeSensingVoltage) annotation(Line(points={{-220,-88.5},{-176,-88.5},{-176,-137}},color={0,0,127}));
  connect(nativeSensing.readCurrent,nativeSensingCurrent) annotation(Line(points={{-220,-98},{-195,-98},{-195,-179},{-176,-179}},color={0,0,127}));
  connect(normalSensing.readVoltage,normalSensingVoltage) annotation(Line(points={{-50,-88.5},{-6,-88.5},{-6,-137}},color={0,0,127}));
  connect(normalSensing.readCurrent,normalSensingCurrent) annotation(Line(points={{-50,-98},{-25,-98},{-25,-179},{-6,-179}},color={0,0,127}));
  connect(faultSensing.readVoltage,faultSensingVoltage) annotation(Line(points={{120,-88.5},{164,-88.5},{164,-137}},color={0,0,127}));
  connect(faultSensing.readCurrent,faultSensingCurrent) annotation(Line(points={{120,-98},{145,-98},{145,-179},{164,-179}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=4,Interval=0.008,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true),graphics={Text(extent={{-298,207},{298,193}},textString="Independent gain, lag and limiter branches with the same input ramp",textColor={34,61,73}),Text(extent={{-272,-5},{272,-19}},textString="Physical current and voltage sensing with nominal loading",textColor={34,61,73})}),
    Documentation(info="<html><p>Use common ramp inputs to compare native, normal and faulted signal blocks and electrical sensors. During [1,2) s, the gain and first-order outputs are captured, the limiter is set to 0.25, the current sensor captures its measurement and the voltage sensor adds 1 V.</p><p>Plot signal and sensing outputs; inspect faultSignal.limit.y and faultSensing.actualCurrent for the unexposed limiter and true circuit current. Use the stored experiment settings.</p></html>"));
end SensorAndSignalFaults;
