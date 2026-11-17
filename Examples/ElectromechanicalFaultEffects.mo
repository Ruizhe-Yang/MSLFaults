within MSLFaults.Examples;
model ElectromechanicalFaultEffects "Compare winding, feedback and magnetic-conversion faults"
  extends Modelica.Icons.Example;
  parameter Integer configuration(min=0,max=3)=1 "Motor case: 0 normal, 1 resistance, 2 feedback gain, 3 combined";
  parameter Boolean dynamicTurns=false "Start the converter turn change during the run when true";
  MSLFaults.Utilities.Internal.ElectromechanicalCircuit motorNative annotation(Placement(transformation(origin={-223,126},extent={{-27,-27},{27,27}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput motorNativeCurrent annotation(Placement(transformation(origin={-118,160},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput motorNativeSpeed annotation(Placement(transformation(origin={-118,119},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput motorNativeTemperature annotation(Placement(transformation(origin={-118,79},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ElectromechanicalCircuit motorFault(redeclare MSLFaults.Electrical.Analog.Basic.FaultableResistor winding(faultMode=if configuration==1 or configuration==3 then MSLFaults.Electrical.Analog.Basic.FaultableResistor.FaultMode.ResistanceStep else MSLFaults.Electrical.Analog.Basic.FaultableResistor.FaultMode.Normal,R_step=4,faultEndTime=6),redeclare MSLFaults.Blocks.Math.FaultableGain feedback(faultMode=if configuration==2 or configuration==3 then MSLFaults.Blocks.Math.FaultableGain.FaultMode.GainDrift else MSLFaults.Blocks.Math.FaultableGain.FaultMode.Normal,kFault=0.05,faultEndTime=6)) annotation(Placement(transformation(origin={49,126},extent={{-27,-27},{27,27}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput motorFaultCurrent annotation(Placement(transformation(origin={154,160},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput motorFaultSpeed annotation(Placement(transformation(origin={154,119},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput motorFaultTemperature annotation(Placement(transformation(origin={154,79},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ElectromagneticCircuit fluxNative annotation(Placement(transformation(origin={-239,-85},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput fluxNativeCurrent annotation(Placement(transformation(origin={-165,-114},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput fluxNativeFlux annotation(Placement(transformation(origin={-165,-166},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ElectromagneticCircuit fluxNormal(redeclare MSLFaults.Magnetic.FluxTubes.Basic.FaultableElectroMagneticConverter converter) annotation(Placement(transformation(origin={-64,-85},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput fluxNormalCurrent annotation(Placement(transformation(origin={10,-114},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput fluxNormalFlux annotation(Placement(transformation(origin={10,-166},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.ElectromagneticCircuit fluxFault(redeclare MSLFaults.Magnetic.FluxTubes.Basic.FaultableElectroMagneticConverter converter(faultMode=MSLFaults.Magnetic.FluxTubes.Basic.FaultableElectroMagneticConverter.FaultMode.TurnLoss,NFault=50,faultStartTime=if dynamicTurns then 1 else 0,faultEndTime=9,transitionTime=if dynamicTurns then 0.3 else 0)) annotation(Placement(transformation(origin={111,-85},extent={{-25,-25},{25,25}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput fluxFaultCurrent annotation(Placement(transformation(origin={185,-114},extent={{-7,-7},{7,7}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput fluxFaultFlux annotation(Placement(transformation(origin={185,-166},extent={{-7,-7},{7,7}},rotation=0)));
initial equation
  assert(configuration>=0 and configuration<=3,"Configuration must be 0 through 3.");
equation
  connect(motorNative.current,motorNativeCurrent) annotation(Line(points={{-196,138.15},{-118,138.15},{-118,160}},color={0,0,127}));
  connect(motorNative.speed,motorNativeSpeed) annotation(Line(points={{-196,126},{-118,126},{-118,119}},color={0,0,127}));
  connect(motorNative.temperature,motorNativeTemperature) annotation(Line(points={{-196,113.85},{-154,113.85},{-154,79},{-118,79}},color={0,0,127}));
  connect(motorFault.current,motorFaultCurrent) annotation(Line(points={{76,138.15},{154,138.15},{154,160}},color={0,0,127}));
  connect(motorFault.speed,motorFaultSpeed) annotation(Line(points={{76,126},{154,126},{154,119}},color={0,0,127}));
  connect(motorFault.temperature,motorFaultTemperature) annotation(Line(points={{76,113.85},{118,113.85},{118,79},{154,79}},color={0,0,127}));
  connect(fluxNative.current,fluxNativeCurrent) annotation(Line(points={{-214,-78},{-165,-78},{-165,-114}},color={0,0,127}));
  connect(fluxNative.flux,fluxNativeFlux) annotation(Line(points={{-214,-92},{-186.5,-92},{-186.5,-166},{-165,-166}},color={0,0,127}));
  connect(fluxNormal.current,fluxNormalCurrent) annotation(Line(points={{-39,-78},{10,-78},{10,-114}},color={0,0,127}));
  connect(fluxNormal.flux,fluxNormalFlux) annotation(Line(points={{-39,-92},{-11.5,-92},{-11.5,-166},{10,-166}},color={0,0,127}));
  connect(fluxFault.current,fluxFaultCurrent) annotation(Line(points={{136,-78},{185,-78},{185,-114}},color={0,0,127}));
  connect(fluxFault.flux,fluxFaultFlux) annotation(Line(points={{136,-92},{163.5,-92},{163.5,-166},{185,-166}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=8,Interval=0.01,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true),graphics={Text(extent={{-301,216},{301,202}},textString="Motor, mechanical load, winding heat and measured feedback",textColor={34,61,73}),Text(extent={{-296,11},{296,-3}},textString="Separate electrical and magnetic conversion experiment",textColor={34,61,73})}),
    Documentation(info="<html><p>The motor paths compare winding resistance and feedback-gain faults using configuration values 0 through 3. Separate magnetic paths compare native, normal and reduced-turn converters. dynamicTurns starts the turn change after initialisation.</p><p>Compare motor current, speed and temperature; inspect converter current and flux as a separate experiment. Use the stored experiment settings.</p></html>"));
end ElectromechanicalFaultEffects;
