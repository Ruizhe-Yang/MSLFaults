within MSLFaults.Fluid.Machines;
model FaultablePump "Shaft-driven pump with prescribed fault torque"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium "Working medium";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.NonSI.AngularVelocity_rpm N_nominal=1500 "Nominal rotational speed for the pump characteristic" ;
  parameter Modelica.Units.SI.VolumeFlowRate V_flow_nominal[3]={0,0.001,0.002} "Volume-flow coordinates of the quadratic pump curve" ;
  parameter Modelica.Units.SI.Position head_nominal[3]={20,10,0} "Head coordinates of the quadratic pump curve" ;
  Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft  "Mechanical shaft connection" annotation(Placement(transformation(extent={{-10,90},{10,110}})));
  type FaultMode=enumeration(Normal "Nominal behaviour", EfficiencyLoss "Additional speed-dependent shaft torque", BearingFrictionIncrease "Additional speed-dependent shaft torque", ImpellerDamage "Sinusoidal shaft torque", ShaftSeizure "Shaft torque proportional to angular speed");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Modelica.Units.SI.Torque faultTorqueAmplitude=1 "Amplitude of the prescribed shaft-torque effect" ;
  parameter Modelica.Units.SI.RotationalDampingConstant seizureDamping=1e9 "Coefficient of the speed-proportional shaft-torque effect" ;
  parameter Modelica.Units.SI.Frequency damageFrequency=20 "Frequency of the sinusoidal shaft-torque effect" ;
  Modelica.Units.SI.Torque faultTorque "Torque command supplied to the fault torque source" ;
  Modelica.Fluid.Machines.Pump nominal(
    redeclare package Medium=Medium,N_nominal=N_nominal,
    redeclare function flowCharacteristic=Modelica.Fluid.Machines.BaseClasses.PumpCharacteristics.quadraticFlow(V_flow_nominal=V_flow_nominal,head_nominal=head_nominal));
protected
  Modelica.Mechanics.Rotational.Sources.Torque faultTorqueSource(useSupport=false)
    "Fixed-topology torque source for shaft faults";
  Modelica.Blocks.Sources.RealExpression faultTorqueCommand(y=faultTorque);

public
  parameter Boolean enabled=true "Enable the selected fault mechanism";
  parameter Boolean useExternalTrigger=false "Use trigger to request activation instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested fault activation";
  output Real effectLevel(min=0,max=1) "Realised severity-scaled fault envelope";
  output Integer activationCount "Number of rising fault activations";

protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  faultTorque=if faultMode==FaultMode.EfficiencyLoss or faultMode==FaultMode.BearingFrictionIncrease then faultActivation*faultTorqueAmplitude*sign(der(shaft.phi)) elseif faultMode==FaultMode.ImpellerDamage then faultActivation*faultTorqueAmplitude*sin(2*Modelica.Constants.pi*damageFrequency*time) elseif faultMode==FaultMode.ShaftSeizure then faultActivation*seizureDamping*der(shaft.phi) else 0;
  connect(shaft,nominal.shaft);
  connect(shaft,faultTorqueSource.flange);
  connect(faultTorqueCommand.y,faultTorqueSource.tau);
  connect(port_a,nominal.port_a);connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-38,-40},{-38,40},{49,0},{-38,-40}}, lineColor={255,255,255}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.5),
        Line(points={{-100,0},{-70,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{70,0},{100,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Rectangle(extent={{-7,70},{7,100}}, lineColor={64,64,64}, fillColor={235,235,235}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.6, radius=0),
        Text(extent={{-43,-79},{43,-104}}, textString="tau", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Combine a shaft-driven MSL <code>Pump</code> and a torque source on the same shaft. Configure the nominal quadratic head curve and connect the fluid ports and mechanical shaft.</p><p><code>EfficiencyLoss</code> and <code>BearingFrictionIncrease</code> both prescribe <code>effectLevel*faultTorqueAmplitude*sign(der(shaft.phi))</code>. <code>ImpellerDamage</code> prescribes a sinusoidal torque, while <code>ShaftSeizure</code> prescribes <code>effectLevel*seizureDamping*der(shaft.phi)</code>. The torque enters the MSL source with this sign. These are imposed shaft-torque effects; the hydraulic efficiency curve and shaft topology remain unchanged, and seizure is not an ideal kinematic lock.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Machines.Pump</code>.</p></html>"));
end FaultablePump;
