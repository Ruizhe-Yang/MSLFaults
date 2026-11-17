within MSLFaults.Fluid.Machines;
model FaultablePrescribedPump "Prescribed-speed pump with speed-command faults"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium "Working medium";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.NonSI.AngularVelocity_rpm N_nominal=1500 "Nominal rotational speed for the pump characteristic" ;
  parameter Boolean use_N_in=false "Expose the rotational-speed input" ;
  parameter Modelica.Units.NonSI.AngularVelocity_rpm N_const=N_nominal "Rotational speed used without an external input" ;
  parameter Modelica.Units.SI.VolumeFlowRate V_flow_nominal[3]={0,0.001,0.002} "Volume-flow coordinates of the quadratic pump curve" ;
  parameter Modelica.Units.SI.Position head_nominal[3]={20,10,0} "Head coordinates of the quadratic pump curve" ;
  Modelica.Blocks.Interfaces.RealInput N_in(unit="rev/min") if use_N_in  "Requested rotational speed" annotation(Placement(transformation(extent={{-20,80},{20,120}})));
  type FaultMode=enumeration(Normal "Nominal behaviour", SpeedLoss "Reduction of the speed-command scale", SpeedDrift "Change of the speed-command scale", StuckSpeed "Blend towards a prescribed speed", PumpTrip "Reduction of the speed command towards zero");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real speedScaleFault=0.5 "Full-effect speed-command scale" ;
  parameter Modelica.Units.NonSI.AngularVelocity_rpm stuckSpeed=0 "Prescribed stuck rotational speed" ;
  Real speedCommand "Rotational-speed command before the fault" ;
  Real speedEffective "Fault-adjusted speed before the positive-speed clamp" ;
  Modelica.Fluid.Machines.PrescribedPump nominal(
    redeclare package Medium=Medium,N_nominal=N_nominal,use_N_in=true,
    redeclare function flowCharacteristic=Modelica.Fluid.Machines.BaseClasses.PumpCharacteristics.quadraticFlow(V_flow_nominal=V_flow_nominal,head_nominal=head_nominal));
protected
  Modelica.Blocks.Interfaces.RealInput N_in_internal(unit="rev/min") "Selected rotational-speed command" ;

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
  connect(N_in,N_in_internal);
  if not use_N_in then
    N_in_internal=N_const;
  end if;
  speedCommand=N_in_internal;
  speedEffective=if faultMode==FaultMode.SpeedLoss or faultMode==FaultMode.SpeedDrift then speedCommand*(1+faultActivation*(speedScaleFault-1)) elseif faultMode==FaultMode.StuckSpeed then speedCommand+faultActivation*(stuckSpeed-speedCommand) elseif faultMode==FaultMode.PumpTrip then speedCommand*(1-faultActivation) else speedCommand;
  nominal.N_in=max(speedEffective,1e-3);
  connect(port_a,nominal.port_a);connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-38,-40},{-38,40},{49,0},{-38,-40}}, lineColor={255,255,255}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.5),
        Line(points={{-100,0},{-70,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{70,0},{100,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{0,100},{0,71}}, color={0,0,255}, thickness=0.6, pattern=LinePattern.Solid),
        Text(extent={{-35,-79},{35,-104}}, textString="N", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Drive the contained <code>PrescribedPump</code> from <code>N_in</code> or <code>N_const</code>. The nominal head curve is quadratic through <code>V_flow_nominal</code> and <code>head_nominal</code>.</p><p><code>SpeedLoss</code> and <code>SpeedDrift</code> use the same envelope-scaled speed factor. <code>StuckSpeed</code> blends towards <code>stuckSpeed</code>, and <code>PumpTrip</code> reduces the command towards zero. The applied speed is clamped to at least <code>1e-3</code> rev/min, including normal operation.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Machines.PrescribedPump</code>.</p></html>"));
end FaultablePrescribedPump;
