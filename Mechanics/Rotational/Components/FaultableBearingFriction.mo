within MSLFaults.Mechanics.Rotational.Components;
model FaultableBearingFriction "Bearing friction with configurable loss, wear and seizure effects"
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialElementaryTwoFlangesAndSupport2;

  parameter Real tau_pos[:, 2]=[0, 1]
    "Positive sliding friction characteristic [N.m] as function of w [rad/s] (w>=0)";
  parameter Real peak(final min=1) = 1
    "Peak for maximum friction torque at w==0 (tau0_max = peak*tau_pos[1,2])";

  extends Modelica.Mechanics.Rotational.Interfaces.PartialFriction;
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;

  Modelica.Units.SI.Angle phi
    "Angle between shaft flanges (flange_a, flange_b) and support";
  Modelica.Units.SI.Torque tau "Friction torque";
  Modelica.Units.SI.AngularVelocity w "Angular velocity relative to the support";
  Modelica.Units.SI.AngularAcceleration a
    "Angular acceleration relative to the support";

  type FaultMode=enumeration(Normal "Nominal component relation", FrictionLoss "Reduced friction", FrictionIncrease "Increased friction", Wear "Configured wear-related friction change", Seizure "Increased finite mechanical resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real frictionLossScale(min=0,max=1)=0.5 "Friction multiplier at full loss effect";
  parameter Real frictionIncreaseScale(min=1)=2 "Friction multiplier at full increase effect";
  parameter Real wearScale(min=1)=3 "Friction multiplier at completed wear progression";
  parameter Modelica.Units.SI.Time wearTime(min=Modelica.Constants.small)=10 "Duration to reach full configured wear progression";
  parameter Real seizureScale(min=1)=1000 "Finite friction multiplier at full seizure effect";
  Real frictionScale "Effective friction multiplier";
  Real wearProgress(min=0,max=1) "Bounded wear progression on the configured absolute time window";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";

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

  // Constant auxiliary variables
  wearProgress=if time<=faultStartTime then 0 else min(1,max(0,(min(time,faultEndTime)-faultStartTime)/wearTime));
  frictionScale=if faultMode==FaultMode.FrictionLoss then 1+faultActivation*(frictionLossScale-1)
    elseif faultMode==FaultMode.FrictionIncrease then 1+faultActivation*(frictionIncreaseScale-1)
    elseif faultMode==FaultMode.Wear then 1+severity*endActivation*wearProgress*(wearScale-1)
    elseif faultMode==FaultMode.Seizure then 1+faultActivation*(seizureScale-1) else 1;
  tau0=frictionScale*Modelica.Math.Vectors.interpolate(tau_pos[:,1],tau_pos[:,2],0,1);
  tau0_max = peak*tau0;
  free = false;

  phi = flange_a.phi - phi_support;
  flange_b.phi = flange_a.phi;

  // Angular velocity and angular acceleration of flanges
  w = der(phi);
  a = der(w);
  w_relfric = w;
  a_relfric = a;

  // Friction torque
  flange_a.tau + flange_b.tau - tau = 0;

  // Friction torque
  tau = frictionScale*(if locked then sa*unitTorque else (
    if startForward then
      Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], w, 1)
    else if startBackward then
      -Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], -w, 1)
    else if pre(mode) == Forward then
      Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], w, 1)
    else
      -Modelica.Math.Vectors.interpolate(tau_pos[:,1], tau_pos[:,2], -w, 1)));
  lossPower = tau*w_relfric;


  annotation (Documentation(info="<html><p>Scale the Coulomb friction characteristic and its static threshold. FrictionLoss, FrictionIncrease and Seizure use the activation envelope. Wear uses a bounded progression from faultStartTime to faultEndTime with time scale wearTime.</p><p>Connect the shaft flanges and optional support; set tau_pos and the selected friction scale, then inspect tau and lossPower.</p><p>Wear follows the configured absolute times even with an external trigger. Its scale is retained while active or while effectLevel is positive, then returns to nominal; it is not a permanent wear state.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.BearingFriction</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
       Icon(coordinateSystem(preserveAspectRatio=true,
      extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
        Rectangle(lineColor={64,64,64}, fillColor={192,192,192}, fillPattern=FillPattern.HorizontalCylinder, extent={{-100,-10},{100,10}}, lineThickness=0.5),
        Rectangle(extent={{-60,-60},{60,-10}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,-25},{60,-10}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,-61},{60,-45}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={255,255,255}, fillPattern=FillPattern.Solid, extent={{-50,-50},{50,-18}}, lineColor={64,64,64}, lineThickness=0.5),
        Polygon(fillColor={160,160,164}, fillPattern=FillPattern.Solid, points={{60,-60},{60,-70},{75,-70},{75,-80},{-75,-80},{-75,-70},{-60,-70},{-60,-60},{60,-60}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-75,-10},{-75,-70}}, color={64,64,64}, thickness=0.5),
        Line(points={{75,-10},{75,-70}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-60,10},{60,60}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,45},{60,60}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={192,192,192}, fillPattern=FillPattern.Solid, extent={{-60,10},{60,25}}, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(fillColor={255,255,255}, fillPattern=FillPattern.Solid, extent={{-50,19},{50,51}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-75,70},{-75,10}}, color={64,64,64}, thickness=0.5),
        Polygon(fillColor={160,160,164}, fillPattern=FillPattern.Solid, points={{60,60},{60,70},{75,70},{75,80},{-75,80},{-75,70},{-60,70},{-60,60},{60,60}}, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{75,70},{75,10}}, color={64,64,64}, thickness=0.5),
        Text(textColor={0,0,255}, extent={{-150,155},{150,125}}, textString="%name"),
        Line(points={{0,-80},{0,-100}}, color={64,64,64}, thickness=0.5),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-35},{2,-35}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableBearingFriction;
