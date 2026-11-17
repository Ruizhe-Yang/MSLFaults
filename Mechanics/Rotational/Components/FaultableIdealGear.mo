within MSLFaults.Mechanics.Rotational.Components;
model FaultableIdealGear "Ideal gear with a variable ratio and prescribed tooth torque"
  extends Modelica.Mechanics.Rotational.Icons.Gear;
  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialElementaryTwoFlangesAndSupport2;
  parameter Real ratio(start=1)
    "Transmission ratio (flange_a.phi/flange_b.phi)";
  Modelica.Units.SI.Angle phi_a
    "Angle between left shaft flange and support";
  Modelica.Units.SI.Angle phi_b
    "Angle between right shaft flange and support";

  type FaultMode=enumeration(Normal "Nominal component relation", GearRatioError "Changed kinematic transmission ratio", ToothDamage "Sinusoidal tooth-related torque", LockedGear "Damping of the exact kinematic residual");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real ratioFault=0.9*ratio "Transmission ratio at full ratio-error effect";
  parameter Modelica.Units.SI.RotationalDampingConstant lockDamping=1e9 "Finite viscous coefficient used by the locking approximation";
  parameter Modelica.Units.SI.Torque toothTorque=1 "Amplitude of the prescribed tooth-torque sinusoid";
  parameter Modelica.Units.SI.Frequency toothFrequency=20 "Frequency of the prescribed tooth-torque sinusoid";
  Real ratio_effective "Transmission ratio in the current kinematic relation";
  Modelica.Units.SI.Torque faultTorque "Additional torque in the faulted balance";

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

  phi_a = flange_a.phi - phi_support;
  phi_b = flange_b.phi - phi_support;
  ratio_effective=if faultMode==FaultMode.GearRatioError then ratio+faultActivation*(ratioFault-ratio) else ratio;
  faultTorque=if faultMode==FaultMode.ToothDamage then faultActivation*toothTorque*sin(2*Modelica.Constants.pi*toothFrequency*time)
    elseif faultMode==FaultMode.LockedGear then faultActivation*lockDamping*der(phi_a-ratio_effective*phi_b) else 0;
  phi_a=ratio_effective*phi_b;
  0=ratio_effective*flange_a.tau+flange_b.tau+faultTorque;


  annotation (
    Documentation(info="<html><p>GearRatioError changes the exact angular relation phi_a=ratio_effective*phi_b. ToothDamage adds a sinusoidal term to the torque balance. LockedGear applies damping to the derivative of that same kinematic residual.</p><p>Connect both shaft flanges and the optional support; configure ratioFault or toothTorque and toothFrequency.</p><p>The ideal constraint makes the LockedGear residual identically zero, so this mode contributes no additional locking torque. A changing ratio is a prescribed kinematic intervention.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.IdealGear</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
       Icon(coordinateSystem(preserveAspectRatio=true,
      extent={{-100,-100},{100,100}}),
      graphics={
        Text(extent={{-153,155},{147,125}}, textColor={0,0,255}, textString="%name"),
        Text(extent={{-146,-49},{154,-79}}, textString="ratio=%ratio"),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealGear;
