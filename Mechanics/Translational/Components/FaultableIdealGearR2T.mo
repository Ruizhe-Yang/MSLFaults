within MSLFaults.Mechanics.Translational.Components;
model FaultableIdealGearR2T "Ideal rotational-to-translational gear with kinematic faults"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialElementaryRotationalToTranslational;
  parameter Real ratio(start=1) "Transmission ratio (flangeR.phi/flangeT.s)";
  type FaultMode=enumeration(Normal "Nominal component relation", GearRatioError "Changed kinematic transmission ratio", Slip "Reduced kinematic transmission scale", LockedGear "Declared gear-locking approximation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Real ratioFault=0.9*ratio "Transmission ratio at full ratio-error effect";
  parameter Real slipRatioFault(min=0,max=1)=0.2 "Reduction of transmission scale at full slip effect";
  Real ratio_effective "Transmission ratio in the current kinematic relation";
  Real transmissionScale "Scale applied in the kinematic transmission relation";

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
  ratio_effective=if faultMode==FaultMode.GearRatioError then ratio+faultActivation*(ratioFault-ratio) else ratio;
  transmissionScale=if faultMode==FaultMode.Slip then 1-faultActivation*slipRatioFault elseif faultMode==FaultMode.LockedGear then 1-faultActivation else 1;
  (flangeR.phi-internalSupportR.phi)=ratio_effective*(flangeT.s-internalSupportT.s)*transmissionScale;
  0=ratio_effective*flangeR.tau+flangeT.f;


  annotation(
    Documentation(info="<html><p>GearRatioError changes ratio_effective. Slip and LockedGear scale the translational term of the exact position relation. The torque-force relation uses ratio_effective without the transmission scale.</p><p>Connect flangeR and flangeT and their required supports; set ratioFault or slipRatioFault and inspect the relative coordinates.</p><p>At full LockedGear effect the rotational displacement relative to support is zero. The slip model changes ideal kinematics and contains no frictional slip or thermal loss law.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.IdealGearR2T</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(graphics={
        Ellipse(extent={{-43,-20},{35,58}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8),
        Ellipse(extent={{-14,8},{6,28}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.55),
        Line(points={{-100,0},{-61,0},{-61,18},{-43,18}}, color={64,64,64}, thickness=0.8, pattern=LinePattern.Solid),
        Line(points={{-50,-41},{-35,-41},{-35,-31},{-20,-31},{-20,-41},{-5,-41},{-5,-31},{10,-31},{10,-41},{25,-41},{25,-31},{40,-31},{40,-41},{80,-41},{80,0},{100,0}}, color={64,64,64}, thickness=0.8, pattern=LinePattern.Solid),
        Text(extent={{-44,-65},{44,-89}}, textString="R/T", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableIdealGearR2T;
