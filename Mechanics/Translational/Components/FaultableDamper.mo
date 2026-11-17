within MSLFaults.Mechanics.Translational.Components;
model FaultableDamper "Translational damper with configurable effective damping"
  extends Modelica.Mechanics.Translational.Interfaces.PartialCompliantWithRelativeStates;
  parameter Modelica.Units.SI.TranslationalDampingConstant d(final min=0, start=0)
    "Damping constant";
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  type FaultMode=enumeration(Normal "Nominal component relation", DampingLoss "Reduced damping", DampingIncrease "Configured damping increase", Leakage "Leakage-related damping change", Seizure "Increased finite mechanical resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.TranslationalDampingConstant dFault=0.5*d "Damping target for the selected fault law";
  parameter Modelica.Units.SI.TranslationalDampingConstant dSeized=1e12 "Finite damping at full seizure effect";
  Modelica.Units.SI.TranslationalDampingConstant d_effective "Damping used by the constitutive relation";

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";
  parameter Modelica.Units.SI.TranslationalDampingConstant dIncrease=2*d "Damping at full increase effect";
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
  d_effective=if faultMode==FaultMode.DampingLoss or faultMode==FaultMode.Leakage then d+faultActivation*(dFault-d) elseif faultMode==FaultMode.DampingIncrease then d+faultActivation*(dIncrease-d) elseif faultMode==FaultMode.Seizure then d+faultActivation*(dSeized-d) else d;
  f=d_effective*v_rel;
  lossPower = f*v_rel;

  assert(d_effective>=0,"A passive damper requires nonnegative damping.");
  annotation (
    Documentation(info="<html><p>The force is d_effective*v_rel. DampingLoss and Leakage use the same interpolation towards dFault; DampingIncrease uses dIncrease and Seizure uses dSeized. The effective coefficient must remain non-negative.</p><p>Connect the two translational flanges and configure dFault, dIncrease or dSeized; inspect f and lossPower.</p><p>Leakage follows the activation envelope without a separate leakage clock. Seizure is large finite damping.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.Damper</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Line(points={{-90,0},{100,0}}, color={0,127,0}, thickness=0.5),
        Line(points={{-60,-30},{-60,30}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-60,30},{30,-30}}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineColor={0,127,0}, lineThickness=0.5),
        Polygon(points={{50,-90},{20,-80},{20,-100},{50,-90}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{-60,-90},{20,-90}}, color={95,127,95}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-45},{150,-75}}, textString="d=%d"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-20},{-14,-20}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Line(points={{60,-30},{-60,-30},{-60,30},{60,30}}, color={0,127,0}, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableDamper;
