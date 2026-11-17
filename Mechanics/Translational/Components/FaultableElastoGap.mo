within MSLFaults.Mechanics.Translational.Components;
model FaultableElastoGap "Unilateral elastic contact with stiffness, damping and gap faults"
  extends Modelica.Mechanics.Translational.Interfaces.PartialCompliantWithRelativeStates;
  parameter Modelica.Units.SI.TranslationalSpringConstant c(final min=0, start=1)
    "Spring constant";
  parameter Modelica.Units.SI.TranslationalDampingConstant d(final min=0, start=1)
    "Damping constant";
  parameter Modelica.Units.SI.Position s_rel0=0 "Unstretched spring length";
  parameter Real n(final min=1) = 1
    "Exponent of spring force ( f_c = -c*|s_rel-s_rel0|^n )";
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;

  /*
Please note that initialisation might fail due to the nonlinear spring characteristic
(spring force is zero for s_rel > s_rel0)
if a positive force is acting on the element and no other force balances this force
(e.g., when setting both initial velocity and acceleration to 0)
*/
  Boolean contact "= true, if contact, otherwise no contact";
  type FaultMode=enumeration(Normal "Nominal component relation", StiffnessLoss "Reduced stiffness", DampingLoss "Reduced damping", GapShift "Changed contact threshold", ContactLoss "Reduced contact stiffness and damping", JammedContact "Increased finite contact stiffness");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.TranslationalSpringConstant cFault=0.5*c "Stiffness target for the selected fault law";
  parameter Modelica.Units.SI.TranslationalDampingConstant dFault=0.5*d "Damping target for the selected fault law";
  parameter Modelica.Units.SI.Position sRel0Fault=s_rel0+0.01 "Unstretched length or contact coordinate at full shift effect";
  Modelica.Units.SI.TranslationalSpringConstant c_effective "Stiffness used by the constitutive relation";
  Modelica.Units.SI.TranslationalDampingConstant d_effective "Damping used by the constitutive relation";
  Modelica.Units.SI.Position s_rel0_effective "Effective unstretched length or contact threshold";
protected
  Modelica.Units.SI.Force f_c "Spring force";
  Modelica.Units.SI.Force f_d2 "Linear damping force";
  Modelica.Units.SI.Force f_d
    "Linear damping force which is limited by spring force (|f_d| <= |f_c|)";

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
  // Modify contact force, so that it is only "pushing" and not
  // "pulling/sticking" and that it is continuous
  c_effective=if faultMode==FaultMode.StiffnessLoss then c+faultActivation*(cFault-c) elseif faultMode==FaultMode.ContactLoss then c*(1-faultActivation) elseif faultMode==FaultMode.JammedContact then c+faultActivation*(1e12-c) else c;
  d_effective=if faultMode==FaultMode.DampingLoss then d+faultActivation*(dFault-d) elseif faultMode==FaultMode.ContactLoss then d*(1-faultActivation) else d;
  s_rel0_effective=if faultMode==FaultMode.GapShift then s_rel0+faultActivation*(sRel0Fault-s_rel0) else s_rel0;
  contact=s_rel<s_rel0_effective;
  f_c=smooth(1,noEvent(if contact then -c_effective*abs(s_rel-s_rel0_effective)^n else 0));
  f_d2=if contact then d_effective*v_rel else 0;
  f_d = smooth(0, noEvent(if contact then (if f_d2 < f_c then f_c else if
    f_d2 > -f_c then -f_c else f_d2) else 0));
  f = f_c + f_d;
  lossPower = f_d*v_rel;


  annotation (
    Documentation(info="<html><p>Contact occurs for s_rel&lt;s_rel0_effective. StiffnessLoss, DampingLoss and GapShift modify the contact parameters. ContactLoss reduces stiffness and damping towards zero; JammedContact raises stiffness towards 1e12. The damping force is limited by the compressive spring force.</p><p>Connect two flanges; set c, d, s_rel0 and exponent n, then inspect contact and f during the selected fault.</p><p>JammedContact retains the unilateral contact condition and does not bond separated surfaces. Initial force and position constraints must be consistent.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.ElastoGap</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Line(points={{-98,0},{-48,0}}, color={0,127,0}, thickness=0.5),
        Line(points={{-48,36},{-48,-38}}, thickness=1, color={0,127,0}),
        Line(points={{-12,-38},{-12,36}}, thickness=1, color={0,127,0}),
        Line(points={{-12,-28},{70,-28},{70,24}}, color={0,127,0}, thickness=0.5),
        Line(points={{70,0},{98,0}}, color={0,127,0}, thickness=0.5),
        Line(points={{-12,24},{0,24},{6,34},{18,14},{30,34},{42,14},{54,34},{60,24},{70,24}}, color={0,127,0}, thickness=0.5),
        Rectangle(extent={{10,-6},{50,-50}}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineColor={0,127,0}, lineThickness=0.5),
        Line(points={{-52,-70},{28,-70}}, color={95,127,95}, thickness=0.5),
        Polygon(points={{58,-70},{28,-60},{28,-80},{58,-70}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-125},{150,-95}}, textString="c=%c"),
        Text(extent={{-150,-160},{150,-130}}, textString="d=%d"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-44},{22,-44},{22,-28}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Line(points={{0,-50},{50,-50},{50,-6},{0,-6}}, color={0,127,0}, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableElastoGap;
