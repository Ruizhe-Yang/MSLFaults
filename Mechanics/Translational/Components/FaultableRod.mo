within MSLFaults.Mechanics.Translational.Components;
model FaultableRod "Rigid massless rod with an additional axial force"
  extends Modelica.Mechanics.Translational.Interfaces.PartialRigid;
  type FaultMode=enumeration(Normal "Nominal component relation", AxialForceDisturbance "Prescribed additional axial force", RodDamage "Sinusoidal axial force", RodLock "Finite viscous resistance to rod motion");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.Force forceFault=1 "Amplitude of the prescribed additional axial force";
  parameter Modelica.Units.SI.Frequency damageFrequency=20 "Frequency of the prescribed damage-related sinusoid";
  Modelica.Units.SI.Force faultForceValue "Additional axial force entering the rod balance";

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
  faultForceValue=if faultMode==FaultMode.AxialForceDisturbance then faultActivation*forceFault elseif faultMode==FaultMode.RodDamage then faultActivation*forceFault*sin(2*Modelica.Constants.pi*damageFrequency*time) elseif faultMode==FaultMode.RodLock then -faultActivation*1e10*der(flange_a.s) else 0;
  0=flange_a.f+flange_b.f+faultForceValue;


  annotation (
    Documentation(info="<html><p>AxialForceDisturbance applies a prescribed force, RodDamage applies a sinusoidal force, and RodLock adds viscous resistance to the absolute flange velocity. The rod retains its rigid length and zero inertia.</p><p>Connect both flanges and set the rigid length; choose forceFault and damageFrequency and inspect faultForceValue.</p><p>RodDamage does not change axial stiffness or disconnect the rod. The additional force is an external mechanical intervention with no explicit reaction port.</p><p>Nominal reference: <code>Modelica.Mechanics.Translational.Components.Rod</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Line(points={{-100,0},{100,0}}, color={0,127,0}, thickness=0.5),
        Polygon(points={{50,-90},{20,-80},{20,-100},{50,-90}}, lineColor={95,127,95}, fillColor={95,127,95}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Line(points={{-60,-90},{20,-90}}, color={95,127,95}, thickness=0.5),
        Rectangle(extent={{-60,10},{60,-10}}, lineColor={0,127,0}, fillColor={160,215,160}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-30},{150,-60}}, textString="L=%L"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableRod;
