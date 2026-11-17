within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableVariableReluctance "Signal-controlled magnetic reluctance with input faults"
  extends Modelica.Magnetic.FluxTubes.Interfaces.TwoPort;
  extends Modelica.Magnetic.FluxTubes.Icons.Reluctance;
  Modelica.Blocks.Interfaces.RealInput R_m(quantity="Reluctance",unit="H-1") "Magnetic reluctance" annotation(Placement(transformation(extent={{-20,-20},{20,20}},rotation=270,origin={0,100})));
  type FaultMode=enumeration(Normal "Nominal component relation", InputDrift "Input multiplied by one plus the effect level", GainError "Changed signal gain", Stuck "Blend towards a prescribed constant", MagneticOpen "Finite magnetic open-path approximation", MagneticShort "Finite magnetic short-path approximation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  parameter Modelica.Units.SI.Reluctance R_mStuck=1 "Prescribed reluctance at full stuck effect";
  parameter Modelica.Units.SI.Reluctance R_mOpen=1e12 "Finite reluctance representing an open magnetic path";
  parameter Modelica.Units.SI.Reluctance R_mShort=1e-12 "Finite reluctance representing a short magnetic path";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law",startActivation(min=0,max=1) "Activation envelope divided by positive severity",endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  Modelica.Units.SI.Reluctance R_m_effective "Magnetic reluctance used by the current relation";

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
  R_m_effective=if faultMode==FaultMode.InputDrift then R_m*(1+faultActivation) elseif faultMode==FaultMode.GainError then R_m*(1-faultActivation) elseif faultMode==FaultMode.Stuck then R_m+faultActivation*(R_mStuck-R_m) elseif faultMode==FaultMode.MagneticOpen then R_m+faultActivation*(R_mOpen-R_m) elseif faultMode==FaultMode.MagneticShort then R_m+faultActivation*(R_mShort-R_m) else R_m;
  V_m=Phi*R_m_effective;


  annotation(
    Documentation(info="<html><p>InputDrift multiplies the input by 1+effectLevel; GainError multiplies it by 1-effectLevel. Stuck blends towards R_mStuck, while MagneticOpen and MagneticShort use the corresponding finite targets.</p><p>Supply R_m in H-1, connect the magnetic ports and configure the stuck or finite open/short target.</p><p>Stuck uses a specified target rather than capturing the input. GainError reaches zero at full severity; check the resulting magnetic constraint and the input range.</p><p>Nominal reference: <code>Modelica.Magnetic.FluxTubes.Basic.VariableReluctance</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    defaultComponentName="reluctance",Icon(graphics={
        Rectangle(extent={{-70,30},{70,-30}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8, radius=0),
        Text(extent={{-43,-48},{43,-77}}, textString="Rm", textColor={64,64,64}, fontSize=0),
        Line(points={{-43,-39},{43,42}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVariableReluctance;
