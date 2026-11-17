within MSLFaults.Electrical.Analog.Sources;
model FaultableSupplyVoltage "Dual-rail voltage supply with rail faults"
  type FaultMode=enumeration(Normal "Nominal behaviour", PositiveRailLoss "Attenuate the positive rail", NegativeRailLoss "Attenuate the negative rail", RailAmplitudeDrift "Scale both rail voltages", GroundShift "Offset both rails relative to ground");
  parameter Modelica.Units.SI.Voltage Vps=+15 "Positive supply";
  parameter Modelica.Units.SI.Voltage Vns=-15 "Negative supply";
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p "Positive electrical terminal" annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n "Negative electrical terminal" annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin ground annotation(Placement(transformation(extent={{-10,-10},{10,10}})));
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter Modelica.Units.SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter Real railScaleFault=0.5 "Scale factor applied to both rails at full effect";
  parameter Modelica.Units.SI.Voltage groundShiftFault=1 "Common rail-voltage offset relative to the ground pin";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Modelica.Units.SI.Voltage Vps_effective "Positive rail voltage before common-mode offset";
  Modelica.Units.SI.Voltage Vns_effective "Negative rail voltage before common-mode offset";

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the scheduled fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    "External fault activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Fault activation request state";
  output Real effectLevel(min=0,max=1) "Severity-scaled fault envelope";
  output Integer activationCount "Number of activation starts";

protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger) "Reversible fault activation envelope";
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  Vps_effective = if faultMode==FaultMode.PositiveRailLoss then Vps*(1-faultActivation)
    elseif faultMode==FaultMode.RailAmplitudeDrift then Vps*(1+faultActivation*(railScaleFault-1)) else Vps;
  Vns_effective = if faultMode==FaultMode.NegativeRailLoss then Vns*(1-faultActivation)
    elseif faultMode==FaultMode.RailAmplitudeDrift then Vns*(1+faultActivation*(railScaleFault-1)) else Vns;
  pin_p.v-ground.v = Vps_effective + (if faultMode==FaultMode.GroundShift then faultActivation*groundShiftFault else 0);
  pin_n.v-ground.v = Vns_effective + (if faultMode==FaultMode.GroundShift then faultActivation*groundShiftFault else 0);
  pin_p.i + pin_n.i + ground.i = 0;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-60,40},{-60,-40}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,40},{40,-40}}, color={0,0,255}, thickness=0.5),
        Line(points={{-40,20},{-40,-20}}, color={0,0,255}, thickness=0.5),
        Line(points={{60,20},{60,-20}}, color={0,0,255}, thickness=0.5),
        Line(points={{-90,0},{-60,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textColor={0,0,255}, textString="%name"),
        Line(points={{-40,0},{-10,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{10,0},{40,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-100,40},{-80,20}}, textColor={0,0,255}, textString="+"),
        Text(extent={{80,40},{100,20}}, textColor={0,0,255}, textString="-"),
        Text(extent={{-10,40},{10,20}}, textColor={0,0,255}, textString="0"),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Prescribe positive and negative voltages relative to the ground pin. Faults attenuate either rail, scale both rails, or add the same offset to both terminal voltages. Terminal currents satisfy the three-pin current balance.</p><p>Connect pin_p, pin_n and ground, set Vps/Vns, and choose railScaleFault or groundShiftFault as required.</p><p>GroundShift offsets both rails relative to the connected ground pin; it does not change the potential imposed on that ground pin by its network.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Sources.SupplyVoltage</code>.</p></html>"));
end FaultableSupplyVoltage;
