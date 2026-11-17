within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableThermalResistor
  "Thermal resistance with configurable finite fault targets"
  extends Modelica.Thermal.HeatTransfer.Interfaces.Element1D;
  parameter Modelica.Units.SI.ThermalResistance R
    "Constant thermal resistance of material";

  type FaultMode=enumeration(Normal "Nominal behaviour", ResistanceDrift "Change in thermal resistance", ResistanceIncrease "Increase in thermal resistance", ThermalOpen "Finite approximation to a thermal open circuit", ThermalShort "Finite approximation to a thermal short circuit");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Modelica.Units.SI.ThermalResistance RFault=2*R "Full-effect thermal-resistance target" ;
  parameter Modelica.Units.SI.ThermalResistance ROpen=1e9 "Finite high-resistance target" ;
  parameter Modelica.Units.SI.ThermalResistance RShort=1e-9 "Finite low-resistance target" ;
  Modelica.Units.SI.ThermalResistance R_effective "Fault-adjusted thermal resistance" ;

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
  R_effective=if faultMode==FaultMode.ResistanceDrift or faultMode==FaultMode.ResistanceIncrease then R+faultActivation*(RFault-R) elseif faultMode==FaultMode.ThermalOpen then R+faultActivation*(ROpen-R) elseif faultMode==FaultMode.ThermalShort then R+faultActivation*(RShort-R) else R;
  dT=R_effective*Q_flow;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Rectangle(extent={{-90,70},{90,-70}}, pattern=LinePattern.None, fillColor={192,192,192}, fillPattern=FillPattern.Forward, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-90,70},{-90,-70}}, thickness=0.5, color={64,64,64}),
        Line(points={{90,70},{90,-70}}, thickness=0.5, color={64,64,64}),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-80},{150,-110}}, textString="R=%R"),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Transfer heat between two ports with <code>dT=R_effective*Q_flow</code>. The component stores no heat.</p><p><code>ResistanceDrift</code> and <code>ResistanceIncrease</code> both interpolate from <code>R</code> to <code>RFault</code>. <code>ThermalOpen</code> and <code>ThermalShort</code> use the finite targets <code>ROpen</code> and <code>RShort</code>. Choose physically suitable non-negative resistances and inspect the resulting heat flow.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.HeatTransfer.Components.ThermalResistor</code>.</p></html>"));
end FaultableThermalResistor;
