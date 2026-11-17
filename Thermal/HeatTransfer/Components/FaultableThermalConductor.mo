within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableThermalConductor
  "Thermal conductance with configurable finite fault targets"
  extends Modelica.Thermal.HeatTransfer.Interfaces.Element1D;
  parameter Modelica.Units.SI.ThermalConductance G
    "Constant thermal conductance of material";

  type FaultMode=enumeration(Normal "Nominal behaviour", ConductanceDrift "Change in thermal conductance", ConductanceLoss "Reduction of thermal conductance", ThermalOpen "Finite approximation to a thermal open circuit", ThermalShort "Finite approximation to a thermal short circuit");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Modelica.Units.SI.ThermalConductance GFault=0.5*G "Full-effect thermal-conductance target" ;
  parameter Modelica.Units.SI.ThermalConductance GOpen=1e-9 "Finite low-conductance target" ;
  parameter Modelica.Units.SI.ThermalConductance GShort=1e9 "Finite high-conductance target" ;
  Modelica.Units.SI.ThermalConductance G_effective "Fault-adjusted thermal conductance" ;

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
  G_effective=if faultMode==FaultMode.ConductanceDrift or faultMode==FaultMode.ConductanceLoss then G+faultActivation*(GFault-G) elseif faultMode==FaultMode.ThermalOpen then G+faultActivation*(GOpen-G) elseif faultMode==FaultMode.ThermalShort then G+faultActivation*(GShort-G) else G;
  Q_flow=G_effective*dT;

  assert(G_effective>=0,"Passive heat transfer requires nonnegative conductance.");
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Rectangle(extent={{-90,70},{90,-70}}, pattern=LinePattern.None, fillColor={192,192,192}, fillPattern=FillPattern.Backward, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-90,70},{-90,-70}}, thickness=0.5, color={64,64,64}),
        Line(points={{90,70},{90,-70}}, thickness=0.5, color={64,64,64}),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-80},{150,-110}}, textString="G=%G"),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Transfer heat between two ports with <code>Q_flow=G_effective*dT</code>. The component stores no heat.</p><p><code>ConductanceDrift</code> and <code>ConductanceLoss</code> both interpolate from <code>G</code> to <code>GFault</code>. <code>ThermalOpen</code> and <code>ThermalShort</code> use the finite targets <code>GOpen</code> and <code>GShort</code>. The connection topology is fixed and the effective conductance must remain non-negative.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.HeatTransfer.Components.ThermalConductor</code>.</p></html>"));
end FaultableThermalConductor;
