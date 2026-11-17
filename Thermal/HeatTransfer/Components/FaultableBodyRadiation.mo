within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableBodyRadiation "Radiative heat transfer with coefficient faults"
  extends Modelica.Thermal.HeatTransfer.Interfaces.Element1D;
  parameter Real Gr(unit="m2")
    "Combined radiation coefficient between the two surfaces";
  type FaultMode=enumeration(Normal "Nominal behaviour", EmissivityDrift "Change in radiation coefficient", SurfaceContamination "Change in radiation coefficient", RadiationLoss "Reduction of radiation coefficient towards zero", RadiationShort "Increase in radiation coefficient");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real GrFault(unit="m2")=0.5*Gr "Full-effect radiation-coefficient target" ;
  parameter Real GrShort(unit="m2")=1000*Gr "Finite high radiation-coefficient target" ;
  Real Gr_effective(unit="m2") "Fault-adjusted radiation coefficient" ;

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
  Gr_effective=if faultMode==FaultMode.EmissivityDrift or faultMode==FaultMode.SurfaceContamination then Gr+faultActivation*(GrFault-Gr) elseif faultMode==FaultMode.RadiationLoss then Gr*(1-faultActivation) elseif faultMode==FaultMode.RadiationShort then Gr+faultActivation*(GrShort-Gr) else Gr;
  Q_flow=Gr_effective*Modelica.Constants.sigma*(port_a.T^4-port_b.T^4);


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Rectangle(extent={{50,80},{90,-80}}, fillColor={192,192,192}, fillPattern=FillPattern.Backward, lineColor={64,64,64}, lineThickness=0.5),
        Rectangle(extent={{-90,80},{-50,-80}}, fillColor={192,192,192}, fillPattern=FillPattern.Backward, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-36,10},{36,10}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,10},{-26,16}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,10},{-26,4}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-10},{36,-10}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,-16},{36,-10}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,-4},{36,-10}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-30},{36,-30}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-30},{-26,-24}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,-30},{-26,-36}}, color={191,0,0}, thickness=0.5),
        Line(points={{-36,30},{36,30}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,24},{36,30}}, color={191,0,0}, thickness=0.5),
        Line(points={{26,36},{36,30}}, color={191,0,0}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{-150,-90},{150,-120}}, textString="Gr=%Gr"),
        Rectangle(extent={{-50,80},{-44,-80}}, lineColor={191,0,0}, fillColor={191,0,0}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Rectangle(extent={{45,80},{50,-80}}, lineColor={191,0,0}, fillColor={191,0,0}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Exchange heat between <code>port_a</code> and <code>port_b</code> through <code>Gr_effective*sigma*(port_a.T^4-port_b.T^4)</code>. Set the nominal combined radiation coefficient <code>Gr</code> in square metres.</p><p><code>EmissivityDrift</code> and <code>SurfaceContamination</code> both blend towards <code>GrFault</code>. <code>RadiationLoss</code> reduces the coefficient towards zero; <code>RadiationShort</code> blends towards finite <code>GrShort</code>. The fault changes the combined coefficient rather than modelling surface geometry or a separate emissivity state.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.HeatTransfer.Components.BodyRadiation</code>.</p></html>"));
end FaultableBodyRadiation;
