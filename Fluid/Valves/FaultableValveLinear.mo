within MSLFaults.Fluid.Valves;
model FaultableValveLinear "Linear valve with opening and flow-coefficient faults"
  extends Modelica.Fluid.Interfaces.PartialTwoPortTransport(redeclare replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium);
  Modelica.Blocks.Interfaces.RealInput opening(min=0,max=1)  "Requested valve opening" annotation(Placement(transformation(origin={0,90},extent={{-20,-20},{20,20}},rotation=270),iconTransformation(origin={0,80},extent={{-20,-20},{20,20}},rotation=270)));
  parameter Modelica.Units.SI.AbsolutePressure dp_nominal=1e5 "Nominal pressure drop";
  parameter Medium.MassFlowRate m_flow_nominal=1 "Nominal mass flow rate";
  parameter Real opening_nominal(min=0,max=1)=1 "Nominal opening; use is specific to the selected valve model";
  parameter Real opening_min(min=0,max=1)=0 "Lower opening limit used by the selected valve model";
  type FaultMode=enumeration(Normal "Nominal behaviour", StuckClosed "Blend towards the closed-command target", StuckOpen "Blend towards the open-command target", OpeningBias "Additive opening-command bias", ActuatorGainLoss "Reduction of actuation gain", LeakageIncrease "Additional leakage conductance", FreezeOpening "Hold the opening command captured at activation");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real stuckOpening(min=0,max=1)=0 "Prescribed opening target for the closed-stuck mode" ;
  parameter Real openingBias=0.1 "Full-effect additive opening bias" ;
  parameter Real actuatorGainFault(min=0,max=1)=0.5 "Full-effect actuation gain" ;
  Real opening_effective(min=0,max=1) "Fault-adjusted valve opening" ;

public
  parameter Boolean enabled=true "Enable the selected fault mechanism";
  parameter Boolean useExternalTrigger=false "Use trigger to request activation instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
     "External activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested fault activation";
  output Real effectLevel(min=0,max=1) "Realised severity-scaled fault envelope";
  output Integer activationCount "Number of rising fault activations";
  final parameter Modelica.Fluid.Types.HydraulicConductance k=m_flow_nominal/dp_nominal;
  parameter Real leakageOpening(min=0,max=1)=0.2 "Parallel flow fraction at zero commanded opening";
  discrete Real capturedOpening(start=0,fixed=true) "Opening captured at initialisation or activation" ;
  Real coefficientScale "Scale applied to the nominal flow coefficient" ;
  Real leakage "Additional dimensionless leakage contribution" ;
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(dp_nominal>0 and m_flow_nominal>=0 and actuatorGainFault>=0 and actuatorGainFault<=1 and leakageOpening>=0 and leakageOpening<=1 and opening_min>=0 and opening_min<=1,"Invalid valve flow parameters.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;

  when {initial(),active} then capturedOpening=opening; end when;
  opening_effective=if faultActivation<=0 then opening else max(opening_min,min(1,
    if faultMode==FaultMode.StuckClosed then opening+faultActivation*(stuckOpening-opening)
    elseif faultMode==FaultMode.StuckOpen then opening+faultActivation*(1-opening)
    elseif faultMode==FaultMode.OpeningBias then opening+faultActivation*openingBias
    elseif faultMode==FaultMode.FreezeOpening then opening+faultActivation*(capturedOpening-opening) else opening));
  coefficientScale=if faultMode==FaultMode.ActuatorGainLoss then 1+faultActivation*(actuatorGainFault-1) else 1;
  leakage=if faultMode==FaultMode.LeakageIncrease then faultActivation*leakageOpening*(1-opening_effective) else 0;
  m_flow=(opening_effective*coefficientScale+leakage)*k*dp;
  port_a.h_outflow=inStream(port_b.h_outflow);
  port_b.h_outflow=inStream(port_a.h_outflow);
  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-100,50},{100,-50},{100,50},{0,0},{-100,-50},{-100,50}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-48,-84},{48,-61}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Use the linear relation <code>m_flow=(opening_effective*coefficientScale+leakage)*k*dp</code>, with <code>k=m_flow_nominal/dp_nominal</code>. Enthalpy is transported without an imposed heat source. The declared <code>opening_nominal</code> is not used in this coefficient.</p><p>Opening faults blend towards a stuck, fully open, biased or captured command. <code>FreezeOpening</code> captures at initialisation and rising activation. <code>ActuatorGainLoss</code> scales the flow coefficient; <code>LeakageIncrease</code> adds <code>effectLevel*leakageOpening*(1-opening_effective)</code>. Opening limits apply only at positive effect; at zero effect the raw command is retained.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Valves.ValveLinear</code>.</p></html>"));
end FaultableValveLinear;
