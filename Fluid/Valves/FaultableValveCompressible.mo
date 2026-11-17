within MSLFaults.Fluid.Valves;
model FaultableValveCompressible "Compressible valve with opening-command faults"
  replaceable package Medium=Modelica.Media.Air.SimpleAir constrainedby Modelica.Media.Interfaces.PartialMedium "Working medium";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.SI.Pressure dp_nominal=1e5 "Nominal pressure drop";
  parameter Medium.MassFlowRate m_flow_nominal=1 "Nominal mass flow rate";
  parameter Real opening_nominal(min=0,max=1)=1 "Nominal opening; use is specific to the selected valve model";
  parameter Real opening_min(min=0,max=1)=0 "Lower opening limit used by the selected valve model";
  parameter Real Fxt_full=0.5 "Full-opening choked-flow factor";
  type FaultMode=enumeration(Normal "Nominal behaviour", StuckClosed "Blend towards the closed-command target", StuckOpen "Blend towards the open-command target", OpeningBias "Additive opening-command bias", ActuatorGainLoss "Reduction of actuation gain", LeakageIncrease "Increased residual flow or opening");
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
  Modelica.Blocks.Interfaces.RealInput opening  "Requested valve opening" annotation(Placement(transformation(origin={0,100},extent={{-20,-20},{20,20}},rotation=270)));
  Real opening_effective(min=0,max=1) "Fault-adjusted valve opening" ;
  Modelica.Fluid.Valves.ValveCompressible nominal(
    redeclare package Medium=Medium,
    dp_nominal=dp_nominal,m_flow_nominal=m_flow_nominal,opening_nominal=opening_nominal,Fxt_full=Fxt_full);

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
  opening_effective=max(opening_min,min(1,if faultMode==FaultMode.StuckClosed then opening+faultActivation*(stuckOpening-opening)
    elseif faultMode==FaultMode.StuckOpen then opening+faultActivation*(1-opening)
    elseif faultMode==FaultMode.OpeningBias then opening+faultActivation*openingBias
    elseif faultMode==FaultMode.ActuatorGainLoss then opening*(1+faultActivation*(actuatorGainFault-1))
    elseif faultMode==FaultMode.LeakageIncrease then max(opening,faultActivation*0.2) else opening));
  nominal.opening=opening_effective;
  connect(port_a,nominal.port_a);
  connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-100,50},{100,-50},{100,50},{0,0},{-100,-50},{-100,50}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-48,-84},{-29,-73},{-9,-66},{15,-62},{48,-61}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{-33,-97},{33,-116}}, textString="gas", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Wrap the MSL compressible valve and retain its choked-flow characteristic. Set the nominal pressure drop, mass flow and <code>Fxt_full</code>, and connect a compatible gas medium.</p><p>The fault blends <code>opening</code> towards <code>stuckOpening</code> or one, adds <code>openingBias</code>, or scales the command with <code>actuatorGainFault</code>. <code>LeakageIncrease</code> imposes an opening floor of <code>0.2*effectLevel</code>. The final command is limited to <code>[opening_min,1]</code>, also in normal operation.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Valves.ValveCompressible</code>.</p></html>"));
end FaultableValveCompressible;
