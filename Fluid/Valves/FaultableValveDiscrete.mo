within MSLFaults.Fluid.Valves;
model FaultableValveDiscrete "Boolean valve with command faults"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium "Working medium";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.SI.AbsolutePressure dp_nominal=1e5 "Nominal pressure drop" ;
  parameter Medium.MassFlowRate m_flow_nominal=1 "Nominal mass flow rate" ;
  parameter Real opening_min(min=0)=0 "Lower opening limit used by the selected valve model" ;
  type FaultMode=enumeration(Normal "Nominal behaviour", StuckClosed "Force a closed command", StuckOpen "Force an open command", CommandInvert "Invert the Boolean command", LeakageIncrease "Force an open command");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  Modelica.Blocks.Interfaces.BooleanInput open  "Requested Boolean valve command" annotation(Placement(transformation(origin={0,80},extent={{-20,-20},{20,20}},rotation=270)));
  Boolean open_effective "Boolean command supplied to the nominal valve" ;
  Modelica.Fluid.Valves.ValveDiscrete nominal(redeclare package Medium=Medium,dp_nominal=dp_nominal,m_flow_nominal=m_flow_nominal,opening_min=opening_min);

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
  open_effective=if faultMode==FaultMode.StuckClosed and faultActivation>0.5 then false elseif faultMode==FaultMode.StuckOpen and faultActivation>0.5 then true elseif faultMode==FaultMode.CommandInvert and faultActivation>0.5 then not open elseif faultMode==FaultMode.LeakageIncrease and faultActivation>0.5 then true else open;
  nominal.open=open_effective;
  connect(port_a,nominal.port_a);connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{-100,50},{100,-50},{100,50},{0,0},{-100,-50},{-100,50}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-48,-83},{-10,-83},{-10,-61},{48,-61}}, color={64,64,64}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Drive the contained MSL valve with the Boolean <code>open</code> command. Configure the nominal flow and pressure drop and the residual closed opening.</p><p>A fault overrides the command only while <code>effectLevel &gt; 0.5</code>. <code>StuckClosed</code> forces false, <code>StuckOpen</code> and <code>LeakageIncrease</code> force true, and <code>CommandInvert</code> negates the input. At or below the threshold, the original command is used.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Valves.ValveDiscrete</code>.</p></html>"));
end FaultableValveDiscrete;
