within MSLFaults.Fluid.Machines;
model FaultableControlledPump "Controlled pump with set-point faults"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium "Working medium";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Medium.AbsolutePressure p_a_nominal=1e5 "Nominal inlet pressure" ;
  parameter Medium.AbsolutePressure p_b_nominal=2e5 "Nominal outlet pressure" ;
  parameter Medium.MassFlowRate m_flow_nominal=1 "Nominal mass flow rate" ;
  parameter Boolean control_m_flow=true "Select mass-flow control rather than outlet-pressure control" ;
  parameter Boolean use_m_flow_set=false "Expose the mass-flow set-point input" ;
  parameter Boolean use_p_set=false "Expose the pressure set-point input" ;
  Modelica.Blocks.Interfaces.RealInput m_flow_set(unit="kg/s") if use_m_flow_set  "Mass-flow set-point" annotation(Placement(transformation(extent={{-70,70},{-30,110}})));
  Modelica.Blocks.Interfaces.RealInput p_set(unit="Pa") if use_p_set  "Outlet-pressure set-point" annotation(Placement(transformation(extent={{30,70},{70,110}})));
  type FaultMode=enumeration(Normal "Nominal behaviour", OutputLoss "Reduction of the flow or pressure-rise command", SetpointBias "Additive set-point bias", GainLoss "Reduction of the command gain", StuckCommand "Blend towards a prescribed set-point");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real gainFault=0.5 "Full-effect gain factor" ;
  parameter Medium.MassFlowRate massFlowBias=0 "Full-effect mass-flow set-point bias" ;
  parameter Modelica.Units.SI.PressureDifference pressureBias=0 "Full-effect pressure set-point bias" ;
  parameter Real stuckMassFlow=0 "Prescribed stuck mass-flow set-point" ;
  parameter Modelica.Units.SI.Pressure stuckPressure=p_b_nominal "Prescribed stuck outlet-pressure set-point" ;
  Real m_flow_command "Mass-flow command before the fault" ;
  Real p_command "Pressure command before the fault" ;
  Modelica.Fluid.Machines.ControlledPump nominal(redeclare package Medium=Medium,p_a_nominal=p_a_nominal,p_b_nominal=p_b_nominal,m_flow_nominal=m_flow_nominal,control_m_flow=control_m_flow,use_m_flow_set=true,use_p_set=true);
protected
  Modelica.Blocks.Interfaces.RealInput m_flow_set_internal(unit="kg/s") "Selected mass-flow set-point" ;
  Modelica.Blocks.Interfaces.RealInput p_set_internal(unit="Pa") "Selected outlet-pressure set-point" ;

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
  connect(m_flow_set,m_flow_set_internal);
  connect(p_set,p_set_internal);
  if not use_m_flow_set then
    m_flow_set_internal=m_flow_nominal;
  end if;
  if not use_p_set then
    p_set_internal=p_b_nominal;
  end if;
  m_flow_command=m_flow_set_internal;
  p_command=p_set_internal;
  nominal.m_flow_set=if faultMode==FaultMode.OutputLoss then m_flow_command*(1-faultActivation) elseif faultMode==FaultMode.SetpointBias then m_flow_command+faultActivation*massFlowBias elseif faultMode==FaultMode.GainLoss then m_flow_command*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.StuckCommand then m_flow_command+faultActivation*(stuckMassFlow-m_flow_command) else m_flow_command;
  nominal.p_set=if faultMode==FaultMode.OutputLoss then port_a.p+(p_command-port_a.p)*(1-faultActivation) elseif faultMode==FaultMode.SetpointBias then p_command+faultActivation*pressureBias elseif faultMode==FaultMode.GainLoss then port_a.p+(p_command-port_a.p)*(1+faultActivation*(gainFault-1)) elseif faultMode==FaultMode.StuckCommand then p_command+faultActivation*(stuckPressure-p_command) else p_command;
  connect(port_a,nominal.port_a);connect(port_b,nominal.port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Polygon(points={{-38,-40},{-38,40},{49,0},{-38,-40}}, lineColor={255,255,255}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.5),
        Line(points={{-100,0},{-70,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Line(points={{70,0},{100,0}}, color={0,127,255}, thickness=0.75, pattern=LinePattern.Solid),
        Text(extent={{-38,-79},{38,-104}}, textString="set", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Wrap <code>ControlledPump</code> with mass-flow or outlet-pressure set-points selected by <code>control_m_flow</code>. Expose the corresponding conditional input or use <code>m_flow_nominal</code> and <code>p_b_nominal</code>.</p><p><code>OutputLoss</code> reduces the flow command towards zero or the pressure rise towards zero. <code>SetpointBias</code> adds the selected bias; <code>GainLoss</code> scales flow or pressure rise; <code>StuckCommand</code> blends towards the specified fixed command. These faults modify commands rather than pump efficiency or geometry.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Machines.ControlledPump</code>.</p></html>"));
end FaultableControlledPump;
