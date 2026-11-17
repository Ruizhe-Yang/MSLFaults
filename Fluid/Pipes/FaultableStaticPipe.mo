within MSLFaults.Fluid.Pipes;
model FaultableStaticPipe "Static pipe with additional quadratic pressure loss"
  replaceable package Medium=Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialMedium "Working medium";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium)
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.Units.SI.Length length=1 "Pipe length";
  parameter Modelica.Units.SI.Diameter diameter=0.1 "Pipe diameter";
  parameter Modelica.Units.SI.Height height_ab=0 "Height of port_b relative to port_a";
  parameter Modelica.Units.SI.Length roughness=2.5e-5 "Average inner-wall roughness";
  parameter Integer nParallel(min=1)=1 "Number of parallel pipes; check forwarding in the selected pipe model";
  parameter Boolean allowFlowReversal=true "Allow flow in either direction" ;
  type FaultMode=enumeration(Normal "Nominal behaviour", RoughnessIncrease "Additional pressure loss at a reduced coefficient", Fouling "Additional pressure loss", PartialBlockage "Increased additional pressure loss", CompleteBlockage "Large finite additional pressure loss");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real blockageCoefficient(unit="Pa.s2/kg2")=1e8 "Reference coefficient of additional quadratic pressure loss" ;
  parameter Real completeBlockageCoefficient(unit="Pa.s2/kg2")=1e14 "Finite pressure-loss coefficient for complete blockage" ;
  Real K_fault(unit="Pa.s2/kg2") "Effective coefficient of additional quadratic pressure loss" ;
  model InlineFaultPressureLoss "Local, topology-invariant fault pressure loss"
    replaceable package Medium=Modelica.Media.Interfaces.PartialMedium;
    Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium);
    Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium=Medium);
    Modelica.Blocks.Interfaces.RealInput K(unit="Pa.s2/kg2") "Coefficient of quadratic pressure loss" ;
  equation
    port_a.m_flow+port_b.m_flow=0;
    port_a.p-port_b.p=K*port_a.m_flow*abs(port_a.m_flow);
    port_a.h_outflow=inStream(port_b.h_outflow);
    port_b.h_outflow=inStream(port_a.h_outflow);
    port_a.Xi_outflow=inStream(port_b.Xi_outflow);
    port_b.Xi_outflow=inStream(port_a.Xi_outflow);
    port_a.C_outflow=inStream(port_b.C_outflow);
    port_b.C_outflow=inStream(port_a.C_outflow);
  end InlineFaultPressureLoss;
  InlineFaultPressureLoss faultLoss(redeclare package Medium=Medium);
  Modelica.Fluid.Pipes.StaticPipe nominal(
    redeclare package Medium=Medium,
    length=length,diameter=diameter,height_ab=height_ab,roughness=roughness,
    nParallel=nParallel,allowFlowReversal=allowFlowReversal);

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
  K_fault=if faultMode==FaultMode.RoughnessIncrease then faultActivation*0.1*blockageCoefficient
    elseif faultMode==FaultMode.Fouling then faultActivation*blockageCoefficient
    elseif faultMode==FaultMode.PartialBlockage then 10*faultActivation*blockageCoefficient
    elseif faultMode==FaultMode.CompleteBlockage then faultActivation*completeBlockageCoefficient else 0;
  faultLoss.K=K_fault;
  connect(port_a,faultLoss.port_a);
  connect(faultLoss.port_b,nominal.port_a);
  connect(nominal.port_b,port_b);


  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-100,44},{100,-44}}, lineColor={255,0,0}, fillColor={0,127,255}, fillPattern=FillPattern.HorizontalCylinder, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{-60,0},{60,0}}, color={255,255,255}, thickness=0.7, pattern=LinePattern.Solid),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),Documentation(info="<html><p>Retain the MSL static-pipe model and insert a loss relation <code>dp=K_fault*m_flow*abs(m_flow)</code> in series. The added element passes enthalpy, species and trace substances through in either flow direction and has zero pressure drop at zero effect.</p><p><code>RoughnessIncrease</code>, <code>Fouling</code> and <code>PartialBlockage</code> apply respectively 0.1, 1 and 10 times <code>blockageCoefficient</code>, scaled by the envelope. <code>CompleteBlockage</code> uses the finite <code>completeBlockageCoefficient</code>. These modes add pressure loss without changing the pipe geometry or roughness parameter.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Fluid.Pipes.StaticPipe</code>.</p></html>"));
end FaultableStaticPipe;
