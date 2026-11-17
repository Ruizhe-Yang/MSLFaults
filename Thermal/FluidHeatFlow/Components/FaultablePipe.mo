within MSLFaults.Thermal.FluidHeatFlow.Components;
model FaultablePipe "Simplified pipe with pressure-loss and heat-coupling faults"
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.TwoPort;
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.SimpleFriction;

  parameter Boolean useHeatPort = false "= true, if HeatPort is enabled"
    annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter Modelica.Units.SI.Length h_g(start=0)
    "Geodetic height (height difference from flowPort_a to flowPort_b)";
  parameter Modelica.Units.SI.Acceleration g(final min=0)=Modelica.Constants.g_n "Gravitation";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort(T=T_q, Q_flow=Q_flowHeatPort) if useHeatPort
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  type FaultMode=enumeration(Normal "Nominal behaviour", PressureLossIncrease "Additional signed pressure drop", Blockage "Additional pressure loss proportional to volume flow", Leakage "Reduced frictional pressure drop", HeatTransferLoss "Scaled heat transfer to the medium");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Modelica.Units.SI.Pressure pressureDropFault=1e5 "Full-effect additional pressure-drop magnitude" ;
  parameter Real heatTransferScaleFault(min=0)=0.5 "Full-effect scale of heat supplied to the medium" ;
  Modelica.Units.SI.Pressure addedPressureDrop "Fault contribution to the pressure drop" ;
  Real heatTransferScale "Scale applied to the heat-port contribution" ;
protected
  Modelica.Units.SI.HeatFlowRate Q_flowHeatPort "Heat flow at conditional heatPort";

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
  if not useHeatPort then
    Q_flowHeatPort=0;
  end if;
  // coupling with FrictionModel
  volumeFlow = V_flow;
  addedPressureDrop=if faultMode==FaultMode.PressureLossIncrease then faultActivation*pressureDropFault*sign(V_flow) elseif faultMode==FaultMode.Blockage then faultActivation*1e9*V_flow elseif faultMode==FaultMode.Leakage then -faultActivation*0.5*pressureDrop else 0;
  dp=pressureDrop+medium.rho*g*h_g+addedPressureDrop;
  // energy exchange with medium
  heatTransferScale=if faultMode==FaultMode.HeatTransferLoss then 1+faultActivation*(heatTransferScaleFault-1) else 1;
  Q_flow=heatTransferScale*Q_flowHeatPort+Q_friction;


annotation (Documentation(info="<html><p>Retain the simplified pipe friction, elevation and optional heat-port relations. <code>PressureLossIncrease</code> adds a signed pressure offset; <code>Blockage</code> adds a finite term proportional to volume flow. <code>Leakage</code> reduces the frictional pressure drop by up to one half and does not remove mass from the pipe.</p><p><code>HeatTransferLoss</code> multiplies the heat-port contribution to the medium by <code>heatTransferScale</code>, while the external heat-port flow retains its original value. Treat this as a prescribed coupling approximation when checking the overall energy balance. Connect <code>heatPort</code> only when <code>useHeatPort=true</code>.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.FluidHeatFlow.Components.Pipe</code>.</p></html>"),
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Rectangle(extent={{-90,20},{90,-20}}, lineColor={255,0,0}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Polygon(visible=useHeatPort, points={{-10,-90},{-10,-40},{0,-20},{10,-40},{10,-90},{-10,-90}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultablePipe;
