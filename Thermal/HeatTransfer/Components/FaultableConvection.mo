within MSLFaults.Thermal.HeatTransfer.Components;
model FaultableConvection
  "Convective heat transfer with conductance faults"
  Modelica.Units.SI.HeatFlowRate Q_flow "Heat flow rate from solid -> fluid";
  Modelica.Units.SI.TemperatureDifference dT "= solid.T - fluid.T";
  Modelica.Blocks.Interfaces.RealInput Gc(unit="W/K")
    "Signal representing the convective thermal conductance in [W/K]"
    annotation (Placement(transformation(
        origin={0,100},
        extent={{-20,-20},{20,20}},
        rotation=270)));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a solid annotation (Placement(transformation(extent={{
            -110,-10},{-90,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b fluid annotation (Placement(transformation(extent={{
            90,-10},{110,10}})));
  type FaultMode=enumeration(Normal "Nominal behaviour", ConductanceDrift "Change in thermal conductance", Fouling "Reduction of convective conductance", FlowLoss "Reduction of convective conductance", Blockage "Reduction of thermal conductance towards zero");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real conductanceScaleFault(min=0)=0.5 "Full-effect scale of convective conductance" ;
  Real Gc_effective(unit="W/K") "Fault-adjusted convective conductance" ;

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
  dT = solid.T - fluid.T;
  solid.Q_flow = Q_flow;
  fluid.Q_flow = -Q_flow;
  Gc_effective=if faultMode==FaultMode.ConductanceDrift or faultMode==FaultMode.Fouling or faultMode==FaultMode.FlowLoss then Gc*(1+faultActivation*(conductanceScaleFault-1)) elseif faultMode==FaultMode.Blockage then Gc*(1-faultActivation) else Gc;
  Q_flow=Gc_effective*dT;

  assert(Gc_effective>=0,"Passive heat transfer requires nonnegative conductance.");
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}),
      graphics={
        Rectangle(extent={{-62,80},{98,-80}}, lineColor={255,255,255}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.5),
        Rectangle(extent={{-90,80},{-60,-80}}, fillColor={192,192,192}, fillPattern=FillPattern.Backward, lineColor={64,64,64}, lineThickness=0.5),
        Text(extent={{-150,-90},{150,-130}}, textString="%name", textColor={0,0,255}),
        Line(points={{100,0},{100,0}}, color={0,127,255}, thickness=0.5),
        Line(points={{-60,20},{76,20}}, color={191,0,0}, thickness=0.5),
        Line(points={{-60,-20},{76,-20}}, color={191,0,0}, thickness=0.5),
        Line(points={{-34,80},{-34,-80}}, color={0,127,255}, thickness=0.5),
        Line(points={{6,80},{6,-80}}, color={0,127,255}, thickness=0.5),
        Line(points={{40,80},{40,-80}}, color={0,127,255}, thickness=0.5),
        Line(points={{76,80},{76,-80}}, color={0,127,255}, thickness=0.5),
        Line(points={{-34,-80},{-44,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{-34,-80},{-24,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{6,-80},{-4,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{6,-80},{16,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{40,-80},{30,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{40,-80},{50,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{76,-80},{66,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{76,-80},{86,-60}}, color={0,127,255}, thickness=0.5),
        Line(points={{56,-30},{76,-20}}, color={191,0,0}, thickness=0.5),
        Line(points={{56,-10},{76,-20}}, color={191,0,0}, thickness=0.5),
        Line(points={{56,10},{76,20}}, color={191,0,0}, thickness=0.5),
        Line(points={{56,30},{76,20}}, color={191,0,0}, thickness=0.5),
        Text(extent={{22,124},{92,98}}, textString="Gc"),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p>Transfer heat from <code>solid</code> to <code>fluid</code> using <code>Q_flow=Gc_effective*(solid.T-fluid.T)</code>. The input <code>Gc</code> supplies the nominal thermal conductance.</p><p><code>ConductanceDrift</code>, <code>Fouling</code> and <code>FlowLoss</code> share the same interpolation towards <code>conductanceScaleFault*Gc</code>. <code>Blockage</code> reduces conductance towards zero. The model conserves heat between its two ports and requires non-negative effective conductance.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.HeatTransfer.Components.Convection</code>.</p></html>"),
       Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={
        Line(points={{100,0},{100,0}}, color={0,127,255}),
        Line(points={{100,0},{100,0}}, color={0,127,255}),
        Line(points={{100,0},{100,0}}, color={0,127,255})}));
end FaultableConvection;
