within MSLFaults.Thermal.FluidHeatFlow.Components;
model FaultableOneWayValve "One-way valve with branch and leakage faults"
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.TwoPort(m(start=0), final tapT=1);

  parameter Modelica.Units.SI.VolumeFlowRate V_flowNominal(start=1) "Nominal volume flow rate (forward)";
  parameter Modelica.Units.SI.Pressure dpForward(displayUnit="bar")=1e-6 "Pressure drop at nominal flow (forward)";
  parameter Modelica.Units.SI.Pressure dpNominal(displayUnit="bar", start=1e5) "Nominal pressure (backward)";
  parameter Modelica.Units.SI.VolumeFlowRate V_flowBackward(start=1E-6) "Leakage volume flow rate (backward)";
  parameter Real frictionLoss(min=0, max=1, start=0)
    "Part of friction losses fed to medium";
  Boolean backward(start=true) "State forward=false / backward=true";
protected
  Real s(start=0, final unit="1")
    "Auxiliary variable for actual position on the valve characteristic";
  /* s < 0: backward, leakage flow
         s > 0: forward, small pressure drop */
  constant Modelica.Units.SI.VolumeFlowRate unitVolumeFlowRate = 1;
public
  constant Modelica.Units.SI.Pressure unitPressureDrop = 1;  type FaultMode=enumeration(Normal "Nominal behaviour", StuckClosed "Force the reverse characteristic branch", StuckOpen "Force the forward characteristic branch", ForwardResistanceIncrease "Increase in forward pressure-drop coefficient", ReverseLeakageIncrease "Increase in reverse-flow coefficient");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Modelica.Units.SI.Pressure dpForwardFault=10*dpForward "Full-effect forward pressure-drop target" ;
  parameter Modelica.Units.SI.VolumeFlowRate reverseLeakFault=100*V_flowBackward "Full-effect reverse-leakage target" ;
  Modelica.Units.SI.Pressure dpForward_effective "Fault-adjusted forward pressure drop" ;
  Modelica.Units.SI.VolumeFlowRate V_flowBackward_effective "Fault-adjusted reverse leakage" ;
  Boolean backward_effective "Selected branch of the valve characteristic" ;

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
  backward_effective=if faultMode==FaultMode.StuckClosed and faultActivation>0.5 then true elseif faultMode==FaultMode.StuckOpen and faultActivation>0.5 then false else s<0;
  backward=backward_effective;
  dpForward_effective=if faultMode==FaultMode.ForwardResistanceIncrease or faultMode==FaultMode.StuckClosed then dpForward+faultActivation*(dpForwardFault-dpForward) else dpForward;
  V_flowBackward_effective=if faultMode==FaultMode.ReverseLeakageIncrease or faultMode==FaultMode.StuckOpen then V_flowBackward+faultActivation*(reverseLeakFault-V_flowBackward) else V_flowBackward;
  dp=(s*unitVolumeFlowRate)*(if backward_effective then 1 else dpForward_effective/V_flowNominal);
  V_flow=(s*unitPressureDrop)*(if backward_effective then V_flowBackward_effective/dpNominal else 1);
  Q_flow = frictionLoss*V_flow*dp;


annotation (Documentation(info="<html><p>Use the simplified MSL one-way-valve characteristic with finite forward pressure drop and reverse leakage. <code>frictionLoss</code> selects the fraction of hydraulic dissipation supplied to the medium.</p><p><code>ForwardResistanceIncrease</code> changes <code>dpForward</code> towards <code>dpForwardFault</code>; <code>ReverseLeakageIncrease</code> changes the reverse-flow target. <code>StuckClosed</code> and <code>StuckOpen</code> additionally force the reverse or forward characteristic branch when <code>effectLevel &gt; 0.5</code>. Their finite branch relations do not impose an ideal closed or lossless path.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.FluidHeatFlow.Components.OneWayValve</code>.</p></html>"),
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Polygon(points={{-90,10},{-60,10},{-60,60},{0,0},{60,60},{60,10},{90,10},{90,-10},{60,-10},{60,-60},{0,0},{-60,-60},{-60,-10},{-90,-10},{-90,10}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Text(extent={{-150,-70},{150,-110}}, textString="%name", textColor={0,0,255}),
        Line(points={{-60,60},{60,-60},{50,-40},{40,-50},{60,-60}}, thickness=0.5, color={64,64,64}),
        Polygon(points={{50,-40},{60,-60},{40,-50},{50,-40}}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableOneWayValve;
