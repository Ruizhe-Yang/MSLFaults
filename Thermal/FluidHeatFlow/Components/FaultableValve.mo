within MSLFaults.Thermal.FluidHeatFlow.Components;
model FaultableValve "Simplified valve with opening and leakage faults"
  extends Modelica.Thermal.FluidHeatFlow.BaseClasses.TwoPort(m(start=0), final tapT=1);

  parameter Boolean LinearCharacteristic(start=true)
    "Type of characteristic"
    annotation(Dialog(group="Standard characteristic"), choices(choice=true "Linear", choice=false
        "Exponential"));
  parameter Real y1(min=small, start=1) "Max. valve opening"
    annotation(Dialog(group="Standard characteristic"));
  parameter Modelica.Units.SI.VolumeFlowRate Kv1(min=small, start=1)
    "Max. flow @ y = y1"
    annotation(Dialog(group="Standard characteristic"));
  parameter Real kv0(min=small,max=1-small, start=0.01)
    "Leakage flow / max.flow @ y = 0"
    annotation(Dialog(group="Standard characteristic"));
  parameter Modelica.Units.SI.Pressure dp0(start=1) "Standard pressure drop"
    annotation(Dialog(group="Standard characteristic"));
  parameter Modelica.Units.SI.Density rho0(start=10)
    "Standard medium's density"
    annotation(Dialog(group="Standard characteristic"));
  parameter Real frictionLoss(min=0, max=1, start=0)
    "Part of friction losses fed to medium";
  type FaultMode=enumeration(Normal "Nominal behaviour", StuckClosed "Blend towards the closed-command target", StuckOpen "Blend towards the open-command target", OpeningBias "Additive opening-command bias", LeakageIncrease "Increase in the closed-flow fraction", Blockage "Reduction of the opening command");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mechanism" ;
  parameter Real severity(min=0,max=1)=1 "Maximum realised fault level" ;
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault window" ;
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal fault window" ;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp" ;
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope" ;
  Real startActivation(min=0,max=1) "Fault envelope normalised by severity" ;
  Real endActivation(min=0,max=1) "Indicator that activation or recovery is present" ;
  parameter Real stuckOpening(min=0)=0 "Prescribed opening target for the closed-stuck mode" ;
  parameter Real openingBias=0.1 "Full-effect additive opening bias" ;
  parameter Real leakageFault(min=Modelica.Constants.small,max=1)=0.1 "Full-effect closed-flow fraction" ;
  Real y_effective "Fault-adjusted opening command" ;
  Real kv0_effective "Fault-adjusted closed-flow fraction" ;
protected
  constant Modelica.Units.SI.VolumeFlowRate unitVolumeFlowRate = 1;
  constant Real small = Modelica.Constants.small;
  constant Modelica.Units.SI.VolumeFlowRate smallVolumeFlowRate = eps*unitVolumeFlowRate;
  constant Real eps = Modelica.Constants.eps;
  Real yLim "Limited fault-affected valve opening";
  Modelica.Units.SI.VolumeFlowRate Kv "Standard flow rate";
public
  Modelica.Blocks.Interfaces.RealInput y annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,100})));

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
initial algorithm
  assert(y1>small, "Valve characteristic: y1 has to be > 0 !");
  assert(Kv1>smallVolumeFlowRate, "Valve characteristic: Kv1 has to be > 0 !");
  assert(kv0>small, "Valve characteristic: kv0 has to be > 0 !");
  assert(kv0<1-eps, "Valve characteristic: kv0 has to be < 1 !");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  y_effective=if faultMode==FaultMode.StuckClosed then y+faultActivation*(stuckOpening-y) elseif faultMode==FaultMode.StuckOpen then y+faultActivation*(y1-y) elseif faultMode==FaultMode.OpeningBias then y+faultActivation*openingBias elseif faultMode==FaultMode.Blockage then y*(1-faultActivation) else y;
  kv0_effective=if faultMode==FaultMode.LeakageIncrease then kv0+faultActivation*(leakageFault-kv0) else kv0;
  yLim=max(min(y_effective,y1),0);
  // evaluate standard characteristic
  Kv/Kv1=if LinearCharacteristic then (kv0_effective+(1-kv0_effective)*yLim/y1) else kv0_effective*exp(Modelica.Math.log(1/kv0_effective)*yLim/y1);
  // pressure drop under real conditions
  dp/dp0 = medium.rho/rho0*(V_flow/Kv)*abs(V_flow/Kv);
  // Fraction of hydraulic dissipation supplied to the medium
  Q_flow = frictionLoss*V_flow*dp;


annotation (Documentation(info="<html><p>Use the simplified linear or exponential valve characteristic with a non-zero closed-flow fraction <code>kv0</code>. The pressure drop depends on the signed quadratic volume-flow relation and medium density.</p><p>Opening faults blend towards <code>stuckOpening</code>, <code>y1</code> or a biased command; <code>Blockage</code> reduces the command towards zero. <code>LeakageIncrease</code> changes <code>kv0</code> towards <code>leakageFault</code>. The effective command is limited to <code>[0,y1]</code>, and the closed characteristic retains its leakage fraction. <code>frictionLoss</code> specifies the dissipated hydraulic power supplied to the medium.</p><p>The selected fault follows the reversible activation envelope. Read <code>effectLevel</code> during recovery as well as <code>active</code>. Nominal counterpart: <code>Modelica.Thermal.FluidHeatFlow.Components.Valve</code>.</p></html>"),
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Polygon(points={{-90,10},{-60,10},{-60,60},{0,0},{60,60},{60,10},{90,10},{90,-10},{60,-10},{60,-60},{0,0},{-60,-60},{-60,-10},{-90,-10},{-90,10}}, lineColor={255,0,0}, fillColor={0,0,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{0,80},{0,0}}, color={0,0,127}, thickness=0.5),
        Text(extent={{-150,-70},{150,-110}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableValve;
