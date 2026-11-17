within MSLFaults.Electrical.Analog.Basic;
model FaultableVariableInductor
  "Variable inductance with input faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  Modelica.Blocks.Interfaces.RealInput L(unit="H") "Commanded inductance" annotation (Placement(
        transformation(
        origin={0,120},
        extent={{-20,-20},{20,20}},
        rotation=270), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,120})));
  SI.MagneticFlux Psi;
  parameter SI.Inductance Lmin=Modelica.Constants.eps
    "Lower bound for variable inductance";
  parameter SI.Current IC=0 "Initial current when UIC is true";
  parameter Boolean UIC=false "Apply IC as the initial current";
  type FaultMode = enumeration(Normal "Nominal behaviour", InductanceScaleError "Scale the commanded inductance", InductanceBias "Bias the commanded inductance", StuckInput "Prescribed input value", InductanceLoss "Interpolate towards the inductance-loss target");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Real scaleFault=0.5 "Input scale factor at full effect";
  parameter SI.Inductance biasFault=1e-3 "Additive offset at full effect in the input or output unit";
  parameter SI.Inductance stuckValue=1e-3 "Prescribed value at full effect";
  parameter SI.Inductance lossValue=Lmin "Parameter value at full loss effect";
  SI.Inductance L_effective "Effective inductance";

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the scheduled fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    "External fault activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Fault activation request state";
  output Real effectLevel(min=0,max=1) "Severity-scaled fault envelope";
  output Integer activationCount "Number of activation starts";

protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger) "Reversible fault activation envelope";
initial equation
  if UIC then
    i = IC;
  end if;
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  assert(L >= 0, "Inductance L_ (= " + String(L) + ") has to be >= 0!");
  // protect solver from index change
  L_effective = if faultMode == FaultMode.InductanceScaleError then L*(1 + faultActivation*(scaleFault - 1))
    elseif faultMode == FaultMode.InductanceBias then L + faultActivation*biasFault
    elseif faultMode == FaultMode.StuckInput then L + faultActivation*(stuckValue - L)
    elseif faultMode == FaultMode.InductanceLoss then L + faultActivation*(lossValue - L) else L;
  Psi = noEvent(max(L_effective, Lmin))*i;
  v = der(Psi);


  annotation (defaultComponentName="inductor",
    Documentation(info="<html><p>Transform input L by scale, bias, a prescribed stuck value or a loss target. Limit the effective inductance below by Lmin when calculating flux linkage Psi; use v=der(Psi) for the port relation. With UIC=true, IC sets the initial current.</p><p>Connect input L, set scaleFault, biasFault, stuckValue or lossValue, and check L_effective together with the lower bound Lmin.</p><p>The declared input L must be nonnegative. StuckInput uses stuckValue and does not capture the input at activation.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.VariableInductor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-90,0},{-60,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,185},{150,155}}, textString="%name", textColor={0,0,255}),
        Line(points={{-60,0},{-59,6},{-52,14},{-38,14},{-31,6},{-30,0}}, color={0,0,255}, smooth=Smooth.Bezier, thickness=0.5),
        Line(points={{-30,0},{-29,6},{-22,14},{-8,14},{-1,6},{0,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{0,0},{1,6},{8,14},{22,14},{29,6},{30,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{30,0},{31,6},{38,14},{52,14},{59,6},{60,0}}, color={255,0,0}, smooth=Smooth.Bezier, thickness=0.8),
        Line(points={{-45,-42},{44,45}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-32,-55},{32,-84}}, textString="L", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVariableInductor;
