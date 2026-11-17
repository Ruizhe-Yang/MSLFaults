within MSLFaults.Electrical.Analog.Basic;
model FaultableVariableCapacitor
  "Variable capacitance with input faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  Modelica.Blocks.Interfaces.RealInput C(unit="F") "Commanded capacitance" annotation (Placement(
        transformation(
        origin={0,120},
        extent={{-20,-20},{20,20}},
        rotation=270)));
  parameter SI.Capacitance Cmin=Modelica.Constants.eps
    "Lower bound for variable capacitance";
  SI.ElectricCharge Q;
  parameter SI.Voltage IC=0 "Initial voltage when UIC is true";
  parameter Boolean UIC=false "Apply IC as the initial voltage";
  type FaultMode = enumeration(Normal "Nominal behaviour", CapacitanceScaleError "Scale the commanded capacitance", CapacitanceBias "Bias the commanded capacitance", StuckInput "Prescribed input value", CapacitanceLoss "Interpolate towards the capacitance-loss target");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  parameter Real scaleFault=0.5 "Input scale factor at full effect";
  parameter SI.Capacitance biasFault=1e-6 "Additive offset at full effect in the input or output unit";
  parameter SI.Capacitance stuckValue=1e-6 "Prescribed value at full effect";
  parameter SI.Capacitance lossValue=Cmin "Parameter value at full loss effect";
  SI.Capacitance C_effective "Effective capacitance";

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
    v = IC;
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
  assert(C >= 0, "Capacitance C (= " + String(C) + ") has to be >= 0!");
  // protect solver from index change
  C_effective = if faultMode == FaultMode.CapacitanceScaleError then C*(1 + faultActivation*(scaleFault - 1))
    elseif faultMode == FaultMode.CapacitanceBias then C + faultActivation*biasFault
    elseif faultMode == FaultMode.StuckInput then C + faultActivation*(stuckValue - C)
    elseif faultMode == FaultMode.CapacitanceLoss then C + faultActivation*(lossValue - C) else C;
  Q = noEvent(max(C_effective, Cmin))*v;
  i = der(Q);


  annotation (defaultComponentName="capacitor",
    Documentation(info="<html><p>Transform input C by scale, bias, a prescribed stuck value or a loss target. Limit the effective capacitance below by Cmin when calculating charge Q; use i=der(Q) for the port relation. With UIC=true, IC sets the initial voltage.</p><p>Connect input C, set scaleFault, biasFault, stuckValue or lossValue, and check C_effective together with the lower bound Cmin.</p><p>The declared input C must be nonnegative. StuckInput uses stuckValue and does not capture the input at activation.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.VariableCapacitor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-90,0},{-6,0}}, color={255,0,0}, thickness=0.8),
        Line(points={{6,0},{90,0}}, color={255,0,0}, thickness=0.8),
        Line(points={{-6,28},{-6,-28}}, color={0,0,255}, thickness=0.5),
        Line(points={{6,28},{6,-28}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,185},{150,155}}, textString="%name", textColor={0,0,255}),
        Line(points={{-45,-42},{44,45}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid, arrow={Arrow.None,Arrow.Filled}),
        Text(extent={{-32,-55},{32,-84}}, textString="C", textColor={64,64,64}, fontSize=0),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableVariableCapacitor;
