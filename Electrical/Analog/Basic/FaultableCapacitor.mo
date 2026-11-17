within MSLFaults.Electrical.Analog.Basic;
model FaultableCapacitor
  "Capacitor with variable capacitance, leakage and series loss"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort(v(start=0));
  parameter SI.Capacitance C(start=1) "Capacitance";

  type FaultMode = enumeration(
    Normal "Nominal behaviour",
    CapacitanceDrift "Progressive bidirectional capacitance drift",
    CapacitanceLoss "Interpolate towards the capacitance-loss target",
    LeakageIncrease "Increased leakage conductance",
    ESRIncrease "Series equivalent resistance increase",
    OpenCircuit "Finite open-circuit equivalent",
    ShortCircuit "Finite short-circuit equivalent");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1
    "Time to reach the full progressive-drift target";
  parameter SI.Capacitance C_drift=1.2*C "Capacitance at completed progressive drift";
  parameter SI.Capacitance C_loss=0.5*C "Capacitance target for the loss mode";
  parameter SI.Conductance G_leakFault=1e-3 "Leakage conductance at full effect";
  parameter SI.Conductance G_short=1e6 "Finite conductance representing a short circuit";
  parameter SI.Resistance R_seriesNominal=1e-9
    "Lower bound applied to the full-effect series-resistance target";
  parameter SI.Resistance R_ESR=1 "Series resistance target for ESRIncrease";
  parameter SI.Resistance R_open=1e10 "Finite resistance representing an open circuit";

  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0,max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0,max=1) "Elapsed activation time divided by driftTime, limited to one";
  SI.Capacitance C_effective "Effective capacitance";
  SI.Conductance G_parallel "Conductance of the internal leakage or short-circuit branch";
  SI.Resistance R_series "Always-present series ESR/open branch";
  SI.Voltage v_cap(start=0) "Voltage across capacitance and leakage branch";


public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the scheduled fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    "External fault activation request" annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Fault activation request state";
  output Real effectLevel(min=0,max=1) "Severity-scaled fault envelope";
  output Integer activationCount "Number of activation starts";
  Modelica.Units.SI.ElectricCharge q "Stored charge";
  Modelica.Units.SI.Current i_leak "Current through the internal leakage branch";
  Modelica.Units.SI.Energy storedEnergy "Stored electrical or magnetic energy";
  Modelica.Units.SI.Power lossPower "Electrical power dissipated in resistive branches",parameterPower "Power associated with prescribed storage-parameter variation";
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger) "Reversible fault activation envelope";
initial equation
  assert(C>0 and C_loss>0 and C_drift>0 and driftTime>0,"Capacitances and drift time must be positive.");
  assert(G_leakFault>=0 and G_short>0 and R_seriesNominal>=0 and R_ESR>=0 and R_open>0,"Invalid capacitor loss targets.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  driftProgress=min(1,max(0,faultGate.elapsedActiveTime/driftTime));
  driftActivation=effectLevel*driftProgress;

  C_effective = if faultMode == FaultMode.CapacitanceDrift then
      C + driftActivation*(C_drift-C)
    elseif faultMode == FaultMode.CapacitanceLoss then
      C + faultActivation*(C_loss-C) else C;
  G_parallel = if faultMode == FaultMode.LeakageIncrease then
      faultActivation*G_leakFault
    elseif faultMode == FaultMode.ShortCircuit then
      faultActivation*G_short else 0;
  R_series = if faultMode == FaultMode.ESRIncrease then
      faultActivation*max(R_seriesNominal,R_ESR)
    elseif faultMode == FaultMode.OpenCircuit then
      faultActivation*max(R_seriesNominal,R_open) else 0;

  v = v_cap + R_series*i;
  q = C_effective*v_cap;
  i_leak=G_parallel*v_cap;
  der(q)=i-i_leak;


  storedEnergy=q*q/(2*C_effective);
  lossPower=R_series*i*i+G_parallel*v_cap*v_cap;
  parameterPower=der(storedEnergy)-v*i+lossPower;
  annotation (
    Documentation(info="<html><p>Evolve stored charge q with q=C_effective*v_cap and der(q)=i-i_leak. CapacitanceDrift uses driftTime, while CapacitanceLoss follows the activation envelope. Leakage and short-circuit faults add a parallel conductance; ESRIncrease and OpenCircuit add a series resistance. Both added branches vanish in Normal mode.</p><p>Set C and the selected target. Inspect v_cap, q, storedEnergy, lossPower and parameterPower when capacitance varies.</p><p>parameterPower accounts for prescribed capacitance variation; it is distinct from dissipative loss. R_seriesNominal is a lower bound on fault targets, not a nominal series resistance.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Basic.Capacitor</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-6,28},{-6,-28}}, color={255,0,0}, thickness=0.8),
        Line(points={{6,28},{6,-28}}, color={255,0,0}, thickness=0.8),
        Line(points={{-90,0},{-6,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{6,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,-40},{150,-80}}, textString="C=%C"),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableCapacitor;
