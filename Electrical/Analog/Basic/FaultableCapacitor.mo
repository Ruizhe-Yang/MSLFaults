within MSLFaults.Electrical.Analog.Basic;
model FaultableCapacitor
  "Fault-enhanced MSL 4.0.0 capacitor with ESR and leakage branches"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort(v(start=0));
  parameter SI.Capacitance C(start=1) "Capacitance";

  type FaultMode = enumeration(
    Normal "Nominal MSL behavior",
    CapacitanceDrift "Progressive bidirectional capacitance drift",
    CapacitanceLoss "Abrupt or ramped capacitance loss",
    LeakageIncrease "Parallel leakage conductance increase",
    ESRIncrease "Series equivalent resistance increase",
    OpenCircuit "Finite series-resistance open circuit",
    ShortCircuit "Finite parallel-conductance short circuit");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter SI.Time faultStartTime=1;
  parameter SI.Time faultEndTime=3;
  parameter SI.Time transitionTime(min=0)=0.05;
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1
    "Time from drift onset to its calibrated target";
  parameter SI.Capacitance C_drift=1.2*C "Capacitance at completed drift";
  parameter SI.Capacitance C_loss=0.5*C "Capacitance after capacity loss";
  parameter SI.Conductance G_leakFault=1e-3 "Leakage target conductance";
  parameter SI.Conductance G_short=1e6 "Finite short-circuit conductance";
  parameter SI.Resistance R_seriesNominal=1e-9
    "Lower bound for the full-fault series resistance target";
  parameter SI.Resistance R_ESR=1 "Target series ESR";
  parameter SI.Resistance R_open=1e10 "Finite open-circuit series resistance";

  Real faultActivation(min=0,max=1);
  Real driftActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  Real driftProgress(min=0,max=1);
  SI.Capacitance C_effective "Fault-adjusted capacitance";
  SI.Conductance G_parallel "Always-present parallel leakage/short branch";
  SI.Resistance R_series "Always-present series ESR/open branch";
  SI.Voltage v_cap(start=0) "Voltage across capacitance and leakage branch";


public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";
  Modelica.Units.SI.ElectricCharge q "Stored charge";
  Modelica.Units.SI.Current i_leak;
  Modelica.Units.SI.Energy storedEnergy;
  Modelica.Units.SI.Power lossPower,parameterPower;
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=false,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
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
    Documentation(info="<html><p><b>Purpose</b> FaultableCapacitor</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort(v(start=0)); Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter SI.Capacitance C(start=1) \"Capacitance\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter SI.Time faultStartTime=1; parameter SI.Time faultEndTime=3; parameter SI.Time transitionTime(min=0)=0.05; parameter SI.Time driftTime(min=Modelica.Constants.small)=1; parameter SI.Capacitance C_drift=1.2*C \"Capacitance at completed drift\"; parameter SI.Capacitance C_loss=0.5*C \"Capacitance after capacity loss\"; parameter SI.Conductance G_leakFault=1e-3 \"Leakage target conductance\"; parameter SI.Conductance G_short=1e6 \"Finite short-circuit conductance\"; parameter SI.Resistance R_seriesNominal=1e-9</p><p><b>Behavior</b> FaultableCapacitor</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.CapacitorDegradation\">MSLFaults.Examples.CapacitorDegradation</a></p><p><b>Purpose</b> Fault-enhanced Capacitor</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> Charge is the storage coordinate. The original terminal voltage retains its initialization attributes. Apply a finite ramp for a resolvable in-run parameter-power trace.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Basic.Capacitor</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.CapacitorDegradation\">MSLFaults.Examples.CapacitorDegradation</a></p><p>用法：将 FaultableCapacitor 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>Normal mode and <code>severity=0</code> reduce algebraically to the MSL 4.0.0
equation <code>i=C*der(v)</code>. The series and parallel branches are always in
the equation set, with exactly zero resistance/conductance when inactive.</p>
<p>Capacity loss, ESR increase and leakage increase are separately observable and
must not share one effective-parameter law. Progressive drift additionally uses
<code>driftTime</code>.</p>
<p><b>Evidence:</b> NASA accelerated-aging studies report capacitance loss
(NTRS 20120013444), ESR degradation (NTRS 20170003492), and leakage-current
growth associated with dielectric defects/oxygen-vacancy migration
(NTRS 20110015253, 20160001192). Evidence level A for directions, B/C for this
lumped fixed-topology mapping.</p>
</html>"),
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
