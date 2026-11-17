within MSLFaults.Electrical.Analog.Semiconductors;
model FaultableNPN "NPN transistor with gain, junction and collector-path faults"
  import SI = Modelica.Units.SI;
  parameter Real Bf=50 "Forward beta";
  parameter Real Br=0.1 "Reverse beta";
  parameter SI.Current Is=1e-16 "Transport saturation current";
  parameter SI.InversePotential Vak=0.02 "Early voltage (inverse)";
  parameter SI.Time Tauf=0.12e-9 "Ideal forward transit time";
  parameter SI.Time Taur=5e-9 "Ideal reverse transit time";
  parameter SI.Capacitance Ccs=1e-12 "Collector-substrate capacitance";
  parameter SI.Capacitance Cje=0.4e-12 "Base-emitter depletion capacitance";
  parameter SI.Capacitance Cjc=0.5e-12 "Base-collector depletion capacitance";
  parameter SI.Voltage Phie=0.8 "Base-emitter diffusion voltage";
  parameter Real Me=0.4 "Base-emitter gradation exponent";
  parameter SI.Voltage Phic=0.8 "Base-collector diffusion voltage";
  parameter Real Mc=0.333 "Base-collector gradation exponent";
  parameter SI.Conductance Gbc=1e-15 "Base-collector conductance";
  parameter SI.Conductance Gbe=1e-15 "Base-emitter conductance";
  parameter Real EMin=-100 "Lower exponential-continuation bound";
  parameter Real EMax=40 "Upper exponential-continuation bound";
  parameter Boolean useTemperatureDependency=false
    "Enable temperature-dependent electrical parameters" annotation(Evaluate=true,HideResult=true,choices(checkBox=true));
  parameter SI.Voltage Vt=0.02585 "Nominal thermal voltage when temperature dependence is disabled" annotation(Dialog(enable=not useTemperatureDependency));
  parameter SI.Temperature Tnom=300.15 "Nominal semiconductor temperature" annotation(Dialog(enable=useTemperatureDependency));
  parameter Real XTI=3 "Temperature exponent of saturation current" annotation(Dialog(enable=useTemperatureDependency));
  parameter Real XTB=0 "Temperature exponent of current gain" annotation(Dialog(enable=useTemperatureDependency));
  parameter SI.Voltage EG=1.11 "Semiconductor energy-gap parameter" annotation(Dialog(enable=useTemperatureDependency));
  parameter Real NF=1.0 "Forward junction emission coefficient";
  parameter Real NR=1.0 "Reverse junction emission coefficient";
  parameter SI.Voltage IC=0 "Initial collector-substrate voltage when UIC is true" annotation(Dialog(enable=UIC));
  parameter Boolean UIC=false "Apply IC as the initial collector-substrate voltage";
  parameter Boolean useSubstrate=false "Expose the substrate pin";
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
    useHeatPort=useTemperatureDependency);

  Modelica.Electrical.Analog.Interfaces.Pin C "Collector"
    annotation (Placement(transformation(extent={{90,50},{110,70}}),iconTransformation(extent={{90,50},{110,70}})));
  Modelica.Electrical.Analog.Interfaces.Pin B "Base"
    annotation (Placement(transformation(extent={{-90,-10},{-110,10}})));
  Modelica.Electrical.Analog.Interfaces.Pin E "Emitter"
    annotation (Placement(transformation(extent={{90,-50},{110,-70}}),iconTransformation(extent={{90,-50},{110,-70}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin S(final i=iS,final v=vS) if useSubstrate "Substrate"
    annotation (Placement(transformation(extent={{110,-10},{90,10}})));

  type FaultMode=enumeration(
    Normal "Nominal behaviour",
    CurrentGainLoss "Progressive reduction of both current gains",
    ForwardGainLoss "Progressive forward current-gain reduction",
    ReverseGainLoss "Progressive reverse current-gain reduction",
    SaturationCurrentDrift "Progressive transport saturation-current drift",
    JunctionLeakageIncrease "Base-emitter and base-collector leakage growth",
    JunctionDegradation "Depletion-capacitance degradation",
    CollectorEmitterOpen "System-level interruption of collector transport",
    CollectorEmitterShort "Finite collector-emitter shunt");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1 "Time to reach the full progressive-drift target";
  parameter Real BfFault(min=Modelica.Constants.small)=0.2*Bf "Forward current gain at completed drift";
  parameter Real BrFault(min=Modelica.Constants.small)=0.2*Br "Reverse current gain at completed drift";
  parameter SI.Current IsFault=10*Is "Transport saturation current at completed drift";
  parameter SI.Conductance GbcFault=1e3*Gbc "Base-collector conductance at full leakage effect";
  parameter SI.Conductance GbeFault=1e3*Gbe "Base-emitter conductance at full leakage effect";
  parameter SI.Capacitance CjeFault=0.5*Cje "Base-emitter depletion capacitance at full effect";
  parameter SI.Capacitance CjcFault=0.5*Cjc "Base-collector depletion capacitance at full effect";
  parameter SI.Resistance RCEShort=1e-3 "Finite collector-emitter shunt resistance";

  SI.Voltage vbc "Base-collector voltage";
  SI.Voltage vbe "Base-emitter voltage";
  SI.Voltage vcs "Collector-substrate voltage";
  Real qbk;
  SI.Current ibc;
  SI.Current ibe;
  SI.Capacitance cbc;
  SI.Capacitance cbe;
  SI.Capacitance Capcje;
  SI.Capacitance Capcjc;
  SI.Current is_t;
  Real br_t;
  Real bf_t;
  SI.Voltage vt_t;
  Real hexp;
  Real htempexp;
  SI.Voltage vS;
  SI.Current iS;
  SI.Current iCEShort "Added fixed-topology collector-emitter shunt current";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0,max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0,max=1) "Elapsed activation time divided by driftTime, limited to one";
  Real Bf_effective "Effective forward current gain";
  Real Br_effective "Effective reverse current gain";
  SI.Current Is_effective "Effective transport saturation current";
  SI.Conductance Gbc_effective "Effective base-collector conductance";
  SI.Conductance Gbe_effective "Effective base-emitter conductance";
  SI.Capacitance Cje_effective "Effective base-emitter depletion capacitance";
  SI.Capacitance Cjc_effective "Effective base-collector depletion capacitance";
  Real collectorTransportFactor(min=0,max=1) "Multiplier applied to collector transport";

protected
  function powlinLocal "MSL semiconductor depletion-capacitance continuation"
    input Real x;
    input Real y;
    output Real z;
  algorithm
    z:=if x>0 then 1+y*x else (1-x)^(-y);
  end powlinLocal;

  function exlinLocal "Exponentially evaluate and linearly continue above Maxexp"
    input Real x;
    input Real Maxexp "Upper argument for exponential-to-linear continuation";
    output Real z;
  algorithm
    z:=if x>Maxexp then exp(Maxexp)*(1+x-Maxexp) else exp(x);
  end exlinLocal;

  function exlin2Local "Linearly continue the exponential at both bounds"
    input Real x;
    input Real Minexp "Lower argument for exponential continuation";
    input Real Maxexp "Upper argument for exponential-to-linear continuation";
    output Real z;
  algorithm
    z:=if x<Minexp then exp(Minexp)*(1+x-Minexp) else exlinLocal(x,Maxexp);
  end exlin2Local;


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
    vcs=IC;
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
  driftProgress=min(1,max(0,faultGate.elapsedActiveTime/driftTime));
  driftActivation=effectLevel*driftProgress;
  Bf_effective=if faultMode==FaultMode.CurrentGainLoss or faultMode==FaultMode.ForwardGainLoss then
    Bf+driftActivation*(BfFault-Bf) else Bf;
  Br_effective=if faultMode==FaultMode.CurrentGainLoss or faultMode==FaultMode.ReverseGainLoss then
    Br+driftActivation*(BrFault-Br) else Br;
  Is_effective=if faultMode==FaultMode.SaturationCurrentDrift then Is+driftActivation*(IsFault-Is) else Is;
  Gbc_effective=if faultMode==FaultMode.JunctionLeakageIncrease then Gbc+faultActivation*(GbcFault-Gbc) else Gbc;
  Gbe_effective=if faultMode==FaultMode.JunctionLeakageIncrease then Gbe+faultActivation*(GbeFault-Gbe) else Gbe;
  Cje_effective=if faultMode==FaultMode.JunctionDegradation then Cje+faultActivation*(CjeFault-Cje) else Cje;
  Cjc_effective=if faultMode==FaultMode.JunctionDegradation then Cjc+faultActivation*(CjcFault-Cjc) else Cjc;
  collectorTransportFactor=if faultMode==FaultMode.CollectorEmitterOpen then 1-faultActivation else 1;
  iCEShort=if faultMode==FaultMode.CollectorEmitterShort then faultActivation*(C.v-E.v)/RCEShort else 0;

  assert(T_heatPort>0,"NPN: temperature must be positive");
  vbc=B.v-C.v;
  vbe=B.v-E.v;
  vcs=C.v-vS;
  qbk=1-vbc*Vak;
  hexp=(T_heatPort/Tnom-1)*EG/vt_t;
  htempexp=smooth(1,exlin2Local(hexp,EMin,EMax));
  is_t=if useTemperatureDependency then Is_effective*(T_heatPort/Tnom)^XTI*htempexp else Is_effective;
  br_t=if useTemperatureDependency then Br_effective*(T_heatPort/Tnom)^XTB else Br_effective;
  bf_t=if useTemperatureDependency then Bf_effective*(T_heatPort/Tnom)^XTB else Bf_effective;
  vt_t=if useTemperatureDependency then (Modelica.Constants.k/Modelica.Constants.q)*T_heatPort else Vt;
  ibc=smooth(1,is_t*(exlin2Local(vbc/(NR*vt_t),EMin,EMax)-1)+vbc*Gbc_effective);
  ibe=smooth(1,is_t*(exlin2Local(vbe/(NF*vt_t),EMin,EMax)-1)+vbe*Gbe_effective);
  Capcjc=smooth(1,Cjc_effective*powlinLocal(vbc/Phic,Mc));
  Capcje=smooth(1,Cje_effective*powlinLocal(vbe/Phie,Me));
  cbc=smooth(1,Taur*is_t/(NR*vt_t)*exlin2Local(vbc/(NR*vt_t),EMin,EMax)+Capcjc);
  cbe=smooth(1,Tauf*is_t/(NF*vt_t)*exlin2Local(vbe/(NF*vt_t),EMin,EMax)+Capcje);
  C.i=collectorTransportFactor*((ibe-ibc)*qbk-ibc/br_t-cbc*der(vbc))-iS+iCEShort;
  B.i=ibe/bf_t+ibc/br_t+cbc*der(vbc)+cbe*der(vbe);
  E.i=-B.i-C.i-iS;
  iS=-Ccs*der(vcs);
  if not useSubstrate then
    vS=0;
  end if;
  LossPower=collectorTransportFactor*(vbc*ibc/br_t+(ibe-ibc)*qbk*(C.v-E.v))+
    vbe*ibe/bf_t+iCEShort*(C.v-E.v);



  annotation(defaultComponentName="npn",
    Documentation(info="<html><p>Retain the bipolar junction, charge-storage and optional temperature/substrate relations. CurrentGainLoss modifies both current gains; ForwardGainLoss, ReverseGainLoss and SaturationCurrentDrift use the separate drift clock. Other modes change junction conductances or depletion capacitances, attenuate collector transport, or add a finite collector-emitter shunt.</p><p>Connect B, C and E and, when enabled, S and heatPort. Configure the selected gain, junction or RCEShort target and inspect terminal currents and LossPower.</p><p>CollectorEmitterOpen attenuates collector transport; base-junction and substrate relations remain. It does not disconnect every terminal current.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Semiconductors.NPN</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-20,40},{-20,-40}}, color={255,0,0}, thickness=0.8),
        Line(points={{-20,0},{-100,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{91,60},{30,60}}, color={0,0,255}, thickness=0.5),
        Line(points={{30,60},{-20,10}}, color={255,0,0}, thickness=0.8),
        Line(points={{-20,-10},{30,-60}}, color={255,0,0}, thickness=0.8),
        Line(points={{30,-60},{91,-60}}, color={0,0,255}, thickness=0.5),
        Polygon(points={{30,-60},{24,-46},{16,-54},{30,-60}}, fillColor={255,0,0}, fillPattern=FillPattern.Solid, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Line(points={{0,0},{90,0}}, color={0,0,255}, pattern=LinePattern.Dash, visible=useSubstrate, thickness=0.5),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableNPN;
