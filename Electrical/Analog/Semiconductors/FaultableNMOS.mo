within MSLFaults.Electrical.Analog.Semiconductors;
model FaultableNMOS "N-channel MOS transistor with channel and leakage faults"
  import SI = Modelica.Units.SI;
  Modelica.Electrical.Analog.Interfaces.Pin D "Drain"
    annotation (Placement(transformation(extent={{90,50},{110,70}}), iconTransformation(extent={{90,50},{110,70}})));
  Modelica.Electrical.Analog.Interfaces.Pin G "Gate"
    annotation (Placement(transformation(extent={{-90,-50},{-110,-70}}), iconTransformation(extent={{-90,-50},{-110,-70}})));
  Modelica.Electrical.Analog.Interfaces.Pin S "Source"
    annotation (Placement(transformation(extent={{90,-50},{110,-70}}), iconTransformation(extent={{90,-50},{110,-70}})));
  Modelica.Electrical.Analog.Interfaces.Pin B "Bulk"
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  parameter SI.Length W=20e-6 "Nominal channel width";
  parameter SI.Length L=6e-6 "Nominal channel length";
  parameter SI.Transconductance Beta=0.041e-3 "Nominal channel transconductance parameter";
  parameter SI.Voltage Vt=0.8 "Nominal threshold voltage";
  parameter Real K2=1.144 "Bulk-voltage threshold coefficient";
  parameter Real K5=0.7311 "Pinch-off correction factor";
  parameter SI.Length dW=-2.5e-6 "Channel-width correction";
  parameter SI.Length dL=-1.5e-6 "Channel-length correction";
  parameter SI.Resistance RDS=1e7 "Nominal drain-source shunt resistance";
  parameter Boolean useTemperatureDependency=false
    "Enable temperature-dependent electrical parameters" annotation(Evaluate=true,HideResult=true,choices(checkBox=true));
  parameter SI.Temperature Tnom=300.15 "Nominal semiconductor temperature" annotation(Dialog(enable=useTemperatureDependency));
  parameter Real kvt=-6.96e-3 "Temperature coefficient of threshold voltage" annotation(Dialog(enable=useTemperatureDependency));
  parameter Real kk2=6e-4 "Temperature coefficient of the bulk factor" annotation(Dialog(enable=useTemperatureDependency));
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
    useHeatPort=useTemperatureDependency);

  type FaultMode=enumeration(
    Normal "Nominal behaviour",
    ThresholdVoltageDrift "Progressive threshold-voltage shift",
    TransconductanceLoss "Progressive channel transconductance loss",
    DrainSourceLeakage "Off-state drain-source leakage increase",
    OnResistanceIncrease "On-state channel conduction degradation",
    OpenCircuit "Finite open-circuit equivalent",
    ShortCircuit "Finite short-circuit equivalent",
    StuckOn "Gate-independent forced channel conduction",
    StuckOff "Gate/channel conduction suppressed with nominal leakage retained");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1 "Time to reach the full progressive-drift target";
  parameter SI.Voltage VtFault=Vt+0.5 "Threshold voltage at completed drift";
  parameter SI.Transconductance BetaFault=0.5*Beta
    "Transconductance parameter at completed drift";
  parameter SI.Resistance RDSLeakFault=1e4 "Drain-source resistance at full leakage effect";
  parameter SI.Resistance ROpen=1e12 "Finite open-circuit resistance";
  parameter SI.Resistance RShort=1e-3 "Finite short-circuit resistance";
  parameter Real onResistanceFactor(min=1)=5
    "Channel-resistance multiplier at full effect";
  parameter SI.Voltage stuckOnOverdrive=5
    "Prescribed channel overdrive at full StuckOn effect";

  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0,max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0,max=1) "Elapsed activation time divided by driftTime, limited to one";
  SI.Transconductance Beta_effective "Effective channel transconductance parameter";
  SI.Voltage Vt_effective "Effective threshold voltage";
  SI.Resistance RDS_effective "Effective drain-source shunt resistance";
protected
  Real v;
  Real uds;
  Real ubs;
  Real ugst;
  Real ugst_nominal;
  Real ud;
  Real us;
  Real id;
  Real id_channel;
  Real gds;
  Real beta_t;
  Real vt_t;
  Real k2_t;
  Real channelFactor(min=0,max=1) "Multiplier applied to channel conduction";

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
  Beta_effective=if faultMode==FaultMode.TransconductanceLoss then Beta+driftActivation*(BetaFault-Beta) else Beta;
  Vt_effective=if faultMode==FaultMode.ThresholdVoltageDrift then Vt+driftActivation*(VtFault-Vt) else Vt;
  RDS_effective=if faultMode==FaultMode.DrainSourceLeakage then RDS+faultActivation*(RDSLeakFault-RDS)
    elseif faultMode==FaultMode.OpenCircuit then RDS+faultActivation*(ROpen-RDS)
    elseif faultMode==FaultMode.ShortCircuit then RDS+faultActivation*(RShort-RDS) else RDS;
  channelFactor=if faultMode==FaultMode.OpenCircuit or faultMode==FaultMode.StuckOff then
      1-faultActivation
    elseif faultMode==FaultMode.OnResistanceIncrease then
      1/(1+faultActivation*(onResistanceFactor-1)) else 1;

  assert(L+dL>0,"NMOS: effective length must be positive");
  assert(W+dW>0,"NMOS: effective width must be positive");
  assert(T_heatPort>0,"NMOS: temperature must be positive");
  gds=if (RDS_effective<1e-20 and RDS_effective>-1e-20) then 1e20 else 1/RDS_effective;
  v=beta_t*(W+dW)/(L+dL);
  ud=smooth(0,if D.v<S.v then S.v else D.v);
  us=smooth(0,if D.v<S.v then D.v else S.v);
  uds=ud-us;
  ubs=smooth(0,if B.v>us then 0 else B.v-us);
  ugst_nominal=(G.v-us-vt_t+k2_t*ubs)*K5;
  ugst=if faultMode==FaultMode.StuckOn then
    ugst_nominal+faultActivation*(stuckOnOverdrive-ugst_nominal) else ugst_nominal;
  id_channel=smooth(0,if ugst<=0 then 0 elseif ugst>uds then
    v*uds*(ugst-uds/2) else v*ugst*ugst/2);
  id=channelFactor*id_channel+uds*gds;
  beta_t=if useTemperatureDependency then Beta_effective*(T_heatPort/Tnom)^(-1.5) else Beta_effective;
  vt_t=if useTemperatureDependency then Vt_effective*(1+(T_heatPort-Tnom)*kvt) else Vt_effective;
  k2_t=if useTemperatureDependency then K2*(1+(T_heatPort-Tnom)*kk2) else K2;
  G.i=0;
  D.i=smooth(0,if D.v<S.v then -id else id);
  S.i=smooth(0,if D.v<S.v then id else -id);
  B.i=0;
  LossPower=D.i*(D.v-S.v);



  annotation(defaultComponentName="nMOS",
    Documentation(info="<html><p>Modify threshold voltage or transconductance progressively using driftTime. DrainSourceLeakage and ShortCircuit change the parallel drain-source resistance. OpenCircuit suppresses channel current and raises that resistance; StuckOff suppresses the channel while retaining nominal leakage. StuckOn interpolates the effective overdrive towards stuckOnOverdrive. OnResistanceIncrease reduces channel conduction.</p><p>Connect D, G, S and B, retain valid effective channel dimensions, and set the relevant target. Use the sign of stuckOnOverdrive appropriate to the channel polarity.</p><p>StuckOn is an overdrive model, not a forced constant resistance. ShortCircuit retains the channel law; gate and bulk currents remain zero.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Semiconductors.NMOS</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-90,-60},{-10,-60}}, color={0,0,255}, thickness=0.5),
        Line(points={{-10,-60},{-10,60}}, color={255,0,0}, thickness=0.8),
        Line(points={{10,80},{10,39}}, color={255,0,0}, thickness=0.8),
        Line(points={{10,20},{10,-21}}, color={255,0,0}, thickness=0.8),
        Line(points={{10,-40},{10,-81}}, color={255,0,0}, thickness=0.8),
        Line(points={{10,60},{91,60}}, color={0,0,255}, thickness=0.5),
        Line(points={{10,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{10,-60},{90,-60}}, color={0,0,255}, thickness=0.5),
        Polygon(points={{40,0},{60,5},{60,-5},{40,0}}, fillColor={255,0,0}, fillPattern=FillPattern.Solid, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableNMOS;
