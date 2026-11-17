within MSLFaults.Electrical.Analog.Semiconductors;
model FaultableDiode "PN diode with junction and leakage faults"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  parameter SI.Current Ids=1e-6 "Saturation current";
  parameter Boolean useTemperatureDependency=false
    "Enable temperature-dependent electrical parameters" annotation(Evaluate=true,HideResult=true,choices(checkBox=true));
  parameter SI.Voltage Vt(min=Modelica.Constants.small)=0.04
    "Nominal junction voltage scale" annotation(Dialog(enable=not useTemperatureDependency));
  parameter Real Maxexp(final min=Modelica.Constants.small)=15 "Upper argument for exponential-to-linear continuation";
  parameter SI.Resistance R=1e8 "Parallel ohmic resistance";
  parameter Real EG=1.11 "Semiconductor energy-gap parameter" annotation(Dialog(enable=useTemperatureDependency));
  parameter Real N=1 "Junction emission coefficient" annotation(Dialog(enable=useTemperatureDependency));
  parameter SI.Temperature TNOM=300.15 "Nominal junction temperature" annotation(Dialog(enable=useTemperatureDependency));
  parameter Real XTI=3 "Temperature exponent of saturation current" annotation(Dialog(enable=useTemperatureDependency));
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
    useHeatPort=useTemperatureDependency);

  type FaultMode=enumeration(
    Normal "Nominal behaviour",
    SaturationCurrentDrift "Progressive transport saturation-current drift",
    ForwardVoltageDrift "Forward-conduction voltage change",
    ReverseLeakageIncrease "Parallel reverse-leakage increase",
    OpenCircuit "Finite open-circuit equivalent",
    ShortCircuit "Finite short-circuit equivalent");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault mode";
  parameter Real severity(min=0,max=1)=1 "Multiplier applied to the fault envelope";
  parameter SI.Time faultStartTime=1 "Inclusive start of the scheduled fault window";
  parameter SI.Time faultEndTime=3 "Exclusive end of the scheduled fault window";
  parameter SI.Time transitionTime(min=0)=0.05 "Duration of a full-scale activation or recovery ramp";
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1 "Time to reach the full progressive-drift target";
  parameter SI.Current IdsFault=10*Ids "Saturation current at completed drift";
  parameter SI.Voltage VtFault=1.5*Vt "Junction voltage scale at completed drift";
  parameter SI.Resistance RLeakFault=R/100 "Parallel resistance at full leakage effect";
  parameter SI.Resistance ROpen=1e12 "Finite open-circuit resistance";
  parameter SI.Resistance RShort=1e-6 "Finite short-circuit resistance";

  SI.Voltage vt_t;
  SI.Voltage junctionVoltageScale;
  SI.Current id;
  SI.Current Ids_effective "Effective saturation current";
  SI.Voltage Vt_effective "Effective junction voltage scale";
  SI.Resistance R_effective "Effective parallel junction resistance";
  Real faultActivation(min=0,max=1) "Severity-scaled fault envelope";
  Real driftActivation(min=0,max=1) "Fault envelope multiplied by progressive drift";
  Real startActivation(min=0,max=1) "Activation envelope divided by severity";
  Real endActivation(min=0,max=1) "One while active or recovering, otherwise zero";
  Real driftProgress(min=0,max=1) "Elapsed activation time divided by driftTime, limited to one";
  Real diodeConductionFactor(min=0,max=1) "Remaining fraction of semiconductor conduction";
protected
  SI.Temperature htemp;
  Real aux;
  Real auxp;
  function exlinLocal
    input Real x;
    input Real Maxexp "Upper argument for exponential-to-linear continuation";
    output Real y;
  algorithm
    y:=if x>Maxexp then exp(Maxexp)*(1+x-Maxexp) else exp(x);
  end exlinLocal;

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

  Ids_effective=if faultMode==FaultMode.SaturationCurrentDrift then
    Ids+driftActivation*(IdsFault-Ids) else Ids;
  Vt_effective=if faultMode==FaultMode.ForwardVoltageDrift then
    Vt+driftActivation*(VtFault-Vt) else Vt;
  R_effective=if faultMode==FaultMode.ReverseLeakageIncrease then
      R+faultActivation*(RLeakFault-R)
    elseif faultMode==FaultMode.OpenCircuit then R+faultActivation*(ROpen-R)
    elseif faultMode==FaultMode.ShortCircuit then R+faultActivation*(RShort-R)
    else R;
  diodeConductionFactor=if faultMode==FaultMode.OpenCircuit then
    1-faultActivation else 1;

  assert(T_heatPort>0,"Diode: temperature must be positive");
  htemp=T_heatPort;
  vt_t=Modelica.Constants.k*htemp/Modelica.Constants.q;
  junctionVoltageScale=if useTemperatureDependency then
    N*vt_t*(Vt_effective/Vt) else Vt_effective;
  id=Ids_effective*(exlinLocal(v/junctionVoltageScale,Maxexp)-1);
  if useTemperatureDependency then
    i=diodeConductionFactor*id*(htemp/TNOM)^(XTI/N)*auxp+v/R_effective;
  else
    i=smooth(1,diodeConductionFactor*id+v/R_effective);
  end if;
  aux=(htemp/TNOM-1)*EG/(N*vt_t);
  auxp=exp(aux);
  LossPower=i*v;



  annotation(defaultComponentName="diode",
    Documentation(info="<html><p>Use an exponential junction law with linear continuation beyond Maxexp. SaturationCurrentDrift and ForwardVoltageDrift use driftTime; ReverseLeakageIncrease lowers the parallel resistance. OpenCircuit suppresses junction conduction and raises that resistance, while ShortCircuit introduces a finite low-resistance shunt.</p><p>Connect p and n; set the selected current, voltage-scale or resistance target. With temperature dependence enabled, connect heatPort and inspect LossPower.</p><p>ForwardVoltageDrift changes the junction voltage scale rather than imposing a constant forward voltage. ShortCircuit retains the semiconductor branch.</p><p>Select <code>faultMode</code> and configure the scheduled window or external trigger. <code>effectLevel</code> can remain nonzero during recovery after <code>active</code> becomes false. Nominal reference: <code>Modelica.Electrical.Analog.Semiconductors.Diode</code>.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(points={{30,0},{-30,40},{-30,-40},{30,0}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, lineThickness=0.75),
        Line(points={{-90,0},{40,0}}, color={255,0,0}, thickness=0.8),
        Line(points={{40,0},{90,0}}, color={255,0,0}, thickness=0.8),
        Line(points={{30,40},{30,-40}}, color={255,0,0}, thickness=0.8),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableDiode;
