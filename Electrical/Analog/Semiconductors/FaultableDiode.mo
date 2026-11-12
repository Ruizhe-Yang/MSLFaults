within MSLFaults.Electrical.Analog.Semiconductors;
model FaultableDiode "Fault-enhanced MSL 4.0.0 PN diode"
  import SI = Modelica.Units.SI;
  extends Modelica.Electrical.Analog.Interfaces.OnePort;
  parameter SI.Current Ids=1e-6 "Saturation current";
  parameter Boolean useTemperatureDependency=false
    annotation(Evaluate=true,HideResult=true,choices(checkBox=true));
  parameter SI.Voltage Vt(min=Modelica.Constants.small)=0.04
    annotation(Dialog(enable=not useTemperatureDependency));
  parameter Real Maxexp(final min=Modelica.Constants.small)=15;
  parameter SI.Resistance R=1e8 "Parallel ohmic resistance";
  parameter Real EG=1.11 annotation(Dialog(enable=useTemperatureDependency));
  parameter Real N=1 annotation(Dialog(enable=useTemperatureDependency));
  parameter SI.Temperature TNOM=300.15 annotation(Dialog(enable=useTemperatureDependency));
  parameter Real XTI=3 annotation(Dialog(enable=useTemperatureDependency));
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
    useHeatPort=useTemperatureDependency);

  type FaultMode=enumeration(
    Normal "Nominal MSL behavior",
    SaturationCurrentDrift "Progressive transport saturation-current drift",
    ForwardVoltageDrift "Progressive forward I-V voltage-scale drift",
    ReverseLeakageIncrease "Parallel reverse-leakage increase",
    OpenCircuit "Suppress junction conduction and retain a finite high resistance",
    ShortCircuit "Finite low-resistance shunt");
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter SI.Time faultStartTime=1;
  parameter SI.Time faultEndTime=3;
  parameter SI.Time transitionTime(min=0)=0.05;
  parameter SI.Time driftTime(min=Modelica.Constants.small)=1;
  parameter SI.Current IdsFault=10*Ids "Saturation current at completed drift";
  parameter SI.Voltage VtFault=1.5*Vt "Voltage scale at completed drift";
  parameter SI.Resistance RLeakFault=R/100 "Parallel resistance after leakage growth";
  parameter SI.Resistance ROpen=1e12 "Finite open resistance";
  parameter SI.Resistance RShort=1e-6 "Finite short resistance";

  SI.Voltage vt_t;
  SI.Voltage junctionVoltageScale;
  SI.Current id;
  SI.Current Ids_effective;
  SI.Voltage Vt_effective;
  SI.Resistance R_effective;
  Real faultActivation(min=0,max=1);
  Real driftActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  Real driftProgress(min=0,max=1);
  Real diodeConductionFactor(min=0,max=1);
protected
  SI.Temperature htemp;
  Real aux;
  Real auxp;
  function exlinLocal
    input Real x;
    input Real Maxexp;
    output Real y;
  algorithm
    y:=if x>Maxexp then exp(Maxexp)*(1+x-Maxexp) else exp(x);
  end exlinLocal;

public
  parameter Boolean enabled=true "Enable the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger instead of the internal time window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched fault";
  output Real effectLevel(min=0,max=1) "Current realized fault effect";
  output Integer activationCount "Number of rising activations";

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
    Documentation(info="<html><p><b>Purpose</b> Fault-enhanced MSL 4.0.0 PN diode</p><p><b>Interfaces</b> extends Modelica.Electrical.Analog.Interfaces.OnePort; extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(</p><p><b>Parameters</b> parameter SI.Current Ids=1e-6 \"Saturation current\"; parameter Boolean useTemperatureDependency=false; parameter SI.Voltage Vt(min=Modelica.Constants.small)=0.04; parameter Real Maxexp(final min=Modelica.Constants.small)=15; parameter SI.Resistance R=1e8 \"Parallel ohmic resistance\"; parameter Real EG=1.11; parameter Real N=1; parameter SI.Temperature TNOM=300.15; parameter Real XTI=3; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter SI.Time faultStartTime=1</p><p><b>Behavior</b> Fault-enhanced MSL 4.0.0 PN diode</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced Diode</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Semiconductors.Diode</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableDiode 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
<p>Copies the MSL 4.0.0 diode constitutive equation and exposes separate effective
parameters for saturation-current drift, forward voltage-scale drift and reverse
leakage. Open and short circuits use finite conductances without changing topology.
Normal mode and zero severity reproduce the original equations.</p>
<p><b>Evidence:</b> NASA radiation reports observe increased diode forward voltage
and reverse leakage (NTRS 19720010112, 19700022435, 20210018053). Evidence level A
for parameter direction and B/C for this lumped equation mapping.</p>
</html>"),
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
