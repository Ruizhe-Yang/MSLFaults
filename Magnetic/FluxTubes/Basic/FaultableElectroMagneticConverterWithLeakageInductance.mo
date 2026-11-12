within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableElectroMagneticConverterWithLeakageInductance "Fault-enhanced converter with leakage inductance"
  Modelica.Magnetic.FluxTubes.Interfaces.PositiveMagneticPort port_p "Positive magnetic port" annotation(Placement(transformation(extent={{90,90},{110,110}}),iconTransformation(extent={{90,90},{110,110}})));
  Modelica.Magnetic.FluxTubes.Interfaces.NegativeMagneticPort port_n "Negative magnetic port" annotation(Placement(transformation(extent={{110,-110},{90,-90}}),iconTransformation(extent={{110,-110},{90,-90}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin" annotation(Placement(transformation(extent={{-90,90},{-110,110}}),iconTransformation(extent={{-90,90},{-110,110}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative electrical pin" annotation(Placement(transformation(extent={{-110,-108},{-90,-88}}),iconTransformation(extent={{-110,-108},{-90,-88}})));
  Modelica.Units.SI.Voltage v;
  Modelica.Units.SI.Current i(start=0,stateSelect=StateSelect.prefer);
  Modelica.Units.SI.MagneticPotentialDifference V_m;
  Modelica.Units.SI.MagneticFlux Phi(stateSelect=StateSelect.never);
  Modelica.Units.SI.MagneticFlux Phi_ind(stateSelect=StateSelect.never);
  Modelica.Units.SI.MagneticFlux Phi_leak(stateSelect=StateSelect.never);
  parameter Real N(start=1,min=Modelica.Constants.eps)=1 "Number of turns";
  parameter Modelica.Units.SI.Length L=10e-3;
  parameter Modelica.Units.SI.Area A=10e-6;
  parameter Modelica.Units.SI.RelativePermeability mu_rel(min=Modelica.Constants.eps)=1;
  final parameter Modelica.Units.SI.Permeance G_m=Modelica.Constants.mu_0*mu_rel*A/L;
  Modelica.Units.SI.MagneticFlux Psi;
  Modelica.Units.SI.Inductance L_stat;
  type FaultMode=enumeration(Normal, TurnLoss, WindingOpen, ShortedTurns, LeakageIncrease, LeakageDecrease);
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real NFault(min=Modelica.Constants.eps)=0.5*N;
  parameter Real leakageFaultFactor(min=Modelica.Constants.eps)=2;
  parameter Modelica.Units.SI.Resistance ROpen=1e9;
  Real faultActivation(min=0,max=1),startActivation(min=0,max=1),endActivation(min=0,max=1),N_effective(min=Modelica.Constants.eps);
  Modelica.Units.SI.Permeance G_m_effective;
  Modelica.Units.SI.Resistance R_series_effective;
protected
  constant Real eps=100*Modelica.Constants.eps;

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
    permanent=faultMode==FaultMode.WindingOpen,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
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
  N_effective=if faultMode==FaultMode.TurnLoss or faultMode==FaultMode.ShortedTurns then N+faultActivation*(NFault-N) else N;
  G_m_effective=G_m*(if faultMode==FaultMode.LeakageIncrease then 1+faultActivation*(leakageFaultFactor-1) elseif faultMode==FaultMode.LeakageDecrease then 1-faultActivation*(1-1/leakageFaultFactor) else 1);
  R_series_effective=if faultMode==FaultMode.WindingOpen then faultActivation*ROpen else 0;
  v=p.v-n.v;
  0=p.i+n.i;
  i=p.i;
  V_m=port_p.V_m-port_n.V_m;
  0=port_p.Phi+port_n.Phi;
  Phi=port_p.Phi;
  V_m=i*N_effective;
  N_effective*der(Phi_ind)=-(v-i*R_series_effective);
  Phi_leak=G_m_effective*V_m;
  Phi=Phi_ind+Phi_leak;
  Psi=N_effective*Phi_ind;
  L_stat=noEvent(if abs(i)>eps then abs(Psi/i) else abs(Psi/eps));


  annotation(
    Documentation(info="<html><p><b>Purpose</b> Fault-enhanced converter with leakage inductance</p><p><b>Interfaces</b> Modelica.Magnetic.FluxTubes.Interfaces.PositiveMagneticPort port_p \"Positive magnetic port\"; Modelica.Magnetic.FluxTubes.Interfaces.NegativeMagneticPort port_n \"Negative magnetic port\"; Modelica.Electrical.Analog.Interfaces.PositivePin p \"Positive electrical pin\"; Modelica.Electrical.Analog.Interfaces.NegativePin n \"Negative electrical pin\"</p><p><b>Parameters</b> parameter Real N(start=1,min=Modelica.Constants.eps)=1 \"Number of turns\"; parameter Modelica.Units.SI.Length L=10e-3; parameter Modelica.Units.SI.Area A=10e-6; parameter Modelica.Units.SI.RelativePermeability mu_rel(min=Modelica.Constants.eps)=1; final parameter Modelica.Units.SI.Permeance G_m=Modelica.Constants.mu_0*mu_rel*A/L; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real NFault(min=Modelica.Constants.eps)=0.5*N; parameter Real leakageFaultFactor(min=Modelica.Constants.eps)=2</p><p><b>Behavior</b> Fault-enhanced converter with leakage inductance</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ElectromechanicalFaultEffects\">MSLFaults.Examples.ElectromechanicalFaultEffects</a></p><p><b>Purpose</b> Fault-enhanced ElectroMagneticConverterWithLeakageInductance</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery. WindingOpen latch until a new simulation. faultEndTime does not repair a latched fault.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverterWithLeakageInductance</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ElectromechanicalFaultEffects\">MSLFaults.Examples.ElectromechanicalFaultEffects</a></p><p>用法：将 FaultableElectroMagneticConverterWithLeakageInductance 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p></html>"),
    defaultComponentName="converter",Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-53,-69},{45,69}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8, radius=0),
        Line(points={{-100,100},{-55,100},{-55,55},{-35,55}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-35,-55},{-55,-55},{-55,-100},{-100,-98}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-35,-55},{-30.34,-52.71},{-26,-50.42},{-22.27,-48.12},{-19.41,-45.83},{-17.61,-43.54},{-17,-41.25},{-17.61,-38.96},{-19.41,-36.67},{-22.27,-34.38},{-26,-32.08},{-30.34,-29.79},{-35,-27.5},{-35,-27.5},{-30.34,-25.21},{-26,-22.92},{-22.27,-20.62},{-19.41,-18.33},{-17.61,-16.04},{-17,-13.75},{-17.61,-11.46},{-19.41,-9.17},{-22.27,-6.88},{-26,-4.58},{-30.34,-2.29},{-35,0},{-35,0},{-30.34,2.29},{-26,4.58},{-22.27,6.88},{-19.41,9.17},{-17.61,11.46},{-17,13.75},{-17.61,16.04},{-19.41,18.33},{-22.27,20.62},{-26,22.92},{-30.34,25.21},{-35,27.5},{-35,27.5},{-30.34,29.79},{-26,32.08},{-22.27,34.38},{-19.41,36.67},{-17.61,38.96},{-17,41.25},{-17.61,43.54},{-19.41,45.83},{-22.27,48.12},{-26,50.42},{-30.34,52.71},{-35,55}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{20,-70},{20,70}}, color={255,127,0}, thickness=0.8, pattern=LinePattern.Solid),
        Line(points={{45,69},{100,100}}, color={255,127,0}, thickness=0.7, pattern=LinePattern.Solid),
        Line(points={{45,-69},{100,-100}}, color={255,127,0}, thickness=0.7, pattern=LinePattern.Solid),
        Text(extent={{-4,-15},{17,15}}, textString="N", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Line(points={{49,44},{70,44},{70,-44},{49,-44}}, color={255,127,0}, thickness=0.7, pattern=LinePattern.Dash),
        Text(extent={{57,-5},{85,20}}, textString="L", textColor={64,64,64}, fontSize=0),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableElectroMagneticConverterWithLeakageInductance;
