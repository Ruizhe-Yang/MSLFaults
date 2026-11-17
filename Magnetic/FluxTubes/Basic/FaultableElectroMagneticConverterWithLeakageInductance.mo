within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableElectroMagneticConverterWithLeakageInductance "Electromagnetic converter with turn loss and magnetic leakage faults"
  Modelica.Magnetic.FluxTubes.Interfaces.PositiveMagneticPort port_p "Positive magnetic port" annotation(Placement(transformation(extent={{90,90},{110,110}}),iconTransformation(extent={{90,90},{110,110}})));
  Modelica.Magnetic.FluxTubes.Interfaces.NegativeMagneticPort port_n "Negative magnetic port" annotation(Placement(transformation(extent={{110,-110},{90,-90}}),iconTransformation(extent={{110,-110},{90,-90}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin" annotation(Placement(transformation(extent={{-90,90},{-110,110}}),iconTransformation(extent={{-90,90},{-110,110}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative electrical pin" annotation(Placement(transformation(extent={{-110,-108},{-90,-88}}),iconTransformation(extent={{-110,-108},{-90,-88}})));
  Modelica.Units.SI.Voltage v "Voltage from p to n";
  Modelica.Units.SI.Current i(start=0,stateSelect=StateSelect.prefer) "Current entering pin p";
  Modelica.Units.SI.MagneticPotentialDifference V_m "Magnetic potential at port_p minus that at port_n";
  Modelica.Units.SI.MagneticFlux Phi(stateSelect=StateSelect.never) "Magnetic flux entering port_p";
  Modelica.Units.SI.MagneticFlux Phi_ind(stateSelect=StateSelect.never) "Flux associated with the induced-voltage relation";
  Modelica.Units.SI.MagneticFlux Phi_leak(stateSelect=StateSelect.never) "Magnetic leakage flux";
  parameter Real N(start=1,min=Modelica.Constants.eps)=1 "Number of turns";
  parameter Modelica.Units.SI.Length L=10e-3 "Length used to calculate leakage permeance";
  parameter Modelica.Units.SI.Area A=10e-6 "Area used to calculate leakage permeance";
  parameter Modelica.Units.SI.RelativePermeability mu_rel(min=Modelica.Constants.eps)=1 "Relative permeability of the leakage path";
  final parameter Modelica.Units.SI.Permeance G_m=Modelica.Constants.mu_0*mu_rel*A/L "Nominal leakage permeance from geometry";
  Modelica.Units.SI.MagneticFlux Psi "Flux linkage used by the winding relation";
  Modelica.Units.SI.Inductance L_stat "Magnitude of apparent inductance derived from flux linkage and current";
  type FaultMode=enumeration(Normal "Nominal component relation", TurnLoss "Reduced effective turn count", WindingOpen "Latched finite series resistance", ShortedTurns "Reduced effective turns with the declared winding approximation", LeakageIncrease "Increased magnetic leakage", LeakageDecrease "Reduced magnetic leakage");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  parameter Real NFault(min=Modelica.Constants.eps)=0.5*N "Effective turn count at full turn-change effect";
  parameter Real leakageFaultFactor(min=Modelica.Constants.eps)=2 "Leakage-permeance multiplier or reciprocal divisor at full effect";
  parameter Modelica.Units.SI.Resistance ROpen=1e9 "Added series resistance at full winding-open effect";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law",startActivation(min=0,max=1) "Activation envelope divided by positive severity",endActivation(min=0,max=1) "One while activation or its recovery envelope remains present",N_effective(min=Modelica.Constants.eps) "Turn count used by the electromagnetic relations";
  Modelica.Units.SI.Permeance G_m_effective "Magnetic permeance used by the current relation";
  Modelica.Units.SI.Resistance R_series_effective "Added winding series resistance";
protected
  constant Real eps=100*Modelica.Constants.eps;

public
  parameter Boolean enabled=true "Enable activation of the selected fault mode";
  parameter Boolean useExternalTrigger=false "Use trigger in place of the internal fault window" annotation(Evaluate=true);
  Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger "External Boolean activation request"
    annotation(Placement(transformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270),
      iconTransformation(origin={60,104},extent={{-10,-10},{10,10}},rotation=270)));
  output Boolean active "Requested or latched activation state";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount "Number of rising activations, including an initially active request";

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
    Documentation(info="<html><p>TurnLoss and ShortedTurns share the turn-count target. LeakageIncrease and LeakageDecrease scale the geometric permeance. WindingOpen adds a finite series resistance and latches. Total flux combines Phi_ind and Phi_leak.</p><p>Connect the electrical and magnetic ports; configure N, leakage geometry and NFault or leakageFaultFactor, then inspect Phi_ind and Phi_leak.</p><p>The electrical relation uses N_effective*der(Phi_ind), without differentiating N_effective. ShortedTurns changes only the effective turns, and this model provides no explicit loss or parameter-power diagnostics.</p><p>Nominal reference: <code>Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverterWithLeakageInductance</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
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
