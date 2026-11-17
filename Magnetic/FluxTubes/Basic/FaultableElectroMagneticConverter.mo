within MSLFaults.Magnetic.FluxTubes.Basic;
model FaultableElectroMagneticConverter "Electromagnetic converter with turn-count and winding-resistance faults"
  Modelica.Magnetic.FluxTubes.Interfaces.PositiveMagneticPort port_p "Positive magnetic port" annotation(Placement(transformation(extent={{90,90},{110,110}}),iconTransformation(extent={{90,90},{110,110}})));
  Modelica.Magnetic.FluxTubes.Interfaces.NegativeMagneticPort port_n "Negative magnetic port" annotation(Placement(transformation(extent={{110,-110},{90,-90}}),iconTransformation(extent={{110,-110},{90,-90}})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive electrical pin" annotation(Placement(transformation(extent={{-90,90},{-110,110}}),iconTransformation(extent={{-90,90},{-110,110}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative electrical pin" annotation(Placement(transformation(extent={{-110,-110},{-90,-90}}),iconTransformation(extent={{-110,-110},{-90,-90}})));
  Modelica.Units.SI.Voltage v "Voltage from p to n";
  Modelica.Units.SI.Current i(start=0,stateSelect=StateSelect.prefer) "Current entering pin p";
  Modelica.Units.SI.MagneticPotentialDifference V_m "Magnetic potential at port_p minus that at port_n";
  Modelica.Units.SI.MagneticFlux Phi "Magnetic flux entering port_p";
  parameter Real N=1 "Number of turns";
  Modelica.Units.SI.MagneticFlux Psi "Flux linkage used by the winding relation";
  Modelica.Units.SI.Inductance L_stat "Magnitude of apparent inductance derived from flux linkage and current";
  type FaultMode=enumeration(Normal "Nominal component relation", TurnCountDrift "Changed effective turn count", TurnLoss "Reduced effective turn count", WindingOpen "Latched finite series resistance", ShortedTurns "Reduced effective turns with the declared winding approximation", CouplingLoss "Reduced effective turn count");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  parameter Real NFault(min=Modelica.Constants.eps)=0.5*N "Effective turn count at full turn-change effect";
  parameter Modelica.Units.SI.Resistance ROpen=1e9 "Added series resistance at full winding-open effect";
  parameter Modelica.Units.SI.Resistance RShortedTurns=1e-6 "Added series resistance at full shorted-turn effect";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law",startActivation(min=0,max=1) "Activation envelope divided by positive severity",endActivation(min=0,max=1) "One while activation or its recovery envelope remains present",N_effective(min=Modelica.Constants.eps) "Turn count used by the electromagnetic relations";
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
  Modelica.Units.SI.Power electricalPower "Electrical power entering the winding",magneticPower "Signed magnetic power from potential difference and flux rate",lossPower "Resistive winding power",parameterPower "Power residual associated with prescribed winding-parameter variation";
protected
  MSLFaults.Utilities.Internal.Activation faultGate(enabled=enabled and faultMode<>FaultMode.Normal,
    permanent=faultMode==FaultMode.WindingOpen,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
    transitionTime=transitionTime,useExternalTrigger=useExternalTrigger);
initial equation
  assert(N>0 and NFault>0 and ROpen>=0 and RShortedTurns>=0,"Invalid winding parameters.");
equation
  connect(trigger,faultGate.externalTrigger);
  if not useExternalTrigger then faultGate.externalTrigger=false; end if;
  active=faultGate.active;
  effectLevel=faultGate.effectLevel;
  activationCount=faultGate.activationCount;

  startActivation=if severity>0 then effectLevel/severity else 0;
  endActivation=if active or effectLevel>0 then 1 else 0;
  faultActivation=effectLevel;
  N_effective=if faultMode==FaultMode.TurnCountDrift or faultMode==FaultMode.TurnLoss or faultMode==FaultMode.ShortedTurns or faultMode==FaultMode.CouplingLoss then N+faultActivation*(NFault-N) else N;
  R_series_effective=if faultMode==FaultMode.WindingOpen then faultActivation*ROpen elseif faultMode==FaultMode.ShortedTurns then faultActivation*RShortedTurns else 0;
  v=p.v-n.v;
  0=p.i+n.i;
  i=p.i;
  V_m=port_p.V_m-port_n.V_m;
  0=port_p.Phi+port_n.Phi;
  Phi=port_p.Phi;
  V_m=i*N_effective;
  der(Psi)=-(v-i*R_series_effective);
  Psi=N_effective*Phi;
  L_stat=noEvent(if abs(i)>eps then abs(Psi/i) else abs(Psi/eps));

  electricalPower=v*i;
  magneticPower=V_m*der(Phi);
  lossPower=R_series_effective*i*i;
  parameterPower=lossPower-electricalPower-magneticPower;
  annotation(
    Documentation(info="<html><p>TurnCountDrift, TurnLoss, CouplingLoss and ShortedTurns vary N_effective. ShortedTurns also adds RShortedTurns; WindingOpen adds ROpen and latches. Flux linkage Psi=N_effective*Phi is differentiated in the electrical relation.</p><p>Connect p and n and both magnetic ports; set N, NFault and winding resistance targets, and inspect the power diagnostics.</p><p>parameterPower=lossPower-electricalPower-magneticPower records the variable-turn contribution under the declared port signs. ShortedTurns is a reduced-order turn/resistance model, not a separate shorted winding circuit.</p><p>Nominal reference: <code>Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverter</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    defaultComponentName="converter",Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-53,-69},{45,69}}, lineColor={255,0,0}, fillColor={255,255,255}, fillPattern=FillPattern.None, pattern=LinePattern.Solid, lineThickness=0.8, radius=0),
        Line(points={{-100,100},{-55,100},{-55,55},{-35,55}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-35,-55},{-55,-55},{-55,-100},{-100,-100}}, color={0,0,255}, thickness=0.65, pattern=LinePattern.Solid),
        Line(points={{-35,-55},{-30.34,-52.71},{-26,-50.42},{-22.27,-48.12},{-19.41,-45.83},{-17.61,-43.54},{-17,-41.25},{-17.61,-38.96},{-19.41,-36.67},{-22.27,-34.38},{-26,-32.08},{-30.34,-29.79},{-35,-27.5},{-35,-27.5},{-30.34,-25.21},{-26,-22.92},{-22.27,-20.62},{-19.41,-18.33},{-17.61,-16.04},{-17,-13.75},{-17.61,-11.46},{-19.41,-9.17},{-22.27,-6.88},{-26,-4.58},{-30.34,-2.29},{-35,0},{-35,0},{-30.34,2.29},{-26,4.58},{-22.27,6.88},{-19.41,9.17},{-17.61,11.46},{-17,13.75},{-17.61,16.04},{-19.41,18.33},{-22.27,20.62},{-26,22.92},{-30.34,25.21},{-35,27.5},{-35,27.5},{-30.34,29.79},{-26,32.08},{-22.27,34.38},{-19.41,36.67},{-17.61,38.96},{-17,41.25},{-17.61,43.54},{-19.41,45.83},{-22.27,48.12},{-26,50.42},{-30.34,52.71},{-35,55}}, color={255,0,0}, thickness=0.85, pattern=LinePattern.Solid),
        Line(points={{20,-70},{20,70}}, color={255,127,0}, thickness=0.8, pattern=LinePattern.Solid),
        Line(points={{45,69},{100,100}}, color={255,127,0}, thickness=0.7, pattern=LinePattern.Solid),
        Line(points={{45,-69},{100,-100}}, color={255,127,0}, thickness=0.7, pattern=LinePattern.Solid),
        Text(extent={{-4,-15},{17,15}}, textString="N", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0),
        Text(extent={{108,94},{139,62}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableElectroMagneticConverter;
