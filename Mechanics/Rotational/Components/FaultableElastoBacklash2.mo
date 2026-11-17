within MSLFaults.Mechanics.Rotational.Components;
model FaultableElastoBacklash2
  "Rotational backlash with stiffness, damping and tooth-loss faults and contact jumps"

  parameter Modelica.Units.SI.RotationalSpringConstant c(final min=Modelica.Constants.small,
      start=1.0e5) "Spring constant (c > 0 required)";
  parameter Modelica.Units.SI.RotationalDampingConstant d(final min=0, start=0)
    "Damping constant";
  parameter Modelica.Units.SI.Angle b(final min=0)=0 "Total backlash";
  parameter Modelica.Units.SI.Angle phi_rel0=0 "Unstretched spring angle";

  extends
    Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;
  extends
    Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
  type FaultMode=enumeration(Normal "Nominal component relation", StiffnessLoss "Reduced stiffness", DampingLoss "Reduced damping", BacklashIncrease "Increased angular backlash", ToothBroken "Latched loss of stiffness and damping", Seizure "Increased finite mechanical resistance");
  parameter FaultMode faultMode=FaultMode.Normal "Selected fault law";
  parameter Real severity(min=0,max=1)=1 "Scale of the activation envelope";
  parameter Modelica.Units.SI.Time faultStartTime=1 "Start of the internal fault request";
  parameter Modelica.Units.SI.Time faultEndTime=3 "End of the internal half-open fault window";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Duration of a complete activation or recovery ramp";
  Real faultActivation(min=0,max=1) "Activation envelope used by the fault law";
  Real startActivation(min=0,max=1) "Activation envelope divided by positive severity";
  Real endActivation(min=0,max=1) "One while activation or its recovery envelope remains present";
  parameter Modelica.Units.SI.RotationalSpringConstant cFault=0.5*c "Stiffness target for the selected fault law";
  parameter Modelica.Units.SI.RotationalDampingConstant dFault=0.5*d "Damping target for the selected fault law";
  parameter Modelica.Units.SI.Angle bFault=b+0.1 "Total backlash at full effect";
  parameter Modelica.Units.SI.RotationalDampingConstant dSeized=1e9 "Finite damping at full seizure effect";
  Modelica.Units.SI.RotationalSpringConstant c_effective "Stiffness used by the constitutive relation";
  Modelica.Units.SI.RotationalDampingConstant d_effective "Damping used by the constitutive relation";
  Modelica.Units.SI.Angle b_effective "Effective total angular backlash";
  Modelica.Units.SI.Angle bMax_effective "Upper boundary of the unloaded angular gap";
  Modelica.Units.SI.Angle bMin_effective "Lower boundary of the unloaded angular gap";
protected
  final parameter Modelica.Units.SI.Angle bMax=b/2
    "Backlash in range bMin <= phi_rel - phi_rel0 <= bMax";
  final parameter Modelica.Units.SI.Angle bMin=-bMax
    "Backlash in range bMin <= phi_rel - phi_rel0 <= bMax";
  Modelica.Units.SI.Torque tau_c;
  Modelica.Units.SI.Torque tau_d;
  Modelica.Units.SI.Angle phi_diff=phi_rel - phi_rel0;

  // A minimum backlash is defined in order to avoid an infinite
  // number of state events if backlash b is set to zero.
  constant Modelica.Units.SI.Angle bEps=1e-10 "Minimum backlash";


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
    permanent=faultMode==FaultMode.ToothBroken,severity=severity,startTime=faultStartTime,endTime=faultEndTime,
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
  c_effective=if faultMode==FaultMode.StiffnessLoss then c+faultActivation*(cFault-c) elseif faultMode==FaultMode.ToothBroken then c*(1-faultActivation) else c;
  d_effective=if faultMode==FaultMode.DampingLoss then d+faultActivation*(dFault-d) elseif faultMode==FaultMode.ToothBroken then d*(1-faultActivation) elseif faultMode==FaultMode.Seizure then d+faultActivation*(dSeized-d) else d;
  b_effective=if faultMode==FaultMode.BacklashIncrease then b+faultActivation*(bFault-b) else b;
  bMax_effective=b_effective/2;
  bMin_effective=-b_effective/2;

  if initial() then
    /* During initialisation the characteristic is modified, in order that
        it is a strict monotone rising function. Otherwise, initialisation might
        result in a singular system when the characteristic has to be
        inverted. The characteristic is modified in the range 1.5*bMin_effective <= phi_rel - phi_rel0 <= 1.5 bMax_effective,
        so that in this range a linear characteristic is present that approaches the original
        function continuously at its limits, e.g.,
          original:  tau(1.5*bMax_effective) = c_effective*(phi_diff - bMax_effective)
                                   = c_effective*(0.5*bMax_effective)
          initial :  tau(1.5*bMax_effective) = (c_effective/3)*phi_diff
                                   = (c_effective/3)*(3/2)*bMax_effective
                                   = (c_effective/2)*bMax_effective
     */
    tau_c = if phi_diff > 1.5*bMax_effective then c_effective*(phi_diff - bMax_effective) else if
      phi_diff < 1.5*bMin_effective then c_effective*(phi_diff - bMin_effective) else (c_effective/3)*phi_diff;
    tau_d = d_effective*w_rel;
    tau = tau_c + tau_d;
    lossPower = tau_d*w_rel;
   else
     /*
     if abs(b_effective) <= bEps then
        tau_c = c_effective*phi_diff;
        tau_d = d_effective*w_rel;
        tau   = tau_c + tau_d;
     elseif phi_diff > bMax_effective then
        tau_c = c_effective*(phi_diff - bMax_effective);
        tau_d = d_effective*w_rel;
        tau   = if tau_c + tau_d <= 0 then 0 else tau_c + tau_d;
     elseif phi_diff < bMin_effective then
        tau_c = c_effective*(phi_diff - bMin_effective);
        tau_d = d_effective*w_rel;
        tau   = if tau_c + tau_d >= 0 then 0 else tau_c + tau_d;
     else
        tau_c = 0;
        tau_d = 0;
        tau   = 0;
     end if;

     This is written in the form below, in order that parameter "b"
     is not evaluated during translation (i.e., in the above form
     it cannot be changed anymore after translation).
   */
    tau_c = if abs(b_effective) <= bEps then
               c_effective*phi_diff
            else if phi_diff > bMax_effective then
               c_effective*(phi_diff - bMax_effective)
            else if phi_diff < bMin_effective then
               c_effective*(phi_diff - bMin_effective)
            else 0;
    tau_d = d_effective*w_rel;

    tau = if abs(b_effective) <= bEps then
             tau_c + tau_d
          else if phi_diff > bMax_effective then
             (if tau_c + tau_d <= 0 then 0 else tau_c + tau_d)
          else if phi_diff < bMin_effective then
             (if tau_c + tau_d >= 0 then 0 else tau_c + tau_d)
          else 0;

    lossPower = if abs(b_effective) <= bEps then
                   tau_d*w_rel
                else if phi_diff > bMax_effective then
                   (if tau_c + tau_d <= 0 then 0 else tau_d*w_rel)
                else if phi_diff < bMin_effective then
                   (if tau_c + tau_d >= 0 then 0 else tau_d*w_rel)
                else 0;
  end if;


  annotation (defaultComponentName="elastoBacklash",
    Documentation(info="<html><p>StiffnessLoss and DampingLoss change c and d; BacklashIncrease changes the total unloaded angular gap. ToothBroken latches and reduces both stiffness and damping towards zero. Seizure raises the damping. The contact law clips the total spring and damping torque; torque may jump when contact begins.</p><p>Connect the shaft flanges; set c, d, total backlash b and the corresponding fault target. Inspect phi_rel and tau.</p><p>The initial contact characteristic is regularised. ToothBroken remains active after the trigger or time window ends; full severity removes stiffness and damping.</p><p>Nominal reference: <code>Modelica.Mechanics.Rotational.Components.ElastoBacklash2</code>. Activation uses the configured half-open fault window or the optional trigger; inspect <code>effectLevel</code> during recovery.</p></html>"),
    Icon(coordinateSystem(preserveAspectRatio=false,
      extent={{-100,-100},{100,100}}),
      graphics={
        Line(points={{-80,32},{-58,32},{-48,0},{-34,61},{-20,0},{-8,60},{0,30},{20,30}}, color={64,64,64}, thickness=0.5),
        Rectangle(extent={{-60,-10},{-10,-50}}, fillColor={192,192,192}, fillPattern=FillPattern.Solid, lineColor={64,64,64}, lineThickness=0.5),
        Line(points={{-60,-50},{0,-50}}, color={64,64,64}, thickness=0.5),
        Line(points={{-60,-10},{0,-10}}, color={64,64,64}, thickness=0.5),
        Line(points={{-10,-30},{20,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,-30},{-60,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-80,32},{-80,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{20,30},{20,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{-90,0},{-80,0}}, color={64,64,64}, thickness=0.5),
        Line(points={{90,0},{80,0}}, color={64,64,64}, thickness=0.5),
        Line(points={{20,0},{60,0},{60,-30}}, color={64,64,64}, thickness=0.5),
        Line(points={{40,-12},{40,-40},{80,-40},{80,0}}, color={64,64,64}, thickness=0.5),
        Text(extent={{-150,-130},{150,-90}}, textString="b=%b"),
        Text(extent={{-150,155},{150,125}}, textColor={0,0,255}, textString="%name"),
        Text(extent={{-152,-92},{148,-52}}, textString="c_effective=%c_effective"),
        Line(visible=useHeatPort, points={{-100,-100},{-100,-43},{-34,-43}}, color={191,0,0}, pattern=LinePattern.Dot, thickness=0.5),
        Text(extent={{20,48},{80,10}}, textColor={95,95,95}, textString="2"),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }));
end FaultableElastoBacklash2;
