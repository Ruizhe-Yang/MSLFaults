within MSLFaults.Electrical.Analog.Sources;
model FaultableSupplyVoltage "Supply voltage (positive and negative) with independent faults"
  type FaultMode=enumeration(Normal "正常", PositiveRailLoss "正电源轨丢失", NegativeRailLoss "负电源轨丢失", RailAmplitudeDrift "电源轨幅值漂移", GroundShift "地电位偏移");
  parameter Modelica.Units.SI.Voltage Vps=+15 "Positive supply";
  parameter Modelica.Units.SI.Voltage Vns=-15 "Negative supply";
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin ground annotation(Placement(transformation(extent={{-10,-10},{10,10}})));
  parameter FaultMode faultMode=FaultMode.Normal;
  parameter Real severity(min=0,max=1)=1;
  parameter Modelica.Units.SI.Time faultStartTime=1;
  parameter Modelica.Units.SI.Time faultEndTime=3;
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05;
  parameter Real railScaleFault=0.5;
  parameter Modelica.Units.SI.Voltage groundShiftFault=1;
  Real faultActivation(min=0,max=1);
  Real startActivation(min=0,max=1);
  Real endActivation(min=0,max=1);
  Modelica.Units.SI.Voltage Vps_effective;
  Modelica.Units.SI.Voltage Vns_effective;

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
  Vps_effective = if faultMode==FaultMode.PositiveRailLoss then Vps*(1-faultActivation)
    elseif faultMode==FaultMode.RailAmplitudeDrift then Vps*(1+faultActivation*(railScaleFault-1)) else Vps;
  Vns_effective = if faultMode==FaultMode.NegativeRailLoss then Vns*(1-faultActivation)
    elseif faultMode==FaultMode.RailAmplitudeDrift then Vns*(1+faultActivation*(railScaleFault-1)) else Vns;
  pin_p.v-ground.v = Vps_effective + (if faultMode==FaultMode.GroundShift then faultActivation*groundShiftFault else 0);
  pin_n.v-ground.v = Vns_effective + (if faultMode==FaultMode.GroundShift then faultActivation*groundShiftFault else 0);
  pin_p.i + pin_n.i + ground.i = 0;


  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}),
      graphics={
        Line(points={{-60,40},{-60,-40}}, color={0,0,255}, thickness=0.5),
        Line(points={{40,40},{40,-40}}, color={0,0,255}, thickness=0.5),
        Line(points={{-40,20},{-40,-20}}, color={0,0,255}, thickness=0.5),
        Line(points={{60,20},{60,-20}}, color={0,0,255}, thickness=0.5),
        Line(points={{-90,0},{-60,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{60,0},{90,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-150,155},{150,125}}, textColor={0,0,255}, textString="%name"),
        Line(points={{-40,0},{-10,0}}, color={0,0,255}, thickness=0.5),
        Line(points={{10,0},{40,0}}, color={0,0,255}, thickness=0.5),
        Text(extent={{-100,40},{-80,20}}, textColor={0,0,255}, textString="+"),
        Text(extent={{80,40},{100,20}}, textColor={0,0,255}, textString="-"),
        Text(extent={{-10,40},{10,20}}, textColor={0,0,255}, textString="0"),
        Ellipse(extent={{-70,70},{70,-70}}, lineColor={255,0,0}, lineThickness=0.75),
        Text(extent={{59,96},{91,63}}, textString="F", textColor={255,0,0}, fontSize=0)
      }),
    Documentation(info="<html><p><b>Purpose</b> Supply voltage (positive and negative) with independent faults</p><p><b>Interfaces</b> Modelica.Electrical.Analog.Interfaces.PositivePin pin_p; Modelica.Electrical.Analog.Interfaces.NegativePin pin_n; Modelica.Electrical.Analog.Interfaces.NegativePin ground; Modelica.Blocks.Interfaces.BooleanInput trigger if useExternalTrigger; output Boolean active \"Requested or latched fault\"; output Real effectLevel(min=0,max=1) \"Current realized fault effect\"; output Integer activationCount \"Number of rising activations\"</p><p><b>Parameters</b> parameter Modelica.Units.SI.Voltage Vps=+15 \"Positive supply\"; parameter Modelica.Units.SI.Voltage Vns=-15 \"Negative supply\"; parameter FaultMode faultMode=FaultMode.Normal; parameter Real severity(min=0,max=1)=1; parameter Modelica.Units.SI.Time faultStartTime=1; parameter Modelica.Units.SI.Time faultEndTime=3; parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05; parameter Real railScaleFault=0.5; parameter Modelica.Units.SI.Voltage groundShiftFault=1; parameter Boolean enabled=true \"Enable the selected fault mode\"; parameter Boolean useExternalTrigger=false \"Use trigger instead of the internal time window\"</p><p><b>Behavior</b> Supply voltage (positive and negative) with independent faults</p><p><b>Initialization</b> Use the declared initial values and host state modifiers.</p><p><b>Activation and recovery</b> Use the declared activation inputs and timing.</p><p><b>Limitations</b> Use the declared parameter range. The catalog identifies simulation coverage.</p><p><b>Observables</b> active; effectLevel; activationCount</p><p><b>Replacement contract</b> Use the stated native class constraint and preserve its modifiers. See the catalog for verified coverage.</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p><b>Purpose</b> Fault-enhanced SupplyVoltage</p><p><b>Interfaces</b> Native visible contract plus the optional Boolean trigger.</p><p><b>Parameters</b> The native modifiers and the local FaultMode are preserved. Default fault interval is 1 to 3 s.</p><p><b>Behavior</b> The established physical equations and fault modes use the common activation and severity law.</p><p><b>Initialization</b> The original initialization modifiers remain available.</p><p><b>Activation and recovery</b> Use the half-open time window or the optional trigger. Effect starts at zero for a positive ramp and transitions on recovery.</p><p><b>Limitations</b> Physical extremes are finite where specified. Dynamic validation coverage is stated in the release catalog.</p><p><b>Observables</b> active, effectLevel, activationCount and the component-specific physical or signal quantities.</p><p><b>Nominal MSL counterpart</b> Modelica.Electrical.Analog.Sources.SupplyVoltage</p><p><b>Example</b> <a href=\"modelica://MSLFaults.Examples.ResistorReplacement\">MSLFaults.Examples.ResistorReplacement</a></p><p>用法：将 FaultableSupplyVoltage 按其 MSL 对应元件连接到系统中。faultMode=Normal 或 severity=0 表示名义行为；选择其他 faultMode，并设置 severity、faultStartTime、faultEndTime 和 transitionTime 后仿真故障响应。</p>
                       <p>This is a simple model of a constant supply voltage with positive and negative supply, the potential between positive and negative supply is accessible.</p>
                       </html>"));
end FaultableSupplyVoltage;
