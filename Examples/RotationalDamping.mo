within MSLFaults.Examples;
model RotationalDamping "Compare damping, inertia and rotational stiffness changes"
  extends Modelica.Icons.Example;
  MSLFaults.Utilities.Internal.RotationalDrive native annotation(Placement(transformation(origin={-265,111},extent={{-23,-23},{23,23}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeSpeed annotation(Placement(transformation(origin={-219,64},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.RotationalDrive normal(redeclare MSLFaults.Mechanics.Rotational.Components.FaultableInertia rotor,redeclare MSLFaults.Mechanics.Rotational.Components.FaultableDamper damper) annotation(Placement(transformation(origin={-114,111},extent={{-23,-23},{23,23}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalSpeed annotation(Placement(transformation(origin={-68,64},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.RotationalDrive dampingFault(redeclare MSLFaults.Mechanics.Rotational.Components.FaultableInertia rotor,redeclare MSLFaults.Mechanics.Rotational.Components.FaultableDamper damper(faultMode=MSLFaults.Mechanics.Rotational.Components.FaultableDamper.FaultMode.DampingIncrease,dIncrease=0.2,faultStartTime=0,faultEndTime=6,transitionTime=0)) annotation(Placement(transformation(origin={37,111},extent={{-23,-23},{23,23}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput dampingFaultSpeed annotation(Placement(transformation(origin={83,64},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.RotationalDrive inertiaFault(redeclare MSLFaults.Mechanics.Rotational.Components.FaultableInertia rotor(faultMode=MSLFaults.Mechanics.Rotational.Components.FaultableInertia.FaultMode.InertiaLoss,JFault=0.05,faultStartTime=0,faultEndTime=6,transitionTime=0),redeclare MSLFaults.Mechanics.Rotational.Components.FaultableDamper damper) annotation(Placement(transformation(origin={188,111},extent={{-23,-23},{23,23}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput inertiaFaultSpeed annotation(Placement(transformation(origin={234,64},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.RotationalSpringPath nativeSpring annotation(Placement(transformation(origin={-230,-86},extent={{-23,-23},{23,23}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput nativeSpringTorque annotation(Placement(transformation(origin={-168,-142},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.RotationalSpringPath normalSpring(redeclare MSLFaults.Mechanics.Rotational.Components.FaultableSpring spring) annotation(Placement(transformation(origin={-59,-86},extent={{-23,-23},{23,23}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput normalSpringTorque annotation(Placement(transformation(origin={3,-142},extent={{-7,-7},{7,7}},rotation=0)));
  MSLFaults.Utilities.Internal.RotationalSpringPath faultSpring(redeclare MSLFaults.Mechanics.Rotational.Components.FaultableSpring spring(faultMode=MSLFaults.Mechanics.Rotational.Components.FaultableSpring.FaultMode.StiffnessDegradation,cDegraded=5,faultStartTime=0,faultEndTime=6,transitionTime=0)) annotation(Placement(transformation(origin={112,-86},extent={{-23,-23},{23,23}},rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput faultSpringTorque annotation(Placement(transformation(origin={174,-142},extent={{-7,-7},{7,7}},rotation=0)));
equation
  connect(native.speed,nativeSpeed) annotation(Line(points={{-242,111},{-219,111},{-219,64}},color={0,0,127}));
  connect(normal.speed,normalSpeed) annotation(Line(points={{-91,111},{-68,111},{-68,64}},color={0,0,127}));
  connect(dampingFault.speed,dampingFaultSpeed) annotation(Line(points={{60,111},{83,111},{83,64}},color={0,0,127}));
  connect(inertiaFault.speed,inertiaFaultSpeed) annotation(Line(points={{211,111},{234,111},{234,64}},color={0,0,127}));
  connect(nativeSpring.torque,nativeSpringTorque) annotation(Line(points={{-207,-86},{-168,-86},{-168,-142}},color={0,0,127}));
  connect(normalSpring.torque,normalSpringTorque) annotation(Line(points={{-36,-86},{3,-86},{3,-142}},color={0,0,127}));
  connect(faultSpring.torque,faultSpringTorque) annotation(Line(points={{135,-86},{174,-86},{174,-142}},color={0,0,127}));
  annotation(experiment(StartTime=0,StopTime=5,Interval=0.01,Tolerance=1e-6),
    Diagram(coordinateSystem(extent={{-360,-240},{360,240}},preserveAspectRatio=true),graphics={Text(extent={{-274,212},{278,198}},textString="Inertia and damper with the same native initial speed",textColor={34,61,73}),Text(extent={{-222,1},{162,-13}},textString="Independent prescribed spring deflection",textColor={34,61,73})}),
    Documentation(info="<html><p>Compare torque-driven rotors with increased damping or reduced inertia, then compare separate prescribed-deflection spring paths. Increased damping changes the steady speed; reduced inertia changes the transient with the nominal damping retained.</p><p>Plot the four speed outputs and three spring-torque outputs; inspect each independent path separately. Use the stored experiment settings.</p></html>"));
end RotationalDamping;
