within MSLFaults.Utilities.Internal;
block Activation "Severity-scaled activation with reversible ramps and optional latching"
parameter Boolean enabled=true "Enable activation requests";
  parameter Boolean permanent=false "Latch the first accepted request";
  parameter Real severity(min=0,max=1)=1 "Maximum activation-envelope level";
  parameter Modelica.Units.SI.Time startTime=1 "Inclusive internal activation start";
  parameter Modelica.Units.SI.Time endTime=3 "Exclusive internal activation end";
  parameter Modelica.Units.SI.Time transitionTime(min=0)=0.05 "Time for a full unit-level transition";
  parameter Boolean useExternalTrigger=false "Use the external trigger instead of the internal fault window";
  Modelica.Blocks.Interfaces.BooleanInput externalTrigger  "External activation request" annotation(Placement(transformation(origin={-100,0},extent={{-10,-10},{10,10}}),iconTransformation(origin={-100,0},extent={{-10,-10},{10,10}})));
  output Boolean active "Requested or latched fault state before transition shaping";
  output Modelica.Units.SI.Time elapsedActiveTime "Current or most recently completed activation duration";
  output Real effectLevel(min=0,max=1) "Severity-scaled activation envelope";
  output Integer activationCount(start=0,fixed=true) "Number of activation starts, including initial activation";
protected
  Boolean request "Enabled activation request before latching";
  discrete Boolean latched(start=false,fixed=true) "Permanent activation state";
  discrete Real fromLevel(start=0,fixed=true) "Unit level at the most recent state change";
  discrete Modelica.Units.SI.Time edgeTime(start=0,fixed=true) "Time of the most recent activation state change";
  Real level "Unit activation level before severity scaling";
  discrete Modelica.Units.SI.Time endedDuration(start=0,fixed=true) "Duration retained after deactivation";
initial equation
  assert(severity>=0 and severity<=1,"Severity outside [0,1].");
  assert(endTime>=startTime,"End time precedes start time.");
  assert(transitionTime>=0,"Negative transition duration.");
equation
  request=enabled and severity>0 and (if useExternalTrigger then externalTrigger else time>=startTime and time<endTime);
  when {initial(),request} then
    latched=(if initial() then false else pre(latched)) or (permanent and request);
  end when;
  active=request or latched;
  when {initial(),change(active)} then
    fromLevel=if initial() or transitionTime<=0 then 0 else
      min(1,max(0,pre(fromLevel)+(if pre(active) then 1 else -1)*(time-pre(edgeTime))/transitionTime));
    edgeTime=time;
    endedDuration=if initial() or active then 0 else time-pre(edgeTime);
    activationCount=(if initial() then 0 else pre(activationCount))+(if active then 1 else 0);
  end when;
  level=if transitionTime<=0 then (if active then 1 else 0) else
    min(1,max(0,fromLevel+(if active then 1 else -1)*(time-edgeTime)/transitionTime));
  elapsedActiveTime=if active then time-edgeTime else endedDuration;
  effectLevel=severity*level;
  annotation(Icon(coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(extent={{-90,-83},{90,83}}, lineColor={64,64,64}, fillColor={255,255,255}, fillPattern=FillPattern.Solid, pattern=LinePattern.Solid, lineThickness=0.55, radius=0),
        Line(points={{-65,54},{-65,-44},{65,-44}}, color={128,128,128}, thickness=0.5, pattern=LinePattern.Solid),
        Line(points={{-58,-29},{-35,-29},{-12,34},{23,34},{46,-29},{61,-29}}, color={255,0,0}, thickness=0.9, pattern=LinePattern.Solid),
        Text(extent={{-49,-56},{49,-78}}, textString="level", textColor={64,64,64}, fontSize=0),
        Text(extent={{-150,155},{150,125}}, textString="%name", textColor={0,0,255}, fontSize=0)
      }),
    Documentation(info="<html><p>Generate a request when <code>enabled</code> is true, <code>severity&gt;0</code> and the internal half-open interval <code>[startTime,endTime)</code> or selected <code>externalTrigger</code> is active. <code>permanent=true</code> latches the first accepted request. <code>active</code> reports this Boolean state.</p><p>A positive <code>transitionTime</code> gives a linear unit-level rise or fall, scaled by <code>severity</code>; reversals continue from the current level. Initial ramps begin at zero. A zero transition changes the level immediately. An initially active request increments <code>activationCount</code>.</p><p><code>elapsedActiveTime</code> is the duration since the latest activation while active and retains the completed active duration after deactivation. It resets on the next activation. The recovery ramp can continue after <code>active</code> becomes false.</p></html>"));
end Activation;
