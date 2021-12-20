within ;
package Project_IV_689
  model Charging
    Modelica.Electrical.Batteries.BatteryStacks.CellStack cellStack(
      Ns=28,
      Np=1,
      cellData(
        Qnom(displayUnit="A.h") = 23400,
        OCVmax=7.2,
        Ri=0.15),
      SOC(fixed=true, start=0.7))
      annotation (Placement(transformation(extent={{-92,-22},{-72,-2}})));
    Modelica.Magnetic.FluxTubes.Sources.SignalMagneticPotentialDifference
      magVoltageSource
      annotation (Placement(transformation(extent={{42,28},{62,48}})));
    Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverter converter1(N=20)
      annotation (Placement(transformation(extent={{-72,18},{-52,38}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-52,-44},{-32,-24}})));
    Modelica.Mechanics.Translational.Components.Vehicle vehicle(
      m=1700,
      J=2,
      R=0.2032)
      annotation (Placement(transformation(extent={{52,-80},{72,-60}})));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_SeriesExcited dcse
      annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
    Modelica.Blocks.Sources.Pulse pulse(amplitude=100,period=10)
      annotation (Placement(transformation(extent={{-24,62},{-4,82}})));
    Modelica.Magnetic.FluxTubes.Basic.Ground ground1
      annotation (Placement(transformation(extent={{90,-2},{110,18}})));
    Modelica.Mechanics.Rotational.Components.Gearbox gearbox(ratio=0.2)
      annotation (Placement(transformation(extent={{0,-72},{20,-52}})));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{90,-66},{110,-46}})));
  equation
    connect(magVoltageSource.port_p, converter1.port_p) annotation (Line(points=
           {{42,38},{-44,38},{-44,42},{-52,42},{-52,38}}, color={255,127,0}));
    connect(magVoltageSource.port_n, converter1.port_n) annotation (Line(points=
           {{62,38},{66,38},{66,20},{-44,20},{-44,18},{-52,18}}, color={255,127,
            0}));
    connect(converter1.n, cellStack.n) annotation (Line(points={{-72,18},{-74,
            18},{-74,-12},{-72,-12}}, color={0,0,255}));
    connect(cellStack.p, converter1.p) annotation (Line(points={{-92,-12},{-96,
            -12},{-96,38},{-72,38}}, color={0,0,255}));
    connect(ground.p, converter1.n) annotation (Line(points={{-42,-24},{-42,4},
            {-74,4},{-74,18},{-72,18}}, color={0,0,255}));
    connect(dcse.pin_an, dcse.pin_ep) annotation (Line(points={{-40,-50},{-48,
            -50},{-48,-54},{-44,-54}}, color={0,0,255}));
    connect(dcse.pin_en, cellStack.n) annotation (Line(points={{-44,-66},{-60,
            -66},{-60,-16},{-72,-16},{-72,-12}}, color={0,0,255}));
    connect(pulse.y, magVoltageSource.V_m)
      annotation (Line(points={{-3,72},{52,72},{52,49}}, color={0,0,127}));
    connect(cellStack.p, dcse.pin_ap) annotation (Line(points={{-92,-12},{-92,
            -44},{-28,-44},{-28,-50}}, color={0,0,255}));
    connect(ground1.port, converter1.port_n) annotation (Line(points={{100,18},
            {82,18},{82,28},{66,28},{66,20},{-44,20},{-44,18},{-52,18}}, color=
            {255,127,0}));
    connect(gearbox.flange_a, dcse.flange) annotation (Line(points={{0,-62},{
            -18,-62},{-18,-60},{-24,-60}}, color={0,0,0}));
    connect(gearbox.flange_b, vehicle.flangeR) annotation (Line(points={{20,-62},
            {46,-62},{46,-70},{52,-70}}, color={0,0,0}));
    connect(speedSensor.flange, vehicle.flangeT) annotation (Line(points={{90,-56},
            {76,-56},{76,-70},{72,-70}},                          color={0,127,
            0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Charging;

  model Controller
    Modelica.Blocks.Interfaces.RealInput Bat1 annotation (Placement(
          transformation(extent={{-110,-12},{-88,10}}), iconTransformation(
            extent={{-110,-12},{-88,10}})));
    Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(
        threshold=4)
      annotation (Placement(transformation(extent={{-34,46},{-14,66}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold=5)
      annotation (Placement(transformation(extent={{-34,-42},{-14,-22}})));
    Modelica.Blocks.Interfaces.BooleanOutput S1
      annotation (Placement(transformation(extent={{90,48},{110,68}})));
    Modelica.Blocks.Interfaces.BooleanOutput S2
      annotation (Placement(transformation(extent={{90,2},{110,22}})));
    Modelica.Blocks.Interfaces.BooleanOutput S3
      annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
  equation
    connect(Bat1, greaterEqualThreshold.u) annotation (Line(points={{-99,-1},{
            -68,-1},{-68,56},{-36,56}}, color={0,0,127}));
    connect(greaterEqualThreshold.y, S1) annotation (Line(points={{-13,56},{86,
            56},{86,58},{100,58}}, color={255,0,255}));
    connect(lessThreshold.y, S2) annotation (Line(points={{-13,-32},{26,-32},{
            26,12},{100,12}}, color={255,0,255}));
    connect(S3, S2) annotation (Line(points={{100,-40},{42,-40},{42,-16},{26,
            -16},{26,12},{100,12}}, color={255,0,255}));
    connect(lessThreshold.u, greaterEqualThreshold.u) annotation (Line(points={
            {-36,-32},{-68,-32},{-68,56},{-36,56}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Controller;

  model Main_III
    Controller controller
      annotation (Placement(transformation(extent={{-72,32},{-52,52}})));
    Modelica.Mechanics.Rotational.Components.Gearbox gearbox(ratio=0.2)
      annotation (Placement(transformation(extent={{14,-46},{34,-26}})));
    Modelica.Mechanics.Translational.Components.Vehicle vehicle(
      m=1700,
      J=2,
      R=0.2032,
      v(start=13.888888888889, fixed=true))
      annotation (Placement(transformation(extent={{52,-44},{72,-24}})));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{90,-46},{110,-26}})));
    Bat_Model_IV bat_Model_IV
      annotation (Placement(transformation(extent={{-64,-24},{-44,-4}})));
  equation
    connect(gearbox.flange_b, vehicle.flangeR) annotation (Line(points={{34,-36},
            {46,-36},{46,-34},{52,-34}}, color={0,0,0}));
    connect(vehicle.flangeT, speedSensor.flange) annotation (Line(points={{72,
            -34},{84,-34},{84,-36},{90,-36}}, color={0,127,0}));
    connect(bat_Model_IV.Bat1, controller.Bat1) annotation (Line(points={{-64.6,
            -16.4},{-82,-16.4},{-82,41.9},{-71.9,41.9}}, color={0,0,127}));
    connect(controller.S3, bat_Model_IV.s3) annotation (Line(points={{-52,38},{
            -40,38},{-40,12},{-70,12},{-70,-18.4},{-66,-18.4}}, color={255,0,
            255}));
    connect(controller.S2, bat_Model_IV.s2) annotation (Line(points={{-52,43.2},
            {-40,43.2},{-40,42},{-38,42},{-38,0},{-74,0},{-74,-18.4},{-66,-18.4}},
          color={255,0,255}));
    connect(controller.S1, bat_Model_IV.s1) annotation (Line(points={{-52,47.8},
            {-44,47.8},{-44,48},{-34,48},{-34,-15.2},{-44.8,-15.2}}, color={255,
            0,255}));
    connect(bat_Model_IV.flange_a, gearbox.flange_a) annotation (Line(points={{
            -44,-18.8},{-30,-18.8},{-30,-18},{14,-18},{14,-36}}, color={0,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Main_III;

  model Bat_Model_IV
    Modelica.Electrical.Analog.Basic.RotationalEMF MG1(k=1.38) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={26,70})));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack Battery1(
      Ns=14,
      Np=1,
      cellData(
        Qnom(displayUnit="A.h") = 23400,
        OCVmax=7.2,
        SOCmax=1,
        Ri=0.15),
      SOC(fixed=true, start=0.7))
                       annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-70,30})));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_SeriesExcited Motor1
      annotation (Placement(transformation(extent={{-66,-54},{-46,-34}})));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack Battery2(
      Ns=14,
      Np=1,
      cellData(
        Qnom(displayUnit="A.h") = 23400,
        OCVmax=7.2,
        Ri=0.15),
      SOC(fixed=true, start=0.7))
                       annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={66,24})));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_SeriesExcited Motor2
      annotation (Placement(transformation(extent={{-20,60},{0,80}})));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-114,28})));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch2
      annotation (Placement(transformation(extent={{-68,56},{-48,76}})));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch3 annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={20,-8})));
    Modelica.Electrical.Analog.Ideal.IdealClosingSwitch switch1 annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-84,-12})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-28,6},{-8,26}})));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor1 annotation (
       Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={120,28})));
    Modelica.Blocks.Interfaces.RealOutput Bat1 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-106,-24})));
    Modelica.Blocks.Interfaces.BooleanInput s1 annotation (Placement(
          transformation(
          extent={{-12,-12},{12,12}},
          rotation=180,
          origin={108,-10}), iconTransformation(extent={{84,-20},{100,-4}})));
    Modelica.Blocks.Interfaces.BooleanInput s2 annotation (Placement(
          transformation(
          extent={{-12,-12},{12,12}},
          rotation=0,
          origin={-106,72}), iconTransformation(extent={{-128,-52},{-112,-36}})));
    Modelica.Blocks.Interfaces.BooleanInput s3 annotation (Placement(
          transformation(
          extent={{-12,-12},{12,12}},
          rotation=180,
          origin={108,78}), iconTransformation(extent={{-128,-52},{-112,-36}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
      annotation (Placement(transformation(extent={{90,-58},{110,-38}})));
  equation
    connect(Motor2.pin_an, Motor2.pin_ep) annotation (Line(points={{-16,80},{
            -16,84},{-24,84},{-24,76},{-20,76}}, color={0,0,255}));
    connect(Motor1.pin_an, Motor1.pin_ep) annotation (Line(points={{-62,-34},{
            -62,-28},{-70,-28},{-70,-38},{-66,-38}}, color={0,0,255}));
    connect(Motor2.flange, MG1.flange)
      annotation (Line(points={{0,70},{16,70}}, color={0,0,0}));
    connect(MG1.p, Battery1.p) annotation (Line(points={{26,60},{26,42},{-70,42},
            {-70,40}}, color={0,0,255}));
    connect(MG1.p, Battery2.p) annotation (Line(points={{26,60},{26,40},{66,40},
            {66,34}}, color={0,0,255}));
    connect(MG1.n, Battery2.n) annotation (Line(points={{26,80},{52,80},{52,88},
            {98,88},{98,6},{66,6},{66,14}}, color={0,0,255}));
    connect(ground.p, Battery1.n) annotation (Line(points={{-18,26},{-22,26},{
            -22,12},{-62,12},{-62,20},{-70,20}}, color={0,0,255}));
    connect(ground.p, Battery2.n) annotation (Line(points={{-18,26},{-18,30},{
            46,30},{46,4},{66,4},{66,14}}, color={0,0,255}));
    connect(voltageSensor.p, Battery1.p) annotation (Line(points={{-114,38},{
            -114,40},{-70,40}}, color={0,0,255}));
    connect(voltageSensor.n, Battery1.n) annotation (Line(points={{-114,18},{
            -92,18},{-92,20},{-70,20}}, color={0,0,255}));
    connect(voltageSensor.v, Bat1) annotation (Line(points={{-125,28},{-132,28},
            {-132,-24},{-106,-24}}, color={0,0,127}));
    connect(s3, switch3.control) annotation (Line(points={{108,78},{42,78},{42,
            10},{20,10},{20,4}}, color={255,0,255}));
    connect(Motor1.flange, flange_a) annotation (Line(points={{-46,-44},{84,-44},
            {84,-48},{100,-48}}, color={0,0,0}));
    connect(s1, switch1.control) annotation (Line(points={{108,-10},{44,-10},{
            44,12},{-2,12},{-2,2},{-78,2},{-78,6},{-84,6},{-84,0}}, color={255,
            0,255}));
    connect(s2, switch2.control) annotation (Line(points={{-106,72},{-72,72},{
            -72,84},{-58,84},{-58,78}}, color={255,0,255}));
    connect(voltageSensor1.p, Battery2.p) annotation (Line(points={{120,18},{
            122,18},{122,12},{82,12},{82,38},{66,38},{66,34}}, color={0,0,255}));
    connect(voltageSensor1.n, Battery2.n) annotation (Line(points={{120,38},{96,
            38},{96,14},{66,14}}, color={0,0,255}));
    connect(Motor1.pin_en, Battery1.n) annotation (Line(points={{-66,-50},{-88,
            -50},{-88,-20},{-62,-20},{-62,6},{-70,6},{-70,20}}, color={0,0,255}));
    connect(Motor1.pin_ap, switch1.p) annotation (Line(points={{-50,-34},{-50,
            -16},{-94,-16},{-94,-12}}, color={0,0,255}));
    connect(switch1.n, Battery1.p) annotation (Line(points={{-74,-12},{-74,12},
            {-94,12},{-94,40},{-70,40}}, color={0,0,255}));
    connect(Motor2.pin_en, Battery1.n) annotation (Line(points={{-20,64},{-36,
            64},{-36,20},{-70,20}}, color={0,0,255}));
    connect(Motor2.pin_ap, switch2.p) annotation (Line(points={{-4,80},{-6,80},
            {-6,88},{-68,88},{-68,66}}, color={0,0,255}));
    connect(switch2.n, Battery1.p) annotation (Line(points={{-48,66},{-48,50},{
            -70,50},{-70,40}}, color={0,0,255}));
    connect(Motor1.pin_en, Battery2.n) annotation (Line(points={{-66,-50},{-66,
            -78},{66,-78},{66,14}}, color={0,0,255}));
    connect(Motor1.pin_ap, switch3.p) annotation (Line(points={{-50,-34},{2,-34},
            {2,-8},{10,-8}}, color={0,0,255}));
    connect(switch3.n, Battery2.p) annotation (Line(points={{30,-8},{44,-8},{44,
            -6},{56,-6},{56,34},{66,34}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Bat_Model_IV;

  model Single_Battery_model

    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_SeriesExcited Motor1
      annotation (Placement(transformation(extent={{-60,-42},{-40,-22}})));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack Battery1(
      Ns=28,
      Np=1,
      cellData(
        Qnom(displayUnit="A.h") = 23400,
        OCVmax=7.2,
        SOCmax=1,
        Ri=0.15),
      SOC(fixed=true, start=0.7))
                        annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-26,44})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{4,-44},{24,-24}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{18,10},{38,30}})));
    Modelica.Mechanics.Rotational.Components.Gearbox gearbox(ratio=0.2)
      annotation (Placement(transformation(extent={{-44,-80},{-24,-60}})));
    Modelica.Mechanics.Translational.Components.Vehicle vehicle(
      m=1700,
      J=2,
      R=0.2032,
      v(start=13.888888888889, fixed=true))
      annotation (Placement(transformation(extent={{-6,-78},{14,-58}})));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor1
      annotation (Placement(transformation(extent={{32,-80},{52,-60}})));
  equation
    connect(Motor1.pin_an, Motor1.pin_ep) annotation (Line(points={{-56,-22},{-58,
            -22},{-58,-16},{-60,-16},{-60,-26}}, color={0,0,255}));
    connect(speedSensor.flange, Motor1.flange) annotation (Line(points={{4,-34},{
            -34,-34},{-34,-32},{-40,-32}}, color={0,0,0}));
    connect(ground.p, Battery1.n) annotation (Line(points={{28,30},{10,30},{10,40},
            {-8,40},{-8,44},{-16,44}}, color={0,0,255}));
    connect(Motor1.pin_en, Battery1.n) annotation (Line(points={{-60,-38},{-74,
            -38},{-74,-40},{-84,-40},{-84,0},{-16,0},{-16,44}}, color={0,0,255}));
    connect(gearbox.flange_b, vehicle.flangeR) annotation (Line(points={{-24,-70},
            {-12,-70},{-12,-68},{-6,-68}}, color={0,0,0}));
    connect(vehicle.flangeT, speedSensor1.flange) annotation (Line(points={{14,
            -68},{26,-68},{26,-70},{32,-70}}, color={0,127,0}));
    connect(gearbox.flange_a, Motor1.flange) annotation (Line(points={{-44,-70},{
            -44,-40},{-50,-40},{-50,-32},{-40,-32}}, color={0,0,0}));
    connect(Battery1.p, Motor1.pin_ap) annotation (Line(points={{-36,44},{-64,
            44},{-64,-12},{-44,-12},{-44,-22}}, color={0,0,255}));
    annotation ();
  end Single_Battery_model;
  annotation (uses(Modelica(version="4.0.0")));
end Project_IV_689;
