# Hysteresis vs. PI control of a heating element (STM32G431KB)
Are you interested in feedback control systems? Do you want to play with a physical plant in the loop? Yes? Good! I've got something for you. One of the cheapest second(-ish) order plants is a temperature sensor attached to a resistor fed by a switching converter. You can build such a plant for under $3 using e.g. a DS18B20 or an MCP9808 temperature sensor and a transistor plus a freewheeling/flyback diode (the latter is there to make it 100% correct since physical resistors introduce not only resistance into the circuit[^1]).

[^1]: In this particular experiment the parasitic/stray inductance of the resistor would not hurt the beefy transistor (overkill is my middle name). Nevertheless, we are here to learn good practices, and thus we keep the second transistor to have its diode present in the circuit. We will learn about the indispensability of such diodes in the presence of inductive currents when playing, inter alia, with electric motors. I'm going to demonstrate to you that even a toy DC motor (3 V, 1 A) can produce, i.e. self-induce, easily 100 V if not accompanied with the freewheeling diode.

We will experiment with several ideas along the way, including:
* a hysteresis controller,
* a PI controller with an anti-windup algorithm (the D part with low-pass filtering is intentionally left for you to add),
* the Serial Wire Viewer[^2] and
* the STM32CubeMonitor.

[^2]: Not all Nucleo boards will let you do that. Check the SWO pin. For example, the [Nucleo-G431KB](/ufnalski/pid_temperature_mcp9808_g431kb/blob/main/Assets/Images/swv_support_nucleo_g431kb.JPG) board vs. the [Nucleo-L432KC](/ufnalski/pid_temperature_mcp9808_g431kb/blob/main/Assets/Images/swv_no_support_nucleo_l432kc.JPG) board.

![MCP9808 and hysteresis controller in action](/Assets/Images/mcp9809_and_hysteresis_controller_in_action.jpg)

![Hysteresis controller in STM32CubeMonitor](/Assets/Images/hysteresis_temperature_control_cube_monitor.JPG)

Visit [/Assets/Images/](/Assets/Images/) for more snapshots.

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Tutorials and libraries
## PID control
* [What Is PID Control? | Understanding PID Control, Part 1](https://www.youtube.com/watch?v=wkfEZmsQqiA) (MatchWorks, Brian Douglas)
* [Anti-windup for PID control | Understanding PID Control, Part 2](https://www.youtube.com/watch?v=NVLXCwc8HzM) (MatchWorks, Brian Douglas)
* [Noise Filtering in PID Control | Understanding PID Control, Part 3](https://www.youtube.com/watch?v=7dUVdrs1e18) (MatchWorks, Brian Douglas)
* [A PID Tuning Guide | Understanding PID Control, Part 4](https://www.youtube.com/watch?v=sFOEsA0Irjs) (MatchWorks, Brian Douglas)
* [3 Ways to Build a Model for Control System Design | Understanding PID Control, Part 5](https://www.youtube.com/watch?v=qhIjIu-Zk10) (MatchWorks, Brian Douglas)
* [Manual and Automatic PID Tuning Methods | Understanding PID Control, Part 6](https://www.youtube.com/watch?v=qj8vTO1eIHo) (MatchWorks, Brian Douglas)
* [Important PID Concepts | Understanding PID Control, Part 7](https://www.youtube.com/watch?v=tbgV6caAVcs) (MatchWorks, Brian Douglas)
* [The Map of Control Theory](https://engineeringmedia.com/maps) (Brian Douglas)
* [Practical PID Control](https://link.springer.com/book/10.1007/1-84628-586-0) (Antonio Visioli)
* [Proportional-Integral-Derivative (PID) Controllers](https://www.mathworks.com/help/control/ug/proportional-integral-derivative-pid-controllers.html) (MathWorks)
* [Discrete-Time Proportional-Integral-Derivative (PID) Controllers](https://www.mathworks.com/help/control/ug/discrete-time-proportional-integral-derivative-pid-controller.html) (MathWorks)
* [Two-Degree-of-Freedom PID Controllers](https://www.mathworks.com/help/control/ug/two-degree-of-freedom-2-dof-pid-controllers.html) (MathWorks)
* [Integral (Reset) Windup, Jacketing Logic and the Velocity PI Form](https://controlguru.com/integral-reset-windup-jacketing-logic-and-the-velocity-pi-form/) (Control Guru)
* [Anti-Windup Control Using PID Controller Block](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html) (MathWorks)
* [PLECS User Manual](https://plexim.com/sites/default/files/plecsmanual.pdf) (Plexim)
## Hysteresis control
* [Hysteresis Control](https://www.sciencedirect.com/topics/engineering/hysteresis-control) (Elsevier ScienceDirect)
## Serial Wire Viewer (STM32CubeIDE)
* [Serial Wire Viewer (SWD + SWO) - fast & native Debugging](https://www.codeinsideout.com/blog/stm32/swv/) (Code Inside Out)
* [Using Serial Wire Trace with CubeIDE (aka a new level of debugging)](https://embedblog.eu/?p=673) (EmbedBlog)
* [STM32CubeIDE Advanced Debug Features: Part 1](https://www.youtube.com/watch?v=4wT9NhlcWP4) (STMicroelectronics)
* [STM32CubeIDE Advanced Debug Features: Part 2](https://www.youtube.com/watch?v=Zqwq9nzTNF8) (STMicroelectronics)
* [STM32CubeIDE Advanced Debug Features: Part 3](https://www.youtube.com/watch?v=Eg_GLvLHM1o) (STMicroelectronics)
* [STM32CubeIDE Advanced Debug Features: Part 4](https://www.youtube.com/watch?v=-X8tndfqTu8) (STMicroelectronics)
* [STM32CubeIDE Advanced Debug Features: Part 5](https://www.youtube.com/watch?v=BZKzwn5w1D8) (STMicroelectronics)
* [Using Printf Debugging, LIVE expressions and SWV Trace in CubeIDE || STM32 || ITM || SWV](https://www.youtube.com/watch?v=sPzQ5CniWtw) (ControllersTech)
## STM32CubeMonitor
* [Introduction to STM32CubeMonitor - 1 Introduction](https://www.youtube.com/watch?v=YWjvHhgqvm4) (STMicroelectronics)
* [Introduction to STM32CubeMonitor - 2 Basic project, global variables](https://www.youtube.com/watch?v=isa1XM5Eeek) (STMicroelectronics)
* [Introduction to STM32CubeMonitor - 3 Access to registers](https://www.youtube.com/watch?v=taiRu-wZmmY) (STMicroelectronics)
* [Introduction to STM32CubeMonitor - 4 Remote access](https://www.youtube.com/watch?v=75eSX60SJOM) (STMicroelectronics)
* [STM32CubeMonitor in practice - How to do a remote monitoring](https://www.youtube.com/watch?v=jlCLrg-yxN8) (STMicroelectronics)
* [STM32CubeMonitor in practice - How to configure ST-Link in shared mode](https://www.youtube.com/watch?v=VhhMxBEFBRQ) (STMicroelectronics)
* [STM32Cube tools in practice - STM32CubeMonitorPower in practice](https://www.youtube.com/watch?v=COOi_BiPE5U) (STMicroelectronics)
* [How to perform RF functional tests on STM32WL - 1 Introduction and theory](https://www.youtube.com/watch?v=lrFiwxmBtkA) (STMicroelectronics)
* [How to perform RF functional tests on STM32WL - 2 STM32CubeMonitor lab](https://www.youtube.com/watch?v=NmLRczlMz18) (STMicroelectronics)
* [Exemplary dashboard for the project at hand](/Assets/CubeMonitor/hysteresis_vs_pi_controller_cube_monitor.json)
## MCP9808
* [Adafruit MCP9808 Library](https://github.com/adafruit/Adafruit_MCP9808_Library)
* [Temperature measurement? Never so easy with STM32 and MCP9808!](https://embeddedespresso.com/temperature-measurement-never-so-easy-with-stm32-and-mcp9808/) (Embedded Espresso)
* [Seeed MCP9808 Library](https://github.com/Seeed-Studio/Grove_Temperature_sensor_MCP9808)
## sprintf()
* [An Embedded-friendly printf Implementation](https://embeddedartistry.com/blog/2019/11/06/an-embedded-friendly-printf-implementation/) (Embedded Artistry)
* [A printf / sprintf Implementation for Embedded Systems](https://github.com/mpaland/printf) (Marco Paland)

# Exemplary hardware
* [Grove MCP9808 temperature sensor (I2C)](https://botland.store/grove-weather-sensors/15183-grove-mcp9808-temperature-sensor-i2c-5903351246828.html) (Seeed Studio)
* [Fermion: MCP9808 High Accuracy I2C Temperature Sensor](https://wiki.dfrobot.com/Fermion_MCP9808_High_Accuracy_I2C_Temperature_Sensor_SKU_SEN0435) (DFRobot)
* [Transistor N-MOSFET IRL540NPBF (suitable for 3.3 V)](https://botland.store/n-mosfet/127-transistor-n-mosfet-irl540npbf-tht-5pcs-5904422308087.html) (Botland)

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_popularyzacji_matematyki/Dzien_Popularyzacji_Matematyki_2024.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [Max Imagination](https://www.youtube.com/@MaxImagination), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Control engineering - do try this at home :sunglasses:

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
