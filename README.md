ELM327 OBD-II Scanner (Subaru + Ford)
=====================================

What it does
------------
- Connects to a USB ELM327 on Windows
- Ford and other CAN vehicles (ISO 15765-4, roughly 2008 onwards)
- Subaru and other K-line vehicles (ISO 9141-2 / KWP2000)
- Reads stored, pending and permanent DTCs per module, with descriptions
- Readiness monitors and MIL status
- Freeze frame, including the code that triggered it
- Ford module scan: ABS, airbag, body, cluster, steering, … (HS-CAN)
- Live data dashboard and table, logged to CSV for the drive-log analyzer
- On CAN, up to 6 values are read per request, so logging is several times faster

Installation
------------
1. Install Python 3 from python.org.
   During installation, enable "Add Python to PATH".

2. Open Command Prompt and run:

   py -m pip install pyserial

3. Connect the ELM327 to the laptop and car.

4. Turn the ignition ON.

5. Close PuTTY and every other OBD program.

6. Double-click RUN_OBD_SCANNER.bat, or run:

   py OBD.py

Recommended settings
--------------------
Port: COM4
Baud rate: 38400
Vehicle: "Ford (CAN)" for Fords, "Subaru / K-line" for older Subarus,
"Auto detect" for anything else.

Trouble codes tab
-----------------
- Read codes: MIL, readiness monitors and stored / pending / permanent codes
  from every emission-related module (engine, transmission, …)
- Read freeze frame: the snapshot saved when the MIL came on
- Ford module scan: reads every module on the high-speed CAN bus.
  Modules on Ford's medium-speed CAN (MS-CAN) cannot be reached by a
  standard ELM327 and show as "no response".
- Clear codes / Clear module codes: ignition ON, engine OFF
- Save report: writes a text file with all codes and details

Unknown codes are described by their category. You can add your own
descriptions in dtc_custom.csv next to OBD.py, one per line:

   P1234;Description of the code

Notes
-----
- Start the engine before monitoring RPM, MAF, fuel trims, and O2 values.
- Keep the Subaru green test-mode connectors disconnected.
- Do not clear DTCs before saving freeze-frame information.
- Generic OBD-II does not expose every manufacturer-specific parameter.
- ISO 9141-2 is slow. A complete monitoring cycle can take several seconds.

Tests
-----
The tests use a simulated adapter and need no car:

   py -m unittest discover tests
