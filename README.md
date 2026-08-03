ELM327 Subaru Scanner
======================

What it does
------------
- Connects to a USB ELM327 on Windows
- Reads stored, pending, and permanent DTCs
- Reads common freeze-frame values
- Displays live engine data
- Logs live data to CSV
- Works with ISO 9141-2 through automatic protocol detection

Installation
------------
1. Install Python 3 from python.org.
   During installation, enable "Add Python to PATH".

2. Open Command Prompt and run:

   py -m pip install pyserial

3. Connect the ELM327 to the laptop and car.

4. Turn the ignition ON.

5. Close PuTTY and every other OBD program.

6. Run:

   py elm327_subaru_scanner.py

Recommended settings
--------------------
Port: COM4
Baud rate: 38400

Notes
-----
- Start the engine before monitoring RPM, MAF, fuel trims, and O2 values.
- Keep the Subaru green test-mode connectors disconnected.
- Do not clear DTCs before saving freeze-frame information.
- Generic OBD-II does not expose every Subaru-specific parameter.
- ISO 9141-2 is slow. A complete monitoring cycle can take several seconds.
