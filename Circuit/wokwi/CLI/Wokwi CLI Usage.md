Wokwi CLI Usage
Create an API token on the Wokwi CI Dashboard. Set the WOKWI_CLI_TOKEN environment variable to the token value.

If you haven't set up your project for Wokwi yet, you can use the init command to configure your project for Wokwi. Run the following command in your project's root directory:

wokwi-cli init

This command will ask you a few questions and will automatically generate wokwi.toml and diagram.json files for your project.

To run the simulation, use the following command:

wokwi-cli <your-project-directory>

The CLI will start the simulation and display the serial output. It will automatically exit after 30 seconds.

CLI Options
You can use the following options to customize the CLI behavior:

Configuration
--elf <path> - ELF file to simulate (default: read from wokwi.toml)
--diagram-file <path> - Path to the diagram.json file, relative to project root (default: diagram.json)
--interactive - Redirect stdin to the simulated serial port
--serial-log-file <path> - Save the serial monitor output to the given file
--timeout <number> - Timeout in simulation milliseconds (default: 30000)
--timeout-exit-code <number> - Process exit code when timeout is reached (default: 42)
Automation
--expect-text <string> - Expect the given text in the output
--fail-text <string> - Fail if the given text is found in the output
--scenario <path> - Path to an automation scenario file, relative to project root
--screenshot-part <string> - Take a screenshot of the given part id (from diagram.json)
--screenshot-time <number> - Time in simulation milliseconds to take the screenshot
--screenshot-file <string> - File name to save the screenshot to (default: screenshot.png)
--vcd-file <path> - Export Logic Analyzer data to a VCD file
General
--help, -h - Prints help information and exit
--quiet, -q - Quiet: do not print version or status messages