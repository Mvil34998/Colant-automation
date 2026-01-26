Wokwi CLI Installation
The CLI allows you to run Wokwi simulations from your terminal, and integrate them with your CI system. We recommend using the Wokwi for VS Code extension for local development, and the CLI for running your tests on CI.

Both the CLI and the VS Code extension use the same project configuration files (wokwi.toml and diagram.json), so you can use the VS Code extension to create and test your project, and then use the CLI to run it on CI.

To install the Wokwi CLI, run the following command:

curl -L https://wokwi.com/ci/install.sh | sh

On Windows, you can use the following command in PowerShell:

iwr https://wokwi.com/ci/install.ps1 -useb | iex

Alternatively, you can download the CLI directly from the GitHub Releases page. Rename the file to wokwi-cli (or wokwi-cli.exe on Windows), make it executable (chmod +x wokwi-cli on Linux/Mac), and move it to a directory in your PATH (e.g. /usr/local/bin on Linux/Mac).