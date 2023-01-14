# 2023-charged-up

Code for 2023 FRC game Charged Up

## Laptop Setup/Requirements

### Visual Studio redistributable

[vc_redist.x64](https://aka.ms/vs/16/release/vc_redist.x64.exe)

### Python

[v3.11.1 (64-bit)](https://www.python.org/ftp/python/3.11.1/python-3.11.1-amd64.exe)

### VS Code

[VS Code](https://code.visualstudio.com)

### FRC Game Tools

Install FRC Game tools from the downloaded ISO image or download from here: 
[FRC Game Tools](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html)

### FRC Radio Configuration Utility

[FRC Configuration Utility](https://firstfrc.blob.core.windows.net/frc2020/Radio/FRC_Radio_Configuration_20_0_0.zip)

### CTRE Phoenix

[Phoenix Tuner](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/download/v5.30.3.0/CTRE_Phoenix_Framework_v5.30.3.0.exe)


### Install robotpy

- **IMPORTANT: Perform ALL operations in a python virtualenv**

#### Create virtualenv (if not previously done)

Recommend placing the virtualenv in the `mentorbot` repo folder under `.venv` (to keep everything together) however the virtualenv is local to your system and should not be uploaded (ignored in `.gitignore`)

```bash
cd <path-to-mentorbot-repo>
py -3 -m venv ./.venv
```

#### Workflow

1. **Activate virtualenv**
   (Virtualenv activation may differ depending on your operating system and terminal)
   - Git Bash (Windows)
     ```bash
     source <path-to-mentorbot-repo>/.venv/Scripts/activate
     ```
   - normal bash (linux, macOS)
     ```bash
     source <path-to-mentorbot-repo>/.venv/bin/activate
     ```
1. **Update pip and wheel. Old versions of pip can prevent binary wheels from being installed. Installing wheel makes other installs faster**
   (must have internet connection)
   ```bash
   python -m pip install --upgrade pip wheel
   ```
1. **Install / update robotpy**
   (must have internet connection)
   ```bash
   python -m pip install -r requirements.txt
   ```
   (examples: `robotpy`, `robotpy[ctre,navx]`, `robotpy[all]`) (see: [robotpy on pypi](https://pypi.org/project/robotpy/))

1.  **Install black**

   ```
   python -m pip install -U black
   ```