
# Lunar Landing Simulation

This is a 2D Lunar Landing Simulation that uses relative control and PID algorithms to safely land a spacecraft on the Moon. The simulation visualizes the descent, logs data in real time, and exports the full simulation data to CSV and Excel files.

## Features

- **Dynamic PID Control:**  
  Vertical speed is controlled via a PID controller. The desired vertical speed decreases as altitude decreases (from 25 m/s at the start to 3 m/s near the ground).

- **Angle Adjustment:**  
  The spacecraft’s target angle is dynamically reduced from 60° (at high altitude) toward 0° as the spacecraft approaches the landing target. When the spacecraft is near the target horizontally, the target angle is forced to 0°.

- **Enhanced Physics:**  
  The simulation applies extra braking on horizontal speed when below a set altitude threshold (3000 m) to ensure a safe landing.

- **Data Export:**  
  At the end of the simulation, all logged data (time, altitude, vertical & horizontal speeds, angle, fuel, throttle, etc.) are exported to CSV and Excel files in an `output` folder.

## Requirements

- Python 3.6 or later
- [numpy](https://numpy.org/)
- [matplotlib](https://matplotlib.org/)
- [pandas](https://pandas.pydata.org/)
- [openpyxl](https://openpyxl.readthedocs.io/)

## Installation Instructions

### Windows

1. **Install Python:**
   - Download and install the latest version of Python from [python.org](https://www.python.org/downloads/windows/).  
   - During installation, check the option **"Add Python to PATH"**.

2. **Clone or Download the Repository:**
   - Clone this repository using Git or download the ZIP and extract it.
   - Open a Command Prompt (cmd) and navigate to the project directory.

3. **Create a Virtual Environment (Optional but Recommended):**
   ```bash
   python -m venv venv
   venv\Scripts\activate
   ```

4. **Install the Required Packages:**
   ```bash
   pip install numpy matplotlib pandas openpyxl
   ```

5. **Run the Simulation:**
   ```bash
   python landing_simulation.py
   ```

### macOS

1. **Install Python:**
   - Download and install the latest version of Python from [python.org](https://www.python.org/downloads/mac-osx/).  
   - או להשתמש ב-Homebrew:
     ```bash
     brew install python
     ```

2. **Clone or Download the Repository:**
   - Clone the repository using Git or download the ZIP file and extract it.
   - Open the Terminal and navigate to the project directory.

3. **Create a Virtual Environment (Optional but Recommended):**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

4. **Install the Required Packages:**
   ```bash
   pip install numpy matplotlib pandas openpyxl
   ```

5. **Run the Simulation:**
   ```bash
   python landing_simulation.py
   ```

## Usage

1. **Start the Simulation:**  
   When you run the program, a window opens displaying the simulation.  
   Click on the **"Start/Reset"** button at the bottom of the window to begin.

2. **Simulation Process:**  
   - The simulation visualizes the descent of the spacecraft, adjusting the throttle and angle according to the PID controllers.
   - The data is logged during the simulation (time, altitude, speeds, angle, fuel, throttle, etc.).

3. **Data Export:**  
   At the end of the simulation (when the spacecraft lands or crashes), the simulation exports the full data to:
   - `output/full_sim_data.csv`
   - `output/full_sim_data.xlsx`

4. **Interpreting the Output:**  
   - The **info text** on the simulation window shows the current state.
   - A message appears at the end indicating whether the landing was **SAFE** or a **CRASH**.
   - The exported data files contain detailed logged data for further analysis.

## Troubleshooting

- **Missing Dependencies:**  
  If you encounter errors, ensure that all required packages are installed correctly.
  
- **Background Image:**  
  If `space.jpg` is not found, a plain background will be used.

- **PID Tuning:**  
  If the simulation does not behave as expected, you may need to adjust the PID constants in the code.

