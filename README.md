# Narrow Band FLIP for Liquid Simulations

This repository contains the implementation and optimization of the **Narrow Band FLIP (NB-FLIP)** algorithm for liquid simulations, as described in the report ["Narrow Band FLIP for Liquid Simulations: Implementation and Optimization"](Report_Narrow_Band_For_Liquid_Simulation_Baptiste_Boulan.pdf) by Baptiste Boulan, inspired by the paper ["Narrow Band FLIP for Liquid Simulations"](Narrow_Band_FLIP_for_Liquid_Simulations.pdf) by Florian Ferstl.

The project focuses on reducing the computational cost of traditional FLIP simulations by maintaining particles only within a narrow band around the liquid surface, while representing the interior on a regular Eulerian grid. The implementation scales from a 2D CPU prototype to a fully 3D GPU-accelerated simulation, capable of handling up to 18 million particles.

---

## Repository Structure

```
.
├── .vscode/                # VSCode configuration (ignored by git)
├── build/                  # Build directory (ignored by git)
├── frames/                 # Output frames from simulations (ignored by git)
├── illustration/           # Illustrations and figures for the report
├── python/                 # Python scripts for video reconstruction
├── shaders/                # GLSL shaders for GPU rendering
├── src/                    # Main C++ source code
├── videos/                 # Simulation videos
├── .gitignore
├── CMakeLists.txt          # CMake configuration
├── Report_Narrow_Band_For_Liquid_Simulation_Baptiste_Boulan.pdf    # Project report
├── Slides_Narrow_Band_FLIP_for_Liquid_Simulations.pdf              # Slides
├── Narrow_Band_FLIP_for_Liquid_Simulations.pdf                     # The original paper
└── README.md

```


### Contents
- **Original paper**: The reference paper introducing Narrow Band FLIP.
- **Slides**: Presentation slides summarizing the project.
- **Report**: Detailed report on the implementation and results.
- **Simulation videos**: Recorded simulations demonstrating the algorithm in action.

---

## Key Features
- **2D and 3D FLIP/PIC solvers** with GPU acceleration.
- **Conjugate Gradient (CG) solver** for stable pressure projection.
- **Narrow Band optimization**: Particles are maintained only near the liquid surface.
- **Level Set computation** using a Dijkstra-like algorithm for smooth distance fields.
- **Cull and Resample** logic to maintain particle density in the narrow band.
- **Marching Cubes** for surface extraction in large-scale simulations.

---

## Performance
- **2x speedup** in stable scenarios compared to standard FLIP.
- **Real-time simulation** of up to 500,000 particles at 50 FPS (2D/3D).
- **Scalability** to 18 million particles on a 128³ grid.

---

## Compilation and Execution

### Prerequisites
- CMake (3.10+)
- C++17-compatible compiler
- OpenGL and GLSL support

### Build Instructions
1. **Configure the project** (first time only):
```sh
   cmake -B build
``` 

2. **Compile**:
```sh
   cmake --build build --config Debug
``` 

3. **Run**:
```sh
   ./build/Debug/renderze.exe
``` 
