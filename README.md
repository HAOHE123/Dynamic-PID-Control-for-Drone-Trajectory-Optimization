### Project Title: Dynamic PID Control for Drone Trajectory Optimization

#### Overview
This repository contains a MATLAB simulation project designed to optimize drone trajectories using a combination of discrete and continuous hybrid optimization methods. The core objective is to efficiently control drone paths over Gaussian Mixture Model (GMM) distributions, employing Proportional-Integral-Derivative (PID) controllers with dynamic gain adjustments to enhance performance near optimal trajectory points.

#### Features
- **Hybrid Optimization Techniques**: Implements both discrete and continuous optimization strategies to manage the drone's flight path over complex distributions.
- **Dynamic PID Control**: Utilizes PID controllers with gains dynamically adjusted based on the drone's proximity to its target, effectively minimizing overshooting and improving stabilization.
- **Error Handling and Visualization**: Provides detailed visual representations of drone trajectories, error metrics, and Gaussian distributions to evaluate performance and accuracy.
- **Mathematical Rigor**: Ensures the correct implementation of mathematical operations, such as handling and validating the positive-definiteness of covariance matrices in GMMs.

#### Simulation Details
The simulation environment in MATLAB showcases:
- Drone movement controlled through predefined paths influenced by Gaussian Mixture Models.
- Comparative analysis of discrete vs. continuous methods, highlighting the impact of each on drone trajectory smoothness and control precision.
- Dynamic adjustment of PID parameters in response to trajectory errors, particularly focusing on rapid adjustments near optimal paths to prevent overshoot.

#### Getting Started
1. **Prerequisites**: Ensure you have MATLAB installed with the necessary toolboxes for handling matrix operations and plotting.
2. **Installation**: Clone the repository and open the project in MATLAB.
3. **Running Simulations**: Execute the main script to start the simulations. Adjust PID settings in the script to experiment with different control dynamics.

#### Contributing
Contributions to the project are welcome! You can contribute by:
- Enhancing the PID control algorithms.
- Expanding the types of optimization methods tested.
- Improving the visualization features to include real-time simulation feedback.
