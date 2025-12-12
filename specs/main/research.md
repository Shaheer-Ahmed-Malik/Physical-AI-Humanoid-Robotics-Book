# Research & Decisions

This document outlines the key research tasks and decisions made during the planning phase of the "Physical AI & Humanoid Robotics" book.

## 1. Simulation Stack

- **Decision**: Use a combination of Gazebo for foundational concepts and NVIDIA Isaac Sim for advanced simulation, photorealistic rendering, and AI training.
- **Rationale**: This dual-platform approach provides a gentle learning curve with the widely used Gazebo while introducing readers to the state-of-the-art capabilities of Isaac Sim for AI-specific tasks. This aligns with the goal of bridging the gap between traditional robotics and modern AI.
- **Alternatives Considered**: Using only Gazebo, only Isaac Sim, or Unity. The combined approach was chosen to provide the most comprehensive learning experience.

## 2. Emphasis on Physical vs. Simulated Pipeline

- **Decision**: Primarily focus on the simulated pipeline throughout the book. Optional sections and callouts will explain how the concepts and code can be transferred to physical hardware.
- **Rationale**: This makes the book accessible to the widest possible audience, as it removes the barrier of expensive and complex hardware. Readers can fully engage with the material using only a capable computer.
- **Alternatives Considered**: A hardware-first approach, which would significantly limit the audience and accessibility of the book.

## 3. Depth of Technical Detail

- **Decision**: Provide a high-level, conceptual understanding of ROS 2, Unity, Isaac, and VLA models. The focus will be on the practical application and integration of these technologies. Deep technical details will be avoided, and readers will be directed to official documentation for more in-depth study.
- **Rationale**: This approach keeps the book focused and accessible, preventing it from becoming an exhaustive reference manual for each individual technology. The goal is to teach the "why" and "how" of integration, not the minutiae of each tool.
- **Alternatives Considered**: A deep-dive into each technology, which would result in a much longer, more complex, and less focused book.

## 4. Structure of the VLA Chapter

- **Decision**: Structure the Vision-Language-Action (VLA) chapter with a "perception-first" workflow. The robot will first learn to perceive and understand its environment, and then act upon voice commands.
- **Rationale**: This follows a logical and intuitive learning progression. A robot must first be able to "see" and "understand" before it can meaningfully respond to high-level commands.
- **Alternatives Considered**: A "voice-first" approach, which would be less pedagogical as it would require the robot to act without a clear understanding of its surroundings.

## 5. Level of Mathematics for Kinematics/Dynamics

- **Decision**: Keep the mathematical content to a minimum. The focus will be on building an intuitive understanding of concepts like kinematics and dynamics. Optional sections or appendices will be provided for readers who wish to explore the underlying mathematics in more detail.
- **Rationale**: This aligns with the goal of making the book accessible to a broad audience, including those without a strong mathematical background.
- **Alternatives Considered**: Including detailed mathematical derivations and proofs, which would likely alienate a significant portion of the target audience.

## 6. Cloud-Based vs. Local Workstation Recommendations

- **Decision**: Recommend local workstations as the primary development environment. A dedicated section will discuss cloud-based alternatives (e.g., AWS RoboMaker, NVIDIA Omniverse Cloud) for users who lack powerful local hardware or need to perform large-scale simulation and training.
- **Rationale**: This provides a practical and cost-effective solution for most readers, while still providing options for those with different needs and resources.
- **Alternatives Considered**: A cloud-only recommendation, which would introduce a recurring cost and potential latency issues for readers.
