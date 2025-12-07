# Quickstart Guide: Physical AI & Humanoid Robotics Book

This guide provides step-by-step instructions to set up your development environment, run simulations, and deploy the "Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World" Docusaurus book.

---

## 1. Prerequisites

Before you begin, ensure you have the following installed:

-   **Git**: For cloning the repository.
    ```bash
    # Debian/Ubuntu
    sudo apt update && sudo apt install git
    # macOS
    brew install git
    # Windows (install Git for Windows)
        ```
-   **Node.js & npm (or Yarn)**: Required for Docusaurus.
    -   Node.js (LTS version, e.g., v18.x or v20.x)
    -   npm (comes with Node.js) or Yarn (install with `npm install -g yarn`)
-   **Python 3.8+**: For ROS 2, VLA, and simulation scripts.
    -   Recommended to use `pyenv` or `conda` for environment management.
-   **Docker & Docker Compose**: For containerized environments (optional, but recommended for ROS 2).
-   **Development Tools**: C++ compiler, CMake, pip (Python package installer).

---

## 2. Docusaurus Environment Setup

### 2.1. Clone the Book Repository

First, clone the book's repository from GitHub:

```bash
git clone https://github.com/your-org/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book
```

### 2.2. Install Docusaurus Dependencies

Navigate to the book's root directory and install the Node.js dependencies:

```bash
npm install
# or
yarn install
```

### 2.3. Run the Docusaurus Development Server

Start the local development server to preview the book:

```bash
npm start
# or
yarn start
```

Your browser should automatically open to `http://localhost:3000`. Any changes you make to the Markdown files will hot-reload.

---

## 3. Robotics Environment Setup

### 3.1. Install ROS 2 (Humble Hawksbill recommended)

Follow the official ROS 2 documentation for your operating system:

-   [ROS 2 Humble Hawksbill Installation](https://docs.ros.org/en/humble/Installation.html)

Verify your installation:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### 3.2. Install Simulation Environment

#### Option A: Gazebo (Garden or Fortress)

Follow the official Gazebo documentation for installation:

-   [Gazebo Installation Guide](https://gazebosim.org/docs/garden/install_ubuntu)

#### Option B: Unity Robotics Hub

1.  Install Unity Hub and Unity Editor (LTS version, e.g., 2022.3).
2.  Clone the Unity Robotics Hub:
    ```bash
    git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
    ```
3.  Open the cloned project in Unity Editor and follow the setup instructions for ROS-Unity integration.

#### Option C: NVIDIA Isaac Sim

1.  Request access and download NVIDIA Omniverse Launcher.
2.  Install Isaac Sim via the Omniverse Launcher.
3.  Refer to the [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html) for setup and ROS 2 bridge configuration.

---

## 4. Running Sample Simulations & Demos

### 4.1. ROS 2 Humanoid Package (Example)

Assuming a ROS 2 package for the humanoid robot (e.g., `humanoid_description`, `humanoid_control`):

```bash
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash
# Build your workspace (if necessary)
cd ~/ros2_ws
colcon build
source install/setup.bash
# Launch the humanoid in Gazebo
ros2 launch humanoid_description display.launch.py # to see URDF
ros2 launch humanoid_control humanoid_gazebo.launch.py # to spawn in Gazebo
```

### 4.2. VLA Demo (Whisper + GPT + Action Simulation)

This demonstration integrates voice command, language understanding, and simulated robot actions.

1.  **Install Python Dependencies**: Ensure you have `openai`, `whisper`, `transformers`, `numpy`, `scipy`.
    ```bash
pip install -r requirements.txt # In the VLA demo directory
    ```
2.  **Configure API Keys**: Set your OpenAI API key as an environment variable (`OPENAI_API_KEY`).
3.  **Run the VLA script**:
    ```bash
    python vla_demo_script.py # This script will simulate voice input and robot action sequences.
    ```
    The script will prompt you to speak a command or use a predefined text input.

---

## 5. Build and Deploy Docusaurus

### 5.1. Build for Production

To generate static HTML, CSS, and JavaScript files for deployment:

```bash
npm run build
# or
yarn build
```

The build output will be in the `build/` directory.

### 5.2. Deploy to GitHub Pages

Configure `docusaurus.config.js` with your GitHub Pages settings:

```javascript
// docusaurus.config.js
module.exports = {
  // ... other configs
  projectName: 'physical-ai-humanoid-robotics-book', // Usually your repo name
  organizationName: 'your-github-username-or-org', // Usually your GitHub org/user name.
  baseUrl: '/physical-ai-humanoid-robotics-book/', // Base URL for GitHub Pages
  // ...
};
```

Then, deploy using the `deploy` command:

```bash
npm run deploy
# or
yarn deploy
```

This will push the `build/` directory content to the `gh-pages` branch of your repository, making your book live at `https://your-github-username-or-org.github.io/physical-ai-humanoid-robotics-book/`.
