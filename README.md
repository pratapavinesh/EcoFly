# <div align="center">EcoFly: Energy-Efficient FANET Protocol</div>

<div align="center">
  <img src="https://raw.githubusercontent.com/pratapavinesh/EcoFly/main/Images/networkAndEnergyModel.png" alt="EcoFly Logo" width="400"/>
</div>

## <div align="center">Project Overview</div>

**EcoFly** is a FANET (Flying Ad-hoc Network) protocol designed to optimize UAV (Unmanned Aerial Vehicle) communication in disaster scenarios. By utilizing Improved Particle Swarm Optimization (IPSO) algorithms, EcoFly enhances communication reliability and reduces energy consumption. This project was implemented from August 2023 to November 2023 and includes realistic UAV simulations validated through comprehensive testing.

<div align="center">
  <img src="https://raw.githubusercontent.com/pratapavinesh/EcoFly/main/Images/networkAndEnergyModel.png" alt="UAV Communication" width="600"/>
</div>

## <div align="center">Implementation Steps</div>

### Step 1: Defining the Energy Model
- **Assumptions:**
  - Anchor nodes and blind nodes.
  - Energy of blind node, position, and velocity.
- **Description:**
  - This step involves defining the energy model for the UAVs. Anchor nodes are assumed to have fixed positions, while blind nodes are mobile. The energy consumption for positioning and movement is considered.

<div align="center">
  <img src="https://raw.githubusercontent.com/pratapavinesh/EcoFly/main/Images/networkAndEnergyModel.png" alt="Energy Model" width="600"/>
</div>

### Step 2: Position Determination using PSO
- **Challenge:**
  - In a disaster scenario, the actual positions of blind nodes are unknown.
- **Solution:**
  - Use anchor nodes to determine the positions of blind nodes using Particle Swarm Optimization (PSO).

<div align="center">
  <img src="https://raw.githubusercontent.com/pratapavinesh/EcoFly/main/Images/uavPositioning.png" alt="PSO Position Determination" width="600"/>
</div>

### Step 3: Clustering using K-Means
- **Description:**
  - Once the positions of blind nodes are determined, clustering is performed using the K-Means algorithm to group the nodes efficiently.

<div align="center">
  <img src="https://raw.githubusercontent.com/pratapavinesh/EcoFly/main/Images/clusterFormation.png" alt="K-Means Clustering" width="600"/>
</div>

### Step 4: Cluster Head Selection using Improved PSO
- **Description:**
  - Improved PSO is used to select the optimal cluster head within each cluster, ensuring efficient communication and energy usage.

<div align="center">
  <img src="https://raw.githubusercontent.com/pratapavinesh/EcoFly/main/Images/clusterHeadSelection.png" alt="Cluster Head Selection" width="600"/>
</div>

### Step 5: Maintenance of Cluster Head
- **Challenge:**
  - The structure of clusters may change due to the movement of nodes.
- **Solution:**
  - Maintenance involves restructuring clusters and reselecting cluster heads as needed.

### Step 6: Determining the Next Hop
- **Description:**
  - The final step is determining the next hop for data transmission using Improved PSO, ensuring reliable and energy-efficient routing.

<div align="center">
  <img src="https://raw.githubusercontent.com/pratapavinesh/EcoFly/main/Images/routingProtocol.png" alt="Next Hop Determination" width="600"/>
</div>

## <div align="center">Technologies Used</div>
- **Algorithms:** Improved Particle Swarm Optimization (IPSO)
- **Simulation Tool:** Matlab
- **Version Control:** GitHub

## <div align="center">Project Repository</div>
- GitHub: [pratapavinesh](https://github.com/pratapavinesh/EcoFly)

## <div align="center">License</div>
Distributed under the MIT License. See `LICENSE` for more information.

---
</div>
