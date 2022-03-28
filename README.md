# Collision Aware Soft Robot Simulator Based on Geometric Computing

![Fig1](https://raw.githubusercontent.com/YingGwan/TyporaUploadImg/main/typora202203/28/135033-102406.png)

- Link: [Paper](https://arxiv.org/pdf/2203.02054.pdf) [Video](https://www.youtube.com/watch?v=DRwwh5kO4io) [Code](https://github.com/YingGwan/collisionAwareSOROSimulator)

- Authors: Guoxin Fang∗, Yingjun Tian∗, Andrew Weightman, and Charlie C.L. Wang

- IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2022), October 23-27, 2022, Kyoto, Japan, under review, 2022

  

  ## Compilation
  
  The code of our paper is compiled using [qmake](https://doc.qt.io/qt-5/qmake-running.html) and [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). A recommended IDE is Visual Studio with embedded qt tool. 
  
  Windows 10 (Recommended Platform): Visual Studio 2019 + Qt 5.12.x
  
  The installation guide of this IDE is here: [Video Demo](https://www.youtube.com/watch?v=6bXrfVrYyxk) 
  
  - After install Qt tool, open visual studio 2019, then select "Extensions" -> "Qt VS Tools" -> "Open Qt Project File (.pro)".  This will **automatically generate the Qt Vs project**.
  
  - Right click "shapeLab", set "shapeLab" as the **start-up project**
  
  - **Enable OpenMP**: ShapeLab Project Property -> 'Configuration Properties' -> c/c++ -> Language -> Open MP Support -> Select 'Yes (/openmp)'
  
  - **Open Console**: ShapeLab Project Property -> 'Configuration Proerties' -> Linker -> System -> Select 'Console (/SUBSYSTEM:CONSOLE)' in 'SubSystem'
  
  - **Dynamic Linking Library (DLL):** after compilation, you might need to move the required QT dynamic linking libraries placed in ./thirdPartyDependency/Qt5/:
  
    - If you run in release mode, copy files from ./thirdPartyDependency/Qt5/release/ to ./ShapeLab/release/
    - If you run in debug mode, copy files from ./thirdPartyDependency/Qt5/debug/ to ./ShapeLab/debug/
  
    
  
  
  
  ## Usage
  
  ### Input Meshes
  
  - Our simulator takes two **surface meshes** as input:
    - $\mathcal{S}_{b}$: Represent the **outer body shape** of soft robots. 
    - $\mathcal{S}_{c}$:  Represent the **Inner chamber shape** which can be inflated.
  
  
  
  - Our algorithm will use ***[TetGen](https://wias-berlin.de/software/tetgen/index.html)***  to generate the tetrahedral mesh inside the  $\mathcal{S}_{b}$ and being segmented by $\mathcal{S}_{c}$. 
  
  - File Location: In ./model/
  
  - We prepare two different sets of meshes in this project to demonstrate the results: 
  
    - Soft finger and its collision with ball:  file name is fingerNew_body.obj and fingerNew_chamber.obj 
    - Twisting robot: twisting_body.obj and twisting_chamber.obj
  
    
  
  ### Steps 
  
  **Soft Finger with Ball**
  
  - Change the volume expansion ratio in the **spin box** (default value: 1.3) 
  
  - Input the iteration time you want to conduct (after "**IterTime"** label)
  
  - press the "**Deformation & Collision Response**" button to conduct:
  
    - Tetrahedral mesh generation and remeshing
  
    - Deformation
  
    - Collision Checking and Response
  
      
  
  **Twisting Robot**
  
  - Press the **Input Twisting Robot** button
  - Press the **Input Obstacle** Button to read obstacle mesh
  - Change the expansion ratio in spin box under **Expansion Ratio** label
  - Press the **Conduct Deformation** button to run deformation
  - Press the **Collision Checking & Response** button to run collision checking and response
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  