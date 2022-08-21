# Collision Aware Soft Robot Simulator Based on Geometric Computing

![Fig1](https://raw.githubusercontent.com/YingGwan/TyporaUploadImg/main/typora202203/28/135033-102406.png)

- Link: [Paper](https://arxiv.org/pdf/2203.02054.pdf) [Video](https://www.youtube.com/watch?v=cI8em6bpqKw) [Code](https://github.com/YingGwan/collisionAwareSOROSimulator)

- Authors: Guoxin Fang∗, Yingjun Tian∗, Andrew Weightman, and Charlie C.L. Wang

- IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2022), October 23-27, 2022, Kyoto, Japan, accepted, 2022

  

  ## Compilation

  The code of our paper is compiled using [qmake](https://doc.qt.io/qt-5/qmake-running.html) and [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). A recommended IDE is Visual Studio with embedded qt tool. 

  Windows 10 (Recommended Platform): Visual Studio 2019 + Qt 5.12.x

  The installation guide of this IDE is here: [Video Demo](https://www.youtube.com/watch?v=6bXrfVrYyxk) 

  - After install Qt tool, open visual studio 2019, then select "Extensions" -> "Qt VS Tools" -> "Open Qt Project File (.pro)".  This will **automatically generate the Qt Vs project**.

  - Right click "shapeLab", set "shapeLab" as the **start-up project**

  - **Enable OpenMP**: ShapeLab Project Property -> 'Configuration Properties' -> c/c++ -> Language -> Open MP Support -> Select 'Yes (/openmp)'

  - **Open Console**: ShapeLab Project Property -> 'Configuration Proerties' -> Linker -> System -> Select 'Console (/SUBSYSTEM:CONSOLE)' in 'SubSystem'

  - **Dynamic Linking Library (DLL):** after compilation, you might need to move the required QT dynamic linking libraries placed in ./thirdPartyDependency/QT_DLL/:

    - If you run in release mode, copy files from ./thirdPartyDependency/QT_DLL/release/ to ./ShapeLab/release/
    - If you run in debug mode, copy files from ./thirdPartyDependency/QT_DLL/debug/ to ./ShapeLab/debug/

    

  

  ## Usage

  - We provide

    

  

  

  

  

  

  

  

  

  

  

  
