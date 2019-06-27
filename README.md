# UMA_VI Dataset Evaluation Tools

Useful C++ and Python tools for the evaluation of visual-inertial odometry/SLAM methods with the UMA-VI dataset

**Authors:** [David Zuñiga-Noël](http://mapir.isa.uma.es/mapirwebsite/index.php/people/270), [Alberto Jaenal](http://mapir.uma.es/mapirwebsite/index.php/people/273), [Ruben Gomez-Ojeda](http://mapir.isa.uma.es/mapirwebsite/index.php/people/164-ruben-gomez), [Javier Gonzalez-Jimenez](http://mapir.isa.uma.es/mapirwebsite/index.php/people/95-javier-gonzalez-jimenez)

**License:**  [GPLv3](https://raw.githubusercontent.com/dzunigan/uma_vi_tools/master/LICENSE.txt)

## 1. Dependencies (C++ only)

* Boost (1.58.0.1ubuntu1 tested)
   ```
   sudo apt install libboost-all-dev
   ```
* CMake (3.5.1-1ubuntu1 tested)
   ```
   sudo apt install cmake
   ```
* Eigen (3.3~beta1-2 tested)
   ```
   sudo apt install libeigen3-dev
   ```
* Gflags (2.1.2-3 tested)
   ```
   sudo apt install libgflags-dev
   ```

## 2. Build

Once all dependencies are installed, proceed to build the source code with the automated build script provided. Simply run the following commands:
```
git clone --recursive https://github.com/dzunigan/uma_vi_tools.git
cd uma_vi_tools
bash build.sh -j2
```

The compiled executable should be inside the `bin` directory.

## 3. Usage


