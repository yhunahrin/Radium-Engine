git submodule radium in my own project
For instance Radium in ./myproject/external/Radium-Engine

```
mkdir external
git submodule add https://github.com/AGGA-IRIT/Radium-Engine/ 
external/Radium-Engine --recursive
git submodule update --init --recursive
```

This corresponds to 
```
.gitmodules
[submodule "Radium-Engine"]
        path = external/Radium-Engine
        url = https://github.com/AGGA-IRIT/Radium-Engine
        branch = master
```


Add a line to include the Radium-Engine cmake module in your main
CMakeLists.txt
```
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/Radium-Engine/CMakeModules)
```


add a ``find_package(Radium REQUIRED)`` in CMakeLists.txt of your
applications that need. If needed add ``set(RADIUM_ROOT "../external/Radium-Engine")``

This will def the following cmake vars 
```
    ${RADIUM_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
    ${ASSIMP_INCLUDE_DIR}
    ${RADIUM_LIB_DIR}
```


access to Radium headers and declarations/defintions
```
include_directories(
    .
    ${RADIUM_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
    ${ASSIMP_INCLUDE_DIR}
)
```

```
# Libs include
link_directories(
    ${RADIUM_LIB_DIR}
)
```

```
# Link good libraries
target_link_libraries( ${EXEC_FILE} # target
    ${RADIUM_LIB_CORE}              # core
    ${RADIUM_LIB_ENGINE}            # engine
    ${RADIUM_LIB_GUIBASE}           # gui
    ${Qt5_LIBRARIES}                # the Qt beast
)
```

Then build radium (as debug and release)
cd external
```
mkdir build-radium-debug
cd build-radium-debug
cmake ../Radium-Engine -DCMAKE_BUILD_TYPE=Debug
make -j8
cd ..
mkdir build-radium-release
cmake ../Radium-Engine -DCMAKE_BUILD_TYPE=Release
make -j8
```


