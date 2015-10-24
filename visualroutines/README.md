```
```
#
``` VisualRoutines
```
Intro to component here

Component to test the "cognitive aaffordances" theory of object perception.

This component requires de installation of the OpenMesh library. Please, download the latest sources from,

    http://www.openmesh.org/download
    
untar the file somewhere outside RoboComp directory tree and follow the installation instructions:

    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
    sudo make install
    sudo ldconfig

## Configuration parameters
As any other component,
``` *VisualRoutines* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <VisualRoutines 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```VisualRoutines ```

    --Ice.Config=config
