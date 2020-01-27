## CarND localization project
#### Implementation notes 
 - commit 6c05ab4 - Initialized the pf init function with weights and measurements
 - commit 48d0625 - Updated the prediction step for one cycle with the given velocity and yaw rate
 - commit 910e9bb - Completed the implementation part. 
 - commit         - Segmentation fault resolved. 
     - Vectors have to assigned a size before populating them
     - Used vector.resize() to initialize all before assignment
     - In updateWeights(), break for loop once landmark has been found
     - Reduce num_particles to 50 for initial trail

#### Build notes
 - Change CMakeLists.txt to include the code for multivariate gaussian dist code

#### Local Installation notes
 - the install_ubuntu file doesnt run automatically. Stops when making the symbolic links.
    I think thats because of the sudo command, but for now forced the symbolic links with -f
    flag ($ ln -s -f )
 - build.sh file has to be run with sudo, or else cannot create the ./build dir to save 
    cmake files and run make command to generate the output file