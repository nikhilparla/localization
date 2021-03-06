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
 - commit         - large error
     - Changed the way weights are sampled. Used a placeholder particles vector
     - weight variable init to 1.0 for double
     - Removed couts now that no runtime errors
 - commit         - reset weights and changed floats to doubles
     - weights for the particles are reset to 1.0 after every iteration
     - couple of particle coordinates are initialized as floats. Should be doubles
     - Reduced the search for landmark coords from whole map to just predicted values
     - See good improvement in error, but still large
 - commit         - catch zero multiplication
     - Change the way weights are calculated when there is a very very low weight
     - Still large error but atleast the weights are updating fine
     - Observe that for many loops, highest weight and avg weight are same
 - commit         - yaw rate division check
     - If yaw rate is very less, change the prediction coordinate calculations
     - prediction vector has to be reset for every particle
 - commit         - successful
     - Added a bunch of couts for debugging
     - changed the observations vector handling. Used pushback calls
     - changed prediction step when yawrate is 0. Div by 0 catch
     - The biggest mistake was the brackets arounf cos and sin theta in map transformations. 
     - Errors min and successful


    

#### Build notes
 - Change CMakeLists.txt to include the code for multivariate gaussian dist code

#### Local Installation notes
 - the install_ubuntu file doesnt run automatically. Stops when making the symbolic links.
    I think thats because of the sudo command, but for now forced the symbolic links with -f
    flag ($ ln -s -f )
 - build.sh file has to be run with sudo, or else cannot create the ./build dir to save 
    cmake files and run make command to generate the output file