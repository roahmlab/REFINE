# REFINE

## Installing Dependencies
* Install Docker, following the instructions on the [Docker website](https://docs.docker.com/engine/install/ubuntu/)
* Download the following into the `data` directory:
  * `coinhsl-2021.05.05.tar.gz` from [here](https://www.hsl.rl.ac.uk/ipopt/) (ACADEMIC LICENCE)
  * `lane_change_Ay_info.mat`, `dir_change_Ay_info.mat`, `car_frs.mat`, `car_frs.txt` from the data folder [here](https://drive.google.com/drive/folders/1WZbFFhCyhYQlMJxuV4caIzNoa-Q9VZkW?usp=share_link)
  * Clone the git repos by running the following:
```bash
cd data
./download-dependencies.sh
```

## Running the Simulation
* In this directory, run `docker build -t roahm/refine_sim .`
* Run `./run-docker.sh`
* In the docker:
  1. Run `./run-matlab.sh`. MATLAB needs to be activated at the first time when it runs in the docker. In case when MATLAB requires for an account verification but fails to automatically open a browser, try to log in your account and verify the account [online](https://matlab.mathworks.com/) 
  2. Run `./build-cpp-opt.sh` to build the necessary MEX file
  3. In MATLAB, open `JL_run_highway_simulation.m` and run the script
