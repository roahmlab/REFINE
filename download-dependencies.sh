cd util/

# Clone dependency repositories
git clone --depth=1 https://github.com/skousik/simulator
git clone --depth=1 https://github.com/pdholmes/zono_RTD_turtlebot_example
git clone --depth=1 https://github.com/ramvasudevan/RTD
git clone --depth=1 https://github.com/skousik/RTD_tutorial

# Install CORA
wget https://tumcps.github.io/CORA/data/archive/version/CORA_2018.zip && \
  unzip CORA_2018.zip
  
rm -f CORA_2018.zip  
  
cd ../Offline_Reachability_Analysis
mkdir FRSdata
cd ..
