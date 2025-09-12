[![INFORMS Journal on Computing Logo](https://INFORMSJoC.github.io/logos/INFORMS_Journal_on_Computing_Header.jpg)](https://pubsonline.informs.org/journal/ijoc)

# TA-MAPF

This archive is distributed in association with the [INFORMS Journal on
Computing](https://pubsonline.informs.org/journal/ijoc) under the [Apache License](http://www.apache.org/licenses/LICENSE-2.0).

The software and data in this repository are a snapshot of the software and data
that were used in the research reported in the paper 
[Solving the Multi-Agent Pathfinding Problem with
Time-Expanded Networks](https://doi.org/10.1287/ijoc.2024.0951) by Tommaso Adamo, Roberto Baldacci, Gianpaolo Ghiani and Emanuela Guerriero. 
The snapshot is based on 
[this SHA](https://github.com/geass88/TA_MAPF/commit/0dcab5493e5b15fd12eb540a8cffb513cf0c6777) 
in the development repository. 

**Important: This code is being developed on an on-going basis at 
[https://github.com/geass88/TA_MAPF](https://github.com/geass88/TA_MAPF). Please go there if you would like to
get a more recent version or would like support**

## Cite

To cite the contents of this repository, please cite both the paper and this repo, using their respective DOIs.

https://doi.org/10.1287/ijoc.2024.0951

https://doi.org/10.1287/ijoc.2024.0951.cd

Below is the BibTex for citing this snapshot of the repository.

```
@misc{TAMAPF,
  author =        {Adamo, Tommaso and Baldacci, Roberto and Ghiani, Gianpaolo and Guerriero, Emanuela},
  publisher =     {INFORMS Journal on Computing},
  title =         {Solving the Multi-Agent Pathfinding Problem with
Time-Expanded Networks},
  year =          {2025},
  doi =           {10.1287/ijoc.2024.0951.cd},
  url =           {https://github.com/INFORMSJoC/2024.0951},
  note =          {Available for download at https://github.com/INFORMSJoC/2024.0951},
}  
```

## Description

The goal of this software is to provide an exact solver for the Multi-Agent Pathfinding Problem using Time-Expanded Networks.

## Requirements

In Linux, to build the version of the software included in this archive, you need to satisfy the following requirements.

Install [Boost](https://www.boost.org/) C++ Libraries.

```bash
sudo apt-get install g++ cmake make
wget https://boostorg.jfrog.io/artifactory/main/release/1.83.0/source/boost_1_83_0.tar.gz
tar xfv boost_1_83_0.tar.gz
cd boost_1_83_0
./bootstrap.sh 
./b2 
```

This project also requires [IBM ILOG CPLEX Optimization Studio](https://www.ibm.com/it-it/products/ilog-cplex-optimization-studio).

IBM offers free academic licenses for CPLEX:
1. Register with your **academic email address** at the [IBM Academic Initiative](https://academic.ibm.com/).
2. Once approved, you can **download IBM ILOG CPLEX Optimization Studio** from the Academic Initiative portal.
3. Select the appropriate version for your operating system (Linux, Windows, macOS).
4. Follow the installation instructions provided in the package.

In Linux, install [cplex](https://www.ibm.com/it-it/products/ilog-cplex-optimization-studio) using the following commands:

```bash
chmod u+x cplex_studio2212.linux_x86_64.bin
./cplex_studio2212.linux_x86_64.bin
```

Set BOOST_ROOT, CPLEX_ROOT_DIR and WDIR (i.e. current working directory) environment variables in CMakeLists.txt

## Building

Use [cmake](https://cmake.org/) to configure the software.

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

The software can be run with the following command:

```bash
./build/TA_MAPF <params.txt>
```

The list of available parameters can be found in the file `params.txt`.

## Visualization

A script for visualizing the result can be found in the scripts folder. Here a simple usage example is shown:

```bash
python3 scripts/visualize.py data/M8/M8_agents2_ex35.yaml out.yaml
```

## Ongoing Development

This code is being developed on an on-going basis at the author's
[Github site](https://github.com/geass88/TA_MAPF).

## Support

For support in using this software, submit an
[issue](https://github.com/geass88/TA_MAPF/issues/new).
