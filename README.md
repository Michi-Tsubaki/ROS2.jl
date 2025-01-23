# ROJ.jl

## Introduction
ROJ (Robot Operation using Julia) is a Julia-based tool for robot development, supporting both real robots and the MuJoCo simulator.

## Install Julia
### For Linux Users
To install the latest version of Julia (v1.11.3 as of January 21, 2025), run the following command in your terminal:
```
curl -fsSL https://install.julialang.org | sh -s -- --yes
```
The installed Julia will be located at:
```
 ${HOME}/.juliaup/bin/julia
```
The `~/.bashrc` file will be automatically modified to include Julia in the PATH. After this, simply typing `julia` will start the Julia REPL.

To verify the installation, run the following in your terminal:
```
julia --version
```
You should see output like `julia version 1.11.3` if the installation was successful.

If you want to update version of julia language, run this in your terminal:
```
juliaup update
```
For more details or if you're using Windows, visit the official download page at `https://julialang.org/downloads/`.

### Setting Environment Variables
To automatically activate the environment corresponding to the `Project.toml` file, run the following in your terminal:
```
export JULIA_PROJECT=@.
```
This will ensure that Julia automatically uses the correct environment when running scripts.

## Cloning the GitHub Repository
To clone the ROJ.jl repository from GitHub, run the following command in your terminal:
```
git clone https://github.com/Michi-Tsubaki/ROJ.jl.git
```
Once cloned, navigate into the repository directory:
```
cd ROJ.jl
```
Then, you can install the necessary dependencies and start working with the project. Welcome to our project!

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

© 2025 Michitoshi Tsubaki (@Michi-Tsubaki)
