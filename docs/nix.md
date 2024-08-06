### Using nix
The [Nix ecosystem](https://nixos.org/) simplifies the usually error-prone process
of obtaining and setting up build dependencies, especially in a C++ project.
If you are new to Nix, please consider reading through an introduction to Nix
such as the excellent [Zero to Nix guide](https://zero-to-nix.com/).

#### Examples using Nix

##### clangd for code completion
Clangd is a C/C++ language server supported by many editors such as Visual Studio Code,
Vim/Neovim and others.
The Nix flake contained in this project includes a shell definition for clangd.
Type the following command in order to enter a clangd-enabled environment:
```
nix develop ".#clangd"
```
And then launch your preferred editor. The `shellHook` will automatically setup
a `compile_commands.json` compilation database that allows clangd to discover the
correct flags.

##### Building the package and running tests
You can simply call `nix build .` to let nix build the environment model as a package
and run tests.

##### direnv: Transparently using the Nix environment in your shell
Consider using [direnv](https://determinate.systems/posts/nix-direnv) which
automatically sets up your shell environment so that all dependencies are available
using Nix.
