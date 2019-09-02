 通过官方PPA安装Ubuntu make

sudo add-apt-repository ppa:ubuntu-desktop/ubuntu-make
sudo apt-get update
sudo apt-get install ubuntu-make
2. 使用命令安装visual studio code



// https://stackoverflow.com/questions/53204589/ubuntu-how-to-complete-purge-visual-studio-code
卸载已经安装的VSCode

umake ide visual-studio-code  --remove


sudo apt-get remove code
sudo apt-get purge code
sudo apt-get update


User specific settings/extensions are located in $HOME/.config/Code and $HOME/.vscode/   .vscode-cpptools
  so remove those folders manually