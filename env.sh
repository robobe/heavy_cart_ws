source /usr/share/gazebo/setup.bash
source install/setup.bash
export PS1="🐳  \[\033[1;36m\]\h \[\033[1;34m\]\W\[\033[0;35m\] \[\033[1;36m\]# \[\033[0m\]"
alias kill_gz="killall -s 9 gzserver; killall -s 9 gzclient"

