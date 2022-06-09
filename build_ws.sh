#!/bin/bash

dir_init=$(pwd)
echo "  -> Initial dir = $dir_init"

echo_frmt() {
    b=0; c=0
    case $2 in 
        r*) c=31;;
        g*) c=32;;
        y*) c=33;;
        b*) c=34;;
    esac

    case $3 in
        b*) b=1;;
    esac

    echo -e "\033["$b";"$c"m$1\033[0m"
}

echo_head() {
	if [ $# -eq 1 ]
		then c="b"
		else c=$2
	fi
	echo_frmt "\n###	$1	###\n" $c b
}
	

dir_work="stride_ws"
echo_head "Build $dir_work"

cd "$dir_work"

cmd="catkin_make"; echo $cmd; $cmd

source "./devel/setup.bash"
