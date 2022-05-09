#!/bin/sh
module="sbull"

echo -n "enter command: "
read cmd

if [ -n "$cmd" ]; then
  case "$cmd" in
    "load" )
      echo "$cmd command, got it"
      /sbin/insmod -f ./$module.ko
      ;;
    "unload" )
      echo "$cmd command, got it"
      /sbin/rmmod $module $*
      ;;
    * )
      echo "wrong command, please try again"
      ;;
    esac
else
  echo "no cmd found =("
fi
