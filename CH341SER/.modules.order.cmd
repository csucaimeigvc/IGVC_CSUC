cmd_/home/igvc/CH341SER/modules.order := {   echo /home/igvc/CH341SER/ch34x.ko; :; } | awk '!x[$$0]++' - > /home/igvc/CH341SER/modules.order
