cmd_/home/igvc/CH341SER/ch34x.mod := printf '%s\n'   ch34x.o | awk '!x[$$0]++ { print("/home/igvc/CH341SER/"$$0) }' > /home/igvc/CH341SER/ch34x.mod
