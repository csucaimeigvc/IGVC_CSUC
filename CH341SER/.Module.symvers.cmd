cmd_/home/igvc/CH341SER/Module.symvers := sed 's/\.ko$$/\.o/' /home/igvc/CH341SER/modules.order | scripts/mod/modpost -m   -o /home/igvc/CH341SER/Module.symvers -e -i Module.symvers   -T -
