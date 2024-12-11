while true
do
    cansend can0 101#FFFFFFFFFFFFFFFD
    # sleep 0.1
    cansend can0 102#FFFFFFFFFFFFFFFD
    cansend can0 103#FFFFFFFFFFFFFFFD
    cansend can0 104#FFFFFFFFFFFFFFFD
    sleep 0.1
done