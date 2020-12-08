echo $1
echo "123456" | sudo -S date "+%Y/%m/%d/ %H:%M:%S" -us "$1"
echo "EXITO"
