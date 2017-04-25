AWS_IP=54.93.207.221

scp carnd@$AWS_IP:~/model.h5 .
python drive.py model.h5
