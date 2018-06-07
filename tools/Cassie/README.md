# Battery Status Checker
The battery status checker requires `python3.` Assuming you have this, it also requires
```
sudo apt install python3-serial
```
To run the script, call
```
python3 BMSStatusChecker.py
```
Note: the user must also have access to the USB device. One option is to instead run
```
sudo python3 BMSStatusChecker.py
```
Alternatively (which has alreayd been done on the operator station `dair01`), you can add the user to the group dialout.
```
sudo adduser $USER dialout
```