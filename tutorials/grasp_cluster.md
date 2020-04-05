# Tutorial for Dairlib (Drake) users

This is a short tutorial for how to use Dairlib on GRASP clusters. 

# Steps

## Log into the server

For security we don't allow password based logins from off campus. This means you will have to send the admin (e.g. Kenneth Chaney from Kostas's group) the public portion of an ssh key so that you can login. Follow this tutorial to set up the ssh key: [https://linuxize.com/post/how-to-set-up-ssh-keys-on-ubuntu-1804/](https://linuxize.com/post/how-to-set-up-ssh-keys-on-ubuntu-1804/)

Assuming you have the access to the server now, run 
```
ssh <your-id>@kostas-ap.seas.upenn.edu
```
to log in.
The temporary password is `PleaseChangeThis`. Run 
```
passwd
```
to change your password.

## Learn about the computing resources

Follow the tutorials at [https://github.com/daniilidis-group/k8-scripts/tree/master/tutorials](https://github.com/daniilidis-group/k8-scripts/tree/master/tutorials)

## Build and Run Dairlib

Note that after you login to the server, you will be on a login node (you will see `<your-id>@kostas-ap$` in the terminal). 
Currently the pre-reqs of Drake (Mar 23, 2020 version) has been installed. The programs (e.g. Bazel) are available on all the compute nodes (NOT the login node).    

Additionally, `/scratch` can be accessed by all the compute nodes, so it's convenient to install Dairlib in `/scratch/<your-id>/`.

It is suggested that you try getting everything working in an interactive debug first, so run 
```
srun --partition debug --qos debug --mem=8G --gres=gpu:1 --pty bash
```
to enter a compute node. 
As an example, when you are in a compute node, you can do the following to clone Dairlib
```
mkdir -p /scratch/<your-id>
cd /scratch/<your-id>
git clone https://github.com/DAIRLab/dairlib.git
```
You can do everything in the debug mode just as how you do on your computer. 

According to the [tutorials](https://github.com/daniilidis-group/k8-scripts/tree/master/tutorials), you could either use `srun` or write a .bash file and use `bash` to request resources to run programs. Here, we demonstrate how you can use `srun` to run your program. Assume that you are on the login node, and you are in `/scratch/<your-id>/dairlib/`. Run 
```
srun --partition=compute --qos=low --mincpus=18 --mem-per-cpu=4G --gres=gpu:0 bazel build --define=WITH_SNOPT=OFF examples/Cassie:run_dircon_squatting
srun --partition=compute --qos=low --mincpus=18 --mem-per-cpu=4G --gres=gpu:0 ./bazel-bin/examples/Cassie/run_dircon_squatting
```

# For SNOPT users

Here is one way to upload SNOPT from your computer to the server: 
```
scp <snopt-path-on-your-computer> <your-id>@kostas-ap.seas.upenn.edu:/scratch/<your-id>/
```
where `<snopt-path-on-your-computer>` looks like `/home/<username>/snopt7.6.tar.gz`

Remember to add the path by 
```
export SNOPT_PATH=<the directory you upload to>/snopt7.6.tar.gz
```
which should be added to `~/.bashrc` if you don't want to add it every time you log in.
