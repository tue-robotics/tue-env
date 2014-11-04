sudo mkdir /home/svn
sudo svnadmin create /home/svn/ros
sudo svnadmin create /home/svn/data
sudo svnadmin create /home/svn/emc
sudo useradd -G www-data subversion
sudo chown -R www-data:subversion /home/svn
sudo chmod -R g+rws /home/svn

sudo apt-get install libapache2-svn

# TODO
