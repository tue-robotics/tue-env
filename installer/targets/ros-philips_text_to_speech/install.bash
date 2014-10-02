if [ ! -d ~/ros/hydro/system/src/philips_text_to_speech ]
then
   URL=https://roboticssrv.wtb.tue.nl/svn/3rdparty/philips_text_to_speech
   DEST="$HOME/ros/$ROS_DISTRO/repos/https://roboticssrv.wtb.tue.nl/svn/3rdparty/philips_text_to_speech"
   LINK=$TUE_SYSTEM_DIR/src/philips_text_to_speech

    tue-install-warning """Cannot install 'philips_text_to_speech' automatically because it is password protected. Please run manually:

    svn co $URL \\
           '$DEST' && \\
    ln -s  '$DEST' \\
           $LINK
"""

fi
