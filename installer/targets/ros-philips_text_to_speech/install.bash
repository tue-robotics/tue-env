dest=$TUE_ENV_DIR/system/src/philips_text_to_speech

if [ ! -d $dest ]
then
    echo "
------------------------------------------------------------
                   PHILIPS TEXT TO SPEECH    
------------------------------------------------------------

    Software is propietary, so a password is needed to
    download the package. Please provide password needed
    to log-in to data@roboticssrv.wtb.tue.nl.

"

    mkdir -p $TUE_ENV_DIR/repos/data
    rsync data@roboticssrv.wtb.tue.nl:/home/data/data/private/data-pkgs/philips_text_to_speech $TUE_ENV_DIR/repos/data/ -av

    ln -s $TUE_ENV_DIR/repos/data/philips_text_to_speech  $dest
fi
