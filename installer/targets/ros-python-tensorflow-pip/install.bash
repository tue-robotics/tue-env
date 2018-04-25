if [ -n "$TUE_CUDA" ]
then
    tue-install-pip 'tensorflow-gpu>=1.4.1'
else
    tue-install-pip 'tensorflow>=1.4.1'
fi
