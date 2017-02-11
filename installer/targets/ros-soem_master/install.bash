SUDO_PAM_LIMITS="session    required   pam_limits.so"
PAM_SUDO_FILE="/etc/pam.d/sudo"

REALTIME_LIMITS="amigo            soft    rtprio          99\namigo            hard    rtprio          99"
LIMITS_FILE="/etc/security/limits.conf"

if ! grep -q "$SUDO_PAM_LIMITS" $PAM_SUDO_FILE ; then
    echo -e "# Load pam_limits when using sudo -u command [auto-added by soem_master install.bash] \n$SUDO_PAM_LIMITS" | sudo tee --append $PAM_SUDO_FILE
    echo -e "# Grant user amigo access to set realtime priorities [auto-added by soem_master install.bash] \n$REALTIME_LIMITS" | sudo tee --append $LIMITS_FILE
fi
