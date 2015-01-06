sudo adduser amigo audio
sudo adduser amigo pulse
sudo adduser amigo pulse-access

red='\033[0;31m'
NC='\033[0m' # No Color
echo -e "${red}\nNow check if the following commands work:\n - mpg123 -o pulse <mp3-file>\n - arecord <wav-file>\n - aplay <wav-file>\n${NC}"
echo -e "${red}If not:\n\nType: aplay -l\n\nCheck which cards id we want to use\n\nSetup the ~/.asoundrc with the following content:\n\npcm.!default {\n  type hw\n  card <NUMBER>\n}\n\nctl.!default {\n  type hw\n  card <NUMBER>\n}\n\nMake sure that the correct sound device starts when you type alsamixer${NC}"
echo -e "${red}\n!! Also make sure that all required channels are not muted (mm) !!${NC}"
